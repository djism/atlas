import { useEffect, useMemo, useRef, useState } from "react";
import { MAX_BOXES, type Field, type FieldStats } from "../field/Field";
import { FieldView } from "../field/FieldView";
import { IcpDemo, type IcpMethod } from "../icp/IcpDemo";
import { formatLength } from "../lib/frames";
import { decodeLab, encodeLab, type LabLimits, type LabMode } from "../lib/share";
import { homeScenario } from "../scenarios/home";
import { Chip, Segmented, driveChip, type Tone } from "./ui";

const MODES: Record<LabMode, { n: number; tab: string; title: string; hint: string }> = {
  match: {
    n: 1, tab: "Match two scans", title: "Match two scans",
    hint: "Drag the blue scan anywhere on the floor and let go. The engine pairs each blue point with its nearest black one and pulls the scan back, pass by pass. Drag it far, and it can settle in the wrong place.",
  },
  lost: {
    n: 2, tab: "Get it lost", title: "Get the robot lost",
    hint: "The blue ring matches each scan against a map of the house; the gray ring only compares scans with each other, so its errors add up. The robot steers by the blue ring. Shorten the range, add noise, or move furniture, then switch the motion guess and watch the difference.",
  },
  plan: {
    n: 3, tab: "Rearrange the house", title: "Rearrange the house",
    hint: "Drag any furniture, or click the floor to drop a box. The planner searches a 5 cm grid from where the robot thinks it is (blue ring) and finds a new route. Block the doorway and see what happens.",
  },
};

const DEFAULT_RANGE = 8;
const DEFAULT_NOISE_CM = 1;
const LIMITS: LabLimits = {
  range: { min: 1, max: 8, step: 0.5, default: DEFAULT_RANGE },
  noiseCm: { min: 0, max: 5, step: 0.5, default: DEFAULT_NOISE_CM },
  maxBoxes: MAX_BOXES,
  width: homeScenario.bounds.width,
  height: homeScenario.bounds.height,
};
const FURNITURE_IDS = homeScenario.furniture.map((f) => f.id);

export function Lab() {
  // A shared link (?lab=...) sets the starting tab, sensor, guess, and furniture.
  const [shared] = useState(() => decodeLab(window.location.search, FURNITURE_IDS, LIMITS));
  const [mode, setMode] = useState<LabMode>(shared?.mode ?? "match");
  const [stats, setStats] = useState<FieldStats | null>(null);
  const [range, setRange] = useState(shared?.range ?? DEFAULT_RANGE);
  const [noiseCm, setNoiseCm] = useState(shared?.noiseCm ?? DEFAULT_NOISE_CM);
  const [wheels, setWheels] = useState(shared?.wheels ?? true);
  const [copied, setCopied] = useState<"idle" | "done" | "failed">("idle");
  const [method, setMethod] = useState<IcpMethod>("p2l");
  const [icpText, setIcpText] = useState({ tone: "run" as Tone, chip: "Loading engine", pass: "–", error: "–", rmse: "–" });
  const panel = useRef<HTMLDivElement>(null);
  const field = useRef<Field | null>(null);
  const icp = useMemo(() => new IcpDemo(), []);

  useEffect(() => void icp.load(), [icp]);

  // Opened from a shared link: rebuild the house and bring the lab into view.
  useEffect(() => {
    if (!shared) return;
    if (shared.mode !== "match") field.current?.loadLayout(shared.room, shared.mapped);
    document.getElementById("lab")?.scrollIntoView({ block: "start" });
  }, [shared]);

  // The address bar always holds a link to the current setup (replaceState: no
  // history entries). The untouched default leaves the URL clean.
  const link = () => {
    const f = field.current;
    // Without a field (no WebGL), keep the shared layout rather than erase it.
    const empty = { moved: {}, boxes: [] };
    const room = f?.layout() ?? shared?.room ?? empty;
    const mapped = f?.mappedLayout() ?? shared?.mapped ?? empty;
    return encodeLab({ mode, range, noiseCm, wheels, room, mapped }, LIMITS);
  };
  useEffect(() => {
    const search = link();
    const want = search === "?lab=match" && !shared ? "" : search;
    if (want !== window.location.search) {
      window.history.replaceState(null, "", `${window.location.pathname}${want}${window.location.hash}`);
    }
  });

  const copyLink = async () => {
    const url = `${window.location.origin}${window.location.pathname}${link()}#lab`;
    try {
      await navigator.clipboard.writeText(url);
      setCopied("done");
    } catch {
      setCopied("failed");
    }
    window.setTimeout(() => setCopied("idle"), 2500);
  };
  const share = (
    <button type="button" className="btn ghost" onClick={copyLink} aria-live="polite">
      {copied === "done" ? "Link copied" : copied === "failed" ? "Copy from the address bar" : "Copy link"}
    </button>
  );

  useEffect(() => field.current?.setSensor(range, noiseCm / 100), [range, noiseCm]);
  useEffect(() => {
    if (field.current) field.current.wheels = wheels;
  }, [wheels, mode]);

  // ICP readouts (the field draws the scans; this mirrors the numbers at 10 Hz)
  useEffect(() => {
    if (mode !== "match") return;
    const id = window.setInterval(() => {
      if (!icp.loaded) return;
      const frame = icp.frame(performance.now());
      const v = icp.verdict(frame);
      const rmse = icp.rmseAt(frame.pass);
      setIcpText({
        tone: v.tone, chip: v.text,
        pass: frame.passes ? `${frame.pass} / ${frame.passes}` : "0",
        error: formatLength(icp.errors(frame.pose).position),
        rmse: rmse === null ? "–" : formatLength(rmse),
      });
    }, 100);
    return () => window.clearInterval(id);
  }, [mode, icp]);

  const choose = (m: LabMode) => {
    if (m === mode) return;
    if (m === "lost") {
      setRange(DEFAULT_RANGE);
      setNoiseCm(DEFAULT_NOISE_CM);
      setWheels(true);
      field.current?.setSensor(DEFAULT_RANGE, DEFAULT_NOISE_CM / 100);
    }
    setMode(m);
  };

  const info = MODES[mode];
  let chip: { tone: Tone; text: string };
  let metrics: [string, string][];
  if (mode === "match") {
    chip = { tone: icpText.tone, text: icpText.chip };
    metrics = [["Pass", icpText.pass], ["Error", icpText.error], ["Pair RMSE", icpText.rmse]];
  } else if (mode === "lost") {
    const s = stats;
    // The chip follows the map localizer; the gray tracker is the comparison.
    if (!s || s.engine !== "ready") chip = driveChip(s);
    else if (s.mapError === null) chip = { tone: "run", text: "Mapping the house" };
    else if (s.bumped) chip = { tone: "bad", text: "Bumped: it was not where it thought" };
    else if (s.mapLost) chip = { tone: "bad", text: "Lost this scan: predicting" };
    else if (s.mapWeak) chip = { tone: "warn", text: "Walls don't pin it down" };
    else if (s.inliers !== null && s.inliers < 0.8) chip = { tone: "warn", text: "House differs from the map" };
    else if (s.mapError < 0.05) chip = { tone: "ok", text: "On track with the map" };
    else chip = { tone: "warn", text: "Off course" };
    metrics = [
      ["With map", s?.mapError == null ? "–" : formatLength(s.mapError)],
      ["No map", s?.error == null ? "–" : formatLength(s.error)],
      ["Map match", s?.inliers == null ? "–" : `${Math.round(s.inliers * 100)}%`],
    ];
  } else {
    chip = driveChip(stats);
    metrics = [
      ["Plans made", String(stats?.plans ?? 0)],
      ["Last plan", `${(stats?.lastPlanMs ?? 0).toFixed(1)} ms`],
      ["Bumps", String(stats?.bumps ?? 0)],
    ];
  }

  return (
    <section className="lab" id="lab">
      <div className="lab-head wrap">
        <div>
          <p className="eyebrow">The lab</p>
          <h2 className="big">Try it yourself.</h2>
        </div>
        <div className="tabs" role="tablist" aria-label="Experiments">
          {(Object.keys(MODES) as LabMode[]).map((m) => (
            <button key={m} role="tab" id={`tab-${m}`} aria-selected={m === mode} aria-controls="lab-panel" onClick={() => choose(m)}>
              {MODES[m].tab}
            </button>
          ))}
        </div>
      </div>
      <div className="lab-stage">
        <FieldView mode={mode} icp={icp} panelRef={panel} fieldRef={field} onStats={setStats} className="lab-field" label={`Experiment: ${info.title}`}>
          <div className="f-ui br"><span className="pill mono">Live C++ engine · Simulated world</span></div>
        </FieldView>
        <div className="panel" id="lab-panel" role="tabpanel" aria-labelledby={`tab-${mode}`} ref={panel} aria-live="polite">
          <div className="panel-body" key={mode}>
            <div className="p-top">
              <Chip tone={chip.tone}>{chip.text}</Chip>
              <span className="p-kicker">{info.n} of 3</span>
            </div>
            <h3>{info.title}</h3>
            <p>{info.hint}</p>
            <dl className="metrics" title="Error compares the engine's answer with the simulator's answer key">
              {metrics.map(([k, v]) => (
                <div key={k}>
                  <dt>{k}</dt>
                  <dd>{v}</dd>
                </div>
              ))}
            </dl>
            <div className="controls">
              {mode === "match" && (
                <>
                  <Segmented
                    className="wide"
                    label="Matching method"
                    value={method}
                    options={[{ value: "p2l", label: "Point-to-line" }, { value: "p2p", label: "Point-to-point" }]}
                    onChange={(m) => {
                      setMethod(m);
                      icp.setMethod(m);
                    }}
                  />
                  <div className="row">
                    <button type="button" className="btn ghost" onClick={() => icp.replay()}>Replay</button>
                    {share}
                  </div>
                </>
              )}
              {mode === "lost" && (
                <>
                  <div>
                    <label htmlFor="lab-range"><span>Sensor range</span><output htmlFor="lab-range">{range.toFixed(1)} m</output></label>
                    <input id="lab-range" type="range" min={1} max={8} step={0.5} value={range} onChange={(e) => setRange(+e.target.value)} />
                  </div>
                  <div>
                    <label htmlFor="lab-noise"><span>Sensor noise</span><output htmlFor="lab-noise">{noiseCm.toFixed(1)} cm</output></label>
                    <input id="lab-noise" type="range" min={0} max={5} step={0.5} value={noiseCm} onChange={(e) => setNoiseCm(+e.target.value)} />
                  </div>
                  <Segmented
                    className="wide"
                    label="Blue ring's motion guess"
                    value={wheels ? "wheels" : "last"}
                    options={[{ value: "wheels", label: "Wheel odometry" }, { value: "last", label: "Repeat last motion" }]}
                    onChange={(v) => setWheels(v === "wheels")}
                  />
                  <div className="row">
                    <button type="button" className="btn ghost" onClick={() => field.current?.restartTracker()}>Re-map and restart</button>
                    {share}
                  </div>
                  <ul className="legend" aria-label="Legend">
                    <li><span className="sw sw-map" aria-hidden="true" />With map: scan-to-map</li>
                    <li><span className="sw sw-base" aria-hidden="true" />No map: scan-to-scan</li>
                    <li><span className="sw sw-dots" aria-hidden="true" />The map{stats?.mapPoints ? ` (${stats.mapPoints.toLocaleString()} points)` : ""}</li>
                  </ul>
                  <p className="fine">The C++ estimators get the scans and the starting pose. Wheel odometry is simulated with assumed noise (2% bias, 5% random). The gray ring always repeats its last motion. The map is surveyed at restart. Errors compare with the simulator's answer key.</p>
                </>
              )}
              {mode === "plan" && (
                <>
                  <div className="row">
                    <button type="button" className="btn" onClick={() => field.current?.dropRandomBox()}>Drop a box</button>
                    <button type="button" className="btn ghost" onClick={() => field.current?.clearBoxes()}>Clear boxes</button>
                    <button type="button" className="btn ghost" onClick={() => field.current?.resetRoom()}>Reset room</button>
                    {share}
                  </div>
                  <p className="fine">The robot plans and steers from its C++ map estimate, not its true position. Simplification: the planner is given the true floor plan; a real robot would plan on its own map.</p>
                </>
              )}
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}
