import { useEffect, useMemo, useRef, useState } from "react";
import type { Field, FieldStats } from "../field/Field";
import { FieldView } from "../field/FieldView";
import { IcpDemo, type IcpMethod } from "../icp/IcpDemo";
import { formatLength } from "../lib/frames";
import { Chip, Segmented, driveChip, type Tone } from "./ui";

type LabMode = "match" | "lost" | "plan";

const MODES: Record<LabMode, { n: number; tab: string; title: string; hint: string }> = {
  match: {
    n: 1, tab: "Match two scans", title: "Match two scans",
    hint: "Drag the blue scan anywhere on the floor and let go. The engine pairs each blue point with its nearest black one and pulls the scan back, pass by pass. Drag it far, and it can settle in the wrong place.",
  },
  lost: {
    n: 2, tab: "Get it lost", title: "Get the robot lost",
    hint: "Two estimators get the same scans. The blue ring matches each scan against a map of the house; the gray ring only compares scans with each other, so its errors add up. Shorten the range, add noise, or move furniture so the house no longer matches the map.",
  },
  plan: {
    n: 3, tab: "Rearrange the house", title: "Rearrange the house",
    hint: "Drag any furniture, or click the floor to drop a box. The planner searches a 5 cm grid and finds a new route. Block the doorway and see what happens.",
  },
};

const DEFAULT_RANGE = 8;
const DEFAULT_NOISE_CM = 1;

export function Lab() {
  const [mode, setMode] = useState<LabMode>("match");
  const [stats, setStats] = useState<FieldStats | null>(null);
  const [range, setRange] = useState(DEFAULT_RANGE);
  const [noiseCm, setNoiseCm] = useState(DEFAULT_NOISE_CM);
  const [method, setMethod] = useState<IcpMethod>("p2l");
  const [icpText, setIcpText] = useState({ tone: "run" as Tone, chip: "Loading engine", pass: "–", error: "–", rmse: "–" });
  const panel = useRef<HTMLDivElement>(null);
  const field = useRef<Field | null>(null);
  const icp = useMemo(() => new IcpDemo(), []);

  useEffect(() => void icp.load(), [icp]);

  useEffect(() => field.current?.setSensor(range, noiseCm / 100), [range, noiseCm]);

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
      ["Cells searched", (stats?.expanded ?? 0).toLocaleString()],
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
                  <div className="row">
                    <button type="button" className="btn ghost" onClick={() => field.current?.restartTracker()}>Re-map and restart</button>
                  </div>
                  <ul className="legend" aria-label="Legend">
                    <li><span className="sw sw-map" aria-hidden="true" />With map: scan-to-map</li>
                    <li><span className="sw sw-base" aria-hidden="true" />No map: scan-to-scan</li>
                    <li><span className="sw sw-dots" aria-hidden="true" />The map{stats?.mapPoints ? ` (${stats.mapPoints.toLocaleString()} points)` : ""}</li>
                  </ul>
                  <p className="fine">Both C++ estimators get only the scans and the starting pose. The map is surveyed from the house as it was at restart. Errors compare with the simulator's answer key; map match is the share of scan points that fit the map.</p>
                </>
              )}
              {mode === "plan" && (
                <>
                  <div className="row">
                    <button type="button" className="btn" onClick={() => field.current?.dropRandomBox()}>Drop a box</button>
                    <button type="button" className="btn ghost" onClick={() => field.current?.clearBoxes()}>Clear boxes</button>
                    <button type="button" className="btn ghost" onClick={() => field.current?.resetRoom()}>Reset room</button>
                  </div>
                  <p className="fine">Simplified planner: it uses the known floor plan and the robot's true position, not its estimate.</p>
                </>
              )}
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}
