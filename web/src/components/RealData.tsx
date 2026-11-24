import { useMemo, useState } from "react";
import { KITTI_RECORDINGS } from "../data/kittiPaths";
import { fitPaths, pairedBars, type PathName } from "../lib/kittiPlot";
import { replayQuery } from "../lib/replay";

const replayHref = `${import.meta.env.BASE_URL}replay/`;

// Recorded results, not computed in the browser. Numbers come from
// `atlas_kitti score` and the streaming runs in docs/kitti.md (Apple M4).
type Seq = "07" | "04";
interface Row { drift: string; turn: string; ms: string; msNote?: string; mem: string }
const RESULTS: Record<Seq, {
  tag: string; title: string; line: string; verdict: string; caption: string;
  best: "atlas" | "kiss"; atlas: Row; kiss: Row; noMap: Row;
}> = {
  "07": {
    tag: "held out",
    title: "Sequence 07 · city loop",
    line: "694 m through town, four sharp turns and a stop, ending 9.5 m from where it began. 1,101 scans.",
    verdict: "Atlas ahead on this drive",
    caption:
      "Seen from above; up is the car's first heading. At this scale Atlas and KISS-ICP sit on the true path; the bars show the difference. Without a map, small turning errors add up and the path swings away at every corner.",
    best: "atlas",
    atlas: { drift: "0.43 %", turn: "0.25°/100 m", ms: "28 ms", mem: "106 MB" },
    kiss: { drift: "0.75 %", turn: "0.37°/100 m", ms: "47 ms", msNote: "15 ms, all cores", mem: "195 MB" },
    noMap: { drift: "5.46 %", turn: "3.18°/100 m", ms: "32 ms", mem: "" },
  },
  "04": {
    tag: "tuned on",
    title: "Sequence 04 · country road",
    line: "394 m almost straight down a country road. 271 scans. Atlas's settings were chosen while studying this drive.",
    verdict: "KISS-ICP ahead on this drive",
    caption:
      "Seen from above; up is the car's first heading. On a straight road the drift is mostly along the road, so it barely shows here; without a map the path slowly leans left. KISS-ICP's largest errors are in the first 30 scans.",
    best: "kiss",
    atlas: { drift: "0.48 %", turn: "0.12°/100 m", ms: "43 ms", mem: "78 MB" },
    kiss: { drift: "0.41 %", turn: "0.19°/100 m", ms: "71 ms", msNote: "19 ms, all cores", mem: "148 MB" },
    noMap: { drift: "3.70 %", turn: "1.92°/100 m", ms: "59 ms", mem: "" },
  },
};

// Drawn back to front: the true path last, thin, on top.
const SERIES: { id: PathName; name: string; cls: string }[] = [
  { id: "noMap", name: "Atlas without a map", cls: "rd-nomap" },
  { id: "kiss", name: "KISS-ICP", cls: "rd-kiss" },
  { id: "atlas", name: "Atlas", cls: "rd-atlas" },
  { id: "truth", name: "True path (GPS/IMU)", cls: "rd-truth" },
];
const LEGEND_ORDER: PathName[] = ["truth", "atlas", "kiss", "noMap"];

export function RealData() {
  const [seq, setSeq] = useState<Seq>("07");
  const [hidden, setHidden] = useState<Partial<Record<PathName, boolean>>>({});
  const rec = KITTI_RECORDINGS[seq];
  const r = RESULTS[seq];
  const plot = useMemo(() => fitPaths(rec.paths), [rec]);
  const bars = useMemo(() => pairedBars(rec.stretchErrorCm.atlas, rec.stretchErrorCm.kiss), [rec]);
  const rows = [
    { id: "atlas" as const, name: "Atlas, scan-to-map", v: r.atlas },
    { id: "kiss" as const, name: "KISS-ICP 1.3.0", v: r.kiss },
    { id: "noMap" as const, name: "Atlas, no map", v: r.noMap },
  ];

  return (
    <section className="realdata wrap" id="results" aria-labelledby="results-title">
      <div className="inside-head">
        <div>
          <p className="eyebrow">Real data · recorded results</p>
          <h2 className="big" id="results-title">Tested on a real car's LiDAR.</h2>
        </div>
        <p>
          The same C++ engine, run on recorded drives from the KITTI dataset: a 64-beam laser scanner on a car in
          Karlsruhe, about ten 360° scans a second. Atlas estimates where the car went from the scans alone; the GPS/IMU
          path is only the answer key.
        </p>
      </div>

      <div className="rd-bar">
        <div className="seg rd-seg" role="group" aria-label="Choose a drive">
          {(["07", "04"] as const).map((id) => (
            <button key={id} type="button" aria-pressed={seq === id} onClick={() => setSeq(id)}>
              Sequence {id} <span className="rd-tag">{RESULTS[id].tag}</span>
            </button>
          ))}
        </div>
        <div className="rd-bar-right">
          <span className="pill mono rd-label">Recorded offline on an Apple M4 · not running live in this page</span>
          <a className="btn-primary rd-replay" href={`${replayHref}${replayQuery(seq, 0)}`}>Open the replay explorer</a>
        </div>
      </div>

      <div className="rd-grid">
        <figure className="rd-figure">
          <div className="rd-plot">
            <div className="rd-chips">
              {LEGEND_ORDER.map((id) => {
                const s = SERIES.find((x) => x.id === id)!;
                return (
                  <button key={id} type="button" className="chip rd-chip" aria-pressed={!hidden[id]}
                    onClick={() => setHidden((h) => ({ ...h, [id]: !h[id] }))}>
                    <svg width="22" height="8" viewBox="0 0 22 8" aria-hidden="true"><line x1="1" y1="4" x2="21" y2="4" className={s.cls} /></svg>
                    {s.name}
                  </button>
                );
              })}
            </div>
            <svg className="rd-svg" viewBox={plot.viewBox} role="img"
              aria-label={`Top-down paths for sequence ${seq}: the true path, Atlas, KISS-ICP and Atlas without a map`}>
              {SERIES.map((s) => (
                <polyline key={s.id} points={plot.points[s.id]} className={s.cls} style={{ opacity: hidden[s.id] ? 0 : 1 }} />
              ))}
              <circle cx={plot.start.x} cy={plot.start.y} r={7} className="rd-start" />
              <text x={plot.start.x + 12} y={plot.start.y + 4} className="rd-svgtext">start</text>
              <line x1={24} y1={plot.height - 14} x2={24 + plot.barLength} y2={plot.height - 14} className="rd-scale" />
              <text x={24} y={plot.height - 22} className="rd-svgtext rd-dim">{plot.barMeters} m</text>
            </svg>
          </div>
          <figcaption>{r.caption}</figcaption>
        </figure>

        <div className="rd-side">
          <div className="rd-meta">
            <p className="eyebrow">{r.title}</p>
            <p>{r.line}</p>
          </div>
          <dl className="rd-stats">
            <div><dd className="rd-blue">{r.atlas.drift}</dd><dt>Atlas drift</dt></div>
            <div><dd>{r.kiss.drift}</dd><dt>KISS-ICP drift</dt></div>
            <div><dd>{r.atlas.ms}</dd><dt>Atlas per scan, one core</dt></div>
          </dl>
          <p className="rd-verdict" data-testid="rd-verdict">{r.verdict}</p>

          <div className="rd-tablewrap">
            <table className="rd-table">
              <thead>
                <tr><th scope="col">Method</th><th scope="col">Drift</th><th scope="col">Turning error</th><th scope="col">Per scan</th><th scope="col">Memory</th></tr>
              </thead>
              <tbody>
                {rows.map((row) => (
                  <tr key={row.id}>
                    <th scope="row">
                      <svg width="18" height="8" viewBox="0 0 18 8" aria-hidden="true"><line x1="1" y1="4" x2="17" y2="4" className={SERIES.find((s) => s.id === row.id)!.cls} /></svg>
                      {row.name}
                    </th>
                    <td className={r.best === row.id ? "rd-best" : undefined}>{row.v.drift}</td>
                    <td>{row.v.turn}</td>
                    <td>{row.v.ms}{row.v.msNote && <span className="rd-sub">{row.v.msNote}</span>}</td>
                    <td>{row.v.mem}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>

          <div className="rd-errors">
            <div className="rd-errhead">
              <p>Error per step along the drive</p>
              <span>{rec.scansPerStretch} scans per pair</span>
            </div>
            <div className="rd-bars" role="group" aria-label={`Average step error per stretch of sequence ${seq}; each pair opens that stretch in the replay explorer`}>
              {bars.map((b, i) => {
                const first = i * rec.scansPerStretch;
                const last = Math.min(rec.scans - 1, first + rec.scansPerStretch);
                const a = rec.stretchErrorCm.atlas[i].toFixed(1), k = rec.stretchErrorCm.kiss[i].toFixed(1);
                return (
                  <a key={i} className="rd-pair" href={`${replayHref}${replayQuery(seq, first)}`}
                    aria-label={`Scans ${first} to ${last}: Atlas ${a} cm, KISS-ICP ${k} cm. Open in the replay explorer.`}
                    title={`Scans ${first}–${last}: Atlas ${a} cm, KISS-ICP ${k} cm. Click to replay.`}>
                    <div className="rd-a" style={{ height: `${Math.max(2, b.a)}%` }} />
                    <div className="rd-k" style={{ height: `${Math.max(2, b.b)}%` }} />
                  </a>
                );
              })}
            </div>
            <div className="rd-axis"><span>scan 0</span><span>scan {(rec.scans - 1).toLocaleString("en-US")}</span></div>
            <p className="rd-note">
              Blue: Atlas. Gray: KISS-ICP. Each pair is the average distance between the estimated and the true step, in
              centimeters, over one stretch of the drive. Click a pair to replay that stretch.
            </p>
          </div>
        </div>
      </div>

      <div className="modules rd-notes">
        <article>
          <span className="file">Comparison</span>
          <h3>One win each</h3>
          <p>KISS-ICP, a published open-source method, ran on the same scans with its default settings. It was ahead on sequence 04 and Atlas on 07. Two drives cannot rank them.</p>
        </article>
        <article>
          <span className="file">Fair test</span>
          <h3>Sequence 07 was held out</h3>
          <p>Every setting was chosen on sequence 04. Sequence 07 was downloaded afterwards and run once, unchanged. Drift is KITTI's own metric: the error over every 100 to 800 m stretch.</p>
        </article>
        <article>
          <span className="file">Limits</span>
          <h3>Odometry, not full SLAM</h3>
          <p>No loop closure, so error still grows with distance. Raw scans, so the numbers are not comparable to the official leaderboard. 1.1 km of driving in all.</p>
        </article>
      </div>

      <p className="rd-credit">
        Data: KITTI Vision Benchmark Suite (Geiger, Lenz, Stiller, Urtasun), CC BY-NC-SA 3.0, non-commercial use.
        Baseline: KISS-ICP 1.3.0 (Vizzo et al., 2023), MIT license. Paths drawn from every third recorded pose.
        Method and commands: <a href="https://github.com/djism/atlas/blob/main/docs/kitti.md">docs/kitti.md</a>.
      </p>
    </section>
  );
}
