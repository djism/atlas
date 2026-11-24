import { historySegments, lengthTicks, logY, matchBars, passChart, type LostSample } from "../lib/labCharts";

const W = 300;

// Log-scale axis labels, placed at their decade lines.
function Ticks({ lo, hi, height, ticks }: { lo: number; hi: number; height: number; ticks: [number, string][] }) {
  return (
    <div className="lc-ticks" style={{ height }} aria-hidden="true">
      {ticks.map(([v, label]) => <span key={label} style={{ top: logY(v, lo, hi, height) - 5 }}>{label}</span>)}
    </div>
  );
}

const ERR_LO = 0.001, ERR_HI = 10, ERR_H = 96, MATCH_H = 30; // the no-map tracker can drift several meters

/** "Get it lost": error with and without the map, and map match, over the last samples. */
export function LostHistory({ samples, capacity, seconds }: { samples: LostSample[]; capacity: number; seconds: number }) {
  const ticks = lengthTicks(ERR_LO, ERR_HI);
  const withMap = historySegments(samples.map((s) => s.withMap), capacity, W, ERR_H, ERR_LO, ERR_HI);
  const noMap = historySegments(samples.map((s) => s.noMap), capacity, W, ERR_H, ERR_LO, ERR_HI);
  const last = samples[samples.length - 1];
  const describe = last?.withMap != null && last.noMap != null
    ? `Position error over the last ${seconds} seconds, log scale. Now: with map ${(last.withMap * 1000).toFixed(1)} mm, no map ${(last.noMap * 100).toFixed(1)} cm.`
    : `Position error over the last ${seconds} seconds, log scale. Waiting for the first estimates.`;
  return (
    <figure className="lc" data-testid="lost-chart">
      <figcaption><span>Error, last {seconds} s</span><span className="lc-note">log scale</span></figcaption>
      <div className="lc-row">
        <Ticks lo={ERR_LO} hi={ERR_HI} height={ERR_H} ticks={ticks} />
        <svg viewBox={`0 0 ${W} ${ERR_H}`} preserveAspectRatio="none" style={{ height: ERR_H }} role="img" aria-label={describe}>
          {ticks.map(([v]) => <line key={v} className="lc-grid" x1={0} x2={W} y1={logY(v, ERR_LO, ERR_HI, ERR_H)} y2={logY(v, ERR_LO, ERR_HI, ERR_H)} />)}
          {noMap.map((p, i) => <polyline key={`n${i}`} className="lc-nomap" points={p} />)}
          {withMap.map((p, i) => <polyline key={`m${i}`} className="lc-map" points={p} />)}
        </svg>
      </div>
      <div className="lc-row">
        <span className="lc-side">match<br />60–100%</span>
        <svg viewBox={`0 0 ${W} ${MATCH_H}`} preserveAspectRatio="none" style={{ height: MATCH_H }} role="img"
          aria-label="Share of each scan matching the map; red marks are scans where the map localizer was lost">
          {matchBars(samples, capacity, W, MATCH_H).map((b, i) => (
            <rect key={i} x={b.x} y={b.y} width={b.w} height={b.h} className={b.lost ? "lc-lost" : "lc-match"} />
          ))}
        </svg>
      </div>
      <div className="lc-key">
        <span><i className="lc-sw lc-sw-map" />With map</span>
        <span><i className="lc-sw lc-sw-nomap" />No map</span>
        <span><i className="lc-sw lc-sw-lost" />Lost</span>
      </div>
    </figure>
  );
}

const PASS_LO = 0.002, PASS_HI = 0.4, PASS_H = 110;

/** "Match two scans": pair RMSE per pass (what the engine sees) and the error against the answer key. */
export function PassHistory({ rmse, error, shown }: { rmse: number[]; error: number[]; shown: number }) {
  const ticks = lengthTicks(PASS_LO, PASS_HI);
  const g = passChart(rmse, error, shown, W, PASS_H, PASS_LO, PASS_HI);
  const cm = (v: number) => (v >= 0.01 ? `${(v * 100).toFixed(1)} cm` : `${(v * 1000).toFixed(1)} mm`);
  const label = `Pass by pass, log scale. Pair RMSE: ${rmse.slice(0, shown).map(cm).join(", ") || "none yet"}. ` +
    `Error against the answer key: ${error.slice(0, shown + 1).map(cm).join(", ")}.`;
  const labelEvery = Math.ceil(error.length / 8); // long point-to-point runs: label every few passes
  return (
    <figure className="lc" data-testid="pass-chart">
      <figcaption><span>Pass by pass</span><span className="lc-note">log scale</span></figcaption>
      <div className="lc-row">
        <Ticks lo={PASS_LO} hi={PASS_HI} height={PASS_H} ticks={ticks} />
        <svg viewBox={`0 0 ${W} ${PASS_H}`} style={{ height: PASS_H }} role="img" aria-label={label}>
          {ticks.map(([v]) => <line key={v} className="lc-grid" x1={0} x2={W} y1={logY(v, PASS_LO, PASS_HI, PASS_H)} y2={logY(v, PASS_LO, PASS_HI, PASS_H)} />)}
          {g.bars.map((b, i) => <rect key={i} x={b.x} y={b.y} width={b.w} height={Math.max(0, b.h)} rx={2} className={b.current ? "lc-bar lc-bar-now" : "lc-bar"} />)}
          <polyline className="lc-err" points={g.line} />
          {g.dots.map((d, i) => <circle key={i} cx={d.x} cy={d.y} r={3.5} className="lc-dot" />)}
        </svg>
      </div>
      <div className="lc-row">
        <span />
        <div className="lc-passes" aria-hidden="true">
          {error.map((_, i) => (
            <span key={i} style={{ left: `${((i + 0.5) / error.length) * 100}%` }}>
              {i === 0 ? "start" : i % labelEvery === 0 || i === error.length - 1 ? i : ""}
            </span>
          ))}
        </div>
      </div>
      <div className="lc-key">
        <span><i className="lc-sw lc-sw-bar" />Pair RMSE: what the engine sees</span>
        <span><i className="lc-sw lc-sw-dot" />Error: the answer key</span>
      </div>
    </figure>
  );
}
