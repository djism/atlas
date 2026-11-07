// What the engine contains. Every number here is measured (tests or the sandbox).
const MODULES = [
  { file: "src/simulation.cpp", title: "Ray-cast LiDAR", text: "360 beams hit walls and furniture. The nearest hit wins, so objects can hide behind each other.", num: "Seeded noise, same on every platform" },
  { file: "src/icp.cpp", title: "Scan matching", text: "ICP in two flavors. Matching points to wall lines instead of single points converges far faster.", num: "0.7 mm in 5 passes vs 3.3 mm in 17" },
  { file: "src/icp.cpp", title: "Ambiguity check", text: "Restarts matching from nearby guesses to catch hallways and repeating poles where many answers look right.", num: "Flags a rival at 18 of 20 inliers" },
  { file: "src/tracker.cpp", title: "Scan tracker", text: "Chains scan-to-scan matches into a path, and reports when a scan can't be registered.", num: "Under 10 cm drift over 120 steps" },
  { file: "src/planner.cpp", title: "Path planner", text: "A* on an inflated occupancy grid, then smoothing. Kept separate from localization and labeled that way.", num: "About 0.3 ms per plan" },
  { file: "wasm/bindings.cpp", title: "WebAssembly build", text: "The same C++ compiled for the browser, running in a background worker so the page stays smooth.", num: "116 KB, about 4.8 ms per scan" },
];

export function Modules() {
  return (
    <section className="inside wrap" id="inside">
      <div className="inside-head">
        <div>
          <p className="eyebrow">What I built</p>
          <h2 className="big">Six parts, one engine.</h2>
        </div>
        <p>
          Every piece is plain C++17 with its own tests, including tests that prove the failure cases fail. No robotics
          libraries, so I had to understand each step.
        </p>
      </div>
      <div className="modules">
        {MODULES.map((m) => (
          <article key={m.title}>
            <span className="file">{m.file}</span>
            <h3>{m.title}</h3>
            <p>{m.text}</p>
            <span className="num">{m.num}</span>
          </article>
        ))}
      </div>
    </section>
  );
}
