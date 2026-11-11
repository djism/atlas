// What the engine contains. Every number here is measured (tests or the sandbox).
const MODULES = [
  { file: "src/simulation.cpp", title: "Ray-cast LiDAR", text: "360 beams hit walls and furniture. The nearest hit wins, so objects can hide behind each other.", num: "Seeded noise, same on every platform" },
  { file: "src/icp.cpp", title: "Scan matching", text: "ICP in two flavors. Matching points to wall lines instead of single points converges far faster.", num: "0.7 mm in 5 passes vs 3.3 mm in 17" },
  { file: "src/mapping.cpp", title: "Occupancy map", text: "Each beam marks the floor it crossed as free and the spot it hit as occupied. Evidence adds up, and a removed box clears after a few clear scans.", num: "Every occupied cell within 5 cm of a wall" },
  { file: "src/localizer.cpp", title: "Map localizer", text: "Matches each scan against the map, so errors stop piling up. Starts each match from wheel odometry, and spots when parallel walls can't pin the position down.", num: "3 to 11 mm worst vs 2 to 19 cm without a map" },
  { file: "src/planner.cpp", title: "Path planner", text: "A* on an inflated occupancy grid, then smoothing. It plans and steers from the robot's estimate, so bad localization means bad driving.", num: "About 0.3 ms per plan" },
  { file: "wasm/bindings.cpp", title: "WebAssembly build", text: "The same C++ compiled for the browser, running in a background worker so the page stays smooth.", num: "132 KB; about 21 ms per scan, both estimators" },
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
