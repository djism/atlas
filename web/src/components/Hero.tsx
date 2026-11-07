import { useRef, useState } from "react";
import type { Field, FieldStats } from "../field/Field";
import { FieldView } from "../field/FieldView";
import { Chip, Segmented, driveChip } from "./ui";

export function Hero() {
  const [stats, setStats] = useState<FieldStats | null>(null);
  const field = useRef<Field | null>(null);
  const chip = driveChip(stats);
  const p = stats?.pose;
  const deg = p ? ((p.theta * 180) / Math.PI + 360) % 360 : 0;

  return (
    <section className="hero">
      <div className="hero-text wrap">
        <p className="eyebrow">Atlas · C++17 localization engine · live in your browser</p>
        <h1>How does a robot know where it is?</h1>
        <p className="lede">
          I built Atlas from scratch in C++ to find out. It reads laser scans, matches them against each other, and works
          out how the robot moved. The same code runs on this page, compiled to WebAssembly.
        </p>
        <div className="cta">
          <a className="btn-primary" href="#lab">Try the lab</a>
          <a className="link" href="#inside">See what's inside</a>
        </div>
        <dl className="stats">
          <div><dt>Position error after 3.7 m</dt><dd>1.1 cm</dd></div>
          <div><dt>Per scan, in the browser</dt><dd>4.8 ms</dd></div>
          <div><dt>External C++ libraries</dt><dd>0</dd></div>
        </dl>
      </div>
      <FieldView
        mode="hero"
        className="hero-field"
        fieldRef={field}
        onStats={setStats}
        label="Interactive 3D home. Drag furniture or click the floor to drop a box; the robot replans around it."
      >
        <div className="f-ui tl">
          <span className="pill hint"><kbd>Drag</kbd> furniture</span>
          <span className="pill hint"><kbd>Click</kbd> the floor to drop a box</span>
        </div>
        <div className="f-ui bl" aria-live="polite">
          <Chip tone={chip.tone}>{chip.text}</Chip>
          {p && <span className="pill mono">x {p.x.toFixed(2)} m · y {p.y.toFixed(2)} m · θ {deg.toFixed(0)}° · world frame</span>}
        </div>
        <div className="f-ui br">
          <Segmented
            label="Camera"
            value={field.current?.currentView ?? "3d"}
            options={[{ value: "3d", label: "3D" }, { value: "top", label: "Top" }]}
            onChange={(v) => field.current?.setView(v)}
          />
        </div>
      </FieldView>
    </section>
  );
}
