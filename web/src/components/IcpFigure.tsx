import { useEffect, useMemo, useRef, useState } from "react";
import { IcpDemo, type IcpMethod } from "../icp/IcpDemo";
import { formatLength } from "../lib/frames";
import { homeScenario } from "../scenarios/home";
import { Chip, Segmented, type Tone } from "./ui";

const VIEW = { x: -0.2, y: -0.2, w: 6.9, h: 8.4 }; // world meters shown: the left room
const TOP = 64; // room for the overlay controls

// 2D figure: the real engine's ICP run, replayed pass by pass. Drag the blue scan.
export function IcpFigure() {
  const canvas = useRef<HTMLCanvasElement>(null);
  const demo = useMemo(() => new IcpDemo(), []);
  const [method, setMethod] = useState<IcpMethod>("p2l");
  const [status, setStatus] = useState<{ tone: Tone; chip: string; caption: string }>({ tone: "run", chip: "Loading engine", caption: "Pass 0" });

  useEffect(() => {
    const cv = canvas.current;
    if (!cv) return;
    void demo.load();
    let raf = 0;
    let visible = true;
    let lastText = 0;
    let scale = 1;
    let dragFrom: { x: number; y: number } | null = null;
    const io = new IntersectionObserver((es) => (visible = es[0].isIntersecting), { rootMargin: "100px" });
    io.observe(cv);

    const draw = (now: number) => {
      raf = requestAnimationFrame(draw);
      if (!visible) return;
      const r = cv.getBoundingClientRect();
      const dpr = Math.min(2, window.devicePixelRatio || 1);
      const W = Math.round(r.width * dpr);
      const H = Math.round(r.height * dpr);
      if (cv.width !== W || cv.height !== H) {
        cv.width = W;
        cv.height = H;
      }
      const pad = 28;
      const s = (scale = Math.min((r.width - 2 * pad) / VIEW.w, (r.height - pad - TOP) / VIEW.h));
      const ox = (r.width - s * VIEW.w) / 2;
      const oy = TOP + (r.height - TOP - pad - s * VIEW.h) / 2;
      const X = (x: number) => ox + (x - VIEW.x) * s;
      const Y = (y: number) => oy + (VIEW.y + VIEW.h - y) * s;
      const ctx = cv.getContext("2d")!;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, r.width, r.height);

      ctx.strokeStyle = "rgba(11,12,14,0.16)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (const w of homeScenario.walls) {
        ctx.moveTo(X(w.a.x), Y(w.a.y));
        ctx.lineTo(X(w.b.x), Y(w.b.y));
      }
      ctx.stroke();
      ctx.fillStyle = "rgba(11,12,14,0.045)";
      for (const f of homeScenario.furniture) ctx.fillRect(X(f.x - f.w / 2), Y(f.y + f.h / 2), f.w * s, f.h * s);

      if (demo.loaded) {
        const frame = demo.frame(now);
        const second = demo.secondInWorld(frame.pose);
        if (frame.pairs) {
          ctx.strokeStyle = "rgba(35,70,255,0.22)";
          ctx.lineWidth = 1;
          ctx.beginPath();
          second.forEach((p, i) => {
            const a = demo.firstWorld[frame.pairs![i]];
            ctx.moveTo(X(p.x), Y(p.y));
            ctx.lineTo(X(a.x), Y(a.y));
          });
          ctx.stroke();
        }
        const dots = (pts: { x: number; y: number }[], fill: string, rad: number) => {
          ctx.fillStyle = fill;
          for (const p of pts) {
            ctx.beginPath();
            ctx.arc(X(p.x), Y(p.y), rad, 0, Math.PI * 2);
            ctx.fill();
          }
        };
        dots(demo.firstWorld, "#0B0C0E", 2.1);
        dots(second, "#2346FF", 2.4);
        if (now - lastText > 100) {
          lastText = now;
          const v = demo.verdict(frame);
          const err = demo.errors(frame.pose);
          const passes = frame.passes ? ` of ${frame.passes}` : "";
          setStatus({
            tone: v.tone, chip: v.text,
            caption: `Pass ${frame.pass}${passes} · error vs answer key ${formatLength(err.position)}, ${((err.heading * 180) / Math.PI).toFixed(2)}°`,
          });
        }
      }
    };
    raf = requestAnimationFrame(draw);

    const down = (e: PointerEvent) => {
      if (!demo.loaded) return;
      dragFrom = { x: e.clientX, y: e.clientY };
      demo.startDrag(performance.now());
      cv.setPointerCapture(e.pointerId);
      cv.classList.add("dragging");
    };
    const move = (e: PointerEvent) => {
      if (!dragFrom) return;
      demo.dragBy({ x: (e.clientX - dragFrom.x) / scale, y: -(e.clientY - dragFrom.y) / scale });
      dragFrom = { x: e.clientX, y: e.clientY };
    };
    const up = () => {
      if (!dragFrom) return;
      dragFrom = null;
      cv.classList.remove("dragging");
      demo.endDrag();
    };
    cv.addEventListener("pointerdown", down);
    cv.addEventListener("pointermove", move);
    cv.addEventListener("pointerup", up);
    cv.addEventListener("pointercancel", up);
    return () => {
      cancelAnimationFrame(raf);
      io.disconnect();
      cv.removeEventListener("pointerdown", down);
      cv.removeEventListener("pointermove", move);
      cv.removeEventListener("pointerup", up);
      cv.removeEventListener("pointercancel", up);
    };
  }, [demo]);

  return (
    <figure className="fig">
      <canvas ref={canvas} aria-label="Two scans of one room corner. Drag the blue scan and release to watch the engine realign it." />
      <div className="f-ui tl spread">
        <Chip tone={status.tone}>{status.chip}</Chip>
        <Segmented
          label="Matching method"
          value={method}
          options={[{ value: "p2l", label: "Point-to-line" }, { value: "p2p", label: "Point-to-point" }]}
          onChange={(m) => {
            setMethod(m);
            demo.setMethod(m);
          }}
        />
      </div>
      <figcaption>
        <span>Black: first scan · Blue: second scan · Drag blue, release</span>
        <span>{status.caption}</span>
      </figcaption>
    </figure>
  );
}
