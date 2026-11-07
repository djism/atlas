import { useEffect, useRef, useState, type MutableRefObject, type ReactNode, type RefObject } from "react";
import type { IcpDemo } from "../icp/IcpDemo";
import { homeScenario } from "../scenarios/home";
import { Field, type FieldMode, type FieldStats } from "./Field";

interface Props {
  mode: FieldMode;
  label: string;
  icp?: IcpDemo;
  panelRef?: RefObject<HTMLElement>; // floating panel on the left; the map shifts clear of it
  fieldRef?: MutableRefObject<Field | null>;
  onStats?: (stats: FieldStats) => void;
  className?: string;
  children?: ReactNode; // overlays (chips, buttons)
}

const hasWebGL = () => {
  try {
    return !!document.createElement("canvas").getContext("webgl2");
  } catch {
    return false;
  }
};

// React shell around the imperative three.js Field. The Field owns its canvas;
// React owns only the overlays.
export function FieldView({ mode, label, icp, panelRef, fieldRef, onStats, className, children }: Props) {
  const host = useRef<HTMLDivElement>(null);
  const field = useRef<Field | null>(null);
  const stats = useRef(onStats);
  stats.current = onStats;
  const [failed, setFailed] = useState(false);

  useEffect(() => {
    if (!host.current) return;
    if (!hasWebGL()) {
      setFailed(true);
      return;
    }
    const f = new Field(host.current, {
      scenario: homeScenario,
      mode,
      icp,
      insetLeft: () => {
        const el = panelRef?.current;
        return el && window.innerWidth > 760 ? el.offsetLeft + el.offsetWidth : 0;
      },
      onStats: (s) => stats.current?.(s),
    });
    field.current = f;
    if (fieldRef) fieldRef.current = f;
    return () => {
      f.dispose();
      field.current = null;
      if (fieldRef) fieldRef.current = null;
    };
    // The Field is created once; mode changes go through setMode below.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  useEffect(() => field.current?.setMode(mode), [mode]);

  return (
    <div className={`field ${className ?? ""}`}>
      <div ref={host} className="field-canvas" role="img" aria-label={label} />
      {children}
      {failed && <div className="field-fallback">This view needs WebGL, which this browser has turned off.</div>}
    </div>
  );
}
