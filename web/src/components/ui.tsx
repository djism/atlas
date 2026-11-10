import type { FieldStats } from "../field/Field";

export type Tone = "ok" | "warn" | "bad" | "run";

export function Chip({ tone, children }: { tone: Tone; children: string }) {
  return <span className={`chip chip-${tone}`}>{children}</span>;
}

interface SegmentedProps<T extends string> {
  label: string;
  value: T;
  options: { value: T; label: string }[];
  onChange: (value: T) => void;
  className?: string;
}

export function Segmented<T extends string>({ label, value, options, onChange, className }: SegmentedProps<T>) {
  return (
    <div className={`seg ${className ?? ""}`} role="group" aria-label={label}>
      {options.map((o) => (
        <button key={o.value} type="button" aria-pressed={o.value === value} onClick={() => onChange(o.value)}>
          {o.label}
        </button>
      ))}
    </div>
  );
}

// Planner / engine status as one chip.
export function driveChip(s: FieldStats | null): { tone: Tone; text: string } {
  if (!s || s.engine === "loading") return { tone: "run", text: "Loading engine" };
  if (s.engine === "error") return { tone: "bad", text: "Engine failed to load" };
  if (s.localizing) return { tone: "run", text: "Localizing" };
  if (s.bumped) return { tone: "bad", text: "Bumped into something: replanning" };
  if (s.drive === "stuck") return { tone: "bad", text: "Stuck: something is on top of the robot" };
  if (s.drive === "no-path") return { tone: "bad", text: "No point is reachable" };
  if (s.drive === "planning") return { tone: "run", text: "Planning" };
  const skipped = s.skipped.length ? ` · skipped ${s.skipped.map((i) => i + 1).join(", ")}` : "";
  return { tone: s.skipped.length ? "warn" : "ok", text: `Driving to point ${s.goal + 1}${skipped}` };
}
