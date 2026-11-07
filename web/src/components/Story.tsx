import { IcpFigure } from "./IcpFigure";

export function Story() {
  return (
    <section className="story wrap" id="story">
      <div>
        <ol className="chapters" aria-label="How it works">
          <li>Scans</li>
          <li aria-current="step">Matching</li>
          <li>Drift</li>
          <li>Planning</li>
        </ol>
        <h2 className="big">Two scans of one room.</h2>
        <p>
          The robot never measures its own motion. Every tenth of a second it gets a cloud of points in its own frame,
          and it has to work out how it moved since the last one.
        </p>
        <p>
          ICP pairs each point with its nearest neighbor, solves for the rotation and shift that best line the pairs up,
          and repeats. The figure replays what the C++ engine did, pass by pass. Drag the blue scan and let go to give it
          a new starting guess.
        </p>
        <div className="compare">
          <div>
            <div className="val"><span>Point-to-line</span><span>0.7 mm · 5 passes</span></div>
            <div className="bar" style={{ width: "29.4%" }} />
          </div>
          <div>
            <div className="val"><span>Point-to-point</span><span>3.3 mm · 17 passes</span></div>
            <div className="bar bar-muted" style={{ width: "100%" }} />
          </div>
          <p className="note">Bars show passes to converge. 360-beam room scan, no noise, measured in the C++ test suite.</p>
        </div>
        <div className="learned">
          <p className="eyebrow">What I learned</p>
          <p>
            "Converged" only means the answer stopped changing. Drag the scan far enough and it settles confidently in
            the wrong place. Atlas checks for that instead of trusting it.
          </p>
        </div>
      </div>
      <IcpFigure />
    </section>
  );
}
