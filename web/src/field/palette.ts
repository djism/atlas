// Scene colors. Mirrors the CSS tokens in styles/tokens.css (three.js needs numbers).
export const color = {
  ink: 0x0b0c0e,
  paper: 0xffffff,
  mist: 0xf2f3f5,
  signal: 0x2346ff, // reserved for what the sensor sees and what the robot believes
  furniture: 0xd3d7de,
  furnitureHover: 0xb9bfc9,
  mapPoint: 0x9fb0ff, // the robot's map: a pale tint of signal, so live scans stand out
  baseline: 0x8a909a, // the no-map tracker's estimate: the comparison, not the answer
} as const;
