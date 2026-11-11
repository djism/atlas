// Smoke tests: does the published site actually work in a browser? Each test
// loads the production build, waits for the C++ engine (WebAssembly, in a Web
// Worker) to produce results, and checks them against loose, meaningful bounds.
// They gate the GitHub Pages deploy. They are not accuracy experiments; those
// are the C++ tests and atlas_experiment.
import { expect, test, type Page } from "@playwright/test";

const errors: string[] = [];

test.beforeEach(async ({ page }) => {
  errors.length = 0;
  // SMOKE_CPU_SLOWDOWN=3 runs the page on a 3x slower CPU, to rehearse slow machines.
  const slowdown = Number(process.env.SMOKE_CPU_SLOWDOWN ?? 1);
  if (slowdown > 1) {
    const cdp = await page.context().newCDPSession(page);
    await cdp.send("Emulation.setCPUThrottlingRate", { rate: slowdown });
  }
  page.on("pageerror", (e) => errors.push(`page error: ${e.message}`));
  page.on("console", (m) => {
    if (m.type() === "error") errors.push(`console error: ${m.text()} (${m.location().url})`);
  });
});

test.afterEach(() => {
  expect(errors, "no errors in the browser console").toEqual([]);
});

// The lab panel's metric value by its label, e.g. "With map" -> "3.7 mm".
const metric = (page: Page, label: string) =>
  page.locator(".panel .metrics div").filter({ has: page.locator("dt", { hasText: label }) }).locator("dd");

// "3.7 mm" / "1.2 cm" / "2.31 m" -> meters; "–" (no value yet) -> NaN.
async function meters(page: Page, label: string): Promise<number> {
  const m = /^([\d.]+) (mm|cm|m)$/.exec((await metric(page, label).innerText()).trim());
  if (!m) return NaN;
  return Number(m[1]) * ({ mm: 0.001, cm: 0.01, m: 1 } as const)[m[2] as "mm" | "cm" | "m"];
}

async function percent(page: Page, label: string): Promise<number> {
  const m = /^(\d+)%$/.exec((await metric(page, label).innerText()).trim());
  return m ? Number(m[1]) : NaN;
}

const panelChip = (page: Page) => page.locator(".panel .chip");

// Readings of a lab metric every 0.5 s for `seconds`, once it has a value.
// Judging a spread of readings, not one, keeps a momentary dip or spike (e.g.
// the doorway) from deciding the test.
async function sample(page: Page, read: () => Promise<number>, seconds = 10): Promise<number[]> {
  await expect.poll(read, { message: "metric has a value", timeout: 30_000 }).not.toBeNaN();
  const values: number[] = [];
  for (let t = 0; t < seconds * 2; t++) {
    const v = await read();
    if (Number.isFinite(v)) values.push(v);
    await page.waitForTimeout(500);
  }
  return values;
}
const median = (v: number[]) => [...v].sort((a, b) => a - b)[Math.floor(v.length / 2)];

test("home: the hero robot drives on the live engine", async ({ page }) => {
  await page.goto("./");
  const chip = page.locator(".hero .f-ui.bl .chip");
  await expect(chip).toHaveText(/Driving to point/, { timeout: 30_000 });
  const pose = page.locator(".hero .f-ui.bl .pill");
  const before = await pose.innerText();
  await expect.poll(() => pose.innerText(), { message: "the robot's pose changes", timeout: 15_000 }).not.toBe(before);
});

test("match two scans: ICP converges near the answer key", async ({ page }) => {
  await page.goto("./?lab=match#lab");
  await expect(panelChip(page)).toHaveText("Converged", { timeout: 30_000 });
  expect(await meters(page, "Error")).toBeLessThan(0.01);
});

test("get it lost: the map localizer stays on track", async ({ page }) => {
  await page.goto("./?lab=lost#lab");
  await expect(page.locator(".legend")).toContainText(/The map \(\d{1},?\d{3} points\)/, { timeout: 30_000 });
  // Judge the C++ map localizer against the answer key while the robot drives.
  const errors = await sample(page, () => meters(page, "With map"));
  expect(Math.max(...errors), "worst map-localizer error over 10 s").toBeLessThan(0.05);
  const match = await sample(page, () => percent(page, "Map match"));
  expect(median(match), "median map match").toBeGreaterThanOrEqual(80);
});

test("shared link: the map comes from the house as it was when mapped", async ({ page }) => {
  // The same moved furniture, twice: once with the map surveyed from the
  // original house (mapped=, empty), once surveyed from the moved house.
  const room = "sofa:3.00,2.00;table:9.00,3.00;bed:4.50,6.00;box:2.00,2.00;box:9.00,6.00";
  await page.goto(`./?lab=lost&room=${room}&mapped=#lab`);
  await expect(page.locator("#tab-lost")).toHaveAttribute("aria-selected", "true");
  const stale = median(await sample(page, () => percent(page, "Map match")));
  const address = decodeURIComponent(new URL(page.url()).search);

  await page.goto(`./?lab=lost&room=${room}#lab`);
  const fresh = median(await sample(page, () => percent(page, "Map match")));
  expect(stale, "map of the original house: median match").toBeLessThan(80);
  expect(fresh, "map of the moved house: median match").toBeGreaterThanOrEqual(80);
  // The address bar keeps the setup (normalized), so the link can be shared on.
  expect(address).toBe(`?lab=lost&room=bed:4.50,6.00;sofa:3.00,2.00;table:9.00,3.00;box:2.00,2.00;box:9.00,6.00&mapped=`);
});

test("rearrange the house: the planner plans and the robot drives", async ({ page }) => {
  await page.goto("./?lab=plan#lab");
  await expect(panelChip(page)).toHaveText(/Driving to point/, { timeout: 30_000 });
  expect(Number(await metric(page, "Plans made").innerText())).toBeGreaterThanOrEqual(1);
  await page.waitForTimeout(5_000);
  expect(await metric(page, "Bumps").innerText()).toBe("0");
});

test("phone width: no sideways scrolling", async ({ page }) => {
  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto("./?lab=lost#lab");
  await expect(panelChip(page)).toBeVisible();
  expect(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth)).toBe(true);
});
