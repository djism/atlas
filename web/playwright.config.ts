import { defineConfig } from "@playwright/test";

// Browser smoke tests against the PRODUCTION build, served under /atlas/ exactly
// as GitHub Pages serves it. Build first: npm run build:pages, then npm run test:e2e.
// Uses the installed Google Chrome (channel "chrome"): no browser download.
export default defineConfig({
  testDir: "e2e",
  timeout: 180_000, // per test; the slowest takes ~60 s with software WebGL and a 3x slower CPU
  retries: 0, // a flaky smoke test should be fixed, not retried into passing
  workers: 1, // one engine-heavy page at a time keeps timings like a visitor's
  reporter: process.env.CI ? [["list"], ["html", { open: "never" }]] : "list",
  use: {
    baseURL: "http://localhost:4173/atlas/",
    channel: "chrome",
    viewport: { width: 1440, height: 900 },
    // Software WebGL when there is no GPU (CI machines); real GPUs still win
    // locally. SMOKE_NO_GPU=1 forces software rendering to rehearse CI.
    launchOptions: { args: ["--enable-unsafe-swiftshader", ...(process.env.SMOKE_NO_GPU ? ["--disable-gpu"] : [])] },
    trace: "retain-on-failure",
  },
  webServer: {
    command: "npx vite preview --base /atlas/ --port 4173 --strictPort",
    url: "http://localhost:4173/atlas/",
    reuseExistingServer: !process.env.CI,
    timeout: 30_000,
  },
});
