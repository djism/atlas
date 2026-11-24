import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  plugins: [react()],
  worker: { format: "es" },
  build: {
    target: "es2022",
    // Two pages: the sandbox home and the replay explorer (replay/index.html).
    rollupOptions: { input: { main: "index.html", replay: "replay/index.html" } }, // paths from web/, where every build runs
  },
});
