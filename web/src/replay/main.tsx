import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { ReplayApp } from "./ReplayApp";
import "../styles/tokens.css";
import "../styles/app.css";
import "../styles/replay.css";

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <ReplayApp />
  </StrictMode>,
);
