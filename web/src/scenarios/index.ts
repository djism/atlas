import { homeScenario } from "./home";
import type { Scenario } from "./types";

// Registry of available worlds. Street-car and warehouse scenarios plug in here.
export const scenarios: Scenario[] = [homeScenario];
export type { Scenario };
