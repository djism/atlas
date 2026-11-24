import { Hero } from "./components/Hero";
import { Lab } from "./components/Lab";
import { Modules } from "./components/Modules";
import { RealData } from "./components/RealData";
import { Story } from "./components/Story";

export function App() {
  return (
    <>
      <main>
        <Hero />
        <Modules />
        <RealData />
        <Story />
        <Lab />
      </main>
      <footer className="footer wrap">
        <span>Atlas · Dhananjay Sharma</span>
        <span>Sandbox and lab: live, the C++ engine runs in your browser via WebAssembly in a simulated world. Real data: recorded results.</span>
      </footer>
    </>
  );
}
