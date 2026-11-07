import { Hero } from "./components/Hero";
import { Lab } from "./components/Lab";
import { Modules } from "./components/Modules";
import { Story } from "./components/Story";

export function App() {
  return (
    <>
      <main>
        <Hero />
        <Modules />
        <Story />
        <Lab />
      </main>
      <footer className="footer wrap">
        <span>Atlas · Dhananjay Sharma</span>
        <span>Live: the C++ engine runs in your browser via WebAssembly. The world is simulated.</span>
      </footer>
    </>
  );
}
