# AegisCommand C4ISR — Joint Situational Awareness & Multi-Domain Operations Digital Twin

[![Live Demo](https://img.shields.io/badge/Live_C4ISR_Twin-Tareq0001.github.io-sky?style=for-the-badge&logo=github)](https://tareq0001.github.io/AegisCommand-C4ISR-Digital-Twin/)
[![Military Standard](https://img.shields.io/badge/Standard-MIL--STD--2525D_%7C_STANAG_4586-blue?style=for-the-badge)](https://tareq0001.github.io/AegisCommand-C4ISR-Digital-Twin/)
[![Three.js](https://img.shields.io/badge/WebGL-Three.js_Holographic_Theater-orange?style=for-the-badge&logo=three.js)](https://tareq0001.github.io/AegisCommand-C4ISR-Digital-Twin/)

> **A mission-critical, interactive C4ISR (Command, Control, Communications, Computers, Intelligence, Surveillance, and Reconnaissance) Joint Operations Center digital twin. Delivers real-time Common Operational Picture (COP), Blue-Force Tracking (BFT), multi-domain sensor fusion across Land, Air, Maritime, and Space, alongside Patriot PAC-3 air defense interception calculations and MGRS military grid plotting.**

---

## ⚡ Core Defense & Systems Engineering Capabilities

1. **3D Holographic Sandtable & Joint Operations Theater**
   - Multi-layer 3D sandtable visualizer built on Three.js (r128) simulating joint operational theaters.
   - Domain filtering toggles: Air Defense (CAP patrols, cruise vectors), Maritime Coastal (naval frigates), Space (reconnaissance satellites in low Earth orbit), and Ground SAM umbrellas.
   - Procedural Web Audio API sound generator for tactical operations room chatter and warning alarms.

2. **Common Operational Picture (COP) & MIL-STD-2525D Symbology**
   - Multi-sensor track correlation unifying radar feeds, ADS-B transponders, AIS maritime coastal tracking, and orbital telemetry into clean operational tracks.
   - Standardized military symbology distinguishing Friendly (Blue Force), Hostile (Red Force), and Unknown contacts.

3. **Air Defense Interception Calculator (Patriot PAC-3 & THAAD)**
   - Dynamic Closest Point of Approach (CPA) and Time-to-Impact (TTI) kinematic solver.
   - Single-shot kill probability ($P_k$) computation based on engagement geometry, target velocity, and interceptor envelope.
   - Interactive authorization protocol for missile launch simulations.

4. **2D Real-Time Airspace Radar PPI Sweep**
   - 12 RPM plan position indicator (PPI) polar radar canvas displaying real-time target vectors, range rings (350 NM), and Mode 5 IFF squawk codes.

---

## 🛠️ Architecture & Tech Stack

- **Graphics:** Three.js (r128), Custom CRT scanline post-processing, Canvas 2D PPI radar sweep.
- **Mathematics:** Kinematic interception vectors, Time-to-Impact formulas, MGRS grid projection.
- **Audio:** Web Audio API oscillator synthesis.
- **UI:** Tactile glassmorphism C4ISR command deck, bilingual English/Arabic interface.

---

## 🚀 Live Deployment

- **Online C4ISR Operations Center:** [https://tareq0001.github.io/AegisCommand-C4ISR-Digital-Twin/](https://tareq0001.github.io/AegisCommand-C4ISR-Digital-Twin/)
- **Author:** Eng. Tareq Aboushi (Tareq0001)
- **Standards:** MIL-STD-2525D | STANAG 4586 | MGRS
