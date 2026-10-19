# AegisCommand C4ISR — Joint Situational Awareness & Multi-Domain Operations Digital Twin

[![Live C4ISR Operations Center](https://img.shields.io/badge/Live_C4ISR_Twin-tareq0001.github.io-0284c7?style=for-the-badge&logo=github)](https://tareq0001.github.io/AegisCommand-C4ISR-Digital-Twin/)
[![Build Status](https://img.shields.io/badge/CI%2FCD-Passing-10b981?style=for-the-badge&logo=githubactions)](https://github.com/Tareq0001/AegisCommand-C4ISR-Digital-Twin/actions)
[![Standards](https://img.shields.io/badge/Defense_Standards-STANAG_5516_%7C_MIL--STD--2525D_%7C_WGS84-blue?style=for-the-badge)](https://tareq0001.github.io/AegisCommand-C4ISR-Digital-Twin/)
[![Tech Stack](https://img.shields.io/badge/Multi--Language-TypeScript_ES2022_%7C_C%2B%2B17_%7C_Python_3.11-6366f1?style=for-the-badge)](https://github.com/Tareq0001/AegisCommand-C4ISR-Digital-Twin)

> **A mission-critical C4ISR (Command, Control, Communications, Computers, Intelligence, Surveillance, and Reconnaissance) Joint Operations Center digital twin. Delivers real-time Common Operational Picture (COP), NATO STANAG 5516 Link-16 J-Series tactical data links, Multi-Hypothesis Kalman Sensor Fusion (JPDA/MHT), and Patriot PAC-3 MSE Threat Evaluation & Weapon Assignment (TEWA) optimization.**

---

## 🏛️ System Architecture

```text
       ┌─────────────────────────────────────────────────────────┐
       │     AegisCommand C4ISR Joint Operations Center Deck     │
       │  (Bilingual EN/AR, DEFCON 2 Alerting, Web Audio Sonar)   │
       └────────────────────────────┬────────────────────────────┘
                                    │
    ┌───────────────────────────────┴──────────────────────────────┐
    │                                                              │
┌───▼──────────────────────┐  ┌───────────────────────────┐  ┌─────▼────────────────────┐
│ TypeScript Core (Vite)   │  │ C++17 Native UKF Engine   │  │ Python SITL CoT Gateway  │
├──────────────────────────┤  ├───────────────────────────┤  ├──────────────────────────┤
│ • WGS84 Geodesy / ENU    │  │ • 9-State Ballistic Vector│  │ • Cursor-on-Target XML   │
│ • Link-16 STANAG 5516    │  │ • Scaled Unscented Filter │  │ • AsyncIO UDP Port 4242  │
│ • JPDA / MHT Fusion      │  │ • RK4 Ballistic Drag/Grav │  │ • Multi-Sensor Loopback  │
│ • TEWA PAC-3 Optimizer   │  │ • Covariance Intersection │  │ • SITL Bridge Node       │
└──────────────────────────┘  └───────────────────────────┘  └──────────────────────────┘
```

---

## ⚡ Core Defense & Engineering Capabilities

### 1. WGS84 Geodetic to ECEF and Local Tangent Plane (ENU)
- **Standard:** WGS 84 (EPSG:4326 / EPSG:4978), IUGG Mean Earth Radius $R = 6371008.8\,\text{m}$.
- **Transformations:**
  - Forward Geodetic $(\phi, \lambda, h) \to \text{ECEF}(X, Y, Z)$ using prime vertical radius $N(\phi) = \frac{a}{\sqrt{1 - e^2 \sin^2\phi}}$.
  - Inverse ECEF to Geodetic via **Bowring's closed-form algorithm** (sub-millimeter terrestrial fidelity).
  - Rotation into Local Tangent Plane **East-North-Up (ENU)** relative to theater reference anchor $(\phi_0, \lambda_0, h_0)$.
  - Great-Circle Haversine distance, forward azimuth bearing, and 3D Closest Point of Approach (CPA) kinematics.

### 2. NATO Link-16 Tactical Data Link (STANAG 5516 / MIL-STD-6016)
- **J-Series Tactical Messages:**
  - `J2.2`: Air PPLI (Precise Participant Location and Identification) with Mode 4/5 IFF, fuel state, and missile inventory.
  - `J3.2`: Air Surveillance Track with 19-bit discrete track number, quantized position/altitude, speed, and heading.
  - `J3.6`: Space & Ballistic Missile Track with boost-phase status, apogee, and predicted impact point.
  - `J9.0`: Weapon Coordination & Engagement Order directive.
- **Wire Serialization:** Packed 75-bit binary frames with parity and network participation group (NPG) tags.
- **Symbology:** Automated generation of **MIL-STD-2525D 15-character SIDC** (e.g., `SHAPCRUI-----` for hostile cruise missile, `SFAPF15S-----` for friendly air patrol).

### 3. Multi-Hypothesis Tracking (MHT) & JPDA Sensor Fusion
- **Kinematic State:** 6-DOF state vector $\mathbf{x} = [x, y, z, \dot{x}, \dot{y}, \dot{z}]^T$ with Discrete White Noise Acceleration (DWNA) process noise covariance $\mathbf{Q}$.
- **Mahalanobis Distance Gating:**
  $$d_M^2 = (\mathbf{z} - \mathbf{H}\hat{\mathbf{x}})^T \mathbf{S}^{-1} (\mathbf{z} - \mathbf{H}\hat{\mathbf{x}}) \le \gamma_{\chi^2}$$
  Validated against $\chi_3^2(0.99) = 11.345$ statistical gate threshold.
- **Joint Association:** Computes Gaussian likelihoods under Poisson clutter density $\lambda_c$ and applies Joint Probabilistic Data Association (JPDA) weights $\beta_i$.
- **Track Lifecycle:** Strict $M$-out-of-$N$ (3-of-5 scans) promotion logic: `TENTATIVE` $\to$ `CONFIRMED`, coasting under missing detections, and dropping stale hypotheses.

### 4. Threat Evaluation & Weapon Assignment (TEWA) Engine
- **Threat Prioritization:** Evaluates multi-criteria threat index based on target lethality, closing velocity $V_c$, time-to-impact (TTI), and defended asset priority weight $V_k$.
- **Weapon Allocation:** Maximum Marginal Return (MMR) optimization allocating Patriot PAC-3 MSE, THAAD, and NASAMS batteries against kinematic engagement envelopes (altitude, slant range, speed).
- **Doctrine Assurance:** Automatic two-missile salvo firing doctrine for ballistic and hypersonic threats, computing cumulative theater kill probability:
  $$P_{\text{kill}} = 1 - \prod_{i=1}^{n} (1 - P_{k, i})$$

### 5. C++17 Native Unscented Kalman Filter (UKF) Engine
- Located in `cpp_core/kalman_track_fusion.cpp`.
- Implements Van der Merwe Scaled Unscented Transform ($\alpha = 10^{-3}, \beta = 2, \kappa = 0$) with Cholesky matrix decomposition.
- Nonlinear equations of motion integrating Earth gravity ($g = 9.80665\,\text{m/s}^2$) and exponential atmospheric density drag $\rho(z) = \rho_0 e^{-z / H_0}$ via 4th-order Runge-Kutta (RK4).
- Radar measurement model fusing slant range, azimuth, elevation, and Doppler radial velocity.
- Multi-sensor Covariance Intersection (CI) for distributed radar nodes.

### 6. Python AsyncIO Cursor-on-Target (CoT) SITL Gateway
- Located in `python_c2/c4isr_sitl_node.py`.
- Generates and parses standard Cursor-on-Target (CoT) XML schema (`<event type="a-f-A" ...>`).
- Non-blocking AsyncIO UDP router for tactical telemetry broadcast and SITL simulator bridging.
- Built-in `--self-test` harness verifying XML generation, round-trip parsing, and UDP socket loopback.

---

## 📂 Repository Layout

```text
.
├── .github/workflows/
│   └── ci.yml                 # Automated Multi-Language CI/CD Pipeline
├── cpp_core/
│   ├── kalman_track_fusion.hpp# C++17 UKF Ballistic Track Fusion Header
│   └── kalman_track_fusion.cpp# C++17 Implementation & Benchmark Harness
├── python_c2/
│   ├── c4isr_sitl_node.py     # Python AsyncIO Cursor-on-Target (CoT) UDP Gateway
│   └── requirements.txt       # SITL Node Python Requirements
├── src/
│   ├── c4isr/
│   │   └── link16_jseries.ts  # NATO STANAG 5516 Link-16 J-Series & SIDC Engine
│   ├── core/
│   │   └── kinematics.ts      # WGS84 Geodesy, ECEF/ENU, Haversine & CPA Solver
│   ├── fusion/
│   │   └── multi_hypothesis_tracker.ts # MHT & JPDA Multi-Sensor Fusion
│   ├── threat/
│   │   └── threat_evaluation_weapon_assignment.ts # TEWA Air Defense Optimizer
│   └── main.ts                # Application Entrypoint & 3D Sandtable Controller
├── tests/
│   └── c4isr_fusion.test.ts   # Vitest Verification Suite
├── index.html                 # Modular Web Entrypoint
├── package.json               # Node.js Dependencies & NPM Scripts
├── tsconfig.json              # Strict TypeScript ES2022 Configuration
├── vite.config.ts             # Vite Production Bundler Configuration
└── README.md                  # Defense Engineering Technical Reference
```

---

## 🛠️ Verification & Build Instructions

### 1. TypeScript Core & Unit Tests
```bash
# Typecheck TypeScript codebase
npm run typecheck

# Run Vitest C4ISR unit tests
npm test

# Build production bundle
npm run build
```

### 2. C++17 UKF Ballistic Fusion Engine
```bash
# Compile shared library
g++ -std=c++17 -O3 -Wall -Wextra -pedantic -fPIC -shared \
  cpp_core/kalman_track_fusion.cpp -o cpp_core/build/libkalman_fusion.so

# Run standalone ballistic benchmark
g++ -std=c++17 -O3 -DAEGIS_STANDALONE_BENCHMARK \
  cpp_core/kalman_track_fusion.cpp -o cpp_core/build/ukf_benchmark
./cpp_core/build/ukf_benchmark
```

### 3. Python Cursor-on-Target (CoT) SITL Router
```bash
# Execute syntax check and automated self-test
python -m py_compile python_c2/c4isr_sitl_node.py
python python_c2/c4isr_sitl_node.py --self-test

# Launch UDP gateway on port 4242
python python_c2/c4isr_sitl_node.py --port 4242
```

---

## 🌐 Live Operations & Deployment

- **Live Tactical Twin URL:** [https://tareq0001.github.io/AegisCommand-C4ISR-Digital-Twin/](https://tareq0001.github.io/AegisCommand-C4ISR-Digital-Twin/)
- **Hosting Infrastructure:** GitHub Pages serving directly from `gh-pages` branch (`/`).
- **Author:** Eng. Tareq Aboushi ([@Tareq0001](https://github.com/Tareq0001))
- **Defense Standards:** NATO STANAG 5516 (Link-16) | MIL-STD-2525D | WGS 84 | MITRE CoT
