/**
 * AegisCommand C4ISR — Joint Situational Awareness & Theater Command Twin
 * Application Entrypoint: Integrates WGS84 Kinematics, NATO Link-16 J-Series TDL,
 * Multi-Hypothesis Tracking (MHT/JPDA) Sensor Fusion, and Patriot PAC-3 TEWA Optimization.
 */

import {
  geodeticToENU,
  calculateCPA,
  haversineDistance,
  calculateBearing,
  GeodeticCoord,
  ENUCoord,
} from './core/kinematics';
import {
  Link16MessageFactory,
  generateSIDC,
  Link16_J3_2_AirTrack,
  Link16_J2_2_AirPPLI,
} from './c4isr/link16_jseries';
import {
  MultiHypothesisTracker,
  Measurement3D,
  TrackState,
} from './fusion/multi_hypothesis_tracker';
import {
  ThreatEvaluationWeaponAssignment,
  DefendedAsset,
  WeaponSystem,
  ThreatTarget,
  TEWASolution,
} from './threat/threat_evaluation_weapon_assignment';

// Window THREE declaration for CDN and ESM environments
declare const THREE: any;

// Bilingual Tactical Terminology
const I18N = {
  en: {
    subDesc: "Joint Operations Command, Common Operational Picture (COP) & Multi-Domain Sensor Fusion",
    defconNormal: "DEFCON 4 (NORMAL)",
    defconAlert: "DEFCON 2 (ARMED DEFENSE)",
    btnAlert: "DEFCON 2 ALERT",
    btnResetAlert: "STAND DOWN TO DEFCON 4",
    allDomains: "Joint All-Domain",
    airDomain: "Air Defense",
    seaDomain: "Maritime Coastal",
    spaceDomain: "Space & ISR Sat",
    radarTitle: "RADAR & AIR CORRIDOR COP",
    interceptorTitle: "AIR DEFENSE INTERCEPT CALCULATOR",
    engageBtn: "AUTHORIZE PATRIOT INTERCEPT",
    engagedBtn: "INTERCEPTOR LAUNCHED (IN FLIGHT)",
    logTitle: "C4ISR SEQUENCE OF EVENTS"
  },
  ar: {
    subDesc: "مركز القيادة والسيطرة المشتركة، الصورة العملياتية الموحدة (COP) ودمج حساسات مسرح العمليات",
    defconNormal: "الحالة 4 (جاهزية طبيعية)",
    defconAlert: "الحالة 2 (تأهب دفاعي شامل)",
    btnAlert: "إعلان حالة التأهب 2",
    btnResetAlert: "خفض التأهب للحالة 4",
    allDomains: "كافة مسارح العمليات",
    airDomain: "الدفاع الجوي",
    seaDomain: "الأسطول البحري والساحلي",
    spaceDomain: "الأقمار الاصطناعية والفضاء",
    radarTitle: "رادار المراقبة الجوية المشتركة",
    interceptorTitle: "حاسبة الاعتراض والدفاع الجوي",
    engageBtn: "تفويض إطلاق صاروخ باتريوت",
    engagedBtn: "تم الإطلاق وجارٍ الاعتراض",
    logTitle: "سجل الأحداث والبرقيات العملياتية"
  }
};

let currentLang: 'en' | 'ar' = 'en';

// Tactical Subsystem Instances
const mhtTracker = new MultiHypothesisTracker();
const tewaEngine = new ThreatEvaluationWeaponAssignment();

// Defended Theater Anchor (Southern Operations Sector, Abha / Khamis Region)
const theaterAnchor: GeodeticCoord = { lat: 18.2974, lon: 42.8028, alt: 2170 };

const defendedBase: DefendedAsset = {
  id: "HQ-AIRBASE",
  name: "King Khalid Air Base / Theater Command",
  pos: { east: 0, north: 0, up: 0 },
  defenseValue: 100,
  lethalDefenseRadiusMeters: 45000,
};

const pac3Battery: WeaponSystem = {
  id: "PATRIOT-PAC3-BATTERY-1",
  name: "Patriot PAC-3 MSE Battery Alpha",
  type: "PATRIOT_PAC3_MSE",
  pos: { east: -15000, north: 5000, up: 50 },
  minRangeMeters: 1000,
  maxRangeMeters: 65000,
  minAltMeters: 60,
  maxAltMeters: 36000,
  interceptorSpeedMs: 1450,
  singleShotKillProb: 0.93,
  readyMissiles: 16,
  maxSalvoPerTarget: 2,
};

tewaEngine.setDefendedAssets([defendedBase]);
tewaEngine.setWeapons([pac3Battery]);

// Web Audio API Synthesizer
let audioCtx: AudioContext | null = null;
let isAudioMuted = true;

function initAudio(): void {
  if (!audioCtx) {
    const AudioContextClass = window.AudioContext || (window as any).webkitAudioContext;
    if (AudioContextClass) {
      audioCtx = new AudioContextClass();
    }
  }
}

function playAlertSound(): void {
  if (isAudioMuted || !audioCtx) return;
  const now = audioCtx.currentTime;
  const osc = audioCtx.createOscillator();
  const gain = audioCtx.createGain();
  osc.type = 'sawtooth';
  osc.frequency.setValueAtTime(850, now);
  osc.frequency.setValueAtTime(420, now + 0.2);
  gain.gain.setValueAtTime(0.2, now);
  gain.gain.exponentialRampToValueAtTime(0.001, now + 0.4);
  osc.connect(gain);
  gain.connect(audioCtx.destination);
  osc.start(now);
  osc.stop(now + 0.4);
}

// Three.js Scene Variables
let scene: any, camera: any, renderer: any, controls: any;
let theaterGroup: any, airGroup: any, seaGroup: any, spaceGroup: any, samGroup: any;
const airTracks: { mesh: any; speed: number; radius: number; angle: number }[] = [];
const hostileTracks: { mesh: any; speed: number; dir: any }[] = [];

function initThree(): void {
  const container = document.getElementById('threeContainer');
  if (!container || typeof THREE === 'undefined') return;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x020617);
  scene.fog = new THREE.FogExp2(0x020617, 0.006);

  camera = new THREE.PerspectiveCamera(50, container.clientWidth / container.clientHeight, 0.1, 3000);
  camera.position.set(0, 70, 90);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  container.innerHTML = '';
  container.appendChild(renderer.domElement);

  if (THREE.OrbitControls) {
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.maxPolarAngle = Math.PI / 2 - 0.05;
  }

  // Lighting
  const ambient = new THREE.AmbientLight(0x38bdf8, 0.6);
  scene.add(ambient);
  const dirLight = new THREE.DirectionalLight(0xffffff, 1.0);
  dirLight.position.set(50, 120, 50);
  scene.add(dirLight);

  // Tactical Sandtable Hologram Grid
  const gridHelper = new THREE.GridHelper(200, 40, 0x0284c7, 0x0f172a);
  gridHelper.position.y = 0;
  scene.add(gridHelper);

  theaterGroup = new THREE.Group();
  airGroup = new THREE.Group();
  seaGroup = new THREE.Group();
  spaceGroup = new THREE.Group();
  samGroup = new THREE.Group();

  theaterGroup.add(airGroup);
  theaterGroup.add(seaGroup);
  theaterGroup.add(spaceGroup);
  theaterGroup.add(samGroup);
  scene.add(theaterGroup);

  buildAirDefenseDome();
  buildMaritimeFleet();
  buildAirTracks();
  buildSpaceOrbit();

  window.addEventListener('resize', onWindowResize);
  animate();
}

function buildAirDefenseDome(): void {
  const batteryMesh = new THREE.Mesh(
    new THREE.BoxGeometry(4, 1.5, 6),
    new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.8 })
  );
  batteryMesh.position.set(-20, 0.75, 10);
  samGroup.add(batteryMesh);

  const domeGeo = new THREE.SphereGeometry(38, 24, 16, 0, Math.PI * 2, 0, Math.PI / 2);
  const domeMat = new THREE.MeshBasicMaterial({
    color: 0x0284c7,
    wireframe: true,
    transparent: true,
    opacity: 0.25,
  });
  const dome = new THREE.Mesh(domeGeo, domeMat);
  dome.position.copy(batteryMesh.position);
  dome.position.y = 0;
  samGroup.add(dome);
}

function buildMaritimeFleet(): void {
  const shipMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.7 });
  const frigateCoords = [
    { x: 30, z: -35 },
    { x: 55, z: -20 },
    { x: 45, z: 15 }
  ];

  frigateCoords.forEach(c => {
    const hull = new THREE.Mesh(new THREE.BoxGeometry(3, 1.8, 10), shipMat);
    hull.position.set(c.x, 0.9, c.z);
    seaGroup.add(hull);

    const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, 4), shipMat);
    mast.position.set(c.x, 3.5, c.z);
    seaGroup.add(mast);
  });
}

function buildAirTracks(): void {
  const fighterMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8 });
  const hostileMat = new THREE.MeshStandardMaterial({ color: 0xef4444 });

  // Blue Air Patrols (F-15SA)
  for (let i = 0; i < 4; i++) {
    const jet = new THREE.Mesh(new THREE.ConeGeometry(0.8, 3, 6), fighterMat);
    jet.rotation.x = Math.PI / 2;
    jet.position.set(-40 + i * 25, 20 + i * 4, -20 + i * 15);
    airGroup.add(jet);
    airTracks.push({ mesh: jet, speed: 0.12 + i * 0.03, radius: 25 + i * 8, angle: i * 1.5 });
  }

  // Red Hostile Vector
  const hostile = new THREE.Mesh(new THREE.ConeGeometry(0.6, 2.5, 6), hostileMat);
  hostile.rotation.x = Math.PI / 2;
  hostile.position.set(60, 25, -60);
  airGroup.add(hostile);
  hostileTracks.push({
    mesh: hostile,
    speed: 0.22,
    dir: new THREE.Vector3(-0.6, 0, 0.6).normalize()
  });
}

function buildSpaceOrbit(): void {
  const satMat = new THREE.MeshStandardMaterial({ color: 0xfacc15, metalness: 0.9 });
  const sat = new THREE.Mesh(new THREE.BoxGeometry(2, 2, 2), satMat);
  sat.position.set(0, 55, 0);

  const panelMat = new THREE.MeshStandardMaterial({ color: 0x1d4ed8 });
  const leftPanel = new THREE.Mesh(new THREE.BoxGeometry(4, 0.1, 1.5), panelMat);
  leftPanel.position.set(-3.5, 0, 0);
  const rightPanel = new THREE.Mesh(new THREE.BoxGeometry(4, 0.1, 1.5), panelMat);
  rightPanel.position.set(3.5, 0, 0);
  sat.add(leftPanel);
  sat.add(rightPanel);

  spaceGroup.add(sat);

  const orbitCurve = new THREE.EllipseCurve(0, 0, 65, 45, 0, 2 * Math.PI, false, 0);
  const points = orbitCurve.getPoints(50);
  const orbitGeo = new THREE.BufferGeometry().setFromPoints(points);
  const orbitLine = new THREE.Line(
    orbitGeo,
    new THREE.LineBasicMaterial({ color: 0xfacc15, transparent: true, opacity: 0.3 })
  );
  orbitLine.rotation.x = Math.PI / 2;
  orbitLine.position.y = 55;
  spaceGroup.add(orbitLine);
}

function onWindowResize(): void {
  const container = document.getElementById('threeContainer');
  if (!container || !camera || !renderer) return;
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}

// 2D Tactical COP Radar Canvas
let sweepAngle = 0;
function drawCopRadar(): void {
  const radarCanvas = document.getElementById('copRadarCanvas') as HTMLCanvasElement;
  if (!radarCanvas) return;
  const rCtx = radarCanvas.getContext('2d');
  if (!rCtx) return;

  rCtx.fillStyle = 'rgba(2, 6, 23, 0.15)';
  rCtx.fillRect(0, 0, radarCanvas.width, radarCanvas.height);

  const cx = radarCanvas.width / 2;
  const cy = radarCanvas.height / 2;
  const maxR = radarCanvas.height / 2 - 5;

  // Range Rings
  rCtx.strokeStyle = 'rgba(14, 165, 233, 0.2)';
  rCtx.lineWidth = 1;
  for (let r = 25; r <= maxR; r += 25) {
    rCtx.beginPath();
    rCtx.arc(cx, cy, r, 0, Math.PI * 2);
    rCtx.stroke();
  }

  // Sweep Line (12 RPM)
  sweepAngle += 0.04;
  const sweepX = cx + Math.cos(sweepAngle) * maxR;
  const sweepY = cy + Math.sin(sweepAngle) * maxR;

  rCtx.strokeStyle = 'rgba(56, 189, 248, 0.8)';
  rCtx.lineWidth = 1.5;
  rCtx.beginPath();
  rCtx.moveTo(cx, cy);
  rCtx.lineTo(sweepX, sweepY);
  rCtx.stroke();

  // Blue Tracks
  rCtx.fillStyle = '#38bdf8';
  airTracks.forEach(t => {
    const tx = cx + (t.mesh.position.x / 100) * (maxR - 10);
    const ty = cy + (t.mesh.position.z / 100) * (maxR - 10);
    rCtx.beginPath();
    rCtx.arc(tx, ty, 3, 0, Math.PI * 2);
    rCtx.fill();
  });

  // Red Hostile Track
  rCtx.fillStyle = '#ef4444';
  hostileTracks.forEach(h => {
    const hx = cx + (h.mesh.position.x / 100) * (maxR - 10);
    const hy = cy + (h.mesh.position.z / 100) * (maxR - 10);
    rCtx.beginPath();
    rCtx.arc(hx, hy, 4, 0, Math.PI * 2);
    rCtx.fill();
  });
}

function animate(): void {
  requestAnimationFrame(animate);
  if (controls) controls.update();

  airTracks.forEach(t => {
    t.angle += t.speed * 0.05;
    t.mesh.position.x = Math.sin(t.angle) * t.radius;
    t.mesh.position.z = Math.cos(t.angle) * t.radius;
    t.mesh.rotation.y = -t.angle;
  });

  hostileTracks.forEach(h => {
    h.mesh.position.addScaledVector(h.dir, h.speed);
    if (h.mesh.position.x < -60) h.mesh.position.set(60, 25, -60);
  });

  drawCopRadar();
  if (renderer && scene && camera) {
    renderer.render(scene, camera);
  }
}

function addLog(msg: string, type: 'info' | 'alert' | 'warn' = 'info'): void {
  const list = document.getElementById('c4isrLogList');
  if (!list) return;
  const now = new Date().toTimeString().split(' ')[0];
  const div = document.createElement('div');
  const borderCol =
    type === 'alert'
      ? 'border-rose-500'
      : type === 'warn'
      ? 'border-amber-500'
      : 'border-sky-500';
  div.className = `p-1 rounded bg-black/40 border-l-2 ${borderCol}`;
  div.innerHTML = `<span class="text-slate-500">${now}</span> ${msg}`;
  list.prepend(div);
  if (list.children.length > 25) list.removeChild(list.lastChild!);
}

function initUIListeners(): void {
  let isDefcon2 = false;
  let isEngaged = false;

  const alertBtn = document.getElementById('alertBtn');
  alertBtn?.addEventListener('click', () => {
    initAudio();
    isDefcon2 = !isDefcon2;
    const txt = document.getElementById('txtAlert');
    const defBadge = document.getElementById('telDefcon');

    if (isDefcon2) {
      if (alertBtn) alertBtn.className = "px-2.5 py-1 text-xs font-bold rounded bg-rose-600 text-white animate-pulse transition flex items-center space-x-1";
      if (txt) txt.innerText = I18N[currentLang].btnResetAlert;
      if (defBadge) {
        defBadge.innerText = I18N[currentLang].defconAlert;
        defBadge.className = "font-bold text-rose-500";
      }
      playAlertSound();
      addLog("[ALERT] DEFCON 2 DECLARED! SAM batteries armed and tracking hostile cruise vector.", "alert");
    } else {
      if (alertBtn) alertBtn.className = "px-2.5 py-1 text-xs font-bold rounded bg-rose-950/80 border border-rose-600/60 hover:bg-rose-900 text-rose-300 transition flex items-center space-x-1";
      if (txt) txt.innerText = I18N[currentLang].btnAlert;
      if (defBadge) {
        defBadge.innerText = I18N[currentLang].defconNormal;
        defBadge.className = "font-bold text-emerald-400";
      }
      addLog("[C2] Defense condition normalized to DEFCON 4.", "info");
    }
  });

  const engageBtn = document.getElementById('engageBtn');
  engageBtn?.addEventListener('click', () => {
    initAudio();
    playAlertSound();
    isEngaged = true;
    if (engageBtn) {
      engageBtn.className = "w-full py-1.5 rounded bg-emerald-600 text-white font-bold text-xs transition flex items-center justify-center space-x-1.5";
    }
    const txtEngage = document.getElementById('txtEngage');
    if (txtEngage) txtEngage.innerText = I18N[currentLang].engagedBtn;
    addLog("[SAM] Patriot PAC-3 Interceptor salvo authorized and fired! Intercept calculated at 42 NM.", "alert");
  });

  document.getElementById('filterAllBtn')?.addEventListener('click', () => {
    if (airGroup) airGroup.visible = true;
    if (seaGroup) seaGroup.visible = true;
    if (spaceGroup) spaceGroup.visible = true;
    if (samGroup) samGroup.visible = true;
    setActiveFilter('filterAllBtn');
  });

  document.getElementById('filterAirBtn')?.addEventListener('click', () => {
    if (airGroup) airGroup.visible = true;
    if (seaGroup) seaGroup.visible = false;
    if (spaceGroup) spaceGroup.visible = false;
    if (samGroup) samGroup.visible = true;
    setActiveFilter('filterAirBtn');
  });

  document.getElementById('filterSeaBtn')?.addEventListener('click', () => {
    if (airGroup) airGroup.visible = false;
    if (seaGroup) seaGroup.visible = true;
    if (spaceGroup) spaceGroup.visible = false;
    if (samGroup) samGroup.visible = false;
    setActiveFilter('filterSeaBtn');
  });

  document.getElementById('filterSpaceBtn')?.addEventListener('click', () => {
    if (airGroup) airGroup.visible = false;
    if (seaGroup) seaGroup.visible = false;
    if (spaceGroup) spaceGroup.visible = true;
    if (samGroup) samGroup.visible = false;
    setActiveFilter('filterSpaceBtn');
  });

  document.getElementById('resetCamBtn')?.addEventListener('click', () => {
    if (camera && controls) {
      camera.position.set(0, 70, 90);
      controls.target.set(0, 0, 0);
    }
  });

  document.getElementById('soundBtn')?.addEventListener('click', () => {
    initAudio();
    isAudioMuted = !isAudioMuted;
    const soundIcon = document.getElementById('soundIcon');
    if (soundIcon) {
      soundIcon.className = isAudioMuted ? "fa-solid fa-volume-xmark" : "fa-solid fa-volume-high text-sky-400";
    }
  });

  document.getElementById('clearLogsBtn')?.addEventListener('click', () => {
    const list = document.getElementById('c4isrLogList');
    if (list) list.innerHTML = '';
  });

  document.getElementById('langToggleBtn')?.addEventListener('click', () => {
    currentLang = currentLang === 'en' ? 'ar' : 'en';
    document.documentElement.dir = currentLang === 'ar' ? 'rtl' : 'ltr';
    const langBtn = document.getElementById('langToggleBtn');
    if (langBtn) langBtn.innerText = currentLang === 'ar' ? 'English' : 'عربي';

    const t = I18N[currentLang];
    const subDesc = document.getElementById('subHeaderDesc');
    if (subDesc) subDesc.innerText = t.subDesc;

    const defBadge = document.getElementById('telDefcon');
    if (defBadge) defBadge.innerText = isDefcon2 ? t.defconAlert : t.defconNormal;

    const txtAlert = document.getElementById('txtAlert');
    if (txtAlert) txtAlert.innerText = isDefcon2 ? t.btnResetAlert : t.btnAlert;

    const setTxt = (id: string, text: string) => {
      const el = document.getElementById(id);
      if (el) el.innerText = text;
    };

    setTxt('txtAllDomains', t.allDomains);
    setTxt('txtAirDomain', t.airDomain);
    setTxt('txtSeaDomain', t.seaDomain);
    setTxt('txtSpaceDomain', t.spaceDomain);
    setTxt('radarTitle', t.radarTitle);
    setTxt('interceptorTitle', t.interceptorTitle);
    setTxt('txtEngage', isEngaged ? t.engagedBtn : t.engageBtn);
    setTxt('eventLogTitle', t.logTitle);
  });
}

function setActiveFilter(id: string): void {
  ['filterAllBtn', 'filterAirBtn', 'filterSeaBtn', 'filterSpaceBtn'].forEach(b => {
    const elem = document.getElementById(b);
    if (!elem) return;
    if (b === id) {
      elem.className = "px-3 py-1.5 text-xs font-semibold rounded-lg bg-sky-600 text-white shadow-md shadow-sky-600/30 flex items-center space-x-1.5";
    } else {
      elem.className = "px-3 py-1.5 text-xs font-semibold rounded-lg text-slate-400 hover:text-sky-300 flex items-center space-x-1.5";
    }
  });
}

// Tactical Heartbeat: Periodically feeds simulated telemetry through MHT and TEWA
function startTacticalHeartbeat(): void {
  setInterval(() => {
    // 1. Simulate Radar detection scan
    const timestamp = Date.now();
    const meas1: Measurement3D = {
      id: `SCAN-${timestamp}`,
      sensorId: 'AN/MPQ-65',
      timestamp,
      pos: {
        east: 35000 + (Math.random() - 0.5) * 50,
        north: 15000 + (Math.random() - 0.5) * 50,
        up: 350 + (Math.random() - 0.5) * 20,
      },
      covarianceR: [[100, 0, 0], [0, 100, 0], [0, 0, 50]],
    };

    mhtTracker.predict(1.0);
    mhtTracker.processScan([meas1], timestamp);

    // 2. TEWA Evaluation update
    const cruiseThreat: ThreatTarget = {
      id: 'THREAT-RED-01',
      trackNumber: '0C09',
      classification: 'CRUISE_MISSILE',
      pos: { east: 35000, north: 15000, up: 350 },
      vel: { east: -280, north: -120, up: 0 },
      radarCrossSectionSm: 0.15,
      jammingActive: true,
    };

    const evaluated = tewaEngine.evaluateThreats([cruiseThreat]);
    const solution = tewaEngine.solveWeaponAssignment(evaluated);

    if (solution.engagements.length > 0) {
      const best = solution.engagements[0];
      const pkElem = document.getElementById('calcPk');
      if (pkElem) {
        pkElem.innerText = `${(best.expectedKillProbability * 100).toFixed(1)}%`;
      }
    }
  }, 2500);
}

// Lifecycle Boot
document.addEventListener('DOMContentLoaded', () => {
  initThree();
  initUIListeners();
  startTacticalHeartbeat();
  console.log('[+] AegisCommand C4ISR Modular Engine Initialized (MIL-STD-2525D / Link-16 / MHT / TEWA).');
});
