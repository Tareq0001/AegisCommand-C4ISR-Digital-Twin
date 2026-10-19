import { describe, it, expect } from 'vitest';
import {
  WGS84,
  geodeticToECEF,
  ecefToGeodetic,
  geodeticToENU,
  enuToGeodetic,
  haversineDistance,
  calculateBearing,
  calculateCPA,
  GeodeticCoord,
  ENUCoord,
} from '../src/core/kinematics';
import {
  Link16MessageFactory,
  encodeLink16_J3_2_Frame,
  decodeLink16_J3_2_Frame,
  generateSIDC,
} from '../src/c4isr/link16_jseries';
import {
  MultiHypothesisTracker,
  Measurement3D,
} from '../src/fusion/multi_hypothesis_tracker';
import {
  ThreatEvaluationWeaponAssignment,
  DefendedAsset,
  WeaponSystem,
  ThreatTarget,
} from '../src/threat/threat_evaluation_weapon_assignment';

describe('AegisCommand C4ISR — Kinematics & Geodesy Engine', () => {
  it('converts Geodetic coordinates to ECEF and back with sub-millimeter precision', () => {
    // King Khalid Air Base, Khamis Mushait / Abha region (2170m elevation)
    const testGeo: GeodeticCoord = {
      lat: 18.2974,
      lon: 42.8028,
      alt: 2170.0,
    };

    const ecef = geodeticToECEF(testGeo);
    expect(ecef.x).toBeDefined();
    expect(ecef.y).toBeDefined();
    expect(ecef.z).toBeDefined();

    const roundTripGeo = ecefToGeodetic(ecef);
    expect(roundTripGeo.lat).toBeCloseTo(testGeo.lat, 6);
    expect(roundTripGeo.lon).toBeCloseTo(testGeo.lon, 6);
    expect(Math.abs(roundTripGeo.alt - testGeo.alt)).toBeLessThan(0.005);
  });

  it('projects local East-North-Up (ENU) coordinates accurately relative to tactical anchor', () => {
    const origin: GeodeticCoord = { lat: 18.25, lon: 42.50, alt: 2200 };
    const target: GeodeticCoord = { lat: 18.35, lon: 42.60, alt: 2500 };

    const enu = geodeticToENU(target, origin);
    expect(enu.east).toBeGreaterThan(0);
    expect(enu.north).toBeGreaterThan(0);
    expect(enu.up).toBeCloseTo(300, 0);

    const reconstructed = enuToGeodetic(enu, origin);
    expect(reconstructed.lat).toBeCloseTo(target.lat, 5);
    expect(reconstructed.lon).toBeCloseTo(target.lon, 5);
  });

  it('computes Haversine surface distance and forward tactical azimuth', () => {
    const abha: GeodeticCoord = { lat: 18.2164, lon: 42.5053, alt: 0 };
    const khamis: GeodeticCoord = { lat: 18.3000, lon: 42.7333, alt: 0 };

    const distanceMeters = haversineDistance(abha, khamis);
    expect(distanceMeters).toBeGreaterThan(25000);
    expect(distanceMeters).toBeLessThan(35000);

    const bearing = calculateBearing(abha, khamis);
    expect(bearing).toBeGreaterThan(60);
    expect(bearing).toBeLessThan(80);
  });

  it('computes Closest Point of Approach (CPA) for intercept kinematics', () => {
    const pos1: ENUCoord = { east: 0, north: 0, up: 5000 };
    const vel1: ENUCoord = { east: 250, north: 0, up: 0 }; // 250 m/s East

    const pos2: ENUCoord = { east: 10000, north: 2000, up: 5000 };
    const vel2: ENUCoord = { east: -250, north: 0, up: 0 }; // Head-on 250 m/s West

    const cpa = calculateCPA(pos1, vel1, pos2, vel2);
    // Relative speed 500 m/s, distance 10000 m -> t_cpa ~ 20 s
    expect(cpa.timeToCPA).toBeCloseTo(20.0, 1);
    expect(cpa.minDistance).toBeCloseTo(2000.0, 1);
  });
});

describe('AegisCommand C4ISR — NATO Link-16 STANAG 5516 Tactical Data Link', () => {
  it('creates valid J2.2 Air PPLI messages', () => {
    const ppli = Link16MessageFactory.createAirPPLI(
      101,
      'VIPER-01',
      { lat: 18.3, lon: 42.8, alt: 8500 },
      { speed: 320, heading: 145, climbRate: 10 },
      115
    );

    expect(ppli.header.messageType).toBe('J2.2');
    expect(ppli.identity).toBe('FRIEND');
    expect(ppli.callsign).toBe('VIPER-01');
    expect(ppli.iffMode4Valid).toBe(true);
    expect(ppli.weaponsInventory.aim120dMissiles).toBe(6);
  });

  it('encodes and decodes Link-16 J3.2 Air Track frames into binary wire format', () => {
    const airTrack = Link16MessageFactory.createAirTrack(
      512,
      'HOSTILE',
      { lat: 18.5, lon: 43.1, alt: 12000 },
      { speed: 580, heading: 220, climbRate: -15 },
      'CRUISE_MISSILE'
    );

    const wireFrame = encodeLink16_J3_2_Frame(airTrack);
    expect(wireFrame.length).toBe(16);

    const decoded = decodeLink16_J3_2_Frame(wireFrame);
    expect(decoded.identity).toBe('HOSTILE');
    expect(decoded.position?.lat).toBeCloseTo(18.5, 1);
    expect(decoded.position?.lon).toBeCloseTo(43.1, 1);
    expect(decoded.position?.alt).toBeCloseTo(12000, -2);
  });

  it('generates standard MIL-STD-2525D SIDC symbology identifiers', () => {
    const hostileAirSIDC = generateSIDC('HOSTILE', 'AIR', 'CRUISE_MISSILE');
    expect(hostileAirSIDC.startsWith('SHAPCRUI')).toBe(true);

    const friendSeaSIDC = generateSIDC('FRIEND', 'SURFACE', 'CORVETTE');
    expect(friendSeaSIDC.startsWith('SFSPCORV')).toBe(true);
  });
});

describe('AegisCommand C4ISR — Multi-Hypothesis Tracking & Sensor Fusion', () => {
  it('gates measurements with Mahalanobis distance under Chi-Square threshold', () => {
    const tracker = new MultiHypothesisTracker();

    const m1: Measurement3D = {
      id: 'M1',
      sensorId: 'AN-MPQ-65',
      timestamp: 1000,
      pos: { east: 5000, north: 8000, up: 3000 },
      covarianceR: [
        [100, 0, 0],
        [0, 100, 0],
        [0, 0, 200],
      ],
    };

    tracker.processScan([m1], 1000);
    const tracks = tracker.getTracks();
    expect(tracks.length).toBe(1);
    expect(tracks[0].status).toBe('TENTATIVE');

    // Simulate 2nd detection following trajectory
    tracker.predict(1.0);
    const m2: Measurement3D = {
      id: 'M2',
      sensorId: 'AN-MPQ-65',
      timestamp: 2000,
      pos: { east: 5005, north: 8010, up: 3000 },
      covarianceR: [
        [100, 0, 0],
        [0, 100, 0],
        [0, 0, 200],
      ],
    };

    const gate = tracker.gateMeasurement(tracks[0], m2);
    expect(gate.inGate).toBe(true);
    expect(gate.mahalanobisDistSq).toBeLessThan(11.345);
  });

  it('confirms tracks after satisfying M-out-of-N criteria', () => {
    const tracker = new MultiHypothesisTracker();

    for (let i = 0; i < 4; i++) {
      tracker.predict(1.0);
      tracker.processScan(
        [
          {
            id: `M-SCAN-${i}`,
            sensorId: 'RADAR-01',
            timestamp: 1000 + i * 1000,
            pos: { east: 1000 + i * 200, north: 2000, up: 4000 },
            covarianceR: [
              [50, 0, 0],
              [0, 50, 0],
              [0, 0, 50],
            ],
          },
        ],
        1000 + i * 1000
      );
    }

    const confirmedTracks = tracker.getTracks().filter((t) => t.status === 'CONFIRMED');
    expect(confirmedTracks.length).toBe(1);
  });
});

describe('AegisCommand C4ISR — Threat Evaluation & Weapon Assignment (TEWA)', () => {
  it('evaluates threats and assigns highest priority to fast ballistic vector', () => {
    const asset: DefendedAsset = {
      id: 'ASSET-HQ',
      name: 'Southern Sector Air Defense Operations Center',
      pos: { east: 0, north: 0, up: 0 },
      defenseValue: 100,
      lethalDefenseRadiusMeters: 45000,
    };

    const ballisticTarget: ThreatTarget = {
      id: 'THREAT-01',
      trackNumber: '0B01',
      classification: 'BALLISTIC_MISSILE',
      pos: { east: 40000, north: 30000, up: 25000 },
      vel: { east: -1200, north: -900, up: -400 }, // Closing fast at 1500 m/s
      radarCrossSectionSm: 0.05,
      jammingActive: false,
    };

    const uavTarget: ThreatTarget = {
      id: 'THREAT-02',
      trackNumber: '0U02',
      classification: 'ATTACK_UAV',
      pos: { east: 80000, north: 70000, up: 4000 },
      vel: { east: -50, north: -40, up: 0 },
      radarCrossSectionSm: 0.1,
      jammingActive: false,
    };

    const tewa = new ThreatEvaluationWeaponAssignment([asset], []);
    const ranked = tewa.evaluateThreats([ballisticTarget, uavTarget]);

    expect(ranked.length).toBe(2);
    expect(ranked[0].target.id).toBe('THREAT-01');
    expect(ranked[0].threatScore).toBeGreaterThan(ranked[1].threatScore);
    expect(ranked[0].recommendedDoctrine).toBe('SALVO_2');
  });

  it('allocates Patriot PAC-3 MSE interceptors optimal engagement within kinematic envelope', () => {
    const asset: DefendedAsset = {
      id: 'ASSET-01',
      name: 'Air Base Complex',
      pos: { east: 0, north: 0, up: 0 },
      defenseValue: 95,
      lethalDefenseRadiusMeters: 40000,
    };

    const pac3Battery: WeaponSystem = {
      id: 'PATRIOT-BTRY-1',
      name: 'Patriot PAC-3 MSE Battery Alpha',
      type: 'PATRIOT_PAC3_MSE',
      pos: { east: 2000, north: 1000, up: 50 },
      minRangeMeters: 1000,
      maxRangeMeters: 65000,
      minAltMeters: 100,
      maxAltMeters: 35000,
      interceptorSpeedMs: 1400, // Mach 4+
      singleShotKillProb: 0.92,
      readyMissiles: 16,
      maxSalvoPerTarget: 2,
    };

    const cruiseTarget: ThreatTarget = {
      id: 'THREAT-CM',
      trackNumber: '0C09',
      classification: 'CRUISE_MISSILE',
      pos: { east: 35000, north: 15000, up: 250 },
      vel: { east: -280, north: -120, up: 0 },
      radarCrossSectionSm: 0.2,
      jammingActive: true,
    };

    const tewa = new ThreatEvaluationWeaponAssignment([asset], [pac3Battery]);
    const evaluated = tewa.evaluateThreats([cruiseTarget]);
    const plan = tewa.solveWeaponAssignment(evaluated);

    expect(plan.engagements.length).toBe(1);
    expect(plan.engagements[0].assignedWeapon.id).toBe('PATRIOT-BTRY-1');
    expect(plan.engagements[0].expectedKillProbability).toBeGreaterThanOrEqual(0.92);
    expect(plan.unengagedThreatCount).toBe(0);
  });
});
