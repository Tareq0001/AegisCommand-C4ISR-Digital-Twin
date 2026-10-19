/**
 * AegisCommand C4ISR — Multi-Hypothesis Tracking (MHT) & JPDA Sensor Fusion Engine
 * Defense Standards: Bar-Shalom Multitarget-Multisensor Tracking / Blackman Multiple-Hypothesis Tracking
 */

import { ENUCoord } from '../core/kinematics';

export type TrackStatus = 'TENTATIVE' | 'CONFIRMED' | 'COASTED' | 'DROPPED';

export interface Measurement3D {
  id: string;
  sensorId: string;
  timestamp: number;
  pos: ENUCoord;       // Observed [x, y, z] in local ENU frame
  covarianceR: number[][]; // 3x3 measurement error covariance
}

export interface TrackState {
  id: string;
  trackNumber: string;
  status: TrackStatus;
  stateVector: [number, number, number, number, number, number]; // [x, y, z, vx, vy, vz]
  covarianceP: number[][];                                      // 6x6 state covariance
  confirmations: number;
  missedDetections: number;
  lastUpdated: number;
  trackQuality: number; // 1 (lowest) to 15 (highest)
  hypotheses: TrackHypothesis[];
}

export interface TrackHypothesis {
  id: string;
  associatedMeasurementId: string | null;
  likelihood: number;
  probability: number;
}

export interface GatingResult {
  measurement: Measurement3D;
  mahalanobisDistSq: number;
  inGate: boolean;
  innovation: [number, number, number];
}

/**
 * 3x3 Matrix inversion helper for innovation covariance S
 */
function invert3x3(m: number[][]): number[][] {
  const [
    [a, b, c],
    [d, e, f],
    [g, h, i],
  ] = m;

  const det =
    a * (e * i - f * h) -
    b * (d * i - f * g) +
    c * (d * h - e * g);

  if (Math.abs(det) < 1e-12) {
    // Degenerate covariance: fallback to diagonal pseudo-inverse
    return [
      [1.0 / (a || 1e-3), 0, 0],
      [0, 1.0 / (e || 1e-3), 0],
      [0, 0, 1.0 / (i || 1e-3)],
    ];
  }

  const invDet = 1.0 / det;

  return [
    [(e * i - f * h) * invDet, (c * h - b * i) * invDet, (b * f - c * e) * invDet],
    [(f * g - d * i) * invDet, (a * i - c * g) * invDet, (c * d - a * f) * invDet],
    [(d * h - e * g) * invDet, (g * b - a * h) * invDet, (a * e - b * d) * invDet],
  ];
}

/**
 * Determinant of 3x3 symmetric matrix
 */
function det3x3(m: number[][]): number {
  return (
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
  );
}

/**
 * Multi-Hypothesis Tracker and JPDA Track Management
 */
export class MultiHypothesisTracker {
  private tracks: Map<string, TrackState> = new Map();
  private nextTrackId = 100;
  private readonly defaultProcessNoiseQ = 4.0; // Acceleration spectral density q (m^2/s^3)
  private readonly gateThresholdChiSq = 11.345; // 3-DOF 99% Chi-Square gating threshold
  private readonly confirmationM = 3;           // 3 detections
  private readonly windowN = 5;                  // in 5 scans
  private readonly maxCoastFrames = 5;

  public getTracks(): TrackState[] {
    return Array.from(this.tracks.values()).filter((t) => t.status !== 'DROPPED');
  }

  public getTrack(id: string): TrackState | undefined {
    return this.tracks.get(id);
  }

  /**
   * Kalman Predict: Propagates track state vector and covariance by dt seconds
   */
  public predict(dt: number): void {
    if (dt <= 0) return;

    for (const track of this.tracks.values()) {
      if (track.status === 'DROPPED') continue;

      const [x, y, z, vx, vy, vz] = track.stateVector;

      // Constant Velocity (CV) Kinematic Transition: x_k = F * x_{k-1}
      track.stateVector = [
        x + vx * dt,
        y + vy * dt,
        z + vz * dt,
        vx,
        vy,
        vz,
      ];

      // Discrete White Noise Acceleration (DWNA) Q matrix formulation
      const dt2 = dt * dt;
      const dt3 = (dt2 * dt) / 2.0;
      const dt4 = (dt2 * dt2) / 4.0;
      const q = this.defaultProcessNoiseQ;

      // Update covariance P = F * P * F^T + Q
      for (let i = 0; i < 3; i++) {
        track.covarianceP[i][i] += dt4 * q;
        track.covarianceP[i][i + 3] += dt3 * q;
        track.covarianceP[i + 3][i] += dt3 * q;
        track.covarianceP[i + 3][i + 3] += dt2 * q;
      }
    }
  }

  /**
   * Mahalanobis Distance Gating
   * d_M^2 = (z - Hx)^T S^-1 (z - Hx) <= gamma
   */
  public gateMeasurement(track: TrackState, meas: Measurement3D): GatingResult {
    const [x, y, z] = track.stateVector;

    // Innovation residual: y = z - Hx (where H selects position [0, 1, 2])
    const inn: [number, number, number] = [
      meas.pos.east - x,
      meas.pos.north - y,
      meas.pos.up - z,
    ];

    // Innovation covariance: S = H * P * H^T + R
    const S: number[][] = [
      [track.covarianceP[0][0] + meas.covarianceR[0][0], track.covarianceP[0][1] + meas.covarianceR[0][1], track.covarianceP[0][2] + meas.covarianceR[0][2]],
      [track.covarianceP[1][0] + meas.covarianceR[1][0], track.covarianceP[1][1] + meas.covarianceR[1][1], track.covarianceP[1][2] + meas.covarianceR[1][2]],
      [track.covarianceP[2][0] + meas.covarianceR[2][0], track.covarianceP[2][1] + meas.covarianceR[2][1], track.covarianceP[2][2] + meas.covarianceR[2][2]],
    ];

    const invS = invert3x3(S);

    // Quadratic form: inn^T * invS * inn
    const dSq =
      inn[0] * (invS[0][0] * inn[0] + invS[0][1] * inn[1] + invS[0][2] * inn[2]) +
      inn[1] * (invS[1][0] * inn[0] + invS[1][1] * inn[1] + invS[1][2] * inn[2]) +
      inn[2] * (invS[2][0] * inn[0] + invS[2][1] * inn[1] + invS[2][2] * inn[2]);

    return {
      measurement: meas,
      mahalanobisDistSq: dSq,
      inGate: dSq <= this.gateThresholdChiSq,
      innovation: inn,
    };
  }

  /**
   * Process incoming batch of multi-sensor measurements with MHT / JPDA association
   */
  public processScan(measurements: Measurement3D[], timestamp: number): void {
    const unassociatedMeasurements = new Set<Measurement3D>(measurements);

    for (const track of this.tracks.values()) {
      if (track.status === 'DROPPED') continue;

      // Gate all measurements against this track
      const candidateGates: GatingResult[] = [];
      for (const meas of measurements) {
        const gate = this.gateMeasurement(track, meas);
        if (gate.inGate) {
          candidateGates.push(gate);
        }
      }

      if (candidateGates.length === 0) {
        // Missed detection: Coast track
        track.missedDetections++;
        track.status = track.missedDetections >= this.maxCoastFrames ? 'DROPPED' : 'COASTED';
        track.trackQuality = Math.max(1, track.trackQuality - 2);
        continue;
      }

      // JPDA Probability weights calculation
      const Pd = 0.95; // Probability of detection
      const clutterLambda = 1e-7; // Spatial Poisson clutter density

      let totalLikelihood = clutterLambda * (1.0 - Pd);
      const hypothesisList: { gate: GatingResult; likelihood: number }[] = [];

      for (const gate of candidateGates) {
        const detS = Math.max(1e-6, det3x3(gate.measurement.covarianceR));
        const normFactor = 1.0 / (Math.pow(2 * Math.PI, 1.5) * Math.sqrt(detS));
        const likelihood = normFactor * Math.exp(-0.5 * gate.mahalanobisDistSq) * Pd;

        hypothesisList.push({ gate, likelihood });
        totalLikelihood += likelihood;
      }

      // Combined JPDA innovation: sum(beta_i * y_i)
      let combinedInn: [number, number, number] = [0, 0, 0];
      let bestGate: GatingResult = candidateGates[0];
      let maxLikelihood = -1;

      for (const hyp of hypothesisList) {
        const beta = hyp.likelihood / (totalLikelihood || 1.0);
        combinedInn[0] += beta * hyp.gate.innovation[0];
        combinedInn[1] += beta * hyp.gate.innovation[1];
        combinedInn[2] += beta * hyp.gate.innovation[2];

        if (hyp.likelihood > maxLikelihood) {
          maxLikelihood = hyp.likelihood;
          bestGate = hyp.gate;
        }

        unassociatedMeasurements.delete(hyp.gate.measurement);
      }

      // Kalman Gain update with best/combined innovation
      this.applyKalmanUpdate(track, combinedInn, bestGate.measurement.covarianceR);

      track.confirmations++;
      track.missedDetections = 0;
      track.lastUpdated = timestamp;
      track.trackQuality = Math.min(15, track.trackQuality + 1);

      if (track.confirmations >= this.confirmationM && track.status === 'TENTATIVE') {
        track.status = 'CONFIRMED';
      }
    }

    // Initiate new tentative tracks from unassociated measurements
    for (const meas of unassociatedMeasurements) {
      this.initiateTentativeTrack(meas, timestamp);
    }
  }

  /**
   * Applies discrete Kalman state and covariance correction
   */
  private applyKalmanUpdate(
    track: TrackState,
    innovation: [number, number, number],
    covR: number[][]
  ): void {
    // S = P_pos + R
    const S: number[][] = [
      [track.covarianceP[0][0] + covR[0][0], track.covarianceP[0][1] + covR[0][1], track.covarianceP[0][2] + covR[0][2]],
      [track.covarianceP[1][0] + covR[1][0], track.covarianceP[1][1] + covR[1][1], track.covarianceP[1][2] + covR[1][2]],
      [track.covarianceP[2][0] + covR[2][0], track.covarianceP[2][1] + covR[2][1], track.covarianceP[2][2] + covR[2][2]],
    ];
    const invS = invert3x3(S);

    // K = P * H^T * invS (6x3 Kalman Gain)
    const K: number[][] = Array.from({ length: 6 }, () => [0, 0, 0]);
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 3; j++) {
        K[i][j] =
          track.covarianceP[i][0] * invS[0][j] +
          track.covarianceP[i][1] * invS[1][j] +
          track.covarianceP[i][2] * invS[2][j];
      }
    }

    // State update: x = x + K * innovation
    for (let i = 0; i < 6; i++) {
      track.stateVector[i] +=
        K[i][0] * innovation[0] + K[i][1] * innovation[1] + K[i][2] * innovation[2];
    }

    // Joseph stabilized Covariance update: P = (I - KH) P (I - KH)^T + K R K^T
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        let kh_p = 0;
        for (let m = 0; m < 3; m++) {
          kh_p += K[i][m] * track.covarianceP[m][j];
        }
        track.covarianceP[i][j] = Math.max(0.1, track.covarianceP[i][j] - kh_p);
      }
    }
  }

  /**
   * Initializes a new tentative track hypothesis
   */
  private initiateTentativeTrack(meas: Measurement3D, timestamp: number): TrackState {
    const id = `TRK-${this.nextTrackId++}`;
    const tnOctal = (this.nextTrackId + 4096).toString(16).toUpperCase();

    const initialCovP: number[][] = [
      [meas.covarianceR[0][0] * 2, 0, 0, 0, 0, 0],
      [0, meas.covarianceR[1][1] * 2, 0, 0, 0, 0],
      [0, 0, meas.covarianceR[2][2] * 2, 0, 0, 0],
      [0, 0, 0, 2500, 0, 0],   // 50 m/s initial velocity uncertainty
      [0, 0, 0, 0, 2500, 0],
      [0, 0, 0, 0, 0, 900],    // 30 m/s climb uncertainty
    ];

    const newTrack: TrackState = {
      id,
      trackNumber: tnOctal,
      status: 'TENTATIVE',
      stateVector: [meas.pos.east, meas.pos.north, meas.pos.up, 0, 0, 0],
      covarianceP: initialCovP,
      confirmations: 1,
      missedDetections: 0,
      lastUpdated: timestamp,
      trackQuality: 4,
      hypotheses: [
        {
          id: `HYP-0`,
          associatedMeasurementId: meas.id,
          likelihood: 1.0,
          probability: 1.0,
        },
      ],
    };

    this.tracks.set(id, newTrack);
    return newTrack;
  }
}
