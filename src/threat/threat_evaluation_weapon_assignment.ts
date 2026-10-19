/**
 * AegisCommand C4ISR — Threat Evaluation & Weapon Assignment (TEWA) Engine
 * Reference: Military Operations Research / Air & Missile Defense Optimization
 * Doctrine: Patriot PAC-3 MSE / THAAD / NASAMS Integrated Air and Missile Defense (IAMD)
 */

import { ENUCoord } from '../core/kinematics';

export type ThreatClass =
  | 'BALLISTIC_MISSILE'
  | 'HYPERSONIC_GLIDE_VEHICLE'
  | 'CRUISE_MISSILE'
  | 'ANTI_RADIATION_MISSILE'
  | 'STRIKE_FIGHTER'
  | 'ATTACK_UAV'
  | 'UNKNOWN_AIR_CONTACT';

export interface DefendedAsset {
  id: string;
  name: string;
  pos: ENUCoord;
  defenseValue: number; // Asset priority weight V_k [1 .. 100]
  lethalDefenseRadiusMeters: number;
}

export interface ThreatTarget {
  id: string;
  trackNumber: string;
  classification: ThreatClass;
  pos: ENUCoord;
  vel: ENUCoord; // Velocity vector in m/s
  radarCrossSectionSm: number; // RCS in m^2
  jammingActive: boolean;
}

export interface WeaponSystem {
  id: string;
  name: string;
  type: 'PATRIOT_PAC3_MSE' | 'THAAD' | 'NASAMS_AMRAAM_ER' | 'DEW_LASER' | 'F15SA_CAP';
  pos: ENUCoord;
  minRangeMeters: number;
  maxRangeMeters: number;
  minAltMeters: number;
  maxAltMeters: number;
  interceptorSpeedMs: number;
  singleShotKillProb: number; // P_k
  readyMissiles: number;
  maxSalvoPerTarget: number;
}

export interface EvaluatedThreat {
  target: ThreatTarget;
  threatScore: number;       // Normalized [0.0 .. 1.0]
  priorityRank: number;      // 1 (highest urgency) to N
  closingVelocityMs: number; // Toward nearest high-value asset
  timeToImpactSeconds: number;
  targetedAsset: DefendedAsset;
  recommendedDoctrine: 'SALVO_2' | 'SINGLE_SHOT' | 'DIRECTED_ENERGY' | 'MONITOR';
}

export interface EngagementPlan {
  threat: EvaluatedThreat;
  assignedWeapon: WeaponSystem;
  salvoCount: number;
  expectedKillProbability: number;
  interceptPointENU: ENUCoord;
  timeToInterceptSeconds: number;
}

export interface TEWASolution {
  timestamp: number;
  evaluatedThreats: EvaluatedThreat[];
  engagements: EngagementPlan[];
  unengagedThreatCount: number;
  theaterResidualRisk: number; // Sum of unintercepted expected damage
}

/**
 * TEWA Optimizer Class
 */
export class ThreatEvaluationWeaponAssignment {
  private defendedAssets: DefendedAsset[] = [];
  private weapons: WeaponSystem[] = [];

  constructor(assets: DefendedAsset[] = [], weapons: WeaponSystem[] = []) {
    this.defendedAssets = assets;
    this.weapons = weapons;
  }

  public setDefendedAssets(assets: DefendedAsset[]): void {
    this.defendedAssets = assets;
  }

  public setWeapons(weapons: WeaponSystem[]): void {
    this.weapons = weapons;
  }

  /**
   * Evaluates and ranks incoming threats based on proximity, closing speed, target lethality, and asset value
   */
  public evaluateThreats(targets: ThreatTarget[]): EvaluatedThreat[] {
    const evaluated: EvaluatedThreat[] = [];

    for (const target of targets) {
      if (this.defendedAssets.length === 0) break;

      // Find highest risk defended asset for this target
      let maxRiskAsset = this.defendedAssets[0];
      let minTimeToImpact = Infinity;
      let maxClosingVel = -Infinity;

      for (const asset of this.defendedAssets) {
        const dx = asset.pos.east - target.pos.east;
        const dy = asset.pos.north - target.pos.north;
        const dz = asset.pos.up - target.pos.up;
        const dist = Math.hypot(dx, dy, dz);

        // Project velocity along line-of-sight: V_closing = (target.vel . r) / dist
        const closingVel = (target.vel.east * dx + target.vel.north * dy + target.vel.up * dz) / Math.max(1.0, dist);

        let tti = dist / Math.max(10.0, closingVel);
        if (closingVel <= 0) {
          // Target moving away
          tti = 9999;
        }

        if (tti < minTimeToImpact) {
          minTimeToImpact = tti;
          maxClosingVel = closingVel;
          maxRiskAsset = asset;
        }
      }

      // Base lethality weight by platform classification
      let lethalityFactor = 0.5;
      switch (target.classification) {
        case 'BALLISTIC_MISSILE':
          lethalityFactor = 1.0;
          break;
        case 'HYPERSONIC_GLIDE_VEHICLE':
          lethalityFactor = 0.98;
          break;
        case 'CRUISE_MISSILE':
          lethalityFactor = 0.88;
          break;
        case 'ANTI_RADIATION_MISSILE':
          lethalityFactor = 0.85;
          break;
        case 'STRIKE_FIGHTER':
          lethalityFactor = 0.75;
          break;
        case 'ATTACK_UAV':
          lethalityFactor = 0.55;
          break;
        default:
          lethalityFactor = 0.4;
          break;
      }

      // Time urgency factor: exponential decay as TTI drops below 180 seconds
      const urgencyFactor = Math.min(1.0, Math.exp(-minTimeToImpact / 120.0));

      // Asset priority factor
      const assetFactor = maxRiskAsset.defenseValue / 100.0;

      // Closing velocity factor (up to 3000 m/s for hypersonics/ballistics)
      const speedFactor = Math.min(1.0, Math.max(0.1, maxClosingVel / 1500.0));

      // Composite Normalized Threat Score T in [0.0 .. 1.0]
      const threatScore = Math.min(
        1.0,
        0.35 * lethalityFactor +
        0.30 * urgencyFactor +
        0.20 * assetFactor +
        0.15 * speedFactor
      );

      // Engagement doctrine recommendation
      let doctrine: 'SALVO_2' | 'SINGLE_SHOT' | 'DIRECTED_ENERGY' | 'MONITOR' = 'SINGLE_SHOT';
      if (target.classification === 'BALLISTIC_MISSILE' || threatScore > 0.85) {
        doctrine = 'SALVO_2'; // High assurance Shoot-Look-Shoot or Ripple fire
      } else if (target.classification === 'ATTACK_UAV' && threatScore < 0.6) {
        doctrine = 'DIRECTED_ENERGY';
      } else if (threatScore < 0.25) {
        doctrine = 'MONITOR';
      }

      evaluated.push({
        target,
        threatScore,
        priorityRank: 0, // Assigned after sorting
        closingVelocityMs: maxClosingVel,
        timeToImpactSeconds: minTimeToImpact,
        targetedAsset: maxRiskAsset,
        recommendedDoctrine: doctrine,
      });
    }

    // Sort descending by threatScore (highest threat first)
    evaluated.sort((a, b) => b.threatScore - a.threatScore);
    evaluated.forEach((item, index) => {
      item.priorityRank = index + 1;
    });

    return evaluated;
  }

  /**
   * Weapon Assignment (WA): Solves multi-target allocation via Maximum Marginal Return (MMR) optimization
   */
  public solveWeaponAssignment(evaluatedThreats: EvaluatedThreat[]): TEWASolution {
    const engagements: EngagementPlan[] = [];
    const remainingInventory = new Map<string, number>();

    this.weapons.forEach((w) => remainingInventory.set(w.id, w.readyMissiles));

    let theaterResidualRisk = 0;
    let unengagedCount = 0;

    for (const threat of evaluatedThreats) {
      if (threat.recommendedDoctrine === 'MONITOR') {
        continue;
      }

      let bestWeapon: WeaponSystem | null = null;
      let bestPk = 0;
      let bestInterceptPoint: ENUCoord = { east: 0, north: 0, up: 0 };
      let bestTimeToIntercept = 0;

      for (const weapon of this.weapons) {
        const available = remainingInventory.get(weapon.id) || 0;
        if (available <= 0) continue;

        // Kinematic Engagement Envelope Check
        const dx = threat.target.pos.east - weapon.pos.east;
        const dy = threat.target.pos.north - weapon.pos.north;
        const dz = threat.target.pos.up - weapon.pos.up;
        const currentDist = Math.hypot(dx, dy, dz);
        const altitude = threat.target.pos.up;

        if (
          altitude < weapon.minAltMeters ||
          altitude > weapon.maxAltMeters ||
          currentDist > weapon.maxRangeMeters
        ) {
          // Out of kinematic interception envelope
          continue;
        }

        // Intercept geometry calculation
        const interceptorTime = currentDist / weapon.interceptorSpeedMs;
        if (interceptorTime > threat.timeToImpactSeconds) {
          // Interceptor cannot reach target prior to asset impact
          continue;
        }

        // Expected single-shot / multi-shot P_k
        const pk = weapon.singleShotKillProb;
        if (pk > bestPk) {
          bestPk = pk;
          bestWeapon = weapon;
          bestTimeToIntercept = interceptorTime;
          bestInterceptPoint = {
            east: threat.target.pos.east + threat.target.vel.east * interceptorTime,
            north: threat.target.pos.north + threat.target.vel.north * interceptorTime,
            up: threat.target.pos.up + threat.target.vel.up * interceptorTime,
          };
        }
      }

      if (bestWeapon && bestPk > 0) {
        const neededSalvo =
          threat.recommendedDoctrine === 'SALVO_2' ? Math.min(2, bestWeapon.maxSalvoPerTarget) : 1;
        const actualSalvo = Math.min(neededSalvo, remainingInventory.get(bestWeapon.id) || 1);

        // Cumulative kill probability: P_kill = 1 - (1 - P_k)^n
        const expectedKillProbability = 1.0 - Math.pow(1.0 - bestWeapon.singleShotKillProb, actualSalvo);

        remainingInventory.set(bestWeapon.id, (remainingInventory.get(bestWeapon.id) || 0) - actualSalvo);

        engagements.push({
          threat,
          assignedWeapon: bestWeapon,
          salvoCount: actualSalvo,
          expectedKillProbability,
          interceptPointENU: bestInterceptPoint,
          timeToInterceptSeconds: bestTimeToIntercept,
        });

        // Unintercepted residual risk
        const residualProb = 1.0 - expectedKillProbability;
        theaterResidualRisk += threat.targetedAsset.defenseValue * residualProb * threat.threatScore;
      } else {
        unengagedCount++;
        theaterResidualRisk += threat.targetedAsset.defenseValue * threat.threatScore;
      }
    }

    return {
      timestamp: Date.now(),
      evaluatedThreats,
      engagements,
      unengagedThreatCount: unengagedCount,
      theaterResidualRisk: Math.round(theaterResidualRisk * 10) / 10,
    };
  }
}
