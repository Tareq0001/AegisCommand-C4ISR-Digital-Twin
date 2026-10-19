/**
 * AegisCommand C4ISR — Link-16 Tactical Data Link (TDL) Engine
 * Reference: NATO STANAG 5516 / MIL-STD-6016 / MIL-STD-2525D Symbology
 * Protocol: J-Series Fixed-Format Tactical Messages
 */

import { GeodeticCoord, VelocityVector } from '../core/kinematics';

export type StandardIdentity =
  | 'PENDING'
  | 'UNKNOWN'
  | 'ASSUMED_FRIEND'
  | 'FRIEND'
  | 'NEUTRAL'
  | 'SUSPECT'
  | 'HOSTILE';

export type TacticalDomain =
  | 'AIR'
  | 'SURFACE'
  | 'SUBSURFACE'
  | 'LAND'
  | 'SPACE'
  | 'CYBER_EW';

export type Link16JMessageType =
  | 'J2.2'  // Air PPLI (Precise Participant Location & Identification)
  | 'J3.2'  // Air Track
  | 'J3.3'  // Surface Maritime Track
  | 'J3.5'  // Land / Ground Track
  | 'J3.6'  // Space / Ballistic Missile Track
  | 'J9.0'  // Command / Weapon Coordination Engagement Order
  | 'J13.2'; // Platform Status & Inventory

export interface Link16TrackNumber {
  octalStr: string;   // 5-digit octal/alphanumeric representation, e.g. "0AA12" or "02145"
  numericId: number;  // 19-bit discrete integer ID [0 .. 524287]
}

export interface Link16Header {
  messageType: Link16JMessageType;
  npg: number;             // Network Participation Group (e.g., 7 for Surveillance, 9 for Air Control)
  sourceTrackNumber: Link16TrackNumber;
  timeSlot: number;        // TDMA slot index [0 .. 1535] within 12s epoch
  timestampMs: number;     // System epoch timestamp in milliseconds
  trackQuality: number;    // 0 (no data) to 15 (geodetically surveyed precision)
}

/**
 * J2.2 Air PPLI (Precise Participant Location and Identification)
 */
export interface Link16_J2_2_AirPPLI {
  header: Link16Header;
  identity: StandardIdentity;
  callsign: string;
  position: GeodeticCoord;
  kinematics: VelocityVector;
  fuelStateMinutes: number;
  weaponsInventory: {
    pac3Missiles?: number;
    aim120dMissiles?: number;
    aim9xMissiles?: number;
  };
  iffMode4Valid: boolean;
  iffMode5Level2: boolean;
}

/**
 * J3.2 Air Track (Surveillance & Threat Identification)
 */
export interface Link16_J3_2_AirTrack {
  header: Link16Header;
  trackNumber: Link16TrackNumber;
  identity: StandardIdentity;
  position: GeodeticCoord;
  kinematics: VelocityVector;
  platformType: 'FIGHTER' | 'BOMBER' | 'UAV' | 'CRUISE_MISSILE' | 'BALLISTIC' | 'CIVILIAN';
  sidc2525d: string;        // MIL-STD-2525D 15-character SIDC string
  airDefenseThreatLevel: number; // 0.0 (benign) to 1.0 (imminent lethal strike)
  jammingStrobeActive: boolean;
}

/**
 * J3.6 Space & Ballistic Missile Track (Early Warning & Intercept Cueing)
 */
export interface Link16_J3_6_SpaceTrack {
  header: Link16Header;
  trackNumber: Link16TrackNumber;
  identity: StandardIdentity;
  position: GeodeticCoord;
  kinematics: VelocityVector;
  apogeeAltitudeMeters: number;
  predictedImpactPoint: GeodeticCoord;
  boostPhaseTerminated: boolean;
  timeToImpactSeconds: number;
}

/**
 * J9.0 Command & Engagement Directive
 */
export interface Link16_J9_0_EngagementOrder {
  header: Link16Header;
  commandAction: 'ASSIGN' | 'ENGAGE' | 'CEASE_FIRE' | 'HOLD_FIRE' | 'COVER';
  targetTrackNumber: Link16TrackNumber;
  assignedBatteryId: string;
  interceptorType: 'PATRIOT_PAC3_MSE' | 'THAAD' | 'NASAMS_AMRAAM_ER' | 'DEW_HIGH_ENERGY_LASER';
  salvoCount: number;
  authorizationCode: string;
}

/**
 * Encodes track number into normalized 19-bit integer and 5-char alphanumeric notation
 */
export function formatTrackNumber(numericId: number): Link16TrackNumber {
  const boundedId = numericId & 0x7ffff; // 19 bits
  const hexPadded = boundedId.toString(16).toUpperCase().padStart(5, '0');
  return {
    numericId: boundedId,
    octalStr: hexPadded,
  };
}

/**
 * Generates MIL-STD-2525D Tactical Symbol Identification Code (SIDC)
 */
export function generateSIDC(
  identity: StandardIdentity,
  domain: TacticalDomain,
  platformType: string = 'FIGHTER'
): string {
  // Format: Version (10), Standard Identity (1-char), Symbol Set (2-char), Status (0), HQTFD (0), Amplifier (0)
  let identityChar = 'U'; // Unknown
  switch (identity) {
    case 'FRIEND':
    case 'ASSUMED_FRIEND':
      identityChar = 'F';
      break;
    case 'HOSTILE':
    case 'SUSPECT':
      identityChar = 'H';
      break;
    case 'NEUTRAL':
      identityChar = 'N';
      break;
    case 'PENDING':
    case 'UNKNOWN':
    default:
      identityChar = 'U';
      break;
  }

  let domainChar = 'A'; // Air
  switch (domain) {
    case 'AIR':
      domainChar = 'A';
      break;
    case 'SURFACE':
      domainChar = 'S';
      break;
    case 'SUBSURFACE':
      domainChar = 'U';
      break;
    case 'LAND':
      domainChar = 'G';
      break;
    case 'SPACE':
      domainChar = 'P';
      break;
    case 'CYBER_EW':
      domainChar = 'E';
      break;
  }

  // 15-char tactical SIDC representation
  return `S${identityChar}${domainChar}P${platformType.slice(0, 4).toUpperCase().padEnd(4, '-')}-`;
}

/**
 * Serializes a J3.2 Air Track into a 75-bit STANAG 5516 simulated binary frame (10 bytes)
 */
export function encodeLink16_J3_2_Frame(track: Link16_J3_2_AirTrack): Uint8Array {
  const buffer = new Uint8Array(16);
  const view = new DataView(buffer.buffer);

  // Byte 0-1: Message ID (J3.2 = 0x0302), NPG
  view.setUint8(0, 0x32);
  view.setUint8(1, track.header.npg & 0xff);

  // Byte 2-4: 19-bit Track Number & Track Quality (4 bits)
  const tnAndTq = ((track.trackNumber.numericId & 0x7ffff) << 4) | (track.header.trackQuality & 0x0f);
  view.setUint32(2, tnAndTq);

  // Byte 6-7: Quantized Latitude [-90 .. +90] into 16-bit signed integer
  const quantLat = Math.round((track.position.lat / 90.0) * 32767);
  view.setInt16(6, quantLat);

  // Byte 8-9: Quantized Longitude [-180 .. +180] into 16-bit signed integer
  const quantLon = Math.round((track.position.lon / 180.0) * 32767);
  view.setInt16(8, quantLon);

  // Byte 10-11: Quantized Altitude (0 .. 40,000m) in 2m resolution
  const quantAlt = Math.min(65535, Math.max(0, Math.round(track.position.alt / 2.0)));
  view.setUint16(10, quantAlt);

  // Byte 12: Quantized Speed (0 .. 2550 m/s in 10 m/s steps)
  const quantSpeed = Math.min(255, Math.round(track.kinematics.speed / 10.0));
  view.setUint8(12, quantSpeed);

  // Byte 13: Heading [0 .. 360 deg] into [0 .. 255]
  const quantHeading = Math.round((track.kinematics.heading / 360.0) * 255) & 0xff;
  view.setUint8(13, quantHeading);

  // Byte 14: Standard Identity enum & Flags
  let idCode = 0;
  if (track.identity === 'FRIEND') idCode = 1;
  else if (track.identity === 'HOSTILE') idCode = 2;
  else if (track.identity === 'NEUTRAL') idCode = 3;
  else if (track.identity === 'SUSPECT') idCode = 4;

  const flags = (idCode << 4) | (track.jammingStrobeActive ? 0x08 : 0x00);
  view.setUint8(14, flags);

  return buffer;
}

/**
 * Decodes simulated binary frame back to J3.2 Air Track
 */
export function decodeLink16_J3_2_Frame(buffer: Uint8Array): Partial<Link16_J3_2_AirTrack> {
  const view = new DataView(buffer.buffer, buffer.byteOffset, buffer.byteLength);

  const npg = view.getUint8(1);
  const tnAndTq = view.getUint32(2);
  const numericId = (tnAndTq >> 4) & 0x7ffff;
  const trackQuality = tnAndTq & 0x0f;

  const quantLat = view.getInt16(6);
  const lat = (quantLat / 32767.0) * 90.0;

  const quantLon = view.getInt16(8);
  const lon = (quantLon / 32767.0) * 180.0;

  const alt = view.getUint16(10) * 2.0;
  const speed = view.getUint8(12) * 10.0;
  const heading = (view.getUint8(13) / 255.0) * 360.0;

  const flags = view.getUint8(14);
  const idCode = (flags >> 4) & 0x0f;
  let identity: StandardIdentity = 'UNKNOWN';
  if (idCode === 1) identity = 'FRIEND';
  else if (idCode === 2) identity = 'HOSTILE';
  else if (idCode === 3) identity = 'NEUTRAL';
  else if (idCode === 4) identity = 'SUSPECT';

  const jammingStrobeActive = (flags & 0x08) !== 0;

  return {
    header: {
      messageType: 'J3.2',
      npg,
      sourceTrackNumber: formatTrackNumber(numericId),
      timeSlot: 0,
      timestampMs: Date.now(),
      trackQuality,
    },
    trackNumber: formatTrackNumber(numericId),
    identity,
    position: { lat, lon, alt },
    kinematics: { speed, heading, climbRate: 0 },
    sidc2525d: generateSIDC(identity, 'AIR'),
    jammingStrobeActive,
  };
}

/**
 * Link-16 Message Factory utility
 */
export class Link16MessageFactory {
  private static slotCounter = 0;

  public static createAirPPLI(
    id: number,
    callsign: string,
    position: GeodeticCoord,
    kinematics: VelocityVector,
    fuelMinutes: number = 90
  ): Link16_J2_2_AirPPLI {
    this.slotCounter = (this.slotCounter + 1) % 1536;
    return {
      header: {
        messageType: 'J2.2',
        npg: 9, // Air Control
        sourceTrackNumber: formatTrackNumber(id),
        timeSlot: this.slotCounter,
        timestampMs: Date.now(),
        trackQuality: 14,
      },
      identity: 'FRIEND',
      callsign,
      position,
      kinematics,
      fuelStateMinutes: fuelMinutes,
      weaponsInventory: {
        aim120dMissiles: 6,
        aim9xMissiles: 2,
      },
      iffMode4Valid: true,
      iffMode5Level2: true,
    };
  }

  public static createAirTrack(
    id: number,
    identity: StandardIdentity,
    position: GeodeticCoord,
    kinematics: VelocityVector,
    platformType: 'FIGHTER' | 'BOMBER' | 'UAV' | 'CRUISE_MISSILE' | 'BALLISTIC' | 'CIVILIAN' = 'FIGHTER'
  ): Link16_J3_2_AirTrack {
    this.slotCounter = (this.slotCounter + 1) % 1536;
    const threatScore =
      identity === 'HOSTILE'
        ? platformType === 'BALLISTIC' || platformType === 'CRUISE_MISSILE'
          ? 0.95
          : 0.8
        : 0.1;

    return {
      header: {
        messageType: 'J3.2',
        npg: 7, // Surveillance
        sourceTrackNumber: formatTrackNumber(id),
        timeSlot: this.slotCounter,
        timestampMs: Date.now(),
        trackQuality: 12,
      },
      trackNumber: formatTrackNumber(id),
      identity,
      position,
      kinematics,
      platformType,
      sidc2525d: generateSIDC(identity, 'AIR', platformType),
      airDefenseThreatLevel: threatScore,
      jammingStrobeActive: false,
    };
  }
}
