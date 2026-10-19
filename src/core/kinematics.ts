/**
 * AegisCommand C4ISR — Kinematics & Geodetic Coordinate Engine
 * Reference: WGS 84 (World Geodetic System 1984, EPSG:4326 / EPSG:4978)
 * Standard: NATO STANAG 4586 / MIL-STD-2525D Navigation Coordinate Transforms
 */

export interface GeodeticCoord {
  lat: number; // Latitude in decimal degrees [-90, +90]
  lon: number; // Longitude in decimal degrees [-180, +180]
  alt: number; // Height above WGS-84 reference ellipsoid in meters
}

export interface ECEFCoord {
  x: number; // X in meters (passes through prime meridian at equator)
  y: number; // Y in meters (passes through 90 deg E meridian at equator)
  z: number; // Z in meters (passes through true north rotational pole)
}

export interface ENUCoord {
  east: number;  // Local East displacement in meters
  north: number; // Local North displacement in meters
  up: number;    // Local Up (zenith) displacement in meters
}

export interface AerCoord {
  azimuth: number;    // Azimuth in degrees [0, 360) measured clockwise from True North
  elevation: number;  // Elevation angle in degrees [-90, +90]
  slantRange: number; // Line-of-sight distance in meters
}

export interface VelocityVector {
  speed: number;     // Ground speed / true airspeed in m/s
  heading: number;   // Track heading in degrees [0, 360)
  climbRate: number; // Vertical velocity in m/s (positive up)
}

export interface CPASolution {
  timeToCPA: number;   // Seconds until Closest Point of Approach (negative if past)
  minDistance: number; // Distance in meters at CPA
  pos1AtCPA: ENUCoord; // Position of Platform 1 at CPA
  pos2AtCPA: ENUCoord; // Position of Platform 2 at CPA
}

/**
 * WGS 84 Ellipsoid Defining and Derived Geodetic Parameters
 */
export const WGS84 = Object.freeze({
  a: 6378137.0,                     // Semi-major axis (equatorial radius) in meters
  f: 1.0 / 298.257223563,           // Flattening of the ellipsoid
  b: 6356752.314245,                // Semi-minor axis (polar radius) in meters
  e2: 0.0066943799901413165,        // First eccentricity squared: e^2 = 2f - f^2
  ePrime2: 0.006739496742276434,    // Second eccentricity squared: e'^2 = (a^2 - b^2) / b^2
  omega: 7.2921151467e-5,           // Nominal Earth angular velocity in rad/s
  mu: 3.986004418e14,               // Earth gravitational constant (GM) in m^3 / s^2
});

const DEG2RAD = Math.PI / 180.0;
const RAD2DEG = 180.0 / Math.PI;

/**
 * Computes prime vertical radius of curvature N(phi)
 * N(phi) = a / sqrt(1 - e^2 * sin^2(phi))
 */
export function getPrimeVerticalRadius(latRad: number): number {
  const sinLat = Math.sin(latRad);
  return WGS84.a / Math.sqrt(1.0 - WGS84.e2 * sinLat * sinLat);
}

/**
 * Transforms WGS 84 Geodetic coordinates (Lat, Lon, Alt) to Cartesian ECEF (X, Y, Z)
 */
export function geodeticToECEF(geo: GeodeticCoord): ECEFCoord {
  const phi = geo.lat * DEG2RAD;
  const lambda = geo.lon * DEG2RAD;
  const h = geo.alt;

  const N = getPrimeVerticalRadius(phi);
  const cosPhi = Math.cos(phi);
  const sinPhi = Math.sin(phi);
  const cosLambda = Math.cos(lambda);
  const sinLambda = Math.sin(lambda);

  return {
    x: (N + h) * cosPhi * cosLambda,
    y: (N + h) * cosPhi * sinLambda,
    z: (N * (1.0 - WGS84.e2) + h) * sinPhi,
  };
}

/**
 * Transforms Cartesian ECEF coordinates (X, Y, Z) to WGS 84 Geodetic (Lat, Lon, Alt)
 * Employs Bowring's closed-form algorithm (accurate to sub-millimeter for terrestrial/airborne altitudes)
 */
export function ecefToGeodetic(ecef: ECEFCoord): GeodeticCoord {
  const { x, y, z } = ecef;
  const p = Math.hypot(x, y);

  if (p < 1e-6) {
    // Platform is located at or near celestial poles
    const lat = z >= 0 ? 90.0 : -90.0;
    const alt = Math.abs(z) - WGS84.b;
    return { lat, lon: 0.0, alt };
  }

  const theta = Math.atan2(z * WGS84.a, p * WGS84.b);
  const sinTheta = Math.sin(theta);
  const cosTheta = Math.cos(theta);

  const phi = Math.atan2(
    z + WGS84.ePrime2 * WGS84.b * sinTheta * sinTheta * sinTheta,
    p - WGS84.e2 * WGS84.a * cosTheta * cosTheta * cosTheta
  );

  const lambda = Math.atan2(y, x);
  const N = getPrimeVerticalRadius(phi);
  const h = p / Math.cos(phi) - N;

  return {
    lat: phi * RAD2DEG,
    lon: lambda * RAD2DEG,
    alt: h,
  };
}

/**
 * Transforms Cartesian ECEF position to Local Tangent Plane East-North-Up (ENU)
 * relative to a geodetic reference anchor point.
 */
export function ecefToENU(targetECEF: ECEFCoord, refGeo: GeodeticCoord): ENUCoord {
  const refECEF = geodeticToECEF(refGeo);
  const dx = targetECEF.x - refECEF.x;
  const dy = targetECEF.y - refECEF.y;
  const dz = targetECEF.z - refECEF.z;

  const phi = refGeo.lat * DEG2RAD;
  const lambda = refGeo.lon * DEG2RAD;

  const sinPhi = Math.sin(phi);
  const cosPhi = Math.cos(phi);
  const sinLambda = Math.sin(lambda);
  const cosLambda = Math.cos(lambda);

  return {
    east: -sinLambda * dx + cosLambda * dy,
    north: -sinPhi * cosLambda * dx - sinPhi * sinLambda * dy + cosPhi * dz,
    up: cosPhi * cosLambda * dx + cosPhi * sinLambda * dy + sinPhi * dz,
  };
}

/**
 * Transforms Local Tangent Plane East-North-Up (ENU) coordinates back to ECEF
 */
export function enuToECEF(enu: ENUCoord, refGeo: GeodeticCoord): ECEFCoord {
  const refECEF = geodeticToECEF(refGeo);
  const phi = refGeo.lat * DEG2RAD;
  const lambda = refGeo.lon * DEG2RAD;

  const sinPhi = Math.sin(phi);
  const cosPhi = Math.cos(phi);
  const sinLambda = Math.sin(lambda);
  const cosLambda = Math.cos(lambda);

  const dx = -sinLambda * enu.east - sinPhi * cosLambda * enu.north + cosPhi * cosLambda * enu.up;
  const dy = cosLambda * enu.east - sinPhi * sinLambda * enu.north + cosPhi * sinLambda * enu.up;
  const dz = cosPhi * enu.north + sinPhi * enu.up;

  return {
    x: refECEF.x + dx,
    y: refECEF.y + dy,
    z: refECEF.z + dz,
  };
}

/**
 * Direct transformation from Geodetic coordinate to Local Tangent Plane ENU
 */
export function geodeticToENU(targetGeo: GeodeticCoord, refGeo: GeodeticCoord): ENUCoord {
  const targetECEF = geodeticToECEF(targetGeo);
  return ecefToENU(targetECEF, refGeo);
}

/**
 * Direct transformation from Local Tangent Plane ENU to Geodetic
 */
export function enuToGeodetic(enu: ENUCoord, refGeo: GeodeticCoord): GeodeticCoord {
  const ecef = enuToECEF(enu, refGeo);
  return ecefToGeodetic(ecef);
}

/**
 * Converts Local Tangent Plane ENU coordinate to Radar Spherical Coordinates (Azimuth, Elevation, Range)
 */
export function enuToAER(enu: ENUCoord): AerCoord {
  const groundRange = Math.hypot(enu.east, enu.north);
  const slantRange = Math.hypot(groundRange, enu.up);

  if (slantRange < 1e-6) {
    return { azimuth: 0, elevation: 0, slantRange: 0 };
  }

  let azimuth = Math.atan2(enu.east, enu.north) * RAD2DEG;
  if (azimuth < 0) azimuth += 360.0;

  const elevation = Math.atan2(enu.up, groundRange) * RAD2DEG;

  return { azimuth, elevation, slantRange };
}

/**
 * Converts Radar Spherical Coordinates (Azimuth, Elevation, Range) to Local Tangent Plane ENU
 */
export function aerToENU(aer: AerCoord): ENUCoord {
  const azRad = aer.azimuth * DEG2RAD;
  const elRad = aer.elevation * DEG2RAD;

  const groundRange = aer.slantRange * Math.cos(elRad);
  return {
    east: groundRange * Math.sin(azRad),
    north: groundRange * Math.cos(azRad),
    up: aer.slantRange * Math.sin(elRad),
  };
}

/**
 * Calculates Great-Circle surface distance between two points using the Haversine formula
 */
export function haversineDistance(p1: GeodeticCoord, p2: GeodeticCoord): number {
  const dLat = (p2.lat - p1.lat) * DEG2RAD;
  const dLon = (p2.lon - p1.lon) * DEG2RAD;
  const lat1 = p1.lat * DEG2RAD;
  const lat2 = p2.lat * DEG2RAD;

  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.sin(dLon / 2) * Math.sin(dLon / 2) * Math.cos(lat1) * Math.cos(lat2);

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  const meanEarthRadius = 6371008.8; // Mean Earth radius in meters (IUGG)
  return meanEarthRadius * c;
}

/**
 * Calculates Initial Forward Azimuth (Bearing) from p1 to p2 in degrees [0, 360)
 */
export function calculateBearing(start: GeodeticCoord, target: GeodeticCoord): number {
  const phi1 = start.lat * DEG2RAD;
  const phi2 = target.lat * DEG2RAD;
  const deltaLambda = (target.lon - start.lon) * DEG2RAD;

  const y = Math.sin(deltaLambda) * Math.cos(phi2);
  const x =
    Math.cos(phi1) * Math.sin(phi2) -
    Math.sin(phi1) * Math.cos(phi2) * Math.cos(deltaLambda);

  let bearing = Math.atan2(y, x) * RAD2DEG;
  return (bearing + 360.0) % 360.0;
}

/**
 * Projects a destination point given starting position, distance in meters, and bearing in degrees
 */
export function projectDestination(
  start: GeodeticCoord,
  distanceMeters: number,
  bearingDeg: number
): GeodeticCoord {
  const R = 6371008.8;
  const delta = distanceMeters / R;
  const theta = bearingDeg * DEG2RAD;

  const phi1 = start.lat * DEG2RAD;
  const lambda1 = start.lon * DEG2RAD;

  const sinPhi2 =
    Math.sin(phi1) * Math.cos(delta) +
    Math.cos(phi1) * Math.sin(delta) * Math.cos(theta);
  const phi2 = Math.asin(sinPhi2);

  const y = Math.sin(theta) * Math.sin(delta) * Math.cos(phi1);
  const x = Math.cos(delta) - Math.sin(phi1) * sinPhi2;
  const lambda2 = lambda1 + Math.atan2(y, x);

  return {
    lat: phi2 * RAD2DEG,
    lon: ((lambda2 * RAD2DEG + 540) % 360) - 180,
    alt: start.alt,
  };
}

/**
 * Converts platform velocity (speed, heading, climbRate) into 3D ENU velocity vector in m/s
 */
export function velocityToENU(vel: VelocityVector): ENUCoord {
  const headingRad = vel.heading * DEG2RAD;
  return {
    east: vel.speed * Math.sin(headingRad),
    north: vel.speed * Math.cos(headingRad),
    up: vel.climbRate,
  };
}

/**
 * Calculates Closest Point of Approach (CPA) between two maneuvering or non-maneuvering tracks
 */
export function calculateCPA(
  pos1: ENUCoord,
  vel1: ENUCoord,
  pos2: ENUCoord,
  vel2: ENUCoord
): CPASolution {
  // Relative position vector: r = pos1 - pos2
  const rx = pos1.east - pos2.east;
  const ry = pos1.north - pos2.north;
  const rz = pos1.up - pos2.up;

  // Relative velocity vector: v = vel1 - vel2
  const vx = vel1.east - vel2.east;
  const vy = vel1.north - vel2.north;
  const vz = vel1.up - vel2.up;

  const vSquared = vx * vx + vy * vy + vz * vz;

  if (vSquared < 1e-4) {
    // Platforms are stationary or moving in parallel with identical velocities
    const currentDist = Math.hypot(rx, ry, rz);
    return {
      timeToCPA: 0,
      minDistance: currentDist,
      pos1AtCPA: { ...pos1 },
      pos2AtCPA: { ...pos2 },
    };
  }

  // Time to CPA: t_cpa = - (r . v) / (v . v)
  const rDotV = rx * vx + ry * vy + rz * vz;
  const timeToCPA = -rDotV / vSquared;

  const t = Math.max(0, timeToCPA);
  const p1AtCPA: ENUCoord = {
    east: pos1.east + vel1.east * t,
    north: pos1.north + vel1.north * t,
    up: pos1.up + vel1.up * t,
  };

  const p2AtCPA: ENUCoord = {
    east: pos2.east + vel2.east * t,
    north: pos2.north + vel2.north * t,
    up: pos2.up + vel2.up * t,
  };

  const minDistance = Math.hypot(
    p1AtCPA.east - p2AtCPA.east,
    p1AtCPA.north - p2AtCPA.north,
    p1AtCPA.up - p2AtCPA.up
  );

  return {
    timeToCPA,
    minDistance,
    pos1AtCPA: p1AtCPA,
    pos2AtCPA: p2AtCPA,
  };
}
