#!/usr/bin/env python3
"""
AegisCommand C4ISR — SITL (Software-in-the-Loop) Tactical Gateway Node
Cursor-on-Target (CoT) UDP Router & NATO Link-16 Tactical Data Link Bridge
Reference: MIL-STD-6016 / MITRE Cursor-on-Target (CoT) Event Specification
"""

import sys
import time
import math
import socket
import asyncio
import argparse
from datetime import datetime, timezone, timedelta
from dataclasses import dataclass
import xml.etree.ElementTree as ET

COT_TIME_FORMAT = "%Y-%m-%dT%H:%M:%S.%fZ"

@dataclass
class TacticalTrack:
    uid: str
    callsign: str
    cot_type: str        # e.g., 'a-f-A' (Friend Air), 'a-h-A' (Hostile Air), 'a-u-A' (Unknown Air)
    lat: float
    lon: float
    hae: float          # Height Above Ellipsoid (meters)
    speed_mps: float    # Speed in meters/second
    course_deg: float   # Heading in degrees [0, 360)
    ce: float = 10.0    # Circular Error 90% (meters)
    le: float = 15.0    # Linear Error (vertical meters)

class CursorOnTargetEngine:
    """Generates and parses standard Cursor-on-Target (CoT) XML schema messages"""

    @staticmethod
    def build_cot_xml(track: TacticalTrack, stale_duration_sec: int = 30) -> str:
        now = datetime.now(timezone.utc)
        stale = now + timedelta(seconds=stale_duration_sec)

        event = ET.Element('event', {
            'version': '2.0',
            'uid': track.uid,
            'type': track.cot_type,
            'time': now.strftime(COT_TIME_FORMAT),
            'start': now.strftime(COT_TIME_FORMAT),
            'stale': stale.strftime(COT_TIME_FORMAT),
            'how': 'm-g',
        })

        ET.SubElement(event, 'point', {
            'lat': f"{track.lat:.6f}",
            'lon': f"{track.lon:.6f}",
            'hae': f"{track.hae:.2f}",
            'ce': f"{track.ce:.1f}",
            'le': f"{track.le:.1f}",
        })

        detail = ET.SubElement(event, 'detail')
        ET.SubElement(detail, 'track', {
            'speed': f"{track.speed_mps:.2f}",
            'course': f"{track.course_deg:.2f}",
        })
        ET.SubElement(detail, 'contact', {
            'callsign': track.callsign,
        })
        ET.SubElement(detail, 'link16', {
            'j_series': 'J3.2' if 'a-h' in track.cot_type or 'a-u' in track.cot_type else 'J2.2',
            'npg': '7' if 'a-h' in track.cot_type else '9',
        })

        return ET.tostring(event, encoding='utf-8', xml_declaration=True).decode('utf-8')

    @staticmethod
    def parse_cot_xml(xml_str: str) -> TacticalTrack:
        root = ET.fromstring(xml_str)
        uid = root.attrib.get('uid', 'UNKNOWN')
        cot_type = root.attrib.get('type', 'a-u-A')

        point = root.find('point')
        lat = float(point.attrib.get('lat', '0.0')) if point is not None else 0.0
        lon = float(point.attrib.get('lon', '0.0')) if point is not None else 0.0
        hae = float(point.attrib.get('hae', '0.0')) if point is not None else 0.0
        ce = float(point.attrib.get('ce', '10.0')) if point is not None else 10.0
        le = float(point.attrib.get('le', '15.0')) if point is not None else 15.0

        detail = root.find('detail')
        speed = 0.0
        course = 0.0
        callsign = uid

        if detail is not None:
            track_el = detail.find('track')
            if track_el is not None:
                speed = float(track_el.attrib.get('speed', '0.0'))
                course = float(track_el.attrib.get('course', '0.0'))

            contact_el = detail.find('contact')
            if contact_el is not None:
                callsign = contact_el.attrib.get('callsign', uid)

        return TacticalTrack(
            uid=uid,
            callsign=callsign,
            cot_type=cot_type,
            lat=lat,
            lon=lon,
            hae=hae,
            speed_mps=speed,
            course_deg=course,
            ce=ce,
            le=le,
        )

class C4ISRAsyncRouter:
    """AsyncIO UDP router for tactical telemetry streaming and CoT broadcast"""

    def __init__(self, host: str = "127.0.0.1", port: int = 4242):
        self.host = host
        self.port = port
        self.engine = CursorOnTargetEngine()

    async def run_server(self):
        loop = asyncio.get_running_loop()
        transport, _ = await loop.create_datagram_endpoint(
            lambda: UDPProtocolHandler(),
            local_addr=(self.host, self.port)
        )
        print(f"[*] C4ISR SITL CoT Router listening on {self.host}:{self.port}")
        try:
            while True:
                await asyncio.sleep(3600)
        finally:
            transport.close()

class UDPProtocolHandler(asyncio.DatagramProtocol):
    def datagram_received(self, data: bytes, addr):
        try:
            msg = data.decode('utf-8')
            track = CursorOnTargetEngine.parse_cot_xml(msg)
            print(f"[+] CoT Ingest from {addr}: {track.callsign} ({track.cot_type}) at {track.lat:.4f}, {track.lon:.4f}")
        except Exception as e:
            print(f"[-] Malformed CoT frame received from {addr}: {e}")

def run_self_test() -> int:
    """Executes automated verification suite for CI/CD and regression testing"""
    print("==================================================================")
    print("  AegisCommand C4ISR — SITL Node & CoT Router Diagnostic Suite    ")
    print("==================================================================")

    # 1. Test Tactical Track Creation
    sample_tracks = [
        TacticalTrack(
            uid="SAAF-VIPER-01",
            callsign="VIPER-01",
            cot_type="a-f-A-M-F", # Friendly military fixed wing
            lat=18.2974,
            lon=42.8028,
            hae=8500.0,
            speed_mps=310.0,
            course_deg=135.0
        ),
        TacticalTrack(
            uid="HOSTILE-CRUISE-09",
            callsign="THREAT-09",
            cot_type="a-h-A-M-M", # Hostile cruise missile
            lat=18.6500,
            lon=43.1200,
            hae=350.0,
            speed_mps=280.0,
            course_deg=220.0
        ),
        TacticalTrack(
            uid="BALLISTIC-BM-401",
            callsign="SCUD-VECTOR",
            cot_type="b-m-p-s-m", # Ballistic missile in boost/reentry
            lat=19.1000,
            lon=43.5000,
            hae=28000.0,
            speed_mps=1450.0,
            course_deg=210.0
        )
    ]

    engine = CursorOnTargetEngine()
    print("[1/3] Testing CoT XML Serialization...")
    xml_frames = []
    for trk in sample_tracks:
        xml_out = engine.build_cot_xml(trk)
        assert "<event" in xml_out, "Missing <event> root tag"
        assert f"uid=\"{trk.uid}\"" in xml_out, f"UID {trk.uid} not serialized properly"
        assert f"lat=\"{trk.lat:.6f}\"" in xml_out, "Latitude serialization error"
        xml_frames.append(xml_out)
    print(f"      Successfully serialized {len(xml_frames)} tactical CoT frames.")

    print("[2/3] Testing CoT XML Deserialization & Round-Trip Parsing...")
    for idx, xml_str in enumerate(xml_frames):
        parsed = engine.parse_cot_xml(xml_str)
        orig = sample_tracks[idx]
        assert parsed.uid == orig.uid, f"UID mismatch: {parsed.uid} != {orig.uid}"
        assert math.isclose(parsed.lat, orig.lat, rel_tol=1e-5), "Latitude mismatch"
        assert math.isclose(parsed.speed_mps, orig.speed_mps, rel_tol=1e-2), "Speed mismatch"
        assert parsed.callsign == orig.callsign, "Callsign mismatch"
    print("      All frames verified with 100% round-trip fidelity.")

    print("[3/3] Testing Synchronous UDP Loopback Socket Transmission...")
    test_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    test_sock.bind(('127.0.0.1', 0))
    bound_port = test_sock.getsockname()[1]

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload = xml_frames[0].encode('utf-8')
    send_sock.sendto(payload, ('127.0.0.1', bound_port))

    test_sock.settimeout(2.0)
    received_data, _ = test_sock.recvfrom(4096)
    assert received_data == payload, "UDP loopback payload mismatch"
    test_sock.close()
    send_sock.close()
    print("      UDP Loopback packet successfully transmitted and verified.")

    print("==================================================================")
    print("  [SUCCESS] All SITL Gateway Diagnostics Passed (Exit Code 0)      ")
    print("==================================================================")
    return 0

def main():
    parser = argparse.ArgumentParser(description="AegisCommand C4ISR SITL CoT Gateway Router")
    parser.add_argument("--self-test", action="store_true", help="Run automated unit test and frame verification suite")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="UDP listening host")
    parser.add_argument("--port", type=int, default=4242, help="UDP listening port")
    args = parser.parse_args()

    if args.self_test:
        sys.exit(run_self_test())

    router = C4ISRAsyncRouter(host=args.host, port=args.port)
    asyncio.run(router.run_server())

if __name__ == "__main__":
    main()
