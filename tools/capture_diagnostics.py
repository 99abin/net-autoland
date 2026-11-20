"""Preserve native SITL logs alongside one flight's ROS recording."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / '.simulation/sitl/logs'
EXPORT_TYPES = {'ATT', 'PIDP', 'PIDR', 'RCOU', 'RCIN', 'AETR', 'CTUN', 'NTUN',
                'TECS', 'MODE', 'MSG', 'PARM', 'IMU', 'XKF1', 'XKF4', 'SIM', 'RATE', 'FTN1'}


def log_inventory():
    return {str(p): [p.stat().st_size, p.stat().st_mtime_ns] for p in LOGS.glob('*.BIN')}


def preserve_native_logs(directory, before):
    target = Path(directory) / 'diagnostics/native'
    target.mkdir(parents=True, exist_ok=True)
    records = []
    for source, stat in log_inventory().items():
        if before.get(source) == stat:
            continue
        path = Path(source)
        destination = target / path.name
        # Read only the observed byte count so a running RTL logger cannot
        # make capture unbounded. A trailing partial record can be ignored.
        with path.open('rb') as incoming, destination.open('wb') as outgoing:
            remaining = stat[0]
            while remaining:
                block = incoming.read(min(remaining, 1024 * 1024))
                if not block:
                    break
                outgoing.write(block)
                remaining -= len(block)
        records.append(dict(source=source, file=str(destination.relative_to(directory)),
            bytes=destination.stat().st_size, sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
            snapshot=True))
    (target.parent / 'native_manifest.json').write_text(json.dumps(records, indent=2))
    return records


def export_native(path, destination):
    # Use the checked-out ArduPilot decoder, without installing Python packages.
    from simulation import settings
    mavlink = Path(settings()['ardupilot_binary']).parents[3] / 'modules/mavlink'
    sys.path.insert(0, str(mavlink))
    from pymavlink import DFReader
    reader = DFReader.DFReader_binary(str(path))
    streams = {}
    writers = {}
    counts = {}
    try:
        while True:
            message = reader.recv_msg()
            if message is None:
                break
            name = message.get_type()
            if name not in EXPORT_TYPES:
                continue
            record = message.to_dict()
            if name not in writers:
                stream = (destination / (name + '.csv')).open('w')
                streams[name] = stream
                writers[name] = csv.DictWriter(stream, fieldnames=list(record))
                writers[name].writeheader()
            writers[name].writerow(record)
            counts[name] = counts.get(name, 0) + 1
    finally:
        for stream in streams.values():
            stream.close()
    return counts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--native-log', type=Path, required=True)
    args = parser.parse_args()
    run = args.run.resolve()
    destination = run / 'diagnostics'
    native = destination / 'native'
    native.mkdir(parents=True, exist_ok=True)
    copied = native / args.native_log.name
    if args.native_log.resolve() != copied.resolve():
        shutil.copy2(args.native_log, copied)
    counts = export_native(copied, destination)
    manifest = dict(native_source=str(args.native_log.resolve()), native_log=copied.name,
        native_sha256=hashlib.sha256(copied.read_bytes()).hexdigest(), csv_records=counts,
        time_alignment='Native TimeUS is FCU boot microseconds. ROS telemetry includes sim_time_s and message stamps. Validate MODE transitions before joining.',
        provenance='Existing completed flight logs. Collection-time sources are identified separately.')
    source = destination / 'collection_time_sources'
    source.mkdir(exist_ok=True)
    for filename in ('fcu_interface.py', 'vision_controller.py', 'vision_guidance.py', 'terminal.py'):
        shutil.copy2(ROOT / 'src/autoland/autoland' / filename, source / filename)
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    (destination / 'README.md').write_text('''# Flight diagnostic data

The parent folder contains telemetry.jsonl with ROS preview and live commands,
pose, velocity, airspeed, vision validity/covariance, ownership, supervisor state,
mission progress and receipt/simulation timestamps. context.json contains the
recorded configuration snapshots. contacts.jsonl contains independent Gazebo
collision evidence. summary.json and actions.json record the test decisions.

The native folder preserves the original ArduPilot DataFlash log. CSV files
export attitude demands/measurements (ATT), rate targets/measurements and
PID/FF terms (PIDP), normalized elevator/mixing outputs (AETR), servo PWM
(RCOU), throttle/navigation/energy control (CTUN/NTUN/TECS), parameters (PARM),
IMU/EKF data, mode transitions and status messages where recorded.

Use native MODE transitions to align TimeUS with ROS sim_time_s. TimeUS uses
microseconds and MAVLink/ROS Euler pitch conventions differ. Keep the native
log for fields the CSV exporter did not select. The collection_time_sources
folder is a later source snapshot. The run-time context is authoritative for
its recorded file versions. An image stream was not recorded in this flight.
''')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
