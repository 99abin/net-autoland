"""Join opt-in publisher, receiver and ownership timing records from one flight."""
import argparse
from collections import defaultdict
import json
from pathlib import Path


def records(path):
    with path.open() as stream:
        for line in stream:
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                # An active recorder may have a final incomplete write.
                continue


def analyse(run):
    paths = sorted((Path(run) / 'timing').glob('*.jsonl'))
    sessions = []
    decisions = []
    for path in paths:
        for r in records(path):
            if r['event'] == 'session':
                sessions.append(r)
            if r['event'] in ('ownership_preview_rejected', 'ownership_preview_accepted'):
                decisions.append(r)
    identities = {(s['host'], s['boot_id']) for s in sessions}
    comparable = bool(sessions) and len(identities) == 1 and all(s['boot_id'] for s in sessions)
    anchors = [r['monotonic_ns'] for r in decisions]
    commands = defaultdict(list)
    windows = defaultdict(list)
    stats = {}
    for path in paths:
        previous_heartbeat = None
        maximum = 0
        callback_max = {}
        dropped = 0
        for r in records(path):
            event = r['event']
            dropped = max(dropped, r.get('dropped', 0))
            if event == 'heartbeat':
                if previous_heartbeat is not None:
                    maximum = max(maximum, r['monotonic_ns'] - previous_heartbeat)
                previous_heartbeat = r['monotonic_ns']
            if event == 'callback_exit':
                name = r['callback']
                if r['duration_ns'] > callback_max.get(name, {}).get('duration_ns', -1):
                    callback_max[name] = r
            if event in ('command_publish_begin', 'command_publish_end', 'command_received'):
                commands[r['command_key']].append(r)
            if any(abs(r['monotonic_ns'] - anchor) <= 1_000_000_000 for anchor in anchors):
                windows[path.name].append(r)
        stats[path.name] = dict(maximum_heartbeat_gap_s=maximum / 1e9,
                               maximum_callbacks=callback_max, dropped_records=dropped)
    joined = []
    for decision in decisions:
        events = commands.get(decision.get('command_key'), [])
        before = [r for r in events if r['monotonic_ns'] <= decision['monotonic_ns']]
        begins = [r for r in before if r['event'] == 'command_publish_begin']
        receives = [r for r in before if r['event'] == 'command_received']
        begin = max(begins, key=lambda r:r['monotonic_ns'], default=None)
        receive = max(receives, key=lambda r:r['monotonic_ns'], default=None)
        ends = [r for r in before if r['event'] == 'command_publish_end'
                and begin and r['monotonic_ns'] >= begin['monotonic_ns']]
        end = min(ends, key=lambda r:r['monotonic_ns'], default=None)
        entry = dict(decision=decision, publisher_begin=begin, publisher_end=end,
                     receiver=receive, repeated_identical_publications=len(begins) > 1)
        if comparable and begin and receive:
            entry['publish_begin_to_receive_s'] = (receive['monotonic_ns'] - begin['monotonic_ns']) / 1e9
        if receive:
            source = receive.get('dds_source_system_ns')
            received = receive.get('dds_received_system_ns')
            if source and received:
                entry['rmw_source_to_received_s'] = (received - source) / 1e9
                entry['rmw_received_to_callback_s'] = (receive['system_ns'] - received) / 1e9
        joined.append(entry)
    return dict(sessions=sessions, shared_monotonic_clock=comparable,
                decisions=joined, process_statistics=stats, decision_windows=windows,
                limitations=[
                    'Callback traces measure execution, not when middleware first marked a callback ready.',
                    'RMW timestamps can be absent or implementation-specific; system clock changes affect them.',
                    'Repeated identical command keys are flagged; no unique wire sequence was added.',
                    'Heartbeats share the process/GIL and disk writer; gaps alone do not prove CPU starvation.',
                    'Check dropped records and compare heartbeat, callback, publication and receive evidence together.',
                ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    result = analyse(args.run)
    target = args.run / 'command_timing_analysis.json'
    target.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'report':str(target), 'sessions':len(result['sessions']),
                      'ownership_decisions':len(result['decisions']),
                      'shared_monotonic_clock':result['shared_monotonic_clock']}, indent=2))


if __name__ == '__main__':
    main()
