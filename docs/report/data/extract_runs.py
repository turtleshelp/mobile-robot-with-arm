#!/usr/bin/env python3
# Copyright 2026 Muhammad Ali Sadiq (github.com/turtleshelp)
# SPDX-License-Identifier: Apache-2.0
"""Extract per-run metrics from saved mission logs into runs.json.

Every run below was a full mission in a freshly started, headless
simulation. 'gazebo_ok' is the ground truth outcome: the cube upright on
the drop pedestal according to Gazebo after the run (not the mission's
own MISSION COMPLETE message). Usage: extract_runs.py <log_dir> <out.json>
"""
import json, os, re, sys

LOG = sys.argv[1]

# (id, log file, group, cube offset [m] or None, gazebo_ok, leftover procs, note)
RUNS = [
  ('T1', 'mission_loc1.log', 'tracking', 0.0, True, False, ''),
  ('T2', 'mission_loc2.log', 'tracking', 0.0, True, False, ''),
  ('T3', 'mission_loc3.log', 'tracking', 0.0, True, False, ''),
  ('T4', 'mission_loc4.log', 'tracking', 0.0, True, False, ''),
  ('T5', 'mission_loc5.log', 'tracking', 0.0, False, False,
   'physics blow-up while opening the gripper at the drop pedestal; robot and cube thrown, mission still reported COMPLETE'),
  ('F1', 'mission_fresh1.log', 'fresh clone', 0.0, True, False, 'fresh clone, before the camera'),
  ('C1', 'mission_rand1.log', 'camera', -0.007, False, True,
   'aborted safely: detections from a leftover test detector in the wrong frame'),
  ('C2', 'mission_rand2.log', 'camera', -0.029, False, True,
   'aborted safely: localization check saw a 0.14 m face (scan while still moving)'),
  ('C3', 'mission_rand3.log', 'camera', 0.022, True, True, ''),
  ('C4', 'mission_rand4.log', 'camera', -0.028, True, True, ''),
  ('C5', 'mission_rand5.log', 'camera', -0.019, True, True, ''),
  ('C6', 'mission_ver1.log', 'camera', -0.009, False, True,
   'aborted safely: 0 detections, TF thread blocked'),
  ('C7', 'mission_ver2_1.log', 'camera', 0.016, True, True, ''),
  ('C8', 'mission_ver3_1.log', 'camera', -0.044, True, True, ''),
  ('C9', 'mission_rec_events.log', 'camera', -0.030, True, False, 'demo recording'),
  ('C10', 'trialcheck/mission_1.log', 'camera', 0.007, True, False, 'run_trials.sh check'),
  ('C11', 'fresh2/trial/mission_1.log', 'camera', 0.025, True, False, 'fresh clone from GitHub'),
]
CUBE_X, CUBE_Y0 = 2.205, -0.30

num = r'([-+]?\d+\.\d+)'
def ts(line):
    m = re.search(r'\[(\d{10}\.\d+)\]', line)
    return float(m.group(1)) if m else None

def parse(path):
    lines = [l.split('|', 1)[-1] for l in open(path, errors='replace')]
    d = {'events': {}, 'arrival': [], 'docked': [], 'grasp_offset': None,
         'camera': None, 'loc_checks': []}
    first = {}
    def mark(key, line):
        if key not in first and ts(line):
            first[key] = ts(line)
    nav_count = dock_count = backup_count = docked_count = 0
    seen_dock_meas = set()
    for l in lines:
        t = ts(l)
        if 'MISSION START' in l: mark('start', l)
        if 'Navigating to x=' in l:
            nav_count += 1; mark(f'nav{nav_count}', l)
        if 'Navigation goal reached' in l:
            mark(f'navdone{nav_count}', l)
        if re.search(r'Docking to (PICKUP|DROP)', l):
            dock_count += 1; mark(f'dock{dock_count}', l)
        m = re.search(r'(PICKUP|DROP) PEDESTAL: dist=' + num + r' m, lateral offset=' + num, l)
        if m and m.group(1) not in seen_dock_meas:
            seen_dock_meas.add(m.group(1))
            d['arrival'].append({'pedestal': m.group(1), 'dist': float(m.group(2)),
                                 'lateral': float(m.group(3))})
        m = re.search(r'Docked at (PICKUP|DROP) PEDESTAL: dist=' + num +
                      r' m, yaw_err=' + num + r' deg, lateral=' + num, l)
        if m:
            docked_count += 1; mark(f'docked{docked_count}', l)
            d['docked'].append({'pedestal': m.group(1), 'dist': float(m.group(2)),
                                'yaw_deg': float(m.group(3)), 'lateral': float(m.group(4))})
        m = re.search(r'Localization check (OK|FAILED): (\w+) PEDESTAL: saw a face ' + num +
                      r' m wide at ' + num, l)
        if m:
            d['loc_checks'].append({'result': m.group(1), 'pedestal': m.group(2),
                                    'width': float(m.group(3)), 'dist': float(m.group(4))})
        m = re.search(r'Cube seen by the camera at \(' + num + ', ' + num + ', ' + num, l)
        if m:
            d['camera'] = [float(m.group(i)) for i in (1, 2, 3)]
        m = re.search(r'object offset from grasp point \(gripper frame\): \[' + num +
                      ', ' + num + ', ' + num, l)
        if m:
            d['grasp_offset'] = [float(m.group(i)) for i in (1, 2, 3)]
        if 'Back-up complete' in l:
            backup_count += 1; mark(f'backup{backup_count}', l)
        if re.search(r'MISSION (COMPLETE|ABORTED)', l):
            mark('end', l)
            d['result'] = re.search(r'MISSION (COMPLETE|ABORTED)', l).group(1)
    e = first
    def span(a, b):
        return round(e[b] - e[a], 1) if a in e and b in e else None
    d['phases_s'] = {
        'drive_to_pickup': span('nav1', 'navdone1'),
        'dock_pickup': span('dock1', 'docked1'),
        'pick': span('docked1', 'backup1'),
        'drive_to_drop': span('nav2', 'navdone2'),
        'dock_drop': span('dock2', 'docked2'),
        'place': span('docked2', 'backup2'),
        'total': span('nav1', 'end'),
    }
    return d

out = []
for rid, f, group, off, ok, orphan, note in RUNS:
    p = os.path.join(LOG, f)
    d = parse(p)
    r = {'id': rid, 'group': group, 'gazebo_ok': ok, 'leftover_processes': orphan,
         'note': note, 'mission_result': d.get('result'), **{k: d[k] for k in
         ('arrival', 'docked', 'grasp_offset', 'loc_checks', 'phases_s')}}
    if d['camera']:
        true = [CUBE_X, CUBE_Y0 + off]
        r['cube_true_xy'] = true
        r['camera_xyz'] = d['camera']
        r['camera_err_mm'] = round(1000 * ((d['camera'][0] - true[0]) ** 2 +
                                           (d['camera'][1] - true[1]) ** 2) ** 0.5, 1)
    out.append(r)

json.dump(out, open(sys.argv[2], 'w'), indent=1)
for r in out:
    print(r['id'], r['mission_result'], 'OK' if r['gazebo_ok'] else 'FAIL',
          'cam_err_mm=', r.get('camera_err_mm'), 'total_s=', r['phases_s']['total'],
          'docked=', [(x['dist'], x['yaw_deg'], x['lateral']) for x in r['docked']])
