#!/usr/bin/env python3
"""Turn a recorded demo run into an edited MP4 and a GIF.

Input:
  record_dir   frames saved by the demo cameras (launch with record:=true),
               one sub-directory per camera: overview/, pickup/, drop/,
               and optionally detector/ (record_detector_view.py), shown
               as an inset while the robot looks for the cube
  events_log   mission output where every line is prefixed with the number
               of overview frames at that moment: "<frames>|<log line>"

The edit switches cameras by mission phase (overview while driving,
close-ups from docking until backing away), speeds driving up more than
manipulation, and adds a short label per phase.

Usage:
  make_demo_video.py <record_dir> <events_log> <out_dir>
"""

import os
import re
import shutil
import subprocess
import sys

FPS_OUT = 30            # output frame rate
CAM_HZ = 10             # camera frames per simulated second
HOLD_S = 1.5            # pause on the camera detection
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def frames(cam_dir):
    files = sorted(
        os.listdir(cam_dir),
        key=lambda f: int(re.search(r'-(\d+)\.jpg$', f).group(1))
    )
    return [os.path.join(cam_dir, f) for f in files]


def event_frames(events_log, overview):
    """Overview frame index of each mission milestone.

    Uses the wall-clock timestamp of each mission log line and the file
    times of the saved frames, so it is exact even if log lines arrive
    late through a pipe."""
    import bisect
    mtimes = [os.path.getmtime(f) for f in overview]
    marks = {}
    order = [
        ('start', r'MOBILE MANIPULATION MISSION START'),
        ('drive1', r'Navigating to x=1\.'),
        ('dock1', r'Docking to PICKUP'),
        ('seen', r'Cube seen by the camera'),
        ('grasp', r'Arm reached GRASP'),
        ('backup1', r'Back-up complete'),
        ('drive2', r'Navigating to x=-1\.'),
        ('dock2', r'Docking to DROP'),
        ('place', r'Arm reached PLACE'),
        ('backup2', r'Back-up complete'),
        ('end', r'MISSION COMPLETE'),
    ]
    lines = open(events_log).read().splitlines()
    pos = 0
    for key, pattern in order:
        for i in range(pos, len(lines)):
            _, _, text = lines[i].partition('|')
            if re.search(pattern, text):
                ts = re.search(r'\[(\d+\.\d+)\]', text)
                marks[key] = bisect.bisect_left(mtimes, float(ts.group(1)))
                pos = i + 1
                break
        else:
            raise SystemExit(f'milestone "{key}" not found in {events_log}')
    return marks


def inset_source(record_dir):
    """Detector debug images with their file times, or None."""
    d = os.path.join(record_dir, 'detector')
    if not os.path.isdir(d) or not os.listdir(d):
        return None
    files = frames(d)
    return [os.path.getmtime(f) for f in files], files


def with_inset(scene_file, detector, out_file):
    """Scene frame with the detector view (nearest in time) top left,
    zoomed in on the part of the camera image where the pedestal is."""
    import bisect
    import cv2
    mtimes, files = detector
    t = os.path.getmtime(scene_file)
    i = min(bisect.bisect_left(mtimes, t), len(files) - 1)
    scene = cv2.imread(scene_file)
    view = cv2.imread(files[i])
    vh, vw = view.shape[:2]
    # 1.6x zoom on the part of the image with the pedestal and the
    # detector's label (which is drawn to the right of the cube)
    view = view[vh // 16:vh // 16 + vh * 5 // 8, vw * 3 // 8:vw]
    w = scene.shape[1] * 2 // 5
    h = view.shape[0] * w // view.shape[1]
    view = cv2.resize(view, (w, h), interpolation=cv2.INTER_AREA)
    x0, y0, b = 16, 16, 3
    scene[y0 - b:y0 + h + b, x0 - b:x0 + w + b] = (255, 255, 255)
    scene[y0:y0 + h, x0:x0 + w] = view
    cv2.putText(scene, 'robot camera', (x0 + 8, y0 + h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
                cv2.LINE_AA)
    cv2.imwrite(out_file, scene, [cv2.IMWRITE_JPEG_QUALITY, 92])


def main(record_dir, events_log, out_dir):

    cams = {c: frames(os.path.join(record_dir, c))
            for c in ('overview', 'pickup', 'drop')}
    m = event_frames(events_log, cams['overview'])
    tail = m['end'] + 2 * CAM_HZ

    # (camera, first frame, last frame, keep every Nth frame, label)
    # every frame kept = 3x real time at 30 fps output
    segments = [
        ('overview', m['drive1'] - 5, m['dock1'], 4, 'Nav2: drive to pickup pedestal'),
        ('pickup', m['dock1'], m['seen'] + 2 * CAM_HZ, 1,
         'LiDAR check + camera finds the cube'),
        ('pickup', m['seen'] + 2 * CAM_HZ, m['grasp'], 2,
         'LiDAR docking + approach the detected cube'),
        ('pickup', m['grasp'], m['backup1'], 1, 'Grasp, lift, carry pose, back off'),
        ('overview', m['backup1'], m['dock2'], 4, 'Nav2: carry cube to drop pedestal'),
        ('drop', m['dock2'], m['place'], 2, 'LiDAR docking + lower cube'),
        ('drop', m['place'], m['backup2'], 1, 'Place, release, back off'),
        ('overview', m['backup2'], tail, 2, 'Mission complete'),
    ]

    work = os.path.join(out_dir, 'edit_frames')
    shutil.rmtree(work, ignore_errors=True)
    os.makedirs(work)

    detector = inset_source(record_dir)

    k = 0
    labels = []
    for idx, (cam, a, b, step, label) in enumerate(segments):
        seq = cams[cam]
        start_t = k / FPS_OUT
        for i in range(max(a, 0), min(b, len(seq)), step):
            # hold the moment the cube is found for HOLD_S seconds
            repeat = 1
            if idx == 1 and i == m['seen']:
                repeat = int(HOLD_S * FPS_OUT)
            for _ in range(repeat):
                out = os.path.join(work, f'{k:06d}.jpg')
                if idx == 1 and detector:
                    with_inset(seq[i], detector, out)
                else:
                    os.symlink(seq[i], out)
                k += 1
        labels.append((start_t, k / FPS_OUT, label))

    print(f'{k} output frames, {k / FPS_OUT:.1f} s')

    text_filters = []
    for t0, t1, label in labels:
        safe = label.replace(':', r'\:').replace(',', r'\,')
        text_filters.append(
            f"drawtext=fontfile={FONT}:text='{safe}':x=24:y=h-60:"
            f"fontsize=30:fontcolor=white:box=1:boxcolor=black@0.55:"
            f"boxborderw=10:enable='between(t,{t0:.2f},{t1:.2f})'"
        )

    mp4 = os.path.join(out_dir, 'mobile_manipulation_demo.mp4')
    subprocess.run([
        'ffmpeg', '-y', '-loglevel', 'error',
        '-framerate', str(FPS_OUT), '-i', os.path.join(work, '%06d.jpg'),
        '-vf', ','.join(text_filters),
        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '22',
        '-movflags', '+faststart', mp4,
    ], check=True)

    # GIF: smaller, faster, own palette for clean colors
    gif = os.path.join(out_dir, 'mobile_manipulation_demo.gif')
    subprocess.run([
        'ffmpeg', '-y', '-loglevel', 'error', '-i', mp4,
        '-vf', 'setpts=0.5*PTS,fps=12,scale=640:-1:flags=lanczos,'
               'split[a][b];[a]palettegen=max_colors=128[p];'
               '[b][p]paletteuse=dither=bayer:bayer_scale=4',
        gif,
    ], check=True)

    for f in (mp4, gif):
        print(f'{f}: {os.path.getsize(f) / 1e6:.1f} MB')


if __name__ == '__main__':
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    main(*sys.argv[1:])
