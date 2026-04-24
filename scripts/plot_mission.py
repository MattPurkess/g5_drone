##!/usr/bin/env python3
"""
Plot planned vs actual trajectory from three ROS 2 bags (MCAP format)
recorded at different acceptance radii (0.5, 1.0, 1.5 m).

Uses rosbag2_py (ships with ROS 2 Jazzy) — no extra pip packages needed.

Usage:
    cd ~/g5_drone
    python3 scripts/plot_mission.py
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ── Configuration ────────────────────────────────────────────────────────

BAGS = {
    '0.5 m': str(Path('~/g5_drone/waypoint_mission_124630').expanduser()),
    '1.0 m': str(Path('~/g5_drone/waypoint_mission_125621').expanduser()),
    '1.5 m': str(Path('~/g5_drone/waypoint_mission_130225').expanduser()),
}

POSE_TOPIC = '/mavros/local_position/pose'

# Must match the MISSION you flew
PLANNED = [
    (0.0,   0.0, 10.0),   # Home (hover)
    (10.0,  0.0, 10.0),   # WP1 - East
    (10.0, 10.0, 10.0),   # WP2 - North-East
    (0.0,  10.0, 10.0),   # WP3 - North
    (0.0,   0.0, 10.0),   # Return to origin
]

WP_LABELS = ['Home', 'WP1', 'WP2', 'WP3', 'Return']

# ── MCAP bag reader using rosbag2_py ─────────────────────────────────────

def read_bag(bag_path: str):
    """Read PoseStamped messages from a ROS 2 MCAP bag."""
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='mcap',
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)

    # Filter to only the pose topic
    filt = rosbag2_py.StorageFilter(topics=[POSE_TOPIC])
    reader.set_filter(filt)

    msg_type = get_message('geometry_msgs/msg/PoseStamped')

    xs, ys, zs, ts_list = [], [], [], []
    t0 = None

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg = deserialize_message(data, msg_type)

        if t0 is None:
            t0 = timestamp

        xs.append(msg.pose.position.x)
        ys.append(msg.pose.position.y)
        zs.append(msg.pose.position.z)
        ts_list.append((timestamp - t0) / 1e9)

    if not xs:
        raise ValueError(f'{POSE_TOPIC} empty in {bag_path}')

    return np.array(xs), np.array(ys), np.array(zs), np.array(ts_list)


def compute_deviations(xs, ys, zs):
    """Compute closest-approach distance to each planned waypoint."""
    devs = []
    for x, y, z in PLANNED:
        d = np.sqrt((xs - x)**2 + (ys - y)**2 + (zs - z)**2)
        devs.append(d.min())
    return devs

# ── Main ─────────────────────────────────────────────────────────────────

data = {}
for label, bag_path in BAGS.items():
    try:
        data[label] = read_bag(bag_path)
        print(f'Loaded {label}: {len(data[label][0])} poses from {Path(bag_path).name}')
    except Exception as e:
        print(f'ERROR loading {label}: {e}')

if not data:
    print('No bags loaded. Check paths above.')
    exit(1)

px = [p[0] for p in PLANNED]
py = [p[1] for p in PLANNED]

run_colours = {
    '0.5 m': '#2563EB',
    '1.0 m': '#16A34A',
    '1.5 m': '#EA580C',
}

# ── Figure 1: Three-panel top-down comparison ────────────────────────────

fig, axes = plt.subplots(1, len(data), figsize=(5 * len(data), 5), squeeze=False)
axes = axes[0]

for ax, (label, (xs, ys, zs, ts)) in zip(axes, data.items()):
    colour = run_colours.get(label, 'blue')

    ax.plot(px, py, 'r--o', lw=1.5, markersize=7, zorder=5,
            label='Planned', markerfacecolor='white', markeredgewidth=1.5)
    for i, (x, y) in enumerate(zip(px, py)):
        ax.annotate(WP_LABELS[i], (x, y),
                     textcoords='offset points', xytext=(7, 7),
                     fontsize=8, color='#666666')

    ax.plot(xs, ys, '-', color=colour, lw=1.3, alpha=0.85, label='Actual')
    ax.plot(xs[0], ys[0], 's', color=colour, markersize=6, label='Start')
    ax.plot(xs[-1], ys[-1], '^', color=colour, markersize=6, label='End')

    devs = compute_deviations(xs, ys, zs)
    mean_dev = np.mean(devs)
    max_dev = np.max(devs)

    ax.set_title(f'Accept radius = {label}\n'
                 f'Mean dev: {mean_dev:.2f} m | Max: {max_dev:.2f} m',
                 fontsize=10, fontweight='bold')
    ax.set_xlabel('X \u2014 East (m)')
    ax.set_ylabel('Y \u2014 North (m)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7, loc='lower right')
    ax.set_xlim(-3, 14)
    ax.set_ylim(-3, 14)

plt.suptitle('Autonomous Waypoint Navigation \u2014 Acceptance Radius Comparison',
             fontsize=12, fontweight='bold', y=1.02)
plt.tight_layout()
plt.savefig('waypoint_trajectory.png', dpi=180, bbox_inches='tight')
print('\nSaved: waypoint_trajectory.png')

# ── Figure 2: Altitude profiles ──────────────────────────────────────────

fig2, axes2 = plt.subplots(len(data), 1, figsize=(10, 3 * len(data)), sharex=False)
if len(data) == 1:
    axes2 = [axes2]

for ax, (label, (xs, ys, zs, ts)) in zip(axes2, data.items()):
    colour = run_colours.get(label, 'blue')
    ax.plot(ts, zs, '-', color=colour, lw=1.2)
    ax.axhline(5, color='green', ls='--', lw=0.8, alpha=0.6, label='Cruise alt (5 m)')
    ax.axhline(10, color='red', ls='--', lw=0.8, alpha=0.6, label='Waypoint alt (10 m)')
    ax.set_ylabel('Altitude (m)')
    ax.set_title(f'Accept radius = {label}', fontsize=10, fontweight='bold')
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7)

axes2[-1].set_xlabel('Time (s)')
plt.suptitle('Altitude Profiles', fontsize=12, fontweight='bold', y=1.01)
plt.tight_layout()
plt.savefig('waypoint_altitude.png', dpi=180, bbox_inches='tight')
print('Saved: waypoint_altitude.png')

# ── Deviation table ──────────────────────────────────────────────────────

print('\n' + '='*65)
print('Per-waypoint closest-approach deviation (metres)')
print('='*65)
header = f'{"Waypoint":<12}'
for label in data:
    header += f'{label:>10}'
print(header)
print('-'*65)

for i, wp_name in enumerate(WP_LABELS):
    row = f'{wp_name:<12}'
    for label, (xs, ys, zs, ts) in data.items():
        devs = compute_deviations(xs, ys, zs)
        row += f'{devs[i]:>10.2f}'
    print(row)

print('-'*65)
row = f'{"Mean":<12}'
for label, (xs, ys, zs, ts) in data.items():
    devs = compute_deviations(xs, ys, zs)
    row += f'{np.mean(devs):>10.2f}'
print(row)
print('='*65)