#!/usr/bin/env python3
"""Read a ROS2 bag and compute landing accuracy metrics."""
import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ROS2 imports needed for CDR deserialization
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

BAG_PATH    = Path('~/apriltag_mission_bag').expanduser()
TAG_X_TRUE  = 10.0   # known world X of tag centre (metres)
TAG_Y_TRUE  = 25.0   # known world Y of tag centre (metres)

def main():
    # 1. Connect to the SQLite database in the bag directory
    # ROS2 bags usually have a .db3 file inside the folder
    db_files = list(BAG_PATH.glob('*.db3'))
    if not db_files:
        print(f"Error: No .db3 file found in {BAG_PATH}")
        return
    db_file = db_files[0]
    
    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()

    # Query topics to get their IDs
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: {'name': row[1], 'type': row[2]} for row in cursor.fetchall()}

    # 2. Query the messages
    cursor.execute("""
        SELECT topic_id, timestamp, data 
        FROM messages 
        ORDER BY timestamp
    """)

    times_sec = []
    altitudes = []
    errors_2d = []
    
    start_time_ns = None
    final_x = None
    final_y = None
    final_z = None
    
    is_armed = False
    disarm_time_sec = None

    # 3. Deserialise each CDR-encoded message
    for topic_id, timestamp_ns, data in cursor.fetchall():
        topic_name = topics[topic_id]['name']
        
        if start_time_ns is None:
            start_time_ns = timestamp_ns
            
        t_sec = (timestamp_ns - start_time_ns) * 1e-9

        if topic_name == '/mavros/local_position/pose':
            msg = deserialize_message(data, PoseStamped)
            
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            # Compute 2D error from known tag location
            err = np.sqrt((x - TAG_X_TRUE)**2 + (y - TAG_Y_TRUE)**2)
            
            times_sec.append(t_sec)
            altitudes.append(z)
            errors_2d.append(err)
            
            final_x, final_y, final_z = x, y, z
            
            # 4. Find the final landed pose (stop processing if disarmed and on ground)
            if not is_armed and final_z is not None and final_z < 0.2 and len(times_sec) > 100:
                # We assume this is the end of the flight
                break
                
        elif topic_name == '/mavros/state':
            msg = deserialize_message(data, State)
            was_armed = is_armed
            is_armed = msg.armed
            if was_armed and not is_armed and final_z is not None and final_z < 1.0:
                disarm_time_sec = t_sec

    conn.close()

    if not times_sec:
        print("No pose data found in the bag!")
        return

    # --- Compute Metrics ---
    final_error_2d = np.sqrt((final_x - TAG_X_TRUE)**2 + (final_y - TAG_Y_TRUE)**2)
    total_time = times_sec[-1]
    
    # Estimate Search Time: 
    # The drone searches until it sees the tag and begins to approach/descend. 
    # We can approximate this as the time spent above ~4.5m before altitude significantly drops.
    alt_array = np.array(altitudes)
    search_end_idx = np.argmax((alt_array > 4.5) & (np.gradient(alt_array) < -0.05)) 
    search_time = times_sec[search_end_idx] if search_end_idx > 0 else total_time / 2.0

    # --- Print Summary ---
    print("=== Landing Performance Metrics ===")
    print(f"Final landing error: {final_error_2d:.3f} m")
    print(f"Final landing alt:   {final_z:.3f} m")
    print(f"Search time (est):   {search_time:.1f} s")
    print(f"Total flight time:   {total_time:.1f} s")
    print("===================================")

    # --- Plot altitude and lateral error vs time ---
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
    
    # Panel 1: Altitude vs Time
    ax1.plot(times_sec, altitudes, label='Altitude (Z)', color='blue')
    ax1.axhline(3.0, color='gray', linestyle='--', alpha=0.5, label='Coarse Alt (3m)')
    ax1.axhline(0.8, color='gray', linestyle=':', alpha=0.5, label='Fine Alt (0.8m)')
    ax1.set_ylabel('Altitude (m)')
    ax1.set_title('Precision Landing Profile')
    ax1.grid(True)
    ax1.legend()

    # Panel 2: Lateral Error vs Time
    ax2.plot(times_sec, errors_2d, label='2D Lateral Error', color='red')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Lateral Error (m)')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()