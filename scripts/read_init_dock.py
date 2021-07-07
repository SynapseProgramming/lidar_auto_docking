#!/usr/bin/env python3
...

import json


# /home/ro/dev_ws/src/lidar_auto_docking/initial_dock_pose

initial_dock_pose = {}

with open(
    "/home/ro/dev_ws/src/lidar_auto_docking/initial_dock_pose/init_dock.json"
) as outfile:
    initial_dock_pose = json.load(outfile)

print(initial_dock_pose["x"])
print(initial_dock_pose["y"])
print(initial_dock_pose["z"])
print(initial_dock_pose["w"])
