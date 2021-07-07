#!/usr/bin/env python3
...

import json


# /home/ro/dev_ws/src/lidar_auto_docking/initial_dock_pose

initial_dock_pose = {}
initial_dock_pose["x"] = 3.14159
initial_dock_pose["y"] = 6.59
initial_dock_pose["z"] = 7.987
initial_dock_pose["w"] = 9.0125

with open(
    "/home/ro/dev_ws/src/lidar_auto_docking/initial_dock_pose/init_dock.json", "w"
) as outfile:
    json.dump(initial_dock_pose, outfile)
