#!/usr/bin/env python3
"""Read-only final process and CAN state evidence."""
import json
from pathlib import Path
import subprocess
import time
names = {'perception_node','vlm_confirm_node','mission_node','command_guard_node',
         'lidar_filter_node','realsense2_camera_node','lslidar_driver_node',
         'bunker_base_node','planner_server','controller_server','bt_navigator','velocity_smoother'}
processes=[]
for p in Path('/proc').glob('[0-9]*'):
    try:
        args=(p/'cmdline').read_bytes().decode(errors='replace').split('\0')
        # Match individual program arguments, never a shell script's text.
        if any(Path(arg).name in names for arg in args[:3]):
            processes.append({'pid':int(p.name),'argv':args})
    except (OSError,ValueError):
        pass
report={'time_unix':time.time(),'can':json.loads(subprocess.check_output(['ip','-j','link','show','type','can'],text=True)),
        'remaining_task_processes':processes}
report['passed']=not processes and not any('UP' in c['flags'] for c in report['can'])
path=Path.home()/'robot_main_task_backup_20261109/final_state.json'
path.write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
