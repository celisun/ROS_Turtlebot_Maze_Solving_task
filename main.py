#!/usr/bin/env python
# TurtleBot Maze Solving

import rospy

from scan_twist_control_loop import ScanTwistControlLoop
from _pd_control import PDControlLoop



# 101 Initialize ros node
rospy.init_node('ros_maze_bot')

# Customize settings
kwargs = {
    'distance_to_wall': 0.2,
    'max_speed': 0.2,

}

scan_monitor = ScanTwistControlLoop(scan_topic="/scan", pub_topic="cmd_vel",
                                    policy='LHR', helper_controller= PDControlLoop, **kwargs)

#run
if __name__ == "__main__":

    scan_monitor.start()


