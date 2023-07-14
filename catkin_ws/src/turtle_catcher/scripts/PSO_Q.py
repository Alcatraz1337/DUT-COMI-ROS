#!/usr/bin/env python

import rospy
import sys
from PSO_Q_Manager import PSO_Q_Manager

def main(argv):
    manager = PSO_Q_Manager(int(argv[1]), int(argv[2]), int(argv[3]))
    manager.initialize()

    is_finished = False
    while not rospy.is_shutdown() and not is_finished:
        try:
            manager.run()
            rospy.loginfo("Finished")
            # manager.exit()
            is_finished = True
            break
        except rospy.ROSInterruptException:
            manager.exit()
            rospy.loginfo("node terminated.")
    print("Exiting...")

if __name__ == '__main__':
    main(sys.argv)