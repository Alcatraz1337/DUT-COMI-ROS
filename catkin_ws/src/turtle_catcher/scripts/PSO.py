#!/usr/bin/env python

import rospy
import sys
from psomanager import PSOManager

def main(argv):
    manager = PSOManager(int(argv[1]), int(argv[2]))
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