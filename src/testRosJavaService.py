#!/usr/bin/env python
import roslib;
#roslib.load_manifest('beginner_tutorials')
roslib.load_manifest('knowledge_ros_service')
import sys

import rospy
from knowledge_ros_service.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AskForActionSequence)
        
        resp1 = add_two_ints('get', 'milk0', None, None)
        #resp1 = add_two_ints(x, y)
        print 'Service call ...'
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__": 
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
