#!/usr/bin/env python
import roslib;
#roslib.load_manifest('beginner_tutorials')
roslib.load_manifest('knowledge_ros_service')
import sys

import rospy
from knowledge_ros_service.srv import *

def add_two_ints_client():
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AskForActionSequence)
        
        resp1 = add_two_ints('get', 'milk0', None, None)
        #resp1 = add_two_ints(x, y)
        print 'Service call ...'
        return resp1.actionSequence
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def querySparQL():
    rospy.wait_for_service('query_sparql')
    try:
        print 'ssss'
        spql = rospy.ServiceProxy('query_sparql', QuerySparQL)
        queryString = "PREFIX house: <http://www.semanticweb.org/ontologies/house.owl#> PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> \n SELECT ?room \n WHERE { ?room rdf:type house:Table . }"
        resp1 = spql(queryString)
        #resp1 = add_two_ints(x, y)
        print 'Service call ...'
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    #print len(add_two_ints_client())
    print querySparQL()
