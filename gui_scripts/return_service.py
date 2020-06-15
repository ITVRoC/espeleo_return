#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client

from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Empty

def handle_return_service(req):
    return_param = rospy.get_param('/espeleo/return_active')
    #return_goal = rospy.get_param('/espeleo/return_finished')
    while return_param:
        return_param = rospy.get_param('/espeleo/return_active')
        return_goal = rospy.get_param('/espeleo/return_finished')
        if return_goal:
            rospy.set_param('/espeleo/return_active', False)
            rospy.set_param('/espeleo/return_finished', False)
            #return_param = rospy.get_param('/espeleo/return_active')
            #print(return_param)
        continue
    return TriggerResponse(success=True, message="made the return")

def return_service():
    rospy.init_node('return_service_node')
    s = rospy.Service('return_service', Trigger, handle_return_service)
    print("return service running")
    rospy.spin()

def emit(config):
    rospy.loginfo('emitting')

if __name__ == "__main__":
    # Handling the timeout when the dynamic reconfigure server is not available
    try:
        dyn_reconfig_client = dynamic_reconfigure.client.Client("espeleo", timeout=3, config_callback=emit)
    except rospy.ROSException as e:
        rospy.logwarn("dynamic reconfigure not found, continuing without it!")
        pass

    return_service()