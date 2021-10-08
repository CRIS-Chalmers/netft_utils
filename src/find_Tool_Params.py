#!/usr/bin/env python
import rospy
from netft_utils.srv import FindToolParams

def handle_find_Tool_Params(req):
    if(req.toDrive):
        print("Returning now calib sensor")
        return True
    return False

def find_Tool_Params_process():
    rospy.init_node('find_Tool_Params_process')
    s = rospy.Service('netft\find_Tool_Params', FindToolParams, handle_find_Tool_Params)
    rospy.spin()

if __name__ == "__main__":
    find_Tool_Params_process()