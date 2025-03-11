#!/usr/bin/env python3.8

import rospy
from robotiq_epick_control.msg import RobotiqEPick_robot_output as outputMsg
from std_srvs.srv import Trigger, TriggerResponse

# 参数含义见 ../robotiq_epick_control
def genCommand(char, command):
    if char == 'a':
        command.rACT = 1
        command.rATR = 0
        command.rGTO = 1
    elif char == 'd':
        command.rATR = 1
    return command

def handle_activate(req):
    """激活吸盘服务回调"""
    command = outputMsg()
    genCommand('a', command)
    pub.publish(command)
    rospy.sleep(1.0)
    return TriggerResponse(success=True, message="EPick activated")

def handle_release(req):
    """释放吸盘服务回调"""
    command = outputMsg()
    genCommand('d', command)
    pub.publish(command)
    rospy.sleep(1.0)
    return TriggerResponse(success=True, message="EPick released")

if __name__ == "__main__":
    rospy.init_node("epick_service_server")
    
    # 控制吸盘
    pub = rospy.Publisher("RobotiqEPickRobotOutput", outputMsg, queue_size=10)
    
    # 通过 rosservice 通信
    rospy.Service("activate_epick", Trigger, handle_activate)
    rospy.Service("release_epick", Trigger, handle_release)
    
    rospy.loginfo("EPick service server is ready")
    rospy.spin()
