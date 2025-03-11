#!/usr/bin/env python3.10

"""basics3_primitive_execution.py

This tutorial executes several basic robot primitives (unit skills). For detailed documentation
on all available primitives, please see [Flexiv Primitives](https://www.flexiv.com/primitives/).
"""

__copyright__ = "Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import spdlog  # pip install spdlog

# Utility methods
from utility import quat2eulerZYX
from utility import list2str

# Flexiv RDK Python library is installed to user site packages
import flexivrdk

import rospy
from std_srvs.srv import Trigger


def main():
    # ROS初始化
    rospy.init_node("flexiv_client", anonymous=True)
    
    rospy.wait_for_service("activate_epick")
    rospy.wait_for_service("release_epick")
    
    activate = rospy.ServiceProxy("activate_epick", Trigger)
    release = rospy.ServiceProxy("release_epick", Trigger)


    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "robot_sn",
        help="Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456",
    )
    args = argparser.parse_args()

    # Define alias
    logger = spdlog.ConsoleLogger("Example")
    mode = flexivrdk.Mode

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial executes several basic robot primitives (unit "
        "skills). For detailed documentation on all available primitives, please see [Flexiv "
        "Primitives](https://www.flexiv.com/primitives/).\n"
    )

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_sn)

        # Clear fault on the connected robot if any
        if robot.fault():
            logger.warn("Fault occurred on the connected robot, trying to clear ...")
            # Try to clear the fault
            if not robot.ClearFault():
                logger.error("Fault cannot be cleared, exiting ...")
                return 1
            logger.info("Fault on the connected robot is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        logger.info("Enabling robot ...")
        robot.Enable()

        # Wait for the robot to become operational
        while not robot.operational():
            time.sleep(1)

        logger.info("Robot is now operational")

        # Execute Primitives
        # ==========================================================================================
        # Switch to primitive execution mode
        robot.SwitchMode(mode.NRT_PRIMITIVE_EXECUTION)

        # 运动到目标角度
        logger.info("Executing primitive: MoveJ")
        robot.ExecutePrimitive("MoveJ", {"target": [0, -49, 0, 90, 0, 49, 30]})
        while not robot.primitive_states()["reachedTarget"]:
            print(robot.primitive_states())
            time.sleep(1)

        # 激活吸盘
        logger.info("激活吸盘")
        activate()
        time.sleep(1.0)


        # 回到初始状态
        logger.info("Executing primitive: Home")
        robot.ExecutePrimitive("Home", dict())
        while not robot.primitive_states()["reachedTarget"]:
            time.sleep(1)


        # 释放吸盘
        logger.info("释放吸盘")
        release()
        time.sleep(1.0)

        # All done, stop robot and put into IDLE mode
        robot.Stop()

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
