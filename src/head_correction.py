#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use setAngles Method"""

import qi
import argparse
import sys
import time
import rospy
from std_srvs.srv import Empty
from pepper_behaviour_srvs.srv import MoveArm, MoveHead, Speak


def move_head(session, msg):
    """
    This example uses the setAngles method.
    """
    # Get the service ALMotion.

    motion_service  = session.service("ALMotion")

    motion_service.setStiffnesses("Head", 1.0)

    #set angles, using a fraction of max speed
    names  = ["HeadYaw", "HeadPitch"]
    angles  = [0.0, 0.0]
    fractionMaxSpeed  = 0.2
    motion_service.setAngles(names, angles, fractionMaxSpeed)

    time.sleep(3.0)
    motion_service.setStiffnesses("Head", 0.5)
    rospy.loginfo("Moved head to position 0.5")
    return "ok"

def move_arm(session, msg):
    motion_service  = session.service("ALMotion")
    arm = "L" if msg.arm == "left" else "R"
    joint_poses = msg.positions

    motion_service.setStiffnesses(arm + "Arm", 1.0)

    neutral_names = ['ShoulderPitch', 'ShoulderRoll', 'ElbowYaw', 'ElbowRoll', 'WristYaw']
    names = [arm + n for n in neutral_names]

    fractionMaxSpeed = 0.2

    motion_service.setAngles(names, joint_poses, fractionMaxSpeed)
    time.sleep(3.0)
    motion_service.setStiffnesses(arm + "Arm", 0.5)
    rospy.loginfo(f"Moved {msg.arm} Arm")
    return "ok"

def speak(session, msg):
    tts = session.service("ALTextToSpeech")

    tts.say(msg.text)

    return "ok"

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    rospy.init_node("pepper_behaviour")
    rospy.Service("move_head_straight", MoveHead, lambda msg: move_head(session, msg))
    rospy.Service("move_arm", MoveArm, lambda msg: move_arm(session, msg))
    rospy.Service("speak", Speak, lambda msg: speak(session, msg))

    rospy.loginfo("Service is running")
    rospy.spin()
