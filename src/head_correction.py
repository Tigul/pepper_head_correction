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
    Moves the head joints, HeadYaw and HeadPitch into the positions specified in
    the Message.
    :param Session: The NAOqi session that connects this node to Pepper.
    :param msg: The ROS message that contains the goal joint values.
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
    """
    Moves one arm into the position specified in the Message. The message consists
    of a list with the joint position for every joint in the arm. The message also
    contains the arm which should be moved.
    :param session: The NAOqi session which connects this node to the Pepper.
    :param msg: The ROS message which contains the arm as well as the goal joint states.
    """
    motion_service  = session.service("ALMotion")
    arm = "L" if msg.arm == "left" else "R"
    joint_poses = msg.positions

    motion_service.setStiffnesses(arm + "Arm", 1.0)

    neutral_names = ['ShoulderPitch', 'ShoulderRoll', 'ElbowYaw', 'ElbowRoll', 'WristYaw']
    # Create correct joint names for the chosen arm
    names = [arm + n for n in neutral_names]

    fractionMaxSpeed = 0.2

    motion_service.setAngles(names, joint_poses, fractionMaxSpeed)
    # Needs to be here since the previous statement is non-blocking and otherwise
    # the stiffness would be set while the arm is still moving.
    time.sleep(3.0)
    motion_service.setStiffnesses(arm + "Arm", 0.5)
    rospy.loginfo(f"Moved {msg.arm} Arm")
    return "ok"

def speak(session, msg):
    """
    Sends a string to the Text-to-Speech engine of NAOqi which generates spoken text
    and plays it over the speaker located in Peppers head.
    :param session: The NAOqi session which connects to Pepper.
    :param msg: The ROS message containing the text to be spoken.
    """
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
    # The NAOqi session which connects to the Pepper robot
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    rospy.init_node("pepper_behaviour")
    # Definition of the ROS services
    rospy.Service("move_head", MoveHead, lambda msg: move_head(session, msg))
    rospy.Service("move_arm", MoveArm, lambda msg: move_arm(session, msg))
    rospy.Service("speak", Speak, lambda msg: speak(session, msg))

    rospy.loginfo("Service is running")
    rospy.spin()
