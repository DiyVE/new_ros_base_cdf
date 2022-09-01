#!/usr/bin/env python
# coding=utf-8
# license removed for brevity
import rospy
from cdf_msgs.msg import Pic_Action
from std_msgs.msg import String
from os import walk

def callback(userPicHandler, motorPicHandler, msg):
    """
    Callback function that writes a specified message to a specified card.

    Parameters
    ----------
    userPicHandler : serial handler
        The serial handler of the user card.
    motorPicHandler : serial handler
        The serial handler of the motor card.
    msg : string
        The message to write to the card.
    """
    rospy.loginfo("writing %s to serial %s" % (msg.action_msg, msg.action_destination))
    if msg.action_destination == 'user':
        pi.serial_write(userPicHandler, msg.action_msg+'\n')
    elif msg.action_destination == 'motor':
        pi.serial_write(motorPicHandler, msg.action_msg+'\n')
    else:
        rospy.logerr("requested serial port destination (%s) not found" % msg.action_destination)

def indentify_PIC_card():
    """
    Identify PIC cards and returns corresponding serial handlers.

    Returns
    -------
    userPicHandler : serial handler
        The serial handler of the user card.
    motorPicHandler : serial handler
        The serial handler of the motor card.
    """
    filenames = next(walk("/dev"), (None, None, []))[2]  # [] if no file
    userPicHandler, motorPicHandler = None, None
    valid_filenames = []
    for filename in filenames:
        if filename.startswith("ttyUSB"):
            valid_filenames.append(filename)
    for filename in valid_filenames:
        rospy.loginfo("trying to connect to %s" % filename)
        try:
            card_handler = pi.serial_open("/dev/" + filename, baudrate, 0)
            pi.serial_write(card_handler, "VERSION\n")
            version = rospy.wait_for_message(versionPic_topic, String, timeout=0.2)
            if version.data == "motor":
                rospy.loginfo("motor card found")
                motorPicHandler = card_handler
            elif version.data == "user":
                rospy.loginfo("user card found")
                userPicHandler = card_handler
            else :
                raise ValueError("unknown card type")
        except rospy.ROSException as e:
            rospy.loginfo("the card isn't responding ")
            pi.serial_close(card_handler)
        except rospy.ERROR as e:
            rospy.loginfo("can't open the usb port")
        except ValueError:
            rospy.loginfo("Unknown card found")
            pi.serial_close(card_handler)
    return userPicHandler, motorPicHandler


if __name__ == '__main__':
    # Node declaration
    # anonymous = False => erreur si on lance cette node plusieurs fois
    rospy.init_node('sender', anonymous=False)

    # Extracting parameters
    baudrate = rospy.get_param('~serial_baudrate', 115200)
    debug_mode = bool(rospy.get_param('~debug_mode', False))
    action_topic = rospy.get_param('~action_topic', '/robot_x/action')
    versionPic_topic = rospy.get_param('~versionPic_topic', '/robot_x/init/version_PIC')

    # In debug mode, we just spin for an infinite time
    if debug_mode:
        rospy.loginfo("debug mode activated")
        rospy.spin()
    # In normal mode, we try to identify the PIC cards and then process all received messages
    else:
        import pigpio

        pi = pigpio.pi()
        userPicHandler, motorPicHandler = indentify_PIC_card()
        while userPicHandler is None or motorPicHandler is None:
            rospy.logerr("One of the PIC cards is not found : Retrying")
            try:
                userPicHandler, motorPicHandler = indentify_PIC_card()
                rospy.sleep(1)
            except pigpio.error as e:
                rospy.logerr("PIGPIO error : %s\nRestarting Pigpiod" % e)

        rospy.loginfo("userPic and motorPic UART Connected")
        try:
            rospy.Subscriber(action_topic, Pic_Action, lambda msg: callback(userPicHandler, motorPicHandler,msg), queue_size=100)
            rospy.spin()
        except RuntimeError as e:
            print(e)
        
