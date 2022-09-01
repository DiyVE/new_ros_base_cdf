#!/usr/bin/env python
# coding=utf-8

from std_msgs.msg import String, Bool
from sender.msg import Evitement

def format_evitement(evitement_msg):
    """
    Formate la réponse de la commande d'évitement

    Paramètres
    ----------
        - evitement_msg (str) : message de réponse de la commande d'évitement

    """
    msg = [bool(int(evitement_msg[i])) for i in range(len(evitement_msg))]
    return msg

msgs_dict = {
    "motor" : {
        0: {
            "topic_rosparam": "~time_pic_topic",
            "topic_default": "/robot_state/time_pic",
            "type": String,
            "publish_on_change": False,
            "value_expression": lambda x: x
        },
        1: {
            "topic_rosparam": "~asserv_motion_done_topic",
            "topic_default": "/robot_state/asserv_motion_done",
            "type": Bool,
            "publish_on_change": False,
            "value_expression": lambda x: bool(int(x))
        },
        2: {
            "topic_rosparam": "~Odom_x",
            "topic_default": "/robot_state/Odomx",
            "type": String,
            "publish_on_change": False,
            "value_expression": lambda x: x
        },
        3: {
            "topic_rosparam": "~Odom_y",
            "topic_default": "/robot_state/Odomy",
            "type": String,
            "publish_on_change": False,
            "value_expression": lambda x: x
        },
        4: {
            "topic_rosparam": "~linear_speed_topic",
            "topic_default": "/robot_state/linear_speed",
            "type": String,
            "publish_on_change": False,
            "value_expression": lambda x: x
        },
        5: {
            "topic_rosparam": "~angular_speed_topic",
            "topic_default": "/robot_state/angular_speed",
            "type": String,
            "publish_on_change": False,
            "value_expression": lambda x: x
        },
        6: {
            "topic_rosparam": "~Odom_t",
            "topic_default": "/robot_state/Odomt",
            "type": String,
            "publish_on_change": False,
            "value_expression": lambda x: x
        }
    },
    "version" : {
        0: {
            "topic_rosparam": "~version_topic",
            "topic_default": "/init/version_PIC",
            "type": String,
            "publish_on_change": False,
            "value_expression": lambda x: x
        }
    },
    "buttons": {
        0: {
            "topic_rosparam": "~button_topic",
            "topic_default": "/buttons",
            "type": String,
            "publish_on_change": False,
            "value_expression": lambda x: x
        }
    },
    "actionAX": {
        0: {
            "topic_rosparam": "~axdone_topic",
            "topic_default": "/ax12_status",
            "type": Bool,
            "publish_on_change": False,
            "value_expression": lambda x: True
        }
    },
    "evit": {
        0: {
            "topic_rosparam": "~evit_topic",
            "topic_default": "/robot_state/evit",
            "type": Evitement,
            "publish_on_change": False,
            "value_expression": lambda x: format_evitement(x)
        }
    },
    "Debug_Sicks": {
        0: {
            "topic_rosparam": "~sick_topic",
            "topic_default": "/debug_pic",
            "type": String,
            "publish_on_change": False,
            "value_expression": lambda x: x
        }
    },
    "match_status": {
            0: {
                "topic_rosparam": "~match_status_topic",
                "topic_default": "/robot_state/match_status",
                "type": String,
                "publish_on_change": False,
                "value_expression": lambda x: x
        }
    }
}

