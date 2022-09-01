from cdf_msgs.msg import RobotData
from std_msgs.msg import String

msgs_dict = {
    "motor" : {
        "robot_data": {
            "topic_rosparam": "~robot_odom_topic",
            "topic_default": "/robot_x/Odom",
            "type": RobotData,
            "publish_on_change": False,
            "structure": {
                0: {
                    "value_expression": lambda x: 1,
                    "path": "velocity.z"
                },
                1: {
                    "value_expression": lambda x: 1,
                    "path": "position.z"
                },
                2: {
                    "value_expression": lambda x: x,
                    "path": "position.x"
                },
                3: {
                    "value_expression": lambda x: x,
                    "path": "position.y"
                },
                4: {
                    "value_expression": lambda x: x,
                    "path": "velocity.x"
                },
                5: {
                    "value_expression": lambda x: x,
                    "path": "velocity.y"
                },
                6: {
                    "value_expression": lambda x: x,
                    "path": "theta"
                }
            }
        }
    },
    "version" : {
        "pic_version": {
            "topic_rosparam": "~version_topic",
            "topic_default": "/robot_x/init/version_PIC",
            "type": String,
            "publish_on_change": False,
            "structure": {
                0: {
                    "value_expression": lambda x: "'" + x + "'",
                    "path": "data"
                }
            }
        }
    }
}