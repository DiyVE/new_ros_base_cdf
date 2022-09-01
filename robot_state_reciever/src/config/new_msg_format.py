from cdf_msgs.msg import RobotData

msgs_dict = {
    "motor" : {
        "robot_data": {
            "topic_rosparam": "~robot_odom_topic",
            "topic_default": "/robot_x/Odom",
            "type": RobotData,
            "publish_on_change": False,
            "structure": {
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
    }
}