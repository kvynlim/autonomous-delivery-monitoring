from infrastructure.utility.logger import Logger
from rospy import wait_for_message, init_node
from robot_status_msgs.msg import RobotStatus
from typing import Dict, List, Optional


class FleetManager:
    def __init__(self):
        self.__robots_id: List[str] = ["tb3_0", "tb3_1", "tb3_2"]

    async def get_robot_status(self, robot_id: Optional[str] = None):
        try:
            init_node("get_robot_status")

            robots_id = list()
            robot_id_robot_status: Dict[str, dict] = dict()
            if robot_id is None:
                robots_id.extend(self.__robots_id)
            else:
                if robot_id in self.__robots_id:
                    robots_id.append(robot_id)
                else:
                    Logger().error(f"FleetManager -- Invalid robot id {robot_id}")
                    return None

            for robot_id in robots_id:
                data = wait_for_message(topic=f"/{robot_id}/robot_status", topic_type=RobotStatus, timeout=5)
                robot_id_robot_status[robot_id] = {
                    "position": {
                        "x": data.pose.position.x,
                        "y": data.pose.position.y,
                        "z": data.pose.position.z,
                    },
                    "battery_level": data.battery_level,
                }
            return robot_id_robot_status
        except Exception as error:
            Logger().error(f"FleetManager -- Get robot -- {type(error).__name__} {error}")
            return None
