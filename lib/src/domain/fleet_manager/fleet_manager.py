from infrastructure.utility.logger import Logger
from rospy import wait_for_message, init_node
from std_msgs.msg import String
from typing import Optional, Union

class FleetManager:
    async def get_robot_status(self, robot_id: Optional[Union[int, str]] = None):
        try:
            init_node("get_robot_status")
            if robot_id is None:
                #TODO: return all available robots status
                return None
            else:
                return {robot_id: str(wait_for_message(topic="", topic_type=String, timeout=5))}
        except Exception as error:
            Logger().error(f"FleetManager -- Get robot -- {type(error).__name__} {error}")
            return None