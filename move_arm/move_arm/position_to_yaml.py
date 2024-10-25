import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from move_arm_interfaces.srv import GetPosition
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory



class PositionToYAMLNode(Node):
    def __init__(self):
        super().__init__('position_to_yaml_node')
        # Service
        self.srv = self.create_service(GetPosition, 'position_to_yaml_server', self.srv_callback)
        # Subscriber
        self.create_subscription(JointState, 'joint_states', self.callback, qos_profile_sensor_data)
        # Value
        self.hand_position = None
        self.position_dict = {}

        self.get_logger().info('Ready to /position_to_yaml_server')

    def callback(self, msg):
        self.hand_position = msg.position 

    def srv_callback(self, req, res):
        if req.state == 'add':
            self.get_logger().info('Add hand position.')
            res.result = self.add_position(req.name)
        elif req.state == 'save':
            self.get_logger().info('Save hand position')
            res.result = self.save_position(req.name)
        else:
            self.get_logger().error(f"State '{req.state}' doesn't exist.")
            res.result = False
        return res

    def add_position(self, name):
        if name in self.position_dict:
            self.get_logger().error(f"'{name}' has been registerd. Please enter a different name.")
            return False
        elif name == '':
            self.get_logger().error('No location name enterd.')
            return False
        else: 
            self.position_dict[name] = list(self.hand_position)
            return True

    def save_position(self, file_name):
        try:
            package_name = 'move_arm'
            save_path = os.path.join(
                    get_package_share_directory(package_name), 'config/')
            with open(save_path + file_name + '.yaml', 'w') as f:
                yaml.dump(self.position_dict, f)
            os.makedirs(save_path, exist_ok=True)
            self.get_logger().info(f"Save to {save_path + file_name}.yaml")
            return True
        except Exception as e:
            self.get_logger().error(f"Could not save. Reason: f{e}")
            return False
            


def main(args=None):
    rclpy.init(args=args)
    node = PositionToYAMLNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
