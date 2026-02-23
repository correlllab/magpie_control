"""
Gripper ROS2 Node - Wraps existing Gripper class for ROS control
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_srvs.srv import Trigger
from magpie_msgs.srv import SetGripperPosition, SetGripperForce
from magpie_msgs.msg import GripperState, DeliGraspParams
from magpie_msgs.action import DeliGrasp
import numpy as np

# Import existing gripper class
from magpie_control.gripper import Gripper


class GripperNode(Node):
    """ROS2 node wrapping Gripper class for control"""
    
    def __init__(self):
        super().__init__('gripper_node')
        
        # Declare parameters
        self.declare_parameter('auto_detect_port', True)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('use_eflesh', False)
        self.declare_parameter('default_speed', 100)
        self.declare_parameter('default_torque', 200)
        
        # Get parameters
        auto_detect = self.get_parameter('auto_detect_port').value
        port = None if auto_detect else self.get_parameter('port').value
        use_eflesh = self.get_parameter('use_eflesh').value
        
        # Initialize gripper
        try:
            self.get_logger().info('Initializing gripper...')
            self.gripper = Gripper(
                servoport=port,
                debug=False,
                use_eflesh=use_eflesh
            )
            
            # Set default parameters
            default_speed = self.get_parameter('default_speed').value
            default_torque = self.get_parameter('default_torque').value
            self.gripper.set_speed(default_speed)
            self.gripper.set_torque(default_torque)
            
            self.get_logger().info('Gripper initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize gripper: {e}')
            raise
        
        # Create services
        self.srv_open = self.create_service(
            Trigger, 'gripper/open', self.open_callback)
        self.srv_close = self.create_service(
            Trigger, 'gripper/close', self.close_callback)
        self.srv_set_position = self.create_service(
            SetGripperPosition, 'gripper/set_position', self.set_position_callback)
        self.srv_set_force = self.create_service(
            SetGripperForce, 'gripper/set_force', self.set_force_callback)
        self.srv_calibrate = self.create_service(
            Trigger, 'gripper/calibrate', self.calibrate_callback)
        
        # Create action server for DeliGrasp
        self.action_server = ActionServer(
            self,
            DeliGrasp,
            'gripper/deligrasp',
            self.deligrasp_execute_callback
        )
        
        # Create publisher for gripper state
        self.pub_state = self.create_publisher(GripperState, 'gripper/state', 10)
        
        # Create timer for publishing state
        self.timer = self.create_timer(0.1, self.publish_state)  # 10 Hz
        
        self.get_logger().info('Gripper Node initialized')
    
    def publish_state(self):
        """Publish current gripper state"""
        try:
            msg = GripperState()
            msg.position = self.gripper.get_gripper_distance() / 1000.0  # Convert mm to m
            msg.force = self.gripper.get_gripper_force() if hasattr(self.gripper, 'get_gripper_force') else 0.0
            msg.is_moving = False  # TODO: implement is_moving detection
            msg.contact_detected = False  # TODO: implement contact detection
            
            # Get individual finger positions if available
            if hasattr(self.gripper, 'get_finger_positions'):
                msg.finger_positions = self.gripper.get_finger_positions()
            
            self.pub_state.publish(msg)
        except Exception as e:
            self.get_logger().warning(f'Error publishing gripper state: {e}')
    
    def open_callback(self, request, response):
        """Service callback to open gripper"""
        try:
            self.get_logger().info('Opening gripper...')
            self.gripper.gripper_open()
            response.success = True
            response.message = 'Gripper opened successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to open gripper: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def close_callback(self, request, response):
        """Service callback to close gripper"""
        try:
            self.get_logger().info('Closing gripper...')
            self.gripper.gripper_test_close()
            response.success = True
            response.message = 'Gripper closed successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to close gripper: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def set_position_callback(self, request, response):
        """Service callback to set gripper position"""
        try:
            # Convert meters to mm
            target_mm = request.position * 1000.0
            
            self.get_logger().info(f'Setting gripper position to {target_mm:.2f} mm')
            self.gripper.gripper_goto_distance(target_mm)
            
            # Get actual position
            actual_mm = self.gripper.get_gripper_distance()
            response.actual_position = actual_mm / 1000.0
            response.success = True
            response.message = f'Gripper position set to {actual_mm:.2f} mm'
        except Exception as e:
            response.success = False
            response.message = f'Failed to set position: {str(e)}'
            response.actual_position = 0.0
            self.get_logger().error(response.message)
        
        return response
    
    def set_force_callback(self, request, response):
        """Service callback to set gripper force limit"""
        try:
            # Map force in Newtons to torque (simplified mapping)
            # TODO: Use proper force-to-torque conversion based on gripper geometry
            torque = int(min(max(request.max_force * 10, 0), 1023))
            
            self.get_logger().info(f'Setting gripper force limit to {request.max_force:.2f} N (torque={torque})')
            self.gripper.set_torque(torque)
            
            response.success = True
            response.message = f'Force limit set to {request.max_force:.2f} N'
        except Exception as e:
            response.success = False
            response.message = f'Failed to set force: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def calibrate_callback(self, request, response):
        """Service callback to calibrate gripper"""
        try:
            self.get_logger().info('Calibrating gripper...')
            # Open fully then close to reset
            self.gripper.gripper_open()
            rclpy.spin_once(self, timeout_sec=2.0)
            self.gripper.gripper_test_close()
            
            response.success = True
            response.message = 'Gripper calibrated successfully'
        except Exception as e:
            response.success = False
            response.message = f'Failed to calibrate: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    async def deligrasp_execute_callback(self, goal_handle):
        """Action callback for DeliGrasp execution"""
        self.get_logger().info('Executing DeliGrasp...')
        
        params = goal_handle.request.params
        feedback_msg = DeliGrasp.Feedback()
        
        try:
            # Phase 1: Approach - move to goal aperture
            feedback_msg.phase = 'approach'
            feedback_msg.current_aperture = params.goal_aperture
            goal_handle.publish_feedback(feedback_msg)
            
            self.gripper.gripper_goto_distance(params.goal_aperture * 1000.0)
            
            # Phase 2: Initial close with force control
            feedback_msg.phase = 'initial_close'
            goal_handle.publish_feedback(feedback_msg)
            
            # Use existing DeliGrasp implementation if available
            if hasattr(self.gripper, 'deligrasp'):
                result = self.gripper.deligrasp(
                    x=params.goal_aperture * 1000.0,
                    fc=params.initial_force,
                    dx=params.additional_closure * 1000.0,
                    df=params.additional_force,
                    complete=params.complete_grasp
                )
                force_log = result if isinstance(result, list) else []
            else:
                # Simple fallback: just close with torque limit
                self.get_logger().warning('DeliGrasp not available, using simple close')
                self.set_force_callback(
                    type('obj', (object,), {'max_force': params.initial_force}),
                    type('obj', (object,), {})()
                )
                self.gripper.gripper_test_close()
                force_log = []
            
            # Return result
            result_msg = DeliGrasp.Result()
            result_msg.success = True
            result_msg.message = 'DeliGrasp completed successfully'
            result_msg.final_aperture = self.gripper.get_gripper_distance() / 1000.0
            result_msg.final_force = params.initial_force + params.additional_force
            result_msg.force_log = force_log
            
            goal_handle.succeed()
            return result_msg
            
        except Exception as e:
            self.get_logger().error(f'DeliGrasp failed: {e}')
            result_msg = DeliGrasp.Result()
            result_msg.success = False
            result_msg.message = f'DeliGrasp failed: {str(e)}'
            goal_handle.abort()
            return result_msg
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down Gripper Node...')
        try:
            # Open gripper before shutdown for safety
            self.gripper.gripper_open()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
