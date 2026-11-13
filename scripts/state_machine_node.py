#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int32, Float64, Bool


class TreasureHuntStateMachine(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.timer = self.create_timer(0.1, self.search_timer_callback)

        # Configurable ROS 2 parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('forward_speed', 0.25)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # FSM State & Memory
        # States: 'SEARCHING', 'VISUAL_SERVOING', 'ROTATING', 'DONE'
        self.state = 'SEARCHING'
        self.visited_ids = set()
        self.active_marker_id = None

        # Map Marker IDs 1-4 to their turn instructions (in radians)
        self.instructions = {
            1: -math.pi / 2.0,  # Turn right by 90 degrees
            2: math.pi,         # Rotate by 180 degrees
            3: math.pi / 2.0,   # Turn left by 90 degrees
            4: 0.0              # Found! Stop: the hunt is over
        }

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.servo_target_pub = self.create_publisher(
            Float64MultiArray, '/treasure_hunt/servo_target', 10
        )
        self.annotated_trigger_pub = self.create_publisher(
            Int32, '/treasure_hunt/publish_annotated_id', 10
        )
        self.rotate_delta_pub = self.create_publisher(
            Float64, '/treasure_hunt/rotate_delta', 10
        )

        # Subscribers
        self.create_subscription(
            Float64MultiArray, '/treasure_hunt/detections', self.detections_callback, 10
        )
        self.create_subscription(
            Int32, '/treasure_hunt/servo_reached', self.servo_reached_callback, 10
        )
        self.create_subscription(
            Bool, '/treasure_hunt/rotation_done', self.rotation_done_callback, 10
        )

        self.get_logger().info('StateMachineNode started in SEARCHING state. Moving ahead...')

    def parse_unvisited_markers(self, data):
        """Extracts unvisited markers (IDs 1..4) from flat array [id, u, v, s, ...]."""
        candidates = []
        for i in range(0, len(data), 4):
            m_id = int(data[i])
            u, v, s = data[i + 1], data[i + 2], data[i + 3]
            if m_id in self.instructions and m_id not in self.visited_ids:
                candidates.append((m_id, u, v, s))
        return candidates

    def detections_callback(self, msg: Float64MultiArray):
        if self.state in ('ROTATING', 'DONE'):
            return

        candidates = self.parse_unvisited_markers(msg.data)

        if self.state == 'SEARCHING':
            if len(candidates) > 0:
                # Lock onto the closest/largest unvisited marker
                best = max(candidates, key=lambda x: x[3])
                self.active_marker_id = best[0]
                self.state = 'VISUAL_SERVOING'
                self.get_logger().info(
                    f'State: SEARCHING -> VISUAL_SERVOING (Target Marker ID: {self.active_marker_id})'
                )
            else:
                # Drive straight ahead looking for the next marker
                twist = Twist()
                twist.linear.x = self.get_parameter('forward_speed').value
                self.cmd_pub.publish(twist)
                return

        if self.state == 'VISUAL_SERVOING':
            # Find the currently tracked marker (or largest unvisited candidate)
            target = None
            for c in candidates:
                if c[0] == self.active_marker_id:
                    target = c
                    break
            if target is None and len(candidates) > 0:
                target = max(candidates, key=lambda x: x[3])
                self.active_marker_id = target[0]

            if target is not None:
                m_id, u, v, s = target
                servo_msg = Float64MultiArray()
                servo_msg.data = [1.0, float(m_id), u, v, s]
                self.servo_target_pub.publish(servo_msg)
            else:
                # Marker temporarily out of view -> creep forward slowly
                twist = Twist()
                twist.linear.x = 0.10
                self.cmd_pub.publish(twist)

    def servo_reached_callback(self, msg: Int32):
        if self.state != 'VISUAL_SERVOING':
            return

        marker_id = msg.data
        if marker_id in self.visited_ids:
            return

        # 1. Disable visual servoing node
        stop_servo = Float64MultiArray()
        stop_servo.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_target_pub.publish(stop_servo)

        # 2. Mark ID as visited so it is ignored if seen again after rotation
        self.visited_ids.add(marker_id)

        # 3. Trigger perception_node to publish annotated image on /marker_image
        trig_msg = Int32()
        trig_msg.data = marker_id
        self.annotated_trigger_pub.publish(trig_msg)

        # 4. Execute marker instruction
        if marker_id == 4:
            self.cmd_pub.publish(Twist())
            self.state = 'DONE'
            self.get_logger().info('=============================================')
            self.get_logger().info('Marker 4 Reached! Found! Stop: the hunt is over!')
            self.get_logger().info('=============================================')
        else:
            delta_yaw = self.instructions[marker_id]
            self.state = 'ROTATING'
            rot_msg = Float64()
            rot_msg.data = delta_yaw
            self.rotate_delta_pub.publish(rot_msg)
            self.get_logger().info(
                f'Marker {marker_id} complete. State: VISUAL_SERVOING -> ROTATING '
                f'({math.degrees(delta_yaw):.0f} deg)'
            )

    def rotation_done_callback(self, msg: Bool):
        if self.state == 'ROTATING' and msg.data:
            self.active_marker_id = None
            self.state = 'SEARCHING'
            self.get_logger().info('State: ROTATING -> SEARCHING. Going ahead for next marker...')

    def search_timer_callback(self):
        """Continuously drives forward while searching for the next marker."""
        if self.state == 'SEARCHING':
            twist = Twist()
            twist.linear.x = float(self.get_parameter('forward_speed').value)
            self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TreasureHuntStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()