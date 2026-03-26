#!/usr/bin/env python3
"""
PyC2 Command Handler Node.

订阅 /pyc2/commands，解析 JSON 命令，
根据 command 字段驱动对应的 FlexBE state 执行，
完成后发布结果到 /pyc2/results。

支持的 command:
  - team_move: params = {team_name, linear_x, angular_z, duration}
  - team_wait: params = {duration}
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class PyC2CommandHandler(Node):
    """ROS2 node that bridges /pyc2/commands to actual robot team topics."""

    COMMAND_TOPIC = '/pyc2/commands'
    RESULT_TOPIC = '/pyc2/results'

    def __init__(self):
        super().__init__('pyc2_command_handler')

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribe to PyC2 commands
        self._cmd_sub = self.create_subscription(
            String, self.COMMAND_TOPIC, self._on_command, qos_best_effort
        )

        # Publish results back to PyC2
        self._result_pub = self.create_publisher(String, self.RESULT_TOPIC, qos_reliable)

        # Cache of velocity publishers per team
        self._vel_pubs = {}
        self._qos_vel = qos_reliable

        self.get_logger().info('PyC2CommandHandler ready, listening on /pyc2/commands')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _get_vel_pub(self, team_name: str):
        """Get or create a cmd_vel publisher for a team."""
        if team_name not in self._vel_pubs:
            topic = f'/{team_name}_team/cmd_vel'
            self._vel_pubs[team_name] = self.create_publisher(
                Twist, topic, self._qos_vel
            )
            self.get_logger().info(f'Created publisher for {topic}')
        return self._vel_pubs[team_name]

    def _publish_result(self, command_id: str, status: str, message: str = ''):
        """Publish result back to PyC2."""
        payload = {
            'command_id': command_id,
            'status': status,
            'message': message,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._result_pub.publish(msg)
        self.get_logger().info(f'Result published: {payload}')

    # ------------------------------------------------------------------
    # Command dispatch
    # ------------------------------------------------------------------

    def _on_command(self, msg: String):
        """Callback for incoming PyC2 commands. Runs in a background thread."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in command: {e}')
            return

        command_id = data.get('command_id', '')
        command = data.get('command', '')
        params = data.get('params', {})

        self.get_logger().info(f'Received command: {command} (id={command_id})')

        # Execute in background thread so we don't block the ROS2 executor
        t = threading.Thread(
            target=self._dispatch,
            args=(command_id, command, params),
            daemon=True
        )
        t.start()

    def _dispatch(self, command_id: str, command: str, params: dict):
        """Dispatch command to the appropriate handler."""
        try:
            if command == 'team_move':
                success = self._handle_team_move(params)
            elif command == 'team_wait':
                success = self._handle_team_wait(params)
            else:
                self.get_logger().warn(f'Unknown command: {command}')
                self._publish_result(command_id, 'failure', f'Unknown command: {command}')
                return

            status = 'success' if success else 'failure'
            self._publish_result(command_id, status)

        except Exception as e:
            self.get_logger().error(f'Error dispatching command {command}: {e}')
            self._publish_result(command_id, 'failure', str(e))

    # ------------------------------------------------------------------
    # Command handlers
    # ------------------------------------------------------------------

    def _handle_team_move(self, params: dict) -> bool:
        """
        Drive a team at the given velocity for the specified duration.
        params: {team_name, linear_x, angular_z, duration}
        """
        team_name = str(params.get('team_name', 'scout'))
        linear_x = float(params.get('linear_x', 0.0))
        angular_z = float(params.get('angular_z', 0.0))
        duration = float(params.get('duration', 1.0))

        pub = self._get_vel_pub(team_name)
        self.get_logger().info(
            f'TeamMove: {team_name} linear_x={linear_x} angular_z={angular_z} duration={duration}s'
        )

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        start = time.time()
        # Publish at ~10 Hz for the duration
        rate_sec = 0.1
        while time.time() - start < duration:
            pub.publish(cmd)
            time.sleep(rate_sec)

        # Stop the team
        stop = Twist()
        pub.publish(stop)
        self.get_logger().info(f'TeamMove: {team_name} stopped')
        return True

    def _handle_team_wait(self, params: dict) -> bool:
        """
        Wait for the specified duration.
        params: {duration}
        """
        duration = float(params.get('duration', 1.0))
        self.get_logger().info(f'TeamWait: waiting {duration}s')
        time.sleep(duration)
        self.get_logger().info('TeamWait: done')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PyC2CommandHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
