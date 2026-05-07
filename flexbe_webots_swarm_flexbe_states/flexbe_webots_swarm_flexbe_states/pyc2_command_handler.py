#!/usr/bin/env python3
"""PyC2 → FlexBE dispatcher.

Subscribes to `/pyc2/commands` (std_msgs/String, JSON payload) and routes
each command to a FlexBE behavior via the
`flexbe/execute_behavior` Action (`flexbe_msgs.action.BehaviorExecution`).
This keeps the PyC2 JSON protocol stable while ensuring the FlexBE engine
is actually in the loop (flexbe_onboard drives the behavior + state machine;
the state publishes Webots String/GOTO control).

Supported commands:
  - team_goto     : {team_name, x, y, tolerance, timeout}
                    → "Team Goto Behavior" (member_ids resolved here)
  - formation_goto: {members, x, y, tolerance, timeout}
                    → "Formation Goto Behavior"
  - team_move     : {team_name, linear_x, angular_z, duration}  [legacy]
                    → "Team Move Behavior"
  - team_wait     : {duration}
                    → "Team Wait Behavior"

Requires the flexbe_widget `be_action_server` to be running (it serves
`flexbe/execute_behavior` and bridges to flexbe_onboard internally).
"""
import json
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from flexbe_msgs.action import BehaviorExecution
from std_msgs.msg import String


SCOUT_IDS = [f'scout_{i}' for i in range(10)]
CARRIER_IDS = [f'carrier_{i}' for i in range(20)]

BEHAVIOR_NAME = {
    'team_goto': 'Team Goto Behavior',
    'formation_goto': 'Formation Goto Behavior',
    'team_move': 'Team Move Behavior',
    'team_wait': 'Team Wait Behavior',
}

EXECUTE_BEHAVIOR_ACTION = 'flexbe/execute_behavior'
GOAL_WAIT_TIMEOUT = 5.0
ACTION_SERVER_READY_TIMEOUT = 30.0


class PyC2CommandHandler(Node):
    """Thin dispatcher: PyC2 JSON → FlexBE BehaviorExecution action."""

    COMMAND_TOPIC = '/pyc2/commands'
    RESULT_TOPIC = '/pyc2/results'

    def __init__(self):
        super().__init__('pyc2_command_handler')

        self._cb_group = ReentrantCallbackGroup()

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

        self._cmd_sub = self.create_subscription(
            String, self.COMMAND_TOPIC, self._on_command, qos_best_effort,
            callback_group=self._cb_group,
        )
        self._result_pub = self.create_publisher(
            String, self.RESULT_TOPIC, qos_reliable,
        )

        self._action_client = ActionClient(
            self, BehaviorExecution, EXECUTE_BEHAVIOR_ACTION,
            callback_group=self._cb_group,
        )

        # Wait (non-blocking to ROS) for the action server. We do this in a
        # background thread so the node constructor returns quickly and the
        # executor can start spinning; messages arriving before the server is
        # up are rejected gracefully in _dispatch.
        self._action_ready = threading.Event()
        threading.Thread(target=self._wait_for_action_server, daemon=True).start()

        self.get_logger().info(
            f'PyC2CommandHandler ready on {self.COMMAND_TOPIC}; '
            f'action client pending {EXECUTE_BEHAVIOR_ACTION}'
        )

    # ------------------------------------------------------------------ #
    # setup
    # ------------------------------------------------------------------ #
    def _wait_for_action_server(self):
        ready = self._action_client.wait_for_server(timeout_sec=ACTION_SERVER_READY_TIMEOUT)
        if ready:
            self._action_ready.set()
            self.get_logger().info(f'✓ action server {EXECUTE_BEHAVIOR_ACTION} is up')
        else:
            self.get_logger().error(
                f'✗ action server {EXECUTE_BEHAVIOR_ACTION} not available after '
                f'{ACTION_SERVER_READY_TIMEOUT}s; commands will fail until it starts.'
            )

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #
    @staticmethod
    def _resolve_members(team_name: str):
        team = (team_name or '').lower()
        if team == 'scout':
            return list(SCOUT_IDS)
        if team == 'carrier':
            return list(CARRIER_IDS)
        return []

    def _publish_result(self, command_id: str, status: str, message: str = ''):
        payload = {
            'command_id': command_id,
            'status': status,
            'message': message,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._result_pub.publish(msg)
        self.get_logger().info(f'Result: {payload}')

    @staticmethod
    def _stringify_args(args: dict):
        """FlexBE arg_values are strings; JSON-encode lists/dicts."""
        arg_keys = []
        arg_values = []
        for k, v in args.items():
            arg_keys.append(str(k))
            if isinstance(v, (list, tuple, dict)):
                arg_values.append(json.dumps(v, ensure_ascii=False))
            elif isinstance(v, bool):
                arg_values.append('True' if v else 'False')
            else:
                arg_values.append(str(v))
        return arg_keys, arg_values

    # ------------------------------------------------------------------ #
    # command routing
    # ------------------------------------------------------------------ #
    def _on_command(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid JSON in command: {exc}')
            return

        command_id = data.get('command_id', '')
        command = data.get('command', '')
        params = data.get('params', {}) or {}

        self.get_logger().info(f'Received command: {command} (id={command_id})')

        try:
            self._dispatch(command_id, command, params)
        except Exception as exc:
            self.get_logger().error(f'dispatch error: {exc}')
            self._publish_result(command_id, 'failure', str(exc))

    def _dispatch(self, command_id: str, command: str, params: dict):
        behavior_name = BEHAVIOR_NAME.get(command)
        if not behavior_name:
            self._publish_result(command_id, 'failure', f'Unknown command: {command}')
            return

        if not self._action_ready.is_set():
            ready = self._action_client.wait_for_server(timeout_sec=GOAL_WAIT_TIMEOUT)
            if ready:
                self._action_ready.set()
            else:
                self._publish_result(
                    command_id, 'failure',
                    f'action server {EXECUTE_BEHAVIOR_ACTION} not ready',
                )
                return

        behavior_args = self._build_behavior_args(command, params)
        if behavior_args is None:
            self._publish_result(command_id, 'failure', f'Bad params for {command}: {params}')
            return

        goal = BehaviorExecution.Goal()
        goal.behavior_name = behavior_name
        arg_keys, arg_values = self._stringify_args(behavior_args)
        goal.arg_keys = arg_keys
        goal.arg_values = arg_values
        goal.input_keys = []
        goal.input_values = []

        self.get_logger().info(
            f'Dispatching behavior "{behavior_name}" with args={behavior_args}'
        )

        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda fut: self._on_goal_response(fut, command_id, behavior_name)
        )

    def _build_behavior_args(self, command: str, params: dict):
        try:
            if command == 'team_goto':
                team_name = str(params['team_name'])
                members = self._resolve_members(team_name)
                if not members:
                    return None
                return {
                    'team_name': team_name,
                    'x': float(params['x']),
                    'y': float(params['y']),
                    'tolerance': float(params.get('tolerance', 0.22)),
                    'timeout': float(params.get('timeout', 60.0)),
                    'member_ids': list(members),
                }
            if command == 'formation_goto':
                members = list(params.get('members') or [])
                if not members:
                    return None
                return {
                    'x': float(params['x']),
                    'y': float(params['y']),
                    'tolerance': float(params.get('tolerance', 0.22)),
                    'timeout': float(params.get('timeout', 60.0)),
                    'member_ids': members,
                }
            if command == 'team_move':
                return {
                    'team_name': str(params.get('team_name', 'scout')),
                    'linear_x': float(params.get('linear_x', 0.0)),
                    'angular_z': float(params.get('angular_z', 0.0)),
                    'duration': float(params.get('duration', 1.0)),
                }
            if command == 'team_wait':
                return {
                    'duration': float(params.get('duration', 1.0)),
                }
        except (KeyError, TypeError, ValueError):
            return None
        return None

    # ------------------------------------------------------------------ #
    # action callbacks
    # ------------------------------------------------------------------ #
    def _on_goal_response(self, goal_future, command_id: str, behavior_name: str):
        try:
            goal_handle = goal_future.result()
        except Exception as exc:
            self.get_logger().error(f'send_goal failed for {behavior_name}: {exc}')
            self._publish_result(command_id, 'failure', f'send_goal error: {exc}')
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected by action server for {behavior_name}')
            self._publish_result(command_id, 'failure', 'goal rejected')
            return

        self.get_logger().info(f'Goal accepted for {behavior_name}; awaiting result')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut: self._on_goal_result(fut, command_id, behavior_name)
        )

    def _on_goal_result(self, result_future, command_id: str, behavior_name: str):
        try:
            wrapped = result_future.result()
        except Exception as exc:
            self.get_logger().error(f'get_result failed for {behavior_name}: {exc}')
            self._publish_result(command_id, 'failure', f'get_result error: {exc}')
            return

        outcome = getattr(wrapped.result, 'outcome', '') or ''
        self.get_logger().info(
            f'Behavior "{behavior_name}" finished: status={wrapped.status} '
            f'outcome="{outcome}"'
        )

        # flexbe_widget/be_action_server sets outcome to 'success' on FINISHED,
        # 'preempted' on preempt, 'failed' on BEStatus.FAILED, 'error' on ERROR.
        if outcome == 'success':
            self._publish_result(command_id, 'success')
        elif outcome == 'preempted':
            self._publish_result(command_id, 'canceled', 'preempted')
        else:
            self._publish_result(command_id, 'failure', outcome or 'unknown')


def main(args=None):
    rclpy.init(args=args)
    node = PyC2CommandHandler()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
