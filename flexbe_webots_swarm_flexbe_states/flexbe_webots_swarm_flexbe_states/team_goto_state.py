#!/usr/bin/env python3
"""TeamGotoState — drive a whole team to absolute (x, y) via Webots control.

Publishes `"GOTO <x> <y>"` (std_msgs/String) once on enter to
`/{team_name}_team/control` (Webots smart_agent_controller accepts this
shape directly — it triggers the on-robot A* / Dijkstra navigator).

Polls `/{robot}/true_pose` (BEST_EFFORT, std_msgs/String JSON with x/y/yaw)
for every member_id; returns `arrived` once ALL members are within
`tolerance` of the goal. Falls back to `failed` on timeout.
"""
import json
import math

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_core.proxy.qos import QOS_DEFAULT, QoSProfile
from rclpy.qos import HistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


# BEST_EFFORT QoS for high-rate telemetry (matches supervisor.py publisher)
TELEMETRY_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)


class TeamGotoState(EventState):
    """
    Drive every member of `team_name` to (x, y) and block until arrived.

    -- team_name    string         集群名称 ('scout' 或 'carrier')
    -- x            float          目标 X (米)
    -- y            float          目标 Y (米)
    -- tolerance    float          到达半径 (米)
    -- timeout      float          超时秒数
    -- member_ids   string         JSON 数组，例如 '["carrier_0","carrier_1",...]'

    <= arrived                     全员到达
    <= failed                      超时 / 成员 ID 解析失败
    """

    def __init__(self, team_name, x, y, tolerance, timeout, member_ids):
        super(TeamGotoState, self).__init__(outcomes=['arrived', 'failed'])

        self._team_name = team_name
        self._goal = (float(x), float(y))
        self._tolerance = float(tolerance)
        self._timeout = float(timeout)
        self._member_ids_raw = member_ids

        self._members = []
        self._control_topic = f'/{team_name}_team/control'
        self._pose_topics = []
        self._start_time = None

        # Control publisher (reliable enough; Webots subscribes with default QoS)
        self._pub = ProxyPublisher({self._control_topic: String})
        self._sub = ProxySubscriberCached()

    # ------------------------------------------------------------------ #
    # lifecycle
    # ------------------------------------------------------------------ #
    def on_enter(self, userdata):
        try:
            self._members = self._parse_members(self._member_ids_raw)
        except Exception as exc:
            Logger.logerr(f'TeamGotoState: bad member_ids "{self._member_ids_raw}": {exc}')
            self._members = []
            return

        if not self._members:
            Logger.logwarn(f'TeamGotoState: empty member list for team {self._team_name}')

        from flexbe_core.core.ros_state import RosState
        self._start_time = RosState._node.get_clock().now()

        # Subscribe to each member's true_pose with BEST_EFFORT QoS
        self._pose_topics = [f'/{m}/true_pose' for m in self._members]
        for topic in self._pose_topics:
            self._sub.subscribe(topic, String, qos=TELEMETRY_QOS)

        # Publish one GOTO command to the team control topic
        msg = String()
        msg.data = f'GOTO {self._goal[0]:.4f} {self._goal[1]:.4f}'
        self._pub.publish(self._control_topic, msg)
        Logger.loginfo(
            f'TeamGoto: {self._team_name} → ({self._goal[0]:.2f}, {self._goal[1]:.2f}) '
            f'tol={self._tolerance:.2f} timeout={self._timeout:.1f}s '
            f'members={len(self._members)}'
        )

    def execute(self, userdata):
        if not self._members:
            return 'failed'

        try:
            from flexbe_core.core.ros_state import RosState
            elapsed = (RosState._node.get_clock().now() - self._start_time).nanoseconds / 1e9

            arrived_ids = []
            no_pose_ids = []        # 永远没收到过 true_pose
            far_ids = []             # 收到了 pose 但还没到达
            for mid, topic in zip(self._members, self._pose_topics):
                pose = self._read_pose(topic)
                if pose is None:
                    no_pose_ids.append(mid)
                    continue
                dx = pose[0] - self._goal[0]
                dy = pose[1] - self._goal[1]
                if math.hypot(dx, dy) <= self._tolerance:
                    arrived_ids.append(mid)
                else:
                    far_ids.append((mid, math.hypot(dx, dy)))

            pending_count = len(no_pose_ids) + len(far_ids)
            if pending_count == 0:
                Logger.loginfo(f'TeamGoto: all {len(arrived_ids)} members arrived')
                return 'arrived'

            if elapsed > self._timeout:
                Logger.logwarn(
                    f'TeamGoto: timeout after {elapsed:.1f}s; '
                    f'arrived={len(arrived_ids)}/{len(self._members)} '
                    f'no_pose={len(no_pose_ids)} far={len(far_ids)}'
                )
                if no_pose_ids:
                    Logger.logwarn(f'  no_pose ids: {no_pose_ids}')
                if far_ids:
                    far_sorted = sorted(far_ids, key=lambda t: t[1], reverse=True)
                    Logger.logwarn(f'  far ids (id, dist): {far_sorted[:10]}')
                return 'failed'
        except Exception as exc:
            Logger.logerr(f'TeamGotoState.execute error: {exc}')
            return 'failed'

        return None

    def on_exit(self, userdata):
        try:
            stop = String()
            stop.data = 'STOP'
            self._pub.publish(self._control_topic, stop)
        except Exception as exc:
            Logger.logerr(f'TeamGotoState.on_exit stop failed: {exc}')

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #
    @staticmethod
    def _parse_members(raw):
        """Accept JSON array or Python list literal or comma-sep string."""
        if isinstance(raw, (list, tuple)):
            return [str(m) for m in raw]
        s = str(raw).strip()
        if not s:
            return []
        try:
            parsed = json.loads(s)
            if isinstance(parsed, list):
                return [str(m) for m in parsed]
        except json.JSONDecodeError:
            pass
        # Fallback: comma-separated
        return [p.strip() for p in s.split(',') if p.strip()]

    def _read_pose(self, topic):
        try:
            if not self._sub.has_msg(topic):
                return None
            msg = self._sub.get_last_msg(topic)
            data = json.loads(msg.data)
            return (float(data['x']), float(data['y']))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return None
