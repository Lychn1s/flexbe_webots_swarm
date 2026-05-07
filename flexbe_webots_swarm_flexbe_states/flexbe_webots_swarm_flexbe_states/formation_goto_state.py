#!/usr/bin/env python3
"""FormationGotoState — anchor+offset + per-member sub_goal arrival.

工作流程:
  1. on_enter 发 FORMATION_GOTO 给 formation_coordinator
     (coordinator 选 anchor + 算 offset + 每成员独立 A*)
  2. 订阅 /formation_coordinator/sub_goals (latched, transient_local)
     拿到 {member_id: (sx, sy)} 映射
  3. execute 周期: 每个 member 距自己 sub_goal 的距离 ≤ tolerance 才算到位
  4. 全员到位 → 'arrived'; elapsed > timeout → 'failed'

注意: tolerance 语义 = "距各自 sub_goal 的半径", 不是 "距全局目标的半径"。
推荐取值 0.15(carrier) / 0.15(scout)。
"""
import json
import math

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from flexbe_core.proxy.qos import QoSProfile
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


# supervisor 用 BEST_EFFORT
TELEMETRY_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)

# coordinator 的 sub_goals 是 latched (TRANSIENT_LOCAL)
SUB_GOALS_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

CONTROL_TOPIC = '/formation_coordinator/control'
SUB_GOALS_TOPIC = '/formation_coordinator/sub_goals'


class FormationGotoState(EventState):
    """
    Formation-coordinator-backed GOTO with per-member sub_goal arrival.

    -- x            float          全局目标 X (米)
    -- y            float          全局目标 Y (米)
    -- tolerance    float          每个成员距自己 sub_goal 的判定半径 (米)
    -- timeout      float          超时秒数
    -- member_ids   string         JSON array: '["scout_0","scout_3",...]'

    <= arrived                     全员都到了各自的 sub_goal
    <= failed                      超时 / 成员解析失败
    """

    def __init__(self, x, y, tolerance, timeout, member_ids):
        super(FormationGotoState, self).__init__(outcomes=['arrived', 'failed'])

        self._goal = (float(x), float(y))
        self._tolerance = float(tolerance)
        self._timeout = float(timeout)
        self._member_ids_raw = member_ids

        self._members = []
        self._pose_topics = []
        self._start_time = None
        self._sub_goals = {}     # member_id -> (sx, sy), 来自 coordinator
        self._sub_goals_global = None  # coordinator 报告的 global_goal, 校验用

        self._pub = ProxyPublisher({CONTROL_TOPIC: String})
        self._sub = ProxySubscriberCached()

    def on_enter(self, userdata):
        try:
            self._members = self._parse_members(self._member_ids_raw)
        except Exception as exc:
            Logger.logerr(f'FormationGoto: bad member_ids "{self._member_ids_raw}": {exc}')
            self._members = []
            return

        if not self._members:
            Logger.logwarn('FormationGoto: empty member list')

        self._sub_goals = {}
        self._sub_goals_global = None

        from flexbe_core.core.ros_state import RosState
        self._start_time = RosState._node.get_clock().now()

        # 订所有成员的 true_pose
        self._pose_topics = [f'/{m}/true_pose' for m in self._members]
        for topic in self._pose_topics:
            self._sub.subscribe(topic, String, qos=TELEMETRY_QOS)

        # 订 coordinator 的 sub_goals 映射 (latched, 后启动也能拿到当前任务的 sub_goals)
        self._sub.subscribe(SUB_GOALS_TOPIC, String, qos=SUB_GOALS_QOS)

        payload = {
            'command': 'FORMATION_GOTO',
            'goal': [self._goal[0], self._goal[1]],
            'members': list(self._members),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub.publish(CONTROL_TOPIC, msg)
        Logger.loginfo(
            f'FormationGoto → goal=({self._goal[0]:.2f}, {self._goal[1]:.2f}) '
            f'tol={self._tolerance:.2f} timeout={self._timeout:.1f}s '
            f'members={len(self._members)} (per-member sub_goal mode)'
        )

    def execute(self, userdata):
        if not self._members:
            return 'failed'

        try:
            from flexbe_core.core.ros_state import RosState
            elapsed = (RosState._node.get_clock().now() - self._start_time).nanoseconds / 1e9

            # 拉最新 sub_goals (与本任务 global_goal 一致才接受)
            self._refresh_sub_goals()

            if not self._sub_goals:
                # coordinator 还没把 sub_goals 推过来, 等
                if elapsed > self._timeout:
                    Logger.logwarn(
                        f'FormationGoto: timeout after {elapsed:.1f}s, '
                        f'never received sub_goals from coordinator'
                    )
                    return 'failed'
                return None

            arrived_ids = []
            no_pose_ids = []
            far_ids = []
            for mid, topic in zip(self._members, self._pose_topics):
                pose = self._read_pose(topic)
                sub_goal = self._sub_goals.get(mid)
                if pose is None or sub_goal is None:
                    no_pose_ids.append(mid)
                    continue
                d = math.hypot(pose[0] - sub_goal[0], pose[1] - sub_goal[1])
                if d <= self._tolerance:
                    arrived_ids.append(mid)
                else:
                    far_ids.append((mid, round(d, 3)))

            pending = len(no_pose_ids) + len(far_ids)
            if pending == 0:
                Logger.loginfo(
                    f'FormationGoto: all {len(arrived_ids)} at sub_goals '
                    f'(elapsed={elapsed:.1f}s)'
                )
                return 'arrived'

            if elapsed > self._timeout:
                Logger.logwarn(
                    f'FormationGoto: timeout after {elapsed:.1f}s; '
                    f'arrived={len(arrived_ids)}/{len(self._members)} '
                    f'no_pose={no_pose_ids} far={far_ids[:10]}'
                )
                return 'failed'
        except Exception as exc:
            Logger.logerr(f'FormationGotoState.execute error: {exc}')
            return 'failed'

        return None

    def on_exit(self, userdata):
        try:
            stop_payload = json.dumps({
                'command': 'STOP',
                'members': list(self._members),
            }, ensure_ascii=False)
            msg = String()
            msg.data = stop_payload
            self._pub.publish(CONTROL_TOPIC, msg)
        except Exception as exc:
            Logger.logerr(f'FormationGotoState.on_exit stop failed: {exc}')

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #
    def _refresh_sub_goals(self):
        """读 latched sub_goals; 校验 global_goal 与本任务一致才采用 (避免拿到上一波残留)."""
        if not self._sub.has_msg(SUB_GOALS_TOPIC):
            return
        try:
            data = json.loads(self._sub.get_last_msg(SUB_GOALS_TOPIC).data)
        except (ValueError, json.JSONDecodeError):
            return
        gg = data.get('global_goal')
        if not (isinstance(gg, list) and len(gg) == 2):
            return
        # 校验: coordinator 报告的 global_goal 与本任务的 _goal 一致(±5cm 容差)
        if math.hypot(gg[0] - self._goal[0], gg[1] - self._goal[1]) > 0.05:
            return
        sg_raw = data.get('sub_goals', {})
        if not isinstance(sg_raw, dict):
            return
        new_sub_goals = {}
        for m, p in sg_raw.items():
            if isinstance(p, list) and len(p) == 2:
                try:
                    new_sub_goals[str(m)] = (float(p[0]), float(p[1]))
                except (TypeError, ValueError):
                    continue
        if new_sub_goals:
            self._sub_goals = new_sub_goals
            self._sub_goals_global = (float(gg[0]), float(gg[1]))

    @staticmethod
    def _parse_members(raw):
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
