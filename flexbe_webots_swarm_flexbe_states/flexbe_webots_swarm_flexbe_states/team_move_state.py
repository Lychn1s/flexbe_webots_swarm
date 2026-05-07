#!/usr/bin/env python3
"""Legacy TeamMove — timed wait against Webots' String/GOTO control surface.

The original version published geometry_msgs/Twist to /{team}_team/cmd_vel,
but Webots' smart_agent_controller never subscribed to cmd_vel — it only
listens on /{team}_team/control for std_msgs/String (GOTO x y | STOP | …).

The velocity/duration semantics therefore cannot be honored 1:1: we can't
stream velocities into the Webots controller. The state is kept for
backwards compatibility with existing behaviors (SwarmMoveExample1), but
it now degrades to "wait `duration` seconds, then publish STOP" — the
previous behavior (publishing Twist into a void) is gone.

New tactics should use TeamGotoState or FormationGotoState.
"""
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String


class TeamMoveState(EventState):
    """
    Legacy team move — timed wait + STOP on /{team}_team/control.

    -- team_name    string    集群名称 ('scout' 或 'carrier')
    -- linear_x     float     (保留字段，不再影响运动；Webots 不接 cmd_vel)
    -- angular_z    float     (保留字段，不再影响运动)
    -- duration     float     等待时间（秒），到点后对该 team 发一次 STOP

    <= done                   完成
    <= failed                 失败
    """

    def __init__(self, team_name, linear_x, angular_z, duration):
        super(TeamMoveState, self).__init__(outcomes=['done', 'failed'])

        self._team_name = team_name
        self._linear_x = linear_x
        self._angular_z = angular_z
        self._duration = duration
        self._start_time = None
        self._topic = f'/{team_name}_team/control'

        self._pub = ProxyPublisher({self._topic: String})

    def on_enter(self, userdata):
        from flexbe_core.core.ros_state import RosState
        self._start_time = RosState._node.get_clock().now()
        Logger.loginfo(
            f'{self._team_name} team move (legacy timed wait): '
            f'linear={self._linear_x}, angular={self._angular_z}, duration={self._duration}s'
        )

    def execute(self, userdata):
        try:
            from flexbe_core.core.ros_state import RosState
            elapsed = (RosState._node.get_clock().now() - self._start_time).nanoseconds / 1e9
            if elapsed >= self._duration:
                return 'done'
        except Exception as exc:
            Logger.logerr(f'Error in TeamMoveState.execute: {exc}')
            return 'failed'

        return None

    def on_exit(self, userdata):
        try:
            stop_msg = String()
            stop_msg.data = 'STOP'
            self._pub.publish(self._topic, stop_msg)
            Logger.loginfo(f'{self._team_name} team STOP sent')
        except Exception as exc:
            Logger.logerr(f'Error stopping team: {exc}')
