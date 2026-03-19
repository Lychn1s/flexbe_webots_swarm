#!/usr/bin/env python3
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from geometry_msgs.msg import Twist

class TeamMoveState(EventState):
    """
    控制整个集群移动

    -- team_name    string    集群名称 ('scout' 或 'carrier')
    -- linear_x     float     前进速度
    -- angular_z    float     转向速度
    -- duration     float     持续时间（秒）

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
        self._topic = f'/{team_name}_team/cmd_vel'

        # 使用 ProxyPublisher
        self._pub = ProxyPublisher({self._topic: Twist})

    def on_enter(self, userdata):
        """进入状态时执行"""
        from flexbe_core.core.ros_state import RosState
        self._start_time = RosState._node.get_clock().now()
        Logger.loginfo(f'{self._team_name} team moving: linear={self._linear_x}, angular={self._angular_z}')

    def execute(self, userdata):
        """循环执行"""
        try:
            from flexbe_core.core.ros_state import RosState
            # 发布速度指令
            cmd = Twist()
            cmd.linear.x = float(self._linear_x)
            cmd.angular.z = float(self._angular_z)
            self._pub.publish(self._topic, cmd)

            # 检查是否超时
            elapsed = (RosState._node.get_clock().now() - self._start_time).nanoseconds / 1e9
            if elapsed >= self._duration:
                return 'done'

        except Exception as e:
            Logger.logerr(f'Error in execute: {e}')
            return 'failed'

        return None

    def on_exit(self, userdata):
        """退出状态时停止机器人"""
        try:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self._pub.publish(self._topic, cmd)
            Logger.loginfo(f'{self._team_name} team stopped')
        except Exception as e:
            Logger.logerr(f'Error stopping team: {e}')
