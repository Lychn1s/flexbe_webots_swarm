#!/usr/bin/env python3
from flexbe_core import EventState, Logger

class TeamWaitState(EventState):
    """
    等待指定时间
    
    -- duration     float     等待时间（秒）
    
    <= done                   完成
    """
    
    def __init__(self, duration):
        super(TeamWaitState, self).__init__(outcomes=['done'])
        self._duration = duration
        self._start_time = None
        
    def on_enter(self, userdata):
        from flexbe_core.core.ros_state import RosState
        self._start_time = RosState._node.get_clock().now()
        Logger.loginfo(f'Waiting for {self._duration} seconds')

    def execute(self, userdata):
        from flexbe_core.core.ros_state import RosState
        elapsed = (RosState._node.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed >= self._duration:
            Logger.loginfo('Wait completed')
            return 'done'
        return None
