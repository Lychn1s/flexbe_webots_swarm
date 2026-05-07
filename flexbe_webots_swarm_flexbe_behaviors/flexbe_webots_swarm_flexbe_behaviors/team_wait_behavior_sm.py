#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Single-state behavior wrapping TeamWaitState for PyC2 dispatch."""

from flexbe_core import Autonomy, Behavior, OperatableStateMachine, initialize_flexbe_core
from flexbe_webots_swarm_flexbe_states.team_wait_state import TeamWaitState


class TeamWaitBehaviorSM(Behavior):
    """Simple wait behavior."""

    def __init__(self, node):
        super().__init__()
        self.name = 'Team Wait Behavior'

        self.add_parameter('duration', 1.0)

        initialize_flexbe_core(node)

    def create(self):
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        with _state_machine:
            OperatableStateMachine.add(
                'TeamWait',
                TeamWaitState(duration=self.duration),
                transitions={'done': 'finished'},
                autonomy={'done': Autonomy.Off},
            )

        return _state_machine
