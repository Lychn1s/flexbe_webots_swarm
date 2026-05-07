#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Single-state behavior wrapping legacy TeamMoveState for PyC2 dispatch."""

from flexbe_core import Autonomy, Behavior, OperatableStateMachine, initialize_flexbe_core
from flexbe_webots_swarm_flexbe_states.team_move_state import TeamMoveState


class TeamMoveBehaviorSM(Behavior):
    """Legacy team_move wrapper (timed wait + STOP; linear_x/angular_z kept for schema)."""

    def __init__(self, node):
        super().__init__()
        self.name = 'Team Move Behavior'

        self.add_parameter('team_name', 'scout')
        self.add_parameter('linear_x', 0.0)
        self.add_parameter('angular_z', 0.0)
        self.add_parameter('duration', 1.0)

        initialize_flexbe_core(node)

    def create(self):
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        with _state_machine:
            OperatableStateMachine.add(
                'TeamMove',
                TeamMoveState(
                    team_name=self.team_name,
                    linear_x=self.linear_x,
                    angular_z=self.angular_z,
                    duration=self.duration,
                ),
                transitions={'done': 'finished', 'failed': 'failed'},
                autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
            )

        return _state_machine
