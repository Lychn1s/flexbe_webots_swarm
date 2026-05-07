#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Single-state behavior wrapping FormationGotoState for PyC2 dispatch."""

from flexbe_core import Autonomy, Behavior, OperatableStateMachine, initialize_flexbe_core
from flexbe_webots_swarm_flexbe_states.formation_goto_state import FormationGotoState


class FormationGotoBehaviorSM(Behavior):
    """PyC2 → FlexBE behavior: drive a named member list to (x, y)."""

    def __init__(self, node):
        super().__init__()
        self.name = 'Formation Goto Behavior'

        self.add_parameter('x', 0.0)
        self.add_parameter('y', 0.0)
        self.add_parameter('tolerance', 0.22)
        self.add_parameter('timeout', 60.0)
        self.add_parameter('member_ids', '[]')

        initialize_flexbe_core(node)

    def create(self):
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        with _state_machine:
            OperatableStateMachine.add(
                'FormationGoto',
                FormationGotoState(
                    x=self.x,
                    y=self.y,
                    tolerance=self.tolerance,
                    timeout=self.timeout,
                    member_ids=self.member_ids,
                ),
                transitions={'arrived': 'finished', 'failed': 'failed'},
                autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
            )

        return _state_machine
