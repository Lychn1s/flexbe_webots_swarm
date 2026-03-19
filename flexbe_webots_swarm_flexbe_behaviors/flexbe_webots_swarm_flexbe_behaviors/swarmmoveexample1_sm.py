#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 ZKW
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.

#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Swarm Move Example 1.

scout集群往前走2秒之后停留2秒，之后Scout集群往左走3秒，再停留1秒，最后Carrier集群往前走2秒

Created on Thu Mar 19 2026
@author: ZKW
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from flexbe_webots_swarm_flexbe_states.team_move_state import TeamMoveState
from flexbe_webots_swarm_flexbe_states.team_wait_state import TeamWaitState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class SwarmMoveExample1SM(Behavior):
    """
    Define Swarm Move Example 1.

    scout集群往前走2秒之后停留2秒，之后Scout集群往左走3秒，再停留1秒，最后Carrier集群往前走2秒
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'Swarm Move Example 1'

        # parameters of this behavior

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]


        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:1476 y:203, x:1173 y:52
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        with _state_machine:
            # x:87 y:71
            OperatableStateMachine.add('ScoutMoveForward2Sec',
                                       TeamMoveState(team_name='scout',
                                                     linear_x=0.3,
                                                     angular_z=0.0,
                                                     duration=2.0),
                                       transitions={'done': 'ScoutWait2Sec',
                                                    'failed': 'failed'  # 610 59 -1 -1 -1 -1
                                                    },
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:1068 y:228
            OperatableStateMachine.add('CarrierMoveForward2Sec',
                                       TeamMoveState(team_name='carrier',
                                                     linear_x=0.3,
                                                     angular_z=0.0,
                                                     duration=2.0),
                                       transitions={'done': 'finished', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:442 y:145
            OperatableStateMachine.add('ScoutTurnLeft3Sec',
                                       TeamMoveState(team_name='scout',
                                                     linear_x=0.0,
                                                     angular_z=0.3,
                                                     duration=3.0),
                                       transitions={'done': 'ScoutWait1Sec', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:764 y:233
            OperatableStateMachine.add('ScoutWait1Sec',
                                       TeamWaitState(duration=1.0),
                                       transitions={'done': 'CarrierMoveForward2Sec'},
                                       autonomy={'done': Autonomy.Off})

            # x:156 y:297
            OperatableStateMachine.add('ScoutWait2Sec',
                                       TeamWaitState(duration=2.0),
                                       transitions={'done': 'ScoutTurnLeft3Sec'},
                                       autonomy={'done': Autonomy.Off})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
