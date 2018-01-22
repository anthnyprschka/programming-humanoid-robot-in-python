'''In this exercise you need to implement the PID controller for joints of robot.

* Task:
    1. complete the control function in PIDController with prediction
    2. adjust PID parameters for NAO in simulation

* Hints:
    1. the motor in simulation can simple modelled by angle(t) = angle(t-1) + speed * dt
    2. use self.y to buffer model prediction
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)),
    '..', 'software_installation'))

# Classic numpy
import numpy as np

# Importing deque
from collections import deque

# Importing both SparkAgent and JOING_CMD_NAMES
from spark_agent import SparkAgent, JOINT_CMD_NAMES


class PIDController(object):
    '''a discretized PID controller, it controls an array of servos,
       e.g. input is an array and output is also an array
    '''
    def __init__(self, dt, size):
        '''
        @param dt: step time
        @param size: number of control values
        @param delay: delay in number of steps
        '''
        self.dt = dt
        self.u = np.zeros(size)
        self.e1 = np.zeros(size)
        self.e2 = np.zeros(size)
        # ADJUST PARAMETERS BELOW
        delay = 0
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.y = deque(np.zeros(size), maxlen=delay + 1)

    def set_delay(self, delay):
        '''
        @param delay: delay in number of steps
        '''
        self.y = deque(self.y, delay + 1)

    def control(self, target, sensor):
        '''apply PID control
        @param target: reference values
        @param sensor: current values from sensor
        @return control signal
        '''
        # YOUR CODE HERE

        return self.u


class PIDAgent(SparkAgent):

    # Note that when instantiating this agent I need to pass all the low level args
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):

        # Instantiating this agent calls SparkAgent's __init__ function
        super(PIDAgent, self).__init__(simspark_ip, simspark_port,
            teamname, player_id, sync_mode)

        # Getting the names of the joints
        self.joint_names = JOINT_CMD_NAMES.keys()

        # Getting the number of joints (used right below)
        number_of_joints = len(self.joint_names)

        # Here I am instantiating a PIDController, defined above
        self.joint_controller = PIDController(dt=0.01, size=number_of_joints)

        # Setting all target joints to 0
        self.target_joints = {k: 0 for k in self.joint_names}


    # Enhance SparkAgent.think() method
    def think(self, perception):

        # What is SparkAgent.think method doing?
        action = super(PIDAgent, self).think(perception)

        # calculate control vector (speeds) from perception.joint, self.target_joints
        joint_angles = np.asarray(
            [perception.joint[joint_id]  for joint_id in JOINT_CMD_NAMES])
        target_angles = np.asarray([self.target_joints.get(joint_id,
            perception.joint[joint_id]) for joint_id in JOINT_CMD_NAMES])

        # Here, PIDController.control() method is called to determine this timestep's actions
        u = self.joint_controller.control(target_angles, joint_angles)

        # Prepare for return
        action.speed = dict(zip(JOINT_CMD_NAMES.iterkeys(), u))  # dict: joint_id -> speed
        return action


if __name__ == '__main__':

    # Instantiating a PIDAgent (that is a child of SparkAgent)
    agent = PIDAgent()

    # Why are we setting this to 1.0? All others are 0.0
    agent.target_joints['HeadYaw'] = 1.0

    # Running the agent
    agent.run()
