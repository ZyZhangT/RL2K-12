from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
import random
from hub import port
import time

# Initialize SPIKE hub and motors
spike = PrimeHub()
R = port.A.motor
L = port.B.motor
state = 0
# Define the state space of your learning agent. In this example, the robot could be
# in any one of the 5 states based on it's facing direction.
StateSpace = [-2,-1,0,1,2]
def setstate():
    angle = spike.motion_sensor.get_yaw_angle()
    if angle < -50:
        state = -2
    if angle > 50:
        state = 2
    if -50 <= angle <= -15 :
        state = -1
    if 15 <= angle <= 50 :
        state = 1
    if -15 < angle < 15:
        state = 0
    return state

# Define action space. When an action is chosen, the speed of two motors would be set accordingly.
ActionSpace = [[20,-45],[20,-60],[40,-40],[60,-20],[45,-20]]

# Epsilon-greedy policy is commonly used in RL for a learning agent to choose actions.
# Epsilon represents the probability of agent choosing random actions to explore the environment (exploration).
# Otherwise, the agent would choose the action it currently thinks to be the best one (exploitation),
# which is the action that has highest q-value in current state.
epsilon = 0.9
def greedy(state):
    k = random.random()
    if epsilon > k:
        action = ActionSpace[random.randint(0, 4)]
    else:
        stateindex = StateSpace.index(state)
        actionarray = qtable[stateindex]
        actionindex = actionarray.index(max(actionarray))
        action = ActionSpace[actionindex]
    return action

# When you test your learning result, the robot would only choose the action with highest
# q-value in current state (exploitation).
def testrun(state):
    stateindex = StateSpace.index(state)
    actionarray = qtable[stateindex]
    actionindex = actionarray.index(max(actionarray))
    action = ActionSpace[actionindex]
    return action

# Define reward space, when the agent arrives at a state, it would receive a corresponding reward.
RewardSpace = [-10,-2,10,-2,-10]

# Bellman Equation. This is how a q-value of one state-action pair is updated after each time step.
# Every time after you move from state A to state B, using action X and receives reward R, the
# q-value of q-table[state A, action X] will be updated. Alpha determines whether the agent should care
# more about past or new learning experience. Gamma determines whether
gamma = 0.9
alpha = 0.1
def update_q(state,action,reward,next_state):
    qvalue = qtable[StateSpace.index(state)][ActionSpace.index(action)]
    # Bellman Equation
    updateq = (1-alpha)*qvalue + alpha*(reward+gamma*max(qtable[StateSpace.index(next_state)]))
    qtable[StateSpace.index(state)][ActionSpace.index(action)] = updateq

# Drive motors with the action chosen by the agent
def move(action):
    L.run_at_speed(speed=action[0])
    R.run_at_speed(speed=action[1])

'''Main program'''
# Initilize total training steps, episodes and mode (0:training, 1:testing)
total_step = 0
episode = 0
mode = 0

# Initilize q-table with all q-value equal to 0.
# The size of the q-table is equal to (number of total states*number of total actions)
qtable = []
for i in range(5):
    qtable.append([0]*5)

# Start training
while True:
    # Show current episode on spike screen
    spike.light_matrix.write(str(episode))
    while True:
        # Choose whether you want to train or test the robot with buttons on the spike hub
        if spike.left_button.is_pressed():
            spike.light_matrix.write("TRAIN")
            print("Train")
            mode = 0
            break
        if spike.right_button.is_pressed():
            spike.light_matrix.write("TEST")
            print("Test")
            mode = 1
            break
    # Reset gyro data before start training each episode
    spike.motion_sensor.reset_yaw_angle()
    step = 0
    if mode == 1:
        step -= 10
    # In each episode, the robot will be trained 15 steps unless it fully turns around.
    while step < 10:
        # Read robot current state
        state = setstate()
        # Choose an action
        if mode == 0:
            action = greedy(state)
        if mode == 1:
            action = testrun(state)
        # Apply action
        move(action)
        time.sleep(0.5)
        # Robot moved, now read the new state of the robot
        new_state = setstate()
        print('action',action)
        print('new_state',new_state)
        # Give robot a reward based on the new state it arrives
        reward = RewardSpace[StateSpace.index(new_state)]
        print('reward',reward)
        # Update q-value
        update_q(state,action,reward,new_state)
        step += 1
        total_step += 1
        # Decrease epsilon as the training process goes, so the robot will explore more at the beginning
        # to learn the environment quickly, and then exploit its past experience more in later training phase
        # to avoid too much uncessesary exploration.
        if epsilon > 0.4:
            epsilon = 0.9 - total_step*0.008
        # If robot turned around, stop this episode
        if spike.motion_sensor.get_yaw_angle() > 160 or spike.motion_sensor.get_yaw_angle() < -160:
            break
        print('step=', step)
    L.run_for_time(0, speed=10)
    R.run_for_time(0, speed=10)
    episode += 1
    print(qtable)