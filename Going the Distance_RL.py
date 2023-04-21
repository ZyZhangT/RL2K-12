from spike.control import wait_for_seconds
import random
from spike import Motor, DistanceSensor, MotorPair, PrimeHub, ForceSensor


class Fetch():
    # Example code for class 4
    def __init__(self):
        # Initialize variables
        self.hub = PrimeHub()
        self.motors = MotorPair('A', 'B')
        self.current_feedback = None
        self.last_feedback = None
        self.reduce_coefficient = 1
        self.distance = None
        self.time = []
        self.t1 = None
        self.t2 = None
        self.testt = None

    def test(self):
        while True:
            print(self.us.get_distance_cm())
            wait_for_seconds(0.1)

    def spike_move_forward(self, time):
        self.motors.move(time, unit='seconds', speed=50)

    def spike_move_backward(self, time):
        self.motors.move(time, unit='seconds', speed=-50)

    def generate_linear_model(self, Input, Output):
        n = len(Input)
        sumxy = 0
        sumx = 0
        sumy = 0
        sumSquarex = 0
        for a in range(len(Input)):
            sumxy += Input[a] * Output[a]
        for i in Input:
            sumx += i
        for j in Output:
            sumy += j
        for k in Input:
            sumSquarex += pow(k, 2)
        self.w1 = (n * sumxy - sumx * sumy) / (n * sumSquarex - pow(sumx, 2))
        self.w0 = (sumSquarex * sumy - sumx * sumxy) / (n * sumSquarex - pow(sumx, 2))
        print(self.w1, self.w0)
        return self.w1, self.w0

    def manual_rl(self, training_distance=[10, 20, 30]):
        self.fs = ForceSensor('F')
        distance = training_distance
        k = 0.1
        train_step = 1
        reduce_coefficient = 1
        feedback = None
        current_feedback = None
        last_feedback = None
        print(1)
        for i in distance:
            # Generate ramdom running time
            self.hub.light_matrix.write('Ready')
            self.fs.wait_until_pressed()
            self.t1 = random.randrange(1, 3)
            finish = 0
            # Convert target distance to target sensor data
            while True:
                print('t=', self.t1)
                # Try to move to first training position
                self.spike_move_forward(self.t1)
                wait_for_seconds(0.5)
                self.hub.light_matrix.write('CHOOSE REWARD')
                # Determine whether the spike moved too far, too close,
                # or stoped accurately
                while True:
                    if self.hub.left_button.is_pressed():
                        self.hub.light_matrix.show_image('ARROW_S')
                        current_feedback = 1
                        break
                    elif self.hub.right_button.is_pressed():
                        current_feedback = -1
                        self.hub.light_matrix.show_image('ARROW_N')
                        break
                    elif self.fs.is_pressed():
                        self.hub.light_matrix.show_image('HAPPY')
                        finish = 1
                        self.hub.speaker.beep(seconds=1)
                        # Record time t when it accurately arrive the target position
                        self.time.append(self.t1)
                        break
                wait_for_seconds(2)
                self.hub.light_matrix.off()
                if finish == 1:
                    break
                # Modify the runninng time based on the feedback generated above.
                if train_step == 1:
                    last_feedback = current_feedback
                    self.t1 -= current_feedback
                    train_step += 1
                elif train_step != 1:
                    current_feedback = current_feedback / reduce_coefficient
                    if current_feedback * last_feedback > 0:
                        self.t1 -= current_feedback
                        last_feedback = current_feedback
                        train_step += 1
                    # If last feedback is opposite to current feedback, half the
                    # time that changes each step
                    if current_feedback * last_feedback < 0:
                        reduce_coefficient = reduce_coefficient * 2
                        current_feedback = current_feedback / reduce_coefficient
                        self.t1 -= current_feedback
                        last_feedback = current_feedback
                        train_step += 1
                self.fs.wait_until_pressed()
                wait_for_seconds(0.5)

    def auto_model_training(self, training_distance=[10, 50, 60, 20, 40],
                            test_distance=[30, 20],
                            model_based_training_size=3):
        print('unplug your Spike and press left button to start')
        wait_for_seconds(1)
        distance = training_distance
        time = []
        self.t1 = 0
        self.t2 = 0
        self.us = DistanceSensor('E')
        k = 0.05
        self.hub.light_matrix.write('READY')
        for i in range(len(distance)):
            self.hub.left_button.wait_until_pressed()
            # Generate ramdom running time
            self.t1 = random.randrange(1, 3)
            train_step = 1
            if self.us.get_distance_cm() == None:
                wait_for_seconds(1)
                self.hub.light_matrix.show_image('SAD')
                while not self.hub.left_button.is_pressed():
                    self.hub.light_matrix.off()
                    pass
            print(self.us.get_distance_cm())
            # Convert target distance to target sensor data
            target = self.us.get_distance_cm() - distance[i]
            print('start=', self.us.get_distance_cm())
            print('target=', target)
            while True:
                print('t=', self.t1)
                # Try to move to first training position
                self.spike_move_forward(self.t1)
                wait_for_seconds(1)
                print(self.us.get_distance_cm())
                # Determine whether the spike moved too far, too close,
                # or stoped accurately
                if self.us.get_distance_cm() == None:
                    self.hub.light_matrix.show_image('SAD')
                    wait_for_seconds(1)
                    while not self.hub.left_button.is_pressed():
                        self.hub.light_matrix.off()
                        pass
                if self.us.get_distance_cm() == target:
                    self.hub.speaker.beep(seconds=1)
                    wait_for_seconds(1)
                    self.spike_move_backward(self.t1)
                    # Record time t when it accurately arrive the target position
                    time.append(self.t1)
                    break
                else:
                    reward = k * (self.us.get_distance_cm() - target)
                    wait_for_seconds(1)
                self.spike_move_backward(self.t1)
                wait_for_seconds(1)
                self.t1 += reward
            self.hub.light_matrix.write('NEXT NEXT')
        # Generate linear model with the time data recorded above and the
        # corresponded distance
        self.generate_linear_model(distance, time)
        self.hub.light_matrix.show_image('HAPPY')
        self.hub.left_button.wait_until_pressed()
        for i in test_distance:
            self.testt = self.w1 * i + self.w0
            self.spike_move_forward(self.testt)
            wait_for_seconds(0.5)
            self.hub.speaker.beep(seconds=1)
            self.spike_move_backward(self.testt)
            wait_for_seconds(0.5)
            print(i, self.testt)
        print(time)
        print(distance)

    def start_auto_training(self, training_distance=10):
        print('unplug your Spike and press left button to start')
        wait_for_seconds(1)
        self.distance = training_distance
        self.time = []
        self.finish = False
        Sum = 0
        self.t1 = 0
        self.t2 = 0
        self.us = DistanceSensor('E')
        self.k = 0.05
        self.hub.light_matrix.write('READY')

    def random_choose_initial_run_time(self):
        self.hub.left_button.wait_until_pressed()
        # Generate ramdom running time
        self.t1 = random.randrange(1, 3)
        if self.us.get_distance_cm() == None:
            wait_for_seconds(1)
            self.hub.light_matrix.show_image('SAD')
            while not self.hub.left_button.is_pressed():
                self.hub.light_matrix.off()
                pass
        print(self.us.get_distance_cm())
        # Convert target distance to target sensor data
        self.target = self.us.get_distance_cm() - self.distance
        print('start=', self.us.get_distance_cm())
        print('target=', self.target)

    def robot_make_a_try(self):
        print('t=', self.t1)
        # Try to move to first training position
        self.spike_move_forward(self.t1)
        wait_for_seconds(1)
        print(self.us.get_distance_cm())
        # Determine whether the spike moved too far, too close,
        # or stoped accurately
        if self.us.get_distance_cm() == None:
            self.hub.light_matrix.show_image('SAD')
            wait_for_seconds(1)
            while not self.hub.left_button.is_pressed():
                self.hub.light_matrix.off()
                pass

    def check_if_accurately_arrive(self):
        if self.us.get_distance_cm() == self.target:
            self.hub.speaker.beep(seconds=1)
            wait_for_seconds(1)
            # Record time t when it accurately arrive the target position
            self.time.append(self.t1)
            self.finish = True
            self.hub.light_matrix.show_image('HAPPY')

    def give_robot_feedback(self):
        self.reward = self.k * (self.us.get_distance_cm() - self.target)
        wait_for_seconds(1)

    def modify_running_time(self):
        self.spike_move_backward(self.t1)
        wait_for_seconds(1)
        self.t1 += self.reward


robot = Fetch()

# Manually give robot feedback to help it train itself
# robot.manual_rl(training_distance=[10,20])

# Watch how robot automatically train itself to reach several distances and generate a model to predict
# how long the motors should run to reach a given distance.
# robot.auto_model_training(training_distance=[10,15],test_distance=[20])

# Correctly replace question marks with folllowing 6 functions to program a robot to automatically
# train itself reach a given distance
# robot.start_auto_training(), robot.random_choose_initial_run_time(), robot.robot_make_a_try(), robot.check_if_accurately_arrive()
# robot.give_robot_feedback(), robot.modify_running_time()

# Two functions should be put before the while loop
# ?
# ?
# while robot.finish!=True:
# four functions should be put after the while loop
# ?
# ?
# ?
# ?