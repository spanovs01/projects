#!/usr/bin/env python3

from numpy.lib.scimath import arctanh, sqrt
import math
import rospy
import time
import numpy as np
from geometry_msgs.msg import Point
'''
from core.srv import ModelService, ServoService, WalkService, ImuService
'''

class Basketball:
    step_length = 32
    side_length = 20
    angle = 0.05
    step_time = 1   #I DON'T KNOW
    side_time = 1   #I DON'T KNOW
    step_time_koef = step_time / step_length
    side_time_koef = side_time / side_length
    '''
    Go Forward -> x
    Go Sideways -> y
    '''

    dead_distance = 0.1

    def __init__(self):
        self.ball = (None, None)                        #coordinates of pixels of ball
        self.basketcase = (None, None)                  #coordinates of pixels of basketcase
        self.ball_self = (None, None)                   #coordinates of ball relatively to the atom
        self.basketcase_self = (None, None)             #coordinates of basketcase relatively to the atom
        self.last_seen_ball = 0
        self.last_seen_basketcase = 0
        self.camera_pan = 0                             #moving head left and right
        self.camera_tilt = 0                            #moving head up and down
        self.flag_ball = False                          #Have we found a ball?
        self.flag_basketcase = False                    #Have we found a basketcase?
        self.distance_to_ball = None                    #relatively to the atom
        self.distance_to_basketcase = None              #relatively to the atom
        self.basketcase_linia = 0                       #tg between basketcase coordinates (y/x)
        self.angle_thinking = 0                         #angle between the direction to ball and to basketcase
        self.ball_coordinates = (0, 0)                  #coordinates of mediana of found balls
        self.basketcase_coordinates = (None, None)      #coordinates of mediana of found basketcases
        self.mediana_ball_distance = None
        self.mediana_basketcase_distance = None



    def update_ball(self, msg):
        self.ball = (msg.x, msg.y)
        self.last_seen_ball = time.time()
        if self.ball != (None, None):
            print(f"Got the ball: {self.ball[0]}, {self.ball[1]}")
            self.ball_self = self.get_ball_distance()
            self.distance_to_ball = sqrt(self.ball_self[0] ** 2 + self.ball_self[1] ** 2)
            #print(f"Got the ball in self coords: {self.ball_self[0]}, {self.ball_self[1]}")


    def update_basketcase(self, msg):
        self.basketcase = (msg.x, msg.y)
        self.last_seen_basketcase = time.time()
        if self.basketcase != (None, None):
            #print(f"Got the basketcase: {self.basketcase[0]}, {self.basketcase[1]}")
            self.basketcase_self = self.get_basketcase_distance()
            self.distance_to_basketcase = sqrt(self.basketcase_self[0] ** 2 + self.basketcase_self[1] ** 2)
            #print(f"Got the basketcase in self coords: {self.basketcase_self[0]}, {self.basketcase_self[1]}")


    def get_ball_distance(self):
        rospy.wait_for_service("model_service")
        try:
            get_coords = rospy.ServiceProxy("model_service", ModelService)
            #print(self.ball[0], self.ball[1], self.camera_pan, self.camera_tilt)
            response = get_coords(int(self.ball[0]), int(self.ball[1]), self.camera_pan, self.camera_tilt, 0.32)
            return (response.x, response.y)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def get_basketcase_distance(self):
        rospy.wait_for_service("model_service")
        try:
            get_coords = rospy.ServiceProxy("model_service", ModelService)
            #print(self.basketcase[0], self.basketcase[1], self.camera_pan, self.camera_tilt)
            response = get_coords(int(self.basketcase[0]), int(self.basketcase[1]), self.camera_pan, self.camera_tilt, 0.32)
            return (response.x, response.y)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def radians_search(frequency):
        res = list(zip(np.zeros(frequency), np.linspace(-np.pi/2, np.pi/2, frequency))) + list(zip(np.zeros(frequency) + np.pi/4, np.linspace(np.pi/2, -np.pi/2, frequency)))
        return res + [[0, 0]]

    def finding_ball(self):
        print("Start finding ball")
        radians = Basketball.radians_search(5)
        l = []
        while len(l) < 2:
            for elem in radians:
                self.move_head(elem[0], elem[1])
                time.sleep(3)
                if self.ball != (None, None):
                    self.ball_self = self.get_ball_distance()
                    print(self.ball_self)
                    l.append(self.ball_self)
            # choose the mediana of all balls in coordinates of robor and look at dispersy, check massive of all finded bolls
                else:
                    print("GOVNO")
        self.ball_coordinates = tuple(np.median(np.array(l), axis = 0))
        print("MEDIANA:")
        print(self.ball_coordinates)
        if self.ball_coordinates != (None, None):
            self.flag_ball = True
            self.mediana_ball_distance = sqrt(self.ball_coordinates[0] ** 2 + self.ball_coordinates[1] ** 2)

    def finding_basketcase(self):
        print("Start finding basketcase")
        radians = Basketball.radians_search(5)
        l = []
        while len(l) < 2:
            for elem in radians:
                self.move_head(elem[0], elem[1])
                time.sleep(1)
                print("it worked")
                if self.basketcase != (None, None):
                    self.basketcase_self = self.get_basketcase_distance()
                    l.append(self.basketcase_self)
            # choose the mediana of all balls in coordinates of robor and look at dispersy, check massive of all finded bolls
                else:
                    print("GOVNO")
        self.basketcase_coordinates = tuple(np.median(np.array(l), axis = 0))
        print(self.basketcase_coordinates)
        if self.basketcase_coordinates != (None, None):
            self.flag_basketcase = True
            self.mediana_basketcase_distance = sqrt(self.basketcase_coordinates[0] ** 2 + self.basketcase_coordinates[1] ** 2)


    def move_head(self, head_pitch = 0., head_yaw = 0.):
        self.camera_pan = head_yaw
        self.camera_tilt = head_pitch
        self.servos_client(["head_yaw", "head_pitch"], [float(head_yaw), float(head_pitch)])

    @staticmethod
    def servos_client(names, positions):
        rospy.wait_for_service('servo_service')
        try:
            servos_service = rospy.ServiceProxy('servo_service', ServoService)
            servos_service(names, positions)
        except rospy.ServiceException as e:
            print("Service call failed:", e)


    def turn_to_ball(self):
        print("Start turning to ball")
        # rotation = np.arctan(self.ball_self[1] / self.ball_self[0])
        degree = np.arctan(self.ball_coordinates[1] / self.ball_coordinates[0]) * 180 / np.pi
        print(str(degree))
        self.rotate(degree)
        time.sleep(1)

    def turn_to_basketcase(self):
        print("Start turning to basketcase")
        # rotation = np.arctan(self.basketcase_self[1] / self.basketcase_self[0])
        '''     Old Logic
        rotation = np.arctan(self.basketcase_coordinates[1] / self.basketcase_coordinates[0])
        time_rotate = (math.fabs(rotation)) * Basketball.const_angle
        self.walk(True, 0, 0, Basketball.angle)
        time.sleep(time_rotate)
        self.walk(False, 0, 0, 0)
        '''
        degree = np.arctan(self.basketcase_coordinates[1] / self.basketcase_coordinates[0]) * 180 / np.pi
        print(str(degree))
        self.rotate(degree)
        time.sleep(1)


    @staticmethod
    def walk(walk_enabled, step_length, side_length, angle):
        print("before waiting for service")
        rospy.wait_for_service("walk_service")
        print("work service in depression")
        try: 
            walk_service = rospy.ServiceProxy('walk_service', WalkService)
            walk_service(walk_enabled, step_length, side_length, angle)
            print("Atom is going")
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def go_to_ball(self, percent_distance):
        print("Start going to ball")
        time_walk = percent_distance * (self.mediana_ball_distance) * Basketball.step_time_koef
        print(self.distance_to_ball, time_walk)
        self.walk(True, Basketball.step_length, 0, 0)
        time.sleep(time_walk)
        self.walk(False, 0, 0, 0)
        time.sleep(1)

    def go_to_ball_try(self):
        print("Start going to ball")
        time_walk = 15
        self.walk(True, 10, 0, 0)
        time.sleep(time_walk)
        # self.ball_coordinates
        self.walk(False, 0, 0, 0)
        time.sleep(1)

    def go_to_basketcase(self, percent_distance):
        print("Start going to basketcase")
        time_walk = percent_distance * (self.mediana_basketcase_distance) * Basketball.step_time_koef
        print(self.distance_to_ball, time_walk)
        self.walk(True, Basketball.step_length, 0, 0)
        time.sleep(time_walk)
        self.walk(False, 0, 0, 0)
        time.sleep(1)

    def thinking_take(self):
        print("Start thinking and taking ball")
        self.basketcase_linia = ((self.basketcase_coordinates[0]) / (self.basketcase_coordinates[1]))
        self.angle_thinking = np.pi/2 - arctanh(self.basketcase_linia)
        if self.ball_self != (None, None):
            self.distance = (math.cos(self.angle_thinking)) * sqrt(self.ball_coordinates[0] ** 2 + self.ball_coordinates[1] ** 2)
        if math.fabs(self.distance) > 0.16:
            # take the ball
            #self.motion.play_Soft_Motion_Slot(name = 'Ball_tennis_throw_v7_2')
            # pox, it won't bring down the holder
            return 0
        elif self.distance < 0:
            # take the ball
            #self.motion.play_Soft_Motion_Slot(name = 'Ball_tennis_throw_v7_2')
            # 3 steps right
            return 1
            time_walk = 3 * Basketball.step_time_koef
            self.walk(True, 0, Basketball.side_length, 0)
            time.sleep(time_walk)
            self.walk(False, 0, 0, 0)
        elif self.distance >= 0:
            # take the ball
            #self.motion.play_Soft_Motion_Slot(name = 'Ball_tennis_throw_v7_2')
            # 3 steps left
            return -1
            time_walk = 3 * Basketball.step_time_koef
            self.walk(True, 0, Basketball.side_length, 0)
            time.sleep(time_walk)
            self.walk(False, 0, 0, 0)

    def imu_client(self):
        rospy.wait_for_service('imu_service')
        try:
            a = rospy.ServiceProxy('imu_service', ImuService)
            return a()
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def walk_client(self, n, step, side, ang):
        rospy.wait_for_service('walk_service')
        try:
            a = rospy.ServiceProxy('walk_service', WalkService)
            a(n, step, side, ang)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def rotate(self, to_rotate_deg):

        imu_start = self.imu_client().x
        imu_end = imu_start - to_rotate_deg
        imu_end %= 360
        rotation = 0.2
        print(to_rotate_deg)

        while True:
            imu = self.imu_client()
            print("imu: ", imu.x)
            print("imu end: ", imu_end)
            if np.abs(imu.x - imu_end) < 3:
                break
            
            if imu.x - imu_end > 0:
                if math.fabs(imu.x - imu_end) <= 180:
                    self.walk_client(True, 0, 0, rotation)
                else:
                    self.walk_client(True, 0, 0, -rotation)
            if  imu.x - imu_end <= 0:
                print("Negative")
                if math.fabs(imu.x - imu_end) <= 180:
                    self.walk_client(True, 0, 0, -rotation)
                else:
                    self.walk_client(True, 0, 0, rotation)
            
        self.walk_client(False, 0, 0, 0)


if __name__ == "__main__":
    rospy.init_node("basketball")
    basketball = Basketball()
    # ball_sub = rospy.Subscriber('ball', Point, basketball.update_ball)
    # basketcase_sub = rospy.Subscriber('basketcase', Point, basketball.update_basketcase)

    basketball.go_to_ball_try()
    # Finding ball and putting it to self.ball_coordinates for future approach.
    # while not (basketball.flag_ball):
    #     basketball.finding_ball()
    # print(basketball.flag_ball)
    
    #Approaching to the ball on 0.8 of distance
    # basketball.flag_ball = False
    # basketball.turn_to_ball()
    # basketball.go_to_ball(0.8)
    
    # Correct the ball position
    # while not (basketball.flag_ball): 
    #     basketball.finding_ball()
    # print(basketball.flag_ball)
    
    # Finally approach a ball
    # basketball.flag_ball = False
    # basketball.turn_to_ball()
    # basketball.go_to_ball(1)
    
    # Searching for basket
    # while not (basketball.flag_basketcase):
    #     basketball.finding_basketcase()
    # print(basketball.flag_basketcase)
    
    # # Take ball and do several steps from ball holder
    # basketball.flag_basketcase = False
    # basketball.thinking_take()
    
    # # Searching for basket again
    # while not (basketball.flag_basketcase):
    #     basketball.finding_basketcase()
    # print(basketball.flag_basketcase)
    
    # # Approach basket first time
    # basketball.flag_basketcase = False
    # basketball.turn_to_basketcase()
    # basketball.go_to_basketcase(0.8)
    
    # # Searching for basket again
    # while not (basketball.flag_basketcase):
    #     basketball.finding_basketcase()
    # print(basketball.flag_basketcase)
    
    # # Finally approach basket
    # basketball.flag_basketcase  = False
    # basketball.go_to_basketcase(1)
    # # put the ball into basketcase
    print("HE HIT THE BALL OR NO. I don't NO")
    # while True:
    #     n = int(input())
    #     basketball.go_to_ball(1, n)