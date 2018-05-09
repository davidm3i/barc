#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed at UC
# Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu) and Greg Marcil (grmarcil@berkeley.edu). The cloud
# services integation with ROS was developed by Kiet Lam
# (kiet.lam@berkeley.edu). The web-server app Dator was based on an open source
# project by Bruce Wootton
# ---------------------------------------------------------------------------

# README: This node serves as an outgoing messaging bus from odroid to arduino
# Subscribes: steering and motor commands on 'ecu'
# Publishes: combined ecu commands as 'ecu_pwm'

from rospy import init_node, Subscriber, Publisher, get_param, get_rostime
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
# from scipy.linalg import expm, inv
from numpy import exp
from barc.msg import ECU, pos_info
import rospy

v0 = 0.0
MPC_solve_time = 0.1

class low_level_control(object):
    motor_pwm = 1500
    servo_pwm = 1580

    motor_pwm_bounds = [500, 2500]	# prevent extreme maneuvers
    servo_pwm_bounds = [1200, 1900]	# make sure servo doesn't get damaged
    # str_ang_max = 35
    # str_ang_min = -35
    ecu_pub = 0
    ecu_cmd = ECU()
    # use this time step for the discretized map from velocity to motor_pwm (same as in MPC!)
    dt = 1.0 / 10 + MPC_solve_time
    # discretize the map from motor_pwm to velocity for accelerating and braking
    # (exact discretization of linear system with (matrix) exponential)
    # vdot(t) = ab[0]*v(t) + ab[1]*motor_pwm(t) => v[k+1] = Ad*v[k] + Bd*motor_pwm[k]
    # accelerating:
    ab     = [-0.9030, 0.0084]
    # ab     = [-0.504303251448, 0.0128286494129]
    # ab     = [-0.632345330573, 0.010641048718]
    # ab     = [-0.732345330573, 0.009641048718]
    Ad_acc = exp(ab[0]*dt)
    Bd_acc = 1/ab[0]*(Ad_acc-1)*ab[1]
    # braking:
    # ab     = [-0.94007, 2.1246*10**(-2)]
    # ab     = [-0.777924522367, 0.00151713150358]
    # ab     = [-0.397097742996, 0.000940711134136]
    Ad_brk = exp(ab[0]*dt)
    Bd_brk = 1/ab[0]*(Ad_brk-1)*ab[1]

    def pwm_converter_callback(self, msg):
        # translate from SI units in vehicle model
        # to pwm angle units (i.e. to send command signal to actuators)
        # convert desired steering angle to degrees, saturate based on input limits

        # Old servo control:
        # self.servo_pwm = 91.365 + 105.6*float(msg.servo)
        # New servo Control
        # if msg.servo < 0.0:         # right curve
        #     self.servo_pwm = 95.5 + 118.8*float(msg.servo)
        # elif msg.servo > 0.0:       # left curve
        #     self.servo_pwm = 90.8 + 78.9*float(msg.servo)
        ab = [-0.000525151156156895, 0.834465187133306]
        # self.servo_pwm = 1580.0 - 833.33*float(msg.servo)
        self.servo_pwm = (float(msg.servo)-ab[1])/ab[0]

        # compute motor command
        # FxR = float(msg.motor)
        v1 = float(msg.motor)
        # print v1, v0
        # if FxR == 0:
        #if v1 == v0:
         #   self.motor_pwm = 1500.0
        # elif FxR > 0:
        if v1 >= v0:
            self.motor_pwm = (v1 - self.Ad_acc*v0)/self.Bd_acc + 1500
            # print 'acc', self.Ad_acc, self.Bd_acc
            # self.motor_pwm = 91 + 6.5*FxR   # using writeMicroseconds() in Arduino
            #self.motor_pwm = max(94,91 + 6.5*FxR)   # using writeMicroseconds() in Arduino

            # self.motor_pwm = max(94,90.74 + 6.17*FxR)
            #self.motor_pwm = 90.74 + 6.17*FxR
            
            #self.motor_pwm = max(94,90.12 + 5.24*FxR)
            #self.motor_pwm = 90.12 + 5.24*FxR
            # Note: Barc doesn't move for u_pwm < 93
        else:               # motor break / slow down
            self.motor_pwm = (v1 - self.Ad_brk*v0)/self.Bd_brk + 1500
            # print 'brk', self.Ad_brk, self.Bd_brk
            # self.motor_pwm = (FxR - ab[0]*v)/ab[1]
            # self.motor_pwm = 93.5 + 46.73*FxR
            # self.motor_pwm = 98.65 + 67.11*FxR
            #self.motor = 69.95 + 68.49*FxR

        if self.servo_pwm < self.servo_pwm_bounds[0]:
            self.servo_pwm = self.servo_pwm_bounds[0]
        if self.servo_pwm > self.servo_pwm_bounds[1]:
            self.servo_pwm = self.servo_pwm_bounds[1]
        if self.motor_pwm < self.motor_pwm_bounds[0]:
            self.motor_pwm = self.motor_pwm_bounds[0]
        if self.motor_pwm > self.motor_pwm_bounds[1]:
            self.motor_pwm = self.motor_pwm_bounds[1]

        self.update_arduino()
        print self.motor_pwm
    def neutralize(self):
        self.motor_pwm = 1400             # slow down first
        self.servo_pwm = 1580
        self.update_arduino()
        rospy.sleep(1)                  # slow down for 1 sec
        self.motor_pwm = 1500
        self.update_arduino()
    def update_arduino(self):
        # self.ecu_cmd.header.stamp = get_rostime()
        self.ecu_cmd.motor = self.motor_pwm
        self.ecu_cmd.servo = self.servo_pwm
        self.ecu_pub.publish(self.ecu_cmd)

def SE_callback(msg):
    global v0
    v0 = msg.v

def v0_callback(msg):
    global v0
    v0 = msg.motor

def arduino_interface():
    # launch node, subscribe to motorPWM and servoPWM, publish ecu
    init_node('arduino_interface')
    llc = low_level_control()

    Subscriber('ecu0', ECU, v0_callback, queue_size = 1)
    Subscriber('ecu2', ECU, llc.pwm_converter_callback, queue_size = 1)
    # Subscriber('pos_info', pos_info, SE_callback, queue_size = 1)
    llc.ecu_pub = Publisher('ecu_pwm', ECU, queue_size = 1)

    # Set motor to neutral on shutdown
    on_shutdown(llc.neutralize)

    # process callbacks and keep alive
    spin()

#############################################################
if __name__ == '__main__':
    try:
        arduino_interface()
    except ROSInterruptException:
        pass
