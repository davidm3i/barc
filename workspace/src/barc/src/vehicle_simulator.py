#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

import rospy
import time
from barc.msg import ECU, Z_KinBkMdl, Encoder
from sensor_msgs.msg import Imu, NavSatFix
from numpy import sin, cos, tan, arctan, array, dot, pi
from numpy import sign, argmin, sqrt, zeros, row_stack, ones, interp
# from bike_model import bikeFE
from system_models import f_KinBkMdl
from lla2flat import flat2lla
from tf import transformations
import math
# input variables
d_f         = 0
acc         = 0

# from encoder
t0          = time.time()

# ecu command update
def ecu_callback(data):
    global acc, d_f
    acc         = data.motor        # input acceleration
    d_f         = data.servo        # input steering angle

# state estimation node
def vehicle_simulator():

    # initialize node
    rospy.init_node('vehicle_simulator', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('ecu', ECU, ecu_callback)
    state_pub   = rospy.Publisher('z_vhcl', Z_KinBkMdl, queue_size = 10)
    imu_pub   = rospy.Publisher('imu/data', Imu, queue_size = 10)
    enc_pub   = rospy.Publisher('vel_est', Encoder, queue_size = 10)
    gps_pub   = rospy.Publisher('fix', NavSatFix, queue_size = 10)
    # servo_pub = rospy.Publisher('ecu_pwm', ECU, queue_size = 10)

    # get external force model
    # a0    = rospy.get_param("air_drag_coeff")
    # m	= rospy.get_param("mass")
    # Ff    = rospy.get_param("friction")
    # x_list     = array([0, 20, 20, 40,  40, 60, 60,  80, 80, 100, 100, 120, 120, 140, 140, 160, 160, 180])
    # theta_list = array([0, 0,  10, 10,  0,  0, -10, -10,  0,  0,  30,  30,  0,   0,   -30, -30, 0,   0])
    
    # set node rate
    loop_rate   = 50
    ts          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    # initialize imudata
    imudata = Imu()
    encdata = Encoder()
    gpsdata = NavSatFix()
    ecu_pwm = ECU()


    # set measurement rates
    dt_GPS      = 1.0/8
    GPS_dt_count= dt_GPS/ts + 0.1   # create a measurement at time 0
    # dt_Enc    = 1.0/50
    # dt_Imu    = 1.0/50

    # load model parameters
    L_a = rospy.get_param("L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("L_b")       # distance from CoG to rear axel
    psio_gps = rospy.get_param("psio_gps") # initial heading GPS

    # set initial conditions 
    x   = 0
    y   = 0
    psi = 0
    # v_x = 0
    # v_y = 0
    v   = 0
    v_FW = 0.0 # front wheel velocity
    # r   = 0

    s    = 0
    ey   = 0
    epsi = 0
    while not rospy.is_shutdown():
        t_ros = rospy.Time.now()

        # theta = interp(x, x_list, theta_list)/180*pi
        # (x, y, psi, v_x) = bikeFE(x, y, psi, v_x, acc, d_f, a0, m, Ff, theta, ts)
        # print x, y, psi, v_x
        # print acc, d_f
        procm = f_KinBkMdl((x, y, psi, 0), (d_f, v_FW), (L_a, L_b), ts, 0)
        (x, y, psi, psi_dr) = procm[0]
        v    = v + ts*(acc -0.63*sign(v)*v**2)
        bta  = math.atan2( L_b * tan(d_f),(L_a + L_b) )
        v_FW = v*cos(bta)/cos(d_f)
        v_RW = v*cos(bta)

        # publish 'true' state information
        state_pub.publish(Z_KinBkMdl(x, y, psi, v) )

        # Publish Imu measurement
        (imudata.orientation.x,imudata.orientation.x,imudata.orientation.z,imudata.orientation.w) = transformations.quaternion_from_euler(0,0,psi)
        imudata.header.stamp = t_ros
        imu_pub.publish(imudata)

        # Publish Encoder measurement
        encdata.FL = v_FW
        encdata.FR = v_FW
        encdata.BL = v_RW
        encdata.BR = v_RW
        enc_pub.publish(encdata)

        # publish GPS measurement
        if ts*GPS_dt_count >= dt_GPS:
            (gpsdata.latitude, gpsdata.longitude, gpsdata.altitude) = flat2lla((x,y,0.0),(0.0, 0.0), psio_gps, 0.0)
            gpsdata.header.stamp = t_ros
            gps_pub.publish(gpsdata)
            GPS_dt_count = -1

        # publish servo_pwm output
        # ab = [-0.000525151156156895, 0.834465187133306]
        # ecu_pwm.servo = (d_f-ab[1])/ab[0]
        # servo_pub.publish(ecu_pwm)

        # wait
        GPS_dt_count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
       vehicle_simulator()
    except rospy.ROSInterruptException:
        pass
