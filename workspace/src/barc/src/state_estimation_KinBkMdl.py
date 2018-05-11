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
import os
import math
from sensor_msgs.msg import Imu, NavSatFix
from barc.msg import ECU, Encoder, Z_KinBkMdl, pos_info
from numpy import pi, cos, sin, eye, array, zeros, unwrap, tan, diag
from observers import ekf, ukf
from system_models import f_KinBkMdl, h_KinBkMdl
from tf import transformations
from lla2flat import lla2flat
from Localization_helpers import Localization
from std_msgs.msg import Header
from SteeringMap import servo2df
# from numpy import unwrap

# input variables [default values]
d_f         = 0         # steering angle [rad]
acc         = 0         # velocity from encoders [m/s] - if state estimator model is changed, this will most likely become acceleration
# desired input variables from MPC [default values]
d_f_cmd = 0				# steering angle [rad]
acc_cmd = 0				# acceleration [m**2/s]

# initial (servo_pwm,curve,bm) (see SteeringMap.py)
# servo_pwm_init = 1500
# args_servo2df = (servo_pwm_init,0,0.0772518905741900-(-0.000371249151551210)*servo_pwm_init)

# raw measurement variables for IMU
yaw_prev = 0
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)
yaw_prev    = 0
yaw_local   = 0
read_yaw0   = False
psi         = 0
psi_meas    = 0
new_imu_meas = False

# from encoder
v_meas      = 0.0
t0          = time.time()
ang_km1     = 0.0
ang_km2     = 0.0
n_FL        = 0.0
n_FR        = 0.0
n_BL        = 0.0
n_BR        = 0.0
r_tire      = rospy.get_param("r_tire") # radius of the tire
new_enc_meas  = False
# for vel_est additionally:
v_est_FL = 0.0
v_est_FR = 0.0
v_est_BL = 0.0
v_est_BR = 0.0


# from gps
x_local = 0.0
y_local = 0.0
z_local = 0.0
gps_first_call = True
new_gps_meas   = False
psio_gps = rospy.get_param("psio_gps") # initial heading angle GPS

sim_mode = rospy.get_param("sim_mode") # indicates whether we are in simulation without /ecu topic

# ecu command update
def ecu_callback(data):
    global acc_cmd, d_f_cmd
    acc_cmd         = data.motor        # input acceleration
    d_f_cmd         = data.servo        # input steering angle

# ecu_pwm command update
def ecu_pwm_callback(data):
    # read only steering angle from ecu_pwm (acceleration map not really good)
    global d_f, acc, args_servo2df
    motor_pwm   = data.motor
    servo_pwm   = data.servo

    # use steering command map to get d_f
    # (d_f,args_servo2df) = servo2df(servo_pwm,args_servo2df)
    # d_f = -0.000527833480631484*servo_pwm+0.836616066800903#0.842150600118722
    d_f = -0.000525151156156895*servo_pwm+0.834465187133306 #0.832364582508679#0.838026451730028 
    #d_f = -0.000479973711852115*servo_pwm+0.758358464726342
    #d_f = -4.838020766400743*10**(-4)*servo_pwm+0.775758994667799 #5/9.0*(-0.000935851818915458*servo_pwm + 1.48382531174335) #5/12.0*(-0.0012*servo_pwm + 1.8962) #

    # assign the acc input the last measurement from the IMU (noisy!) in direction of the v for KinBkMdl
    # L_a = vhMdl[0]
    # L_b = vhMdl[1]
    # bta = math.atan2( L_b * tan(d_f),(L_a + L_b) )
    # v_x_meas = v_meas*cos(d_f)
    # acc = (a_x+w_z*v_x_meas*tan(bta))*cos(bta)+(a_y-w_z*v_x_meas)*sin(bta)
    # acc = a_x
    acc = v_meas

# GPS measurement update
def gps_callback(data):
    global x_local, y_local, z_local, gps_first_call, gps_lat_init, gps_lng_init, gps_alt_init, new_gps_meas
    gps_latitude = data.latitude
    gps_longitude = data.longitude
    gps_altitude = data.altitude
	
	# save the initial measurement (after MPC has been solved the first time)
    if gps_first_call:
        gps_lat_init = gps_latitude
        gps_lng_init = gps_longitude
        gps_alt_init = gps_altitude
        if acc_cmd!=0 or sim_mode:
            gps_first_call = False

	# compute x,y,z coordinates respectively
    (x_gps, y_gps, z_gps) = lla2flat((gps_latitude, gps_longitude, gps_altitude),(gps_lat_init, gps_lng_init), psio_gps, gps_alt_init) # 115: FifteenThreePihalf.bag -yaw0*180/pi+115
    x_local = x_gps
    y_local = y_gps 
    z_gps = z_gps

	# indicate that a new measurement has been received
    new_gps_meas = True

# imu measurement update
def imu_callback(data):
    # units: [rad] and [rad/s]
    global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
    global yaw_prev, yaw0, read_yaw0, yaw_local, psi_meas
    global new_imu_meas

    # get orientation from quaternion data, and convert to roll, pitch, yaw
    # extract angular velocity and linear acceleration data
    ori         = data.orientation
    quaternion  = (ori.x, ori.y, ori.z, ori.w)
    (roll, pitch, yaw) = transformations.euler_from_quaternion(quaternion)

    # save initial measurements (after MPC has been solved the first time)
    if not read_yaw0:
        if acc_cmd!=0 or sim_mode:
            read_yaw0   = True
        yaw_prev    = yaw
        yaw0        = yaw
    
    # unwrap measurement
    yaw         = unwrap(array([yaw_prev, yaw]), discont = pi)[1]
    yaw_prev    = yaw
    yaw_local   = yaw - yaw0
    psi_meas    = yaw_local
    
    # extract angular velocity and linear acceleration data
    w_x = data.angular_velocity.x
    w_y = data.angular_velocity.y
    w_z = data.angular_velocity.z
    a_x = data.linear_acceleration.x
    a_y = data.linear_acceleration.y
    a_z = data.linear_acceleration.z

	# indicate that a new measurement has been received
    new_imu_meas = True

# encoder measurement update
def enc_callback(data):
    global t0, v_meas, new_enc_meas
    global n_FL, n_FR, n_BL, n_BR
    global ang_km1, ang_km2

    n_FL = data.FL
    n_FR = data.FR
    n_BL = data.BL
    n_BR = data.BR

    # compute the average encoder measurement (no slip at front wheels)
    n_mean = (n_FL + n_FR)/2.0

    # transfer the encoder measurement to angular displacement
    ang_mean = n_mean*2*pi/8.0

    # compute time elapsed
    tf = time.time()
    dt = tf - t0
    
    # compute speed with second-order, backwards-finite-difference estimate
    v_meas    = r_tire*(3*ang_mean - 4*ang_km1 + ang_km2)/(2*dt)

    # update old data
    ang_km2 = ang_km1
    ang_km1 = ang_mean
    t0      = time.time()

	# indicate that a new measurement has been received
    # Uncomment this if the encoder measurements are used to correct the estimates
    # new_enc_meas = True

# velocity estimation callback
def vel_est_callback(data):
    # This relies on the published information to the topic vel_est (estimate
    # produced in Arduino) and is not used for now since the second order 
    # finite backwards difference is more accurate (see enc_callback)
    global v_meas, new_enc_meas

    v_est_FL = data.FL
    v_est_FR = data.FR
    v_est_BL = data.BL
    v_est_BR = data.BR

    # (once it comes to wheels taking off or wrong enc meas. from one wheel, think about what makes sense)
    # idea: Get an estimate also from the integration of acceleration from the IMU and compare to vel_est (drifting?)
    v_meas = (v_est_FL + v_est_FR)/2.0

	# indicate that a new measurement has been received
    # Uncomment this if the encoder measurements are used to correct the estimates
    # new_enc_meas = True



# state estimation node
def state_estimation():
    global dt_v_enc
    global v_meas, psi_meas
    global x_local, y_local
    global new_enc_meas, new_gps_meas, new_imu_meas
    global vhMdl

    # initialize node
    rospy.init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('imu/data', Imu, imu_callback)
    rospy.Subscriber('encoder', Encoder, enc_callback)
    # rospy.Subscriber('vel_est', Encoder, vel_est_callback)
    rospy.Subscriber('ecu', ECU, ecu_callback)
    rospy.Subscriber('ecu_pwm', ECU, ecu_pwm_callback)
    rospy.Subscriber('fix', NavSatFix, gps_callback)
    state_pub = rospy.Publisher('state_estimate', Z_KinBkMdl, queue_size = 10)
    state_pub_pos = rospy.Publisher('pos_info', pos_info, queue_size = 10)

    # for debugging
    meas_pub    = rospy.Publisher('meas', Z_KinBkMdl, queue_size = 10)
    u_pub       = rospy.Publisher('u', ECU, queue_size = 10)

    # get vehicle dimension parameters
    L_a = rospy.get_param("L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("L_b")       # distance from CoG to rear axel
    vhMdl   = (L_a, L_b)

    # get encoder parameters
    dt_v_enc = rospy.get_param("state_estimation/dt_v_enc") # time interval to compute v_x from encoders

    # get EKF observer properties
    q_std   = rospy.get_param("state_estimation/q_std")             # std of process noise
    r_std   = rospy.get_param("state_estimation/r_std")             # std of measurementnoise

    # get measurement model type according to incorporated sensors
    est_mode = rospy.get_param("state_estimation/est_mode")

    ##################################################################################################################################
    # Set up track parameters (for simulation, this needs to be changed according to the track the sensor data was recorded for)
    l = Localization()
    l.create_track()
    est_counter = 0
    ##################################################################################################################################

    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    # initial state vector (mean)
    x_EKF       = zeros(4)

    # estimation variables for the filter (covariances)
    # make the uncertainty in the initial psi and/or psi_drift bigger the less we know about the relationship between the relationship
    # between the GPS coordinate system (namely NESW) and the IMU coordinate system
    # Precisely: where is the car heading initially w.r.t. NESW?
    P           = diag([0.1,0.1,50*q_std**2,0.01])                  # initial state covariance matrix
    Q           = diag([q_std**2,q_std**2,10*q_std**2,2*q_std**2])  # process noise covariance matrix (drift: 5)
    R           = diag([r_std**2,r_std**2,2*r_std**2,r_std**2])     # measurement noise covariance matrix

    # publish initial state estimate
    (x, y, psi, psi_drift) = x_EKF
    # print x,y,psi,v
    v = acc
    state_pub.publish( Z_KinBkMdl(x, y, psi, v) )
    # collect input
    u   = [ d_f, acc ]
    # u_pub.publish( ECU(u[1],u[0]) )

    # wait
    rate.sleep()

    while not rospy.is_shutdown():        

        # collect measurements
        z   = array([x_local, y_local, psi_meas, v_meas])
        # print
        # print z
        z_new = array([new_gps_meas, new_gps_meas, new_imu_meas, new_enc_meas])
        meas_pub.publish(Z_KinBkMdl(x_local,y_local,psi_meas,v_meas))
        z   = z[z_new]
        

        # Reset booleans for new measurements for the next iteration
        new_enc_meas = False
        new_gps_meas = False
        new_imu_meas = False

        # print u

        # reshape covariance matrix according to number of received measurements
        R_k = R[:,z_new][z_new,:]

        # decide which measurement model to use based on which measurements came in
        if          z_new[0] and        z_new[2] and        z_new[3]:
            est_mode = 1
        elif    not z_new[0] and        z_new[2] and        z_new[3]:
            est_mode = 2
        elif        z_new[0] and    not z_new[2] and    not z_new[3]:
            est_mode = 3
        elif        z_new[0] and    not z_new[2] and        z_new[3]:
            est_mode = 4
        elif        z_new[0] and        z_new[2] and    not z_new[3]:
            est_mode = 5
        elif    not z_new[0] and    not z_new[2] and        z_new[3]:
            est_mode = 6
        elif    not z_new[0] and        z_new[2] and    not z_new[3]:
            est_mode = 7
        elif    not z_new[0] and    not z_new[2] and    not z_new[3]:
            est_mode = 8
        else:
            est_mode = 0    # a case is not covered -> error

        args = (u,vhMdl,dt,est_mode)

        # collect input for the next iteration
        u   = [ d_f, acc ]
        # u_pub.publish( ECU(u[1],u[0]) )

        # print est_mode

        # apply EKF and get each state estimate
        (x_EKF,P) = ekf(f_KinBkMdl, x_EKF, P, h_KinBkMdl, z, Q, R_k, args )

        # publish state estimate
        # (x, y, psi, v) = x_EKF
        v = acc
        (x, y, psi, psi_drift) = x_EKF
        # print x,y,psi,psi_drift

        # publish information
        state_pub.publish( Z_KinBkMdl(x, y, psi, v) )

        # Update track position (x,y,psi,v_x,v_y,psi_dot)
        l.set_pos(x, y, psi, v, 0, 0)
        # Comment: in find_s, the linear and angular velocities are not needed,
        # therefore we can just ignore splitting up v here

        # Calculate new s, ey, epsi (only 12.5 Hz, enough for controller that runs at 10 Hz)
        if est_counter%4 == 0:
            l.find_s()

            # and then publish position info
            ros_t = rospy.get_rostime()
            state_pub_pos.publish(pos_info(Header(stamp=ros_t), l.s, l.ey, l.epsi, v, l.s_start, l.x, l.y, l.v_x, l.v_y,
                                       l.psi, l.psiDot, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0,
                                       (0,), (0,), (0,), l.coeffCurvature.tolist()))
        est_counter += 1

        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       state_estimation()
    except rospy.ROSInterruptException:
        pass
