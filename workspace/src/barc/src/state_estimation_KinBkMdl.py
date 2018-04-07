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
from numpy import pi, cos, sin, eye, array, zeros, unwrap, tan
from observers import ekf
from system_models import f_KinBkMdl, h_KinBkMdl
from tf import transformations
from lla2flat import lla2flat
from Localization_helpers import Localization
from std_msgs.msg import Header
# from numpy import unwrap

# input variables [default values]
d_f         = 0         # steering angle [deg]
acc         = 0         # acceleration [m/s]

# raw measurement variables
yaw_prev = 0
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)
yaw_prev    = 0
yaw_local   = 0
read_yaw0   = False
psi         = 0
psi_meas    = 0
new_imu_meas = False

# from encoder
# v           = 0
# v_meas      = 0
# t0          = time.time()
# n_FL        = 0                     # counts in the front left tire
# n_FR        = 0                     # counts in the front right tire
# n_BL        = 0                     # counts in the back left tire
# n_BR        = 0                     # counts in the back right tire
# n_FL_prev   = 0
# n_FR_prev   = 0
# n_BL_prev   = 0
# n_BR_prev   = 0
# r_tire      = 0.036                  # radius from tire center to perimeter along magnets [m]
# dx_qrt      = 2.0*pi*r_tire/4.0     # distance along quarter tire edge [m]

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


# from gps
x_local = 0.0
y_local = 0.0
z_local = 0.0
gps_first_call = True
new_gps_meas   = False
 
# def lla2flat(lla, llo, psio, href):
#     '''
#     lla  -- array of geodetic coordinates 
#             (latitude, longitude, and altitude), 
#             in [degrees, degrees, meters]. 
 
#             Latitude and longitude values can be any value. 
#             However, latitude values of +90 and -90 may return 
#             unexpected values because of singularity at the poles.
 
#     llo  -- Reference location, in degrees, of latitude and 
#             longitude, for the origin of the estimation and 
#             the origin of the flat Earth coordinate system.
 
#     psio -- Angular direction of flat Earth x-axis 
#             (degrees clockwise from north), which is the angle 
#             in degrees used for converting flat Earth x and y 
#             coordinates to the North and East coordinates.
 
#     href -- Reference height from the surface of the Earth to 
#             the flat Earth frame with regard to the flat Earth 
#             frame, in meters.
 
#     usage: print(lla2flat((0.1, 44.95, 1000.0), (0.0, 45.0), 5.0, -100.0))
 
#     '''
 
#     R = 6378137.0  # Equator radius in meters
#     f = 0.00335281066474748071  # 1/298.257223563, inverse flattening

#     Lat_p = lla[0] * math.pi / 180.0  # from degrees to radians
#     Lon_p = lla[1] * math.pi / 180.0  # from degrees to radians
#     Alt_p = lla[2]  # meters
 
#     # Reference location (lat, lon), from degrees to radians
#     Lat_o = llo[0] * math.pi / 180.0
#     Lon_o = llo[1] * math.pi / 180.0
     
#     psio = psio * math.pi / 180.0  # from degrees to radians
 
#     dLat = Lat_p - Lat_o
#     dLon = Lon_p - Lon_o
 
#     ff = (2.0 * f) - (f ** 2)  # Can be precomputed
 
#     sinLat = math.sin(Lat_o)
 
#     # Radius of curvature in the prime vertical
#     Rn = R / math.sqrt(1 - (ff * (sinLat ** 2)))
 
#     # Radius of curvature in the meridian
#     Rm = Rn * ((1 - ff) / (1 - (ff * (sinLat ** 2))))
 
#     dNorth = (dLat) / math.atan2(1, Rm)
#     dEast = (dLon) / math.atan2(1, (Rn * math.cos(Lat_o)))
 
#     # Rotate matrice clockwise
#     Xp = (dNorth * math.cos(psio)) + (dEast * math.sin(psio))
#     Yp = (-dNorth * math.sin(psio)) + (dEast * math.cos(psio))
#     Zp = -Alt_p - href
 
#     return Xp, Yp, Zp


# ecu command update
def ecu_callback(data):
    global acc, d_f
    acc         = data.motor        # input acceleration
    d_f         = data.servo        # input steering angle

def ecu_pwm_callback(data):
    # read only steering angle from ecu_pwm (acceleration map not really good)
    global d_f, acc
    motor_pwm   = data.motor
    servo_pwm   = data.servo

    # use steering command map to get d_f
    d_f = -0.0007*servo_pwm + 1.1345 #5/12.0*(-0.0012*servo_pwm + 1.8962)

    # assign the acc input the last measurement from the IMU (noisy!) in direction of the v for KinBkMdl
    # L_a = vhMdl[0]
    # L_b = vhMdl[1]
    # acc = a_x*cos(math.atan2( L_b * tan(d_f),(L_a + L_b) ))+a_y*sin(math.atan2( L_b * tan(d_f),(L_a + L_b) ))
    acc = a_x

# GPS measurement update
def gps_callback(data):
    global x_local, y_local, z_local, gps_first_call, gps_lat_init, gps_lng_init, gps_alt_init, new_gps_meas
    gps_latitude = data.latitude
    gps_longitude = data.longitude
    gps_altitude = data.altitude

    if gps_first_call:
        gps_lat_init = gps_latitude
        gps_lng_init = gps_longitude
        gps_alt_init = gps_altitude
        gps_first_call = False

    (x_gps, y_gps, z_gps) = lla2flat((gps_latitude, gps_longitude, gps_altitude),(gps_lat_init, gps_lng_init), -yaw0*180/pi+90, gps_alt_init) # -2.12098490166
    x_local = x_gps
    y_local = y_gps 
    z_gps = z_gps
    # rospy.logwarn("x = {}, y = {}".format(x_local,y_local))

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

    # save initial measurements
    if not read_yaw0:
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

    new_imu_meas = True

# encoder measurement update
# def enc_callback(data):
#     global v, t0, dt_v_enc, v_meas
#     global n_FL, n_FR, n_FL_prev, n_FR_prev
#     global n_BL, n_BR, n_BL_prev, n_BR_prev

#     n_FL = data.FL
#     n_FR = data.FR
#     n_BL = data.BL
#     n_BR = data.BR

#     # compute time elapsed
#     tf = time.time()
#     dt = tf - t0

#     # if enough time elapse has elapsed, estimate v_x
#     if dt >= dt_v_enc:
#         # compute speed :  speed = distance / time
#         v_FL = float(n_FL - n_FL_prev)*dx_qrt/dt
#         v_FR = float(n_FR - n_FR_prev)*dx_qrt/dt
#         v_BL = float(n_BL - n_BL_prev)*dx_qrt/dt
#         v_BR = float(n_BR - n_BR_prev)*dx_qrt/dt

#         # Uncomment/modify according to your encoder setup
#         # v_meas    = (v_FL + v_FR)/2.0
#         # Modification for 3 working encoders
#         v_meas = (v_FL + v_BL + v_BR)/3.0
#         # Modification for bench testing (driven wheels only)
#         # v = (v_BL + v_BR)/2.0

#         # update old data
#         n_FL_prev   = n_FL
#         n_FR_prev   = n_FR
#         n_BL_prev   = n_BL
#         n_BR_prev   = n_BR
#         t0          = time.time()

# encoder measurement update
def enc_callback(data):
    global t0, v_meas, new_enc_meas
    global n_FL, n_FR, n_BL, n_BR
    global ang_km1, ang_km2

    n_FL = data.FL
    n_FR = data.FR
    n_BL = data.BL
    n_BR = data.BR

    # compute the average encoder measurement
    n_mean = (n_FL + n_FR + n_BL + n_BR)/4

    # transfer the encoder measurement to angular displacement
    ang_mean = n_mean*2*pi/8

    # compute time elapsed
    tf = time.time()
    dt = tf - t0
    
    # compute speed with second-order, backwards-finite-difference estimate
    v_meas    = r_tire*(3*ang_mean - 4*ang_km1 + ang_km2)/(2*dt)
    # rospy.logwarn("speed = {}".format(v_meas))

    # update old data
    ang_km1 = ang_mean
    ang_km2 = ang_km1
    t0      = time.time()

    new_enc_meas = True

# velocity estimation callback
def vel_est_callback(data):
    global v_meas, new_enc_meas

    v_est_FL = data.FL
    v_est_FR = data.FR
    v_est_BL = data.BL
    v_est_BR = data.BR

    # This is pretty random for now: (once it comes to wheels taking off or wrong enc meas. from one wheel, think about what makes sense)
    # idea: Get an estimate also from the integration of acceleration from the IMU and compare to vel_est (drifting?)
    v_meas = (v_FL + v_BL + v_BR)/3.0

    new_enc_meas = True



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
    # rospy.Subscriber('ecu', ECU, ecu_callback)
    rospy.Subscriber('ecu_pwm', ECU, ecu_pwm_callback)  # use this line only in simulation without controller (no ECU published) -> read commands from recorded data
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

    # state estimates
    x_EKF       = zeros(4)

    # estimation variables for EKF
    P           = eye(4)                # initial dynamics covariance matrix
    Q           = array([[q_std**2,0.0,0.0,0.0],
                         [0.0,q_std**2,0.0,0.0],
                         [0.0,0.0,q_std**2,0.0],
                         [0.0,0.0,0.0,q_std**2]])     # process noise covariance matrix
    R           = array([[0.1,0.0,0.0,0.0],
                         [0.0,0.1,0.0,0.0],
                         [0.0,0.0,0.1,0.0],
                         [0.0,0.0,0.0,r_std]])    # measurement noise covariance matrix

    # publish initial state estimate
    (x, y, psi, v) = x_EKF
    # print x,y,psi,v
    state_pub.publish( Z_KinBkMdl(x, y, psi, v) )
    # collect input
    u   = [ d_f, acc ]
    # u_pub.publish( ECU(u[1],u[0]) )

    # wait
    rate.sleep()

    while not rospy.is_shutdown():        

        # collect measurements
        z   = array([x_local, y_local, psi_meas, v_meas])   # v_meas from arduino calc instead of enc data directly?
        meas_pub.publish(Z_KinBkMdl(x_local,y_local,psi_meas,v_meas))
        # print
        # print z
        z_new = array([new_gps_meas, new_gps_meas, new_imu_meas, new_enc_meas])
        z   = z[z_new]
        

        # Reset booleans for new measurements for the next iteration
        new_enc_meas = False
        new_gps_meas = False
        new_imu_meas = False

        print u

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
        u_pub.publish( ECU(u[1],u[0]) )

        # print est_mode

        # apply EKF and get each state estimate
        (x_EKF,P) = ekf(f_KinBkMdl, x_EKF, P, h_KinBkMdl, z, Q, R_k, args )

        # publish state estimate
        (x, y, psi, v) = x_EKF
        print x,y,psi,v

        # publish information
        state_pub.publish( Z_KinBkMdl(x, y, psi, v) )

        # Update track position (x,y,psi,v_x,v_y,psi_dot)
        l.set_pos(x, y, psi, v, 0, 0)

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
