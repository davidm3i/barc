#!/usr/bin/env python

import rosbag
import subprocess
import os
from os import listdir
from os.path import isfile, join
import yaml
import numpy as np
import cv2
from tf import transformations
from numpy import pi
import matplotlib.pyplot as plt
from lla2flat import lla2flat
import Tkinter, tkFileDialog

rosbag_dir = os.path.expanduser("~") + '/simulations'
initafterecu = False

# select bag file
root = Tkinter.Tk()
root.withdraw()
bag_file = tkFileDialog.askopenfilename(initialdir = rosbag_dir, title = "select bag file" , filetypes = [("Bags","*.bag")])
bag         = rosbag.Bag( bag_file )
print bag_file
meta_data   = bag.get_type_and_topic_info()[1]
n_msgs      = {}
topics      = meta_data.keys()

# extract number of messages transmitted on each topic
for i in range( len(topics) ):
    topic = topics[i]
    n_msgs[topic] = meta_data[topic][1]
print topics

# declare storage variables
if '/fix' in topics:
    idx_gps             = 0
    n_fix               = n_msgs['/fix']
    latitude   		    = np.zeros( n_fix )
    longitude  		    = np.zeros( n_fix )
    altitude            = np.zeros( n_fix )
    X                   = np.zeros( n_fix )
    Y                   = np.zeros( n_fix )
    t_gps               = np.zeros( n_fix )

if '/imu/data' in topics:
    idx_imu             = 0
    n_imu               = n_msgs['/imu/data']
    roll   		        = np.zeros( n_imu )
    pitch  		        = np.zeros( n_imu )
    yaw   		        = np.zeros( n_imu )
    ang_vel_x           = np.zeros( n_imu )
    ang_vel_y           = np.zeros( n_imu )
    ang_vel_z           = np.zeros( n_imu )
    lin_acc_x           = np.zeros( n_imu )
    lin_acc_y           = np.zeros( n_imu )
    lin_acc_z           = np.zeros( n_imu )
    t_imu               = np.zeros( n_imu )

if '/ecu_pwm' in topics:
    idx_rc              = 0
    n_rc                = n_msgs['/ecu_pwm']
    throttle            = np.zeros( n_rc )
    steering            = np.zeros( n_rc )
    t_rc                = np.zeros( n_rc )

if '/state_estimate' in topics:
    idx_se              = 0
    n_se                = n_msgs['/state_estimate']
    X_se                = np.zeros( n_se )
    Y_se                = np.zeros( n_se )
    v_se                = np.zeros( n_se )
    psi_se              = np.zeros( n_se )
    t_se                = np.zeros( n_se )

if '/meas' in topics:
    idx_meas            = 0
    n_meas              = n_msgs['/meas']
    X_meas              = np.zeros( n_meas )
    Y_meas              = np.zeros( n_meas )
    v_meas              = np.zeros( n_meas )
    psi_meas            = np.zeros( n_meas )
    t_meas              = np.zeros( n_meas )

if '/u' in topics:
    idx_u               = 0
    n_u                 = n_msgs['/u']
    acc                 = np.zeros( n_u )
    d_f                 = np.zeros( n_u )
    t_u                 = np.zeros( n_u )

if '/ecu' in topics:
    idx_ecu             = 0
    n_ecu               = n_msgs['/ecu']
    acc_ecu             = np.zeros( n_ecu )
    d_f_ecu             = np.zeros( n_ecu )
    t_ecu               = np.zeros( n_ecu )

if '/encoder' in topics:
    idx_enc             = 0
    n_enc               = n_msgs['/encoder']
    n_FL                = np.zeros( n_enc )
    n_FR                = np.zeros( n_enc )
    n_BL                = np.zeros( n_enc )
    n_BR                = np.zeros( n_enc )
    t_enc               = np.zeros( n_enc )

# initialize index for each measurement
t0 = -1
for topic, msg, t in bag.read_messages(topics=['/ecu_pwm','/ecu','/imu/data','/state_estimate','/meas','/u','/encoder']) :
    
    # initial system time
    if t0 == -1:
        t0                  = t.secs + t.nsecs/(10.0**9)
    ts                  = t.secs + t.nsecs/(10.0**9) - t0

    if topic == '/imu/data':
        ori                 = msg.orientation
        quaternion          = (ori.x, ori.y, ori.z, ori.w)
        (r, p, y)           = transformations.euler_from_quaternion(quaternion)
        t_imu[idx_imu]      = ts 
        roll[idx_imu]        = r
        pitch[idx_imu]      = p
        yaw[idx_imu]        = y
        ang_vel_x[idx_imu]  = msg.angular_velocity.x
        ang_vel_y[idx_imu]  = msg.angular_velocity.y
        ang_vel_z[idx_imu]  = msg.angular_velocity.z
        lin_acc_x[idx_imu]  = msg.linear_acceleration.x
        lin_acc_y[idx_imu]  = msg.linear_acceleration.y
        lin_acc_z[idx_imu]  = msg.linear_acceleration.z
        idx_imu     += 1
        # print yaw[0]

    if topic == '/ecu_pwm':
        t_rc[idx_rc]        = ts 
        throttle[idx_rc]    = msg.motor
        steering[idx_rc]    = msg.servo
        idx_rc += 1

    if topic == '/state_estimate':
        t_se[idx_se]        = ts 
        X_se[idx_se]        = msg.x
        Y_se[idx_se]        = msg.y
        v_se[idx_se]        = msg.v
        psi_se[idx_se]      = msg.psi
        idx_se += 1

    if topic == '/meas':
        t_meas[idx_meas]      = ts 
        X_meas[idx_meas]      = msg.x
        Y_meas[idx_meas]      = msg.y
        v_meas[idx_meas]      = msg.v
        psi_meas[idx_meas]    = msg.psi
        idx_meas += 1

    if topic == '/u':
        t_u[idx_u]          = ts 
        acc[idx_u]          = msg.motor
        d_f[idx_u]          = msg.servo
        idx_u += 1

    if topic == '/ecu':
        t_ecu[idx_ecu]      = ts 
        acc_ecu[idx_ecu]    = msg.motor
        d_f_ecu[idx_ecu]    = msg.servo
        idx_ecu += 1

    if topic == '/encoder':
        t_enc[idx_enc]      = ts
        n_FL[idx_enc]       = msg.FL
        n_FR[idx_enc]       = msg.FR
        n_BL[idx_enc]       = msg.BL
        n_BR[idx_enc]       = msg.BR

t0 = -1
gpsinit = False
for topic, msg, t in bag.read_messages(topics=['/fix']):
    # initial system time
    if t0 == -1:
        t0                  = t.secs + t.nsecs/(10.0**9)
    ts                  = t.secs + t.nsecs/(10.0**9) - t0

    if topic == '/fix':
        lng                         = msg.longitude
        lat                         = msg.latitude
        alt                         = msg.altitude
        t_gps[idx_gps]              = ts 
        longitude[idx_gps]          = lng
        latitude[idx_gps]           = lat
        altitude[idx_gps]           = alt
        # if there is an ecu topic, initialize the GPS at the point when the first command is sent (before the car could not possibly move)
        if '/ecu' in topics and initafterecu:
            if not gpsinit:
                lat0 = lat
                lng0 = lng
                # print ts, t_ecu
                # j = next(j for j,w in enumerate(t_ecu) if w <= ts)
                # print j
                # if acc_ecu[j] != 0:
                if ts >= t_ecu[0]:
                    gpsinit = True
            (X[idx_gps], Y[idx_gps],_)  = lla2flat((lat,lng,alt), (lat0, lng0), 80.0, altitude[0])
        else:
            (X[idx_gps], Y[idx_gps],_)  = lla2flat((lat,lng,alt), (latitude[0], longitude[0]), 236.5, altitude[0]) #-yaw[0]*180/pi+115

        idx_gps += 1

bag.close()

font = {'family':'serif',
        'color' : 'black',
        'weight': 'normal',
        'size'  : 12}
fig_sz = (14,4)


# visualize data collected
if '/fix' in topics:
    plt.figure( figsize = fig_sz)
    plt.subplot(211)
    plt.plot(t_gps, longitude)
    plt.xlabel('t [sec]')
    plt.ylabel('longitude [deg]')
    plt.subplot(212)
    plt.plot(t_gps, latitude)
    plt.xlabel('t [sec]')
    plt.ylabel('latitude [deg]')

    plt.figure()
    plt.plot(X,Y,'k*')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.grid(axis = 'both')

if '/ecu_pwm' in topics:
    plt.figure( figsize = fig_sz)
    plt.subplot(211)
    plt.plot(t_rc, throttle)
    plt.xlabel('t [sec]')
    plt.ylabel('throttle command')
    plt.grid(axis = 'both')
    plt.subplot(212)
    plt.plot(t_rc, steering)
    plt.xlabel('t [sec]')
    plt.ylabel('steering command')
    plt.grid(axis = 'both')

    # plt.figure('d_f', figsize = fig_sz)
    # plt.plot(t_rc, (-0.0010*steering+1.5)*180/pi)    #(-0.0012*steering+1.8962)*180/pi
    # plt.xlabel('t [sec]')
    # plt.ylabel('steering angle [deg]')
    # plt.grid(axis = 'both')

if '/imu/data' in topics:
    plt.figure( figsize = fig_sz)
    plt.subplot(311)
    plt.plot(t_imu, roll)
    plt.plot(t_imu, pitch)
    plt.plot(t_imu, yaw)
    plt.xlabel('t [sec]')
    plt.ylabel('Euler angles')
    plt.grid(axis = 'both')
    plt.subplot(312)
    plt.plot(t_imu, ang_vel_x)
    plt.plot(t_imu, ang_vel_y)
    plt.plot(t_imu, ang_vel_z)
    plt.xlabel('t [sec]')
    plt.ylabel('angular velocity')
    plt.grid(axis = 'both')
    plt.subplot(313)
    plt.plot(t_imu, lin_acc_x)
    plt.plot(t_imu, lin_acc_y)
    plt.plot(t_imu, lin_acc_z)
    plt.xlabel('t [sec]')
    plt.ylabel('linear acceleration')
    plt.legend(('a_x','a_y','a_z'))
    plt.grid(axis = 'both')

if '/state_estimate' in topics:
    plt.figure(figsize = fig_sz)
    plt.plot(X,Y,'k*')
    plt.hold('on')
    plt.plot(X_se,Y_se,'b+')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.grid(axis = 'both')

    plt.figure(figsize = fig_sz)
    plt.subplot(211)
    plt.plot(t_gps,X,'k*')
    plt.hold('on')
    plt.plot(t_se,X_se,'b+')
    plt.xlabel('t [sec]')
    plt.ylabel('X [m]')
    plt.subplot(212)
    plt.plot(t_gps,Y,'k*')
    plt.plot(t_se,Y_se,'b+')
    plt.xlabel('t [sec]')
    plt.ylabel('Y [m]')

    plt.figure(figsize = fig_sz)
    plt.plot(t_imu, (yaw-yaw[0]*np.ones(len(yaw)))*180/pi)
    plt.hold('on')
    plt.plot(t_se,psi_se*180/pi,'b+')
    plt.xlabel('t [sec]')
    plt.ylabel('psi [deg]')
    plt.grid(axis = 'both')

    plt.figure('v',figsize = fig_sz) # if what we measure is v
    plt.plot(t_se,v_se,'b+')
    plt.xlabel('t [sec]')
    plt.ylabel('v [m/s]')
    plt.grid(axis = 'both')
    # if '/u' in topics:    # if what we measure is v_x
    #     plt.figure('v',figsize = fig_sz)
    #     bta         = np.arctan( 0.5 * np.tan(d_f) )
    #     plt.plot(t_se,v_se*np.cos(bta),'b+')
    #     plt.xlabel('t [sec]')
    #     plt.ylabel('v_x [m/s]')
    #     plt.grid(axis = 'both')

if '/ecu' in topics:
    plt.figure( figsize = fig_sz)
    plt.subplot(211)
    plt.plot(t_ecu, acc_ecu)
    plt.xlabel('t [sec]')
    plt.ylabel('acceleration ECU')
    plt.grid(axis = 'both')
    plt.subplot(212)
    plt.plot(t_ecu, d_f_ecu)
    plt.xlabel('t [sec]')
    plt.ylabel('steering angle ECU')
    plt.grid(axis = 'both')

# if '/meas' in topics and '/u' in topics:    # transform measurement to v
#     plt.figure('v')
#     plt.hold('on')
#     bta         = np.arctan( 0.5 * np.tan(d_f) )
#     plt.plot(t_meas,v_meas*np.cos(d_f)/np.cos(bta),'k*')

# if '/u' in topics:
#     plt.figure('d_f')
#     plt.hold('on')
#     plt.plot(t_u,d_f*180/pi)

#     if '/imu/data' in topics:
#         plt.figure(figsize = fig_sz)
#         plt.plot(t_imu, lin_acc_x)
#         plt.hold('on')
#         plt.plot(t_u,acc)
#         plt.xlabel('t [sec]')
#         plt.ylabel('linear acceleration')
#         plt.grid(axis = 'both')

# if '/encoder' in topics:
#     plt.figure(figsize = fig_sz)
#     plt.plot(t_enc,n_FL)
#     plt.hold('on')
#     plt.plot(t_enc,n_FR)
#     plt.plot(t_enc,n_BL)
#     plt.plot(t_enc,n_BR)
#     plt.xlabel('t [sec]')
#     plt.ylabel('encoder counting')
#     plt.grid(axis = 'both')
#     plt.legend(('n_FL','n_FR','n_BL','n_BR'))


plt.show()

