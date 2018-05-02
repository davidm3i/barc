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

from numpy import array, dot, eye, copy
from numpy import dot, zeros, add, sum, newaxis
from scipy.linalg import inv, cholesky
import rospy

C = array([[1, 0]])
B = eye(2)
def kinematicLuembergerObserver(vhat_x, vhat_y, w_z, a_x, a_y, v_x_enc, aph, dt):
    """
    inputs: 
        * current longitudinal velocity estimate vhat_x [m/s]
        * current lateral velocity estimate vhat_y [m/s]
        * yaw rate measurement from gyroscope [rad/s]
        * a_x measurement from IMU [m/s^2]
        * a_y measurement from IMU [m/s^2]
        * v_x estimate from encoder (v_x_enc)  [m/s]
    output:
        * longitudinal velocity estimate vhat_x at next time step [m/s]
        * lateral velocity estimate vhat_y at next time step [m/s]

    reference:
        Farrelly, Jim, and Peter Wellstead. "Estimation of vehicle lateral velocity."
        Control Applications, 1996. Proceedings of the 1996 IEEE International Conference
        on IEEE, 1996
        Equations (25) - (31)
    """

    # compute the observer gain
    # note, the reshape(-1,1) transforms the array into a n x 1 vector
    K = array([ 2*aph*abs(w_z), (aph**2 - 1)*w_z ]).reshape(-1,1)

    # if car is not moving, then acclerations should be zero 
    if v_x_enc == 0:
        a_x = 0
        a_y = 0
        vhat_x = 0
        vhat_y = 0

    # build system matrices
    A = array([[ 0,     w_z], \
               [-w_z,   0  ]])
    u = array([a_x, a_y]).reshape(-1,1)
    vhat_k = array([vhat_x, vhat_y]).reshape(-1,1)

    # apply observer
    vhat_kp1 = vhat_k + dt*( dot( (A - dot(K,C)), vhat_k) + dot(B,u) + K*v_x_enc)

    return vhat_kp1


def ekf(f, mx_km1, P_km1, h, y_k, Q, R, args):
    """
     EKF   Extended Kalman Filter for nonlinear dynamic systems
     ekf(f,mx,P,h,z,Q,R) returns state estimate, x and state covariance, P 
     for nonlinear dynamic system:
               x[k] = f(x[k-1],u[k-1]) + v[k-1]
               y[k] = h(x[k]) + w[k]
     where v ~ N(0,Q) meaning v is gaussian noise with covariance Q
           w ~ N(0,R) meaning w is gaussian noise with covariance R
    Inputs:    f: function handle for f(x)
               mx_km1: "a priori" state estimate
               P_km1: "a priori" estimated state covariance
               h: fanction handle for h(x)
               y_k: current measurement
               Q: process noise covariance 
               R: measurement noise covariance
               args: additional arguments to f(x, *args)
    Output:    mx_k: "a posteriori" state estimate
               P_k: "a posteriori" state covariance
               
    Notation: mx_k = E[x_k] and my_k = E[y_k], where m stands for "mean of"
    """
    
    # prior update
    xDim    = mx_km1.size                       # dimension of the state
    procm   = f(mx_km1, *args)                  # evaluate process model
    mx_k    = procm[0]                          # predict next state
    # A       = numerical_jac(f, mx_km1, *args)   # linearize process model about current state
    A       = procm[1]                          # linearize process model about current state
    P_k     = dot(dot(A,P_km1),A.T) + Q         # propagate variance

    # measurement update
    if args[3] != 8:
        # print args[3]
        measm   = h(mx_k, *args)                    # evaluate measurement model
        my_k    = measm[0]                          # predict future output
        # H       = numerical_jac(h, mx_k, *args)     # linearize measurement model about predicted next state
        H       = measm[1]                          # linearize measurement model about predicted next state
        P12     = dot(P_k, H.T)                     # cross covariance
        # print H, P12, my_k, R
        # print dot(H,P12) + R
        K       = dot(P12, inv( dot(H,P12) + R))    # Kalman filter gain
        mx_k    = mx_k + dot(K,(y_k - my_k))        # state estimate
        # print K, mx_k
        P_k     = dot(dot(K,R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_k)  ,  (eye(xDim) - dot(K,H)).T )

    # print mx_k

    return (mx_k, P_k)

def ukf(f, mx_km1, P_km1, h, y_k, Q, R, args):
    """
     EKF   Extended Kalman Filter for nonlinear dynamic systems
     ekf(f,mx,P,h,z,Q,R) returns state estimate, x and state covariance, P 
     for nonlinear dynamic system:
               x[k] = f(x[k-1],u[k-1]) + v[k-1]
               y[k] = h(x[k]) + w[k]
     where v ~ N(0,Q) meaning v is gaussian noise with covariance Q
           w ~ N(0,R) meaning w is gaussian noise with covariance R
    Inputs:    f: function handle for f(x)
               mx_km1: state estimate last time step
               P_km1: estimated state covariance last time step
               h: fanction handle for h(x)
               y_k: current measurement
               Q: process noise covariance 
               R: measurement noise covariance
               args: additional arguments to f(x, *args)
    Output:    mx_k: "a posteriori" state estimate
               P_k: "a posteriori" state covariance
               
    Notation: mx_k = E[x_k] and my_k = E[y_k], where m stands for "mean of"
    """
    
    # prior update
    # generate sigma-points
    xDim        = mx_km1.size                       # dimension of the state
    sqrtnP      = cholesky(xDim*P_km1)
    sm_km1      = list(add(mx_km1,sqrtnP))
    sm_km1.extend(list(add(mx_km1,-sqrtnP)))
    numSigP     = len(sm_km1)
    # compute prior sigma points
    sp_k        = []
    for s in sm_km1:
        sp_k.append(f(s, *args)[0])                 # evaluate process model for each sigma point
    # compute prior statistics
    mx_k        = 1.0/numSigP*sum(sp_k, axis=0)     # estimate prior mean from prior sigma points
    P_k         = zeros(P_km1.shape)
    for i in range(numSigP):                        # estimate prior variance from prior sigma points
        P_k     = P_k+1.0/numSigP*(array(sp_k[i])[newaxis].T-array(mx_k)[newaxis].T)*(array(sp_k[i])[newaxis]-array(mx_k)[newaxis])
    P_k = P_k + Q                                   # additive noise for testing purposes (add non-linear noise later)

    # measurement update
    if args[3] != 8:
        # print args[3]
        measm   = h(mx_k, *args)                    # evaluate measurement model
        my_k    = measm[0]                          # predict future output
        # H       = numerical_jac(h, mx_k, *args)     # linearize measurement model about predicted next state
        H       = measm[1]                          # linearize measurement model about predicted next state
        P12     = dot(P_k, H.T)                     # cross covariance
        # print H, P12, my_k, R
        # print dot(H,P12) + R
        K       = dot(P12, inv( dot(H,P12) + R))    # Kalman filter gain
        mx_k    = mx_k + dot(K,(y_k - my_k))        # state estimate
        # print K, mx_k
        P_k     = dot(dot(K,R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_k)  ,  (eye(xDim) - dot(K,H)).T )

    # print mx_k

    return (mx_k, P_k)


    
def numerical_jac(f,x, *args):
    """
    Function to compute the numerical jacobian of a vector valued function 
    using final differences
    """
    # numerical gradient and diagonal hessian
    y = f(x, *args)[0]
    
    jac = zeros( (y.size,x.size) )
    eps = 1e-5
    xp = copy(x)
    
    for i in range(x.size):
        xp[i] = x[i] + eps/2.0
        yhi = f(xp, *args)[0]
        xp[i] = x[i] - eps/2.0
        ylo = f(xp, *args)[0]
        xp[i] = x[i]
        jac[:,i] = (yhi - ylo) / eps
    return jac
