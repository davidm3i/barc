#!/usr/bin/env python

from SteeringMap import servo2df
import numpy as np
import matplotlib.pyplot as plt

plt.ion()

# time step size
dt = 0.03
steps = 500
# initial (servo_pwm,curve,bm) (see SteeringMap.py)
servo_pwm_init = 1500
args = (servo_pwm_init,0,0.0772518905741900-(-0.000371249151551210)*servo_pwm_init)

servo_pwm = np.zeros(steps)
d_f = np.zeros(steps)

for k in range(steps):
    t = k*dt
    servo_pwm[k] = 300*np.sin(2*np.pi*1*t)+1500
    (d_f[k],args) = servo2df(servo_pwm[k],args)
    print args

    plt.figure(1)
    plt.plot(servo_pwm[k],d_f[k],'kx')
    plt.draw()

    raw_input("Press Enter")

plt.show()