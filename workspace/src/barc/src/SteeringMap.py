#!/usr/bin/env python

def servo2df(servo,args):
    '''
    --------
    inputs:
    --------

    servo        -- servo_pwm input on the scale of [1000, 2000].

    args:

    servo_km1    -- servo_pwm input of the last time step.

    curve        -- indicates on which curve the previous servo_pwm was.
                    -1: left steep curve
                     0: curve with low slope
                    +1: right steep curve

    bm           -- current offset of the curve with the low gradient (curve 0)
 
    --------
    outputs:
    --------

    d_f          -- steering angle in [rad]

    args         -- (see above, but for next function call)
 
    --------
    usage:
    --------

    print(servo2df(1520, (1500,0,)))

    '''

    # General idea behind this steering map concept:
    # The two steep relationships between servo_pwm and d_f stay the same lines 
    # all the time. But once the car changes turning direction, the relationship 
    # moves along the curve with the low gradient (0) until it reaches one of the steep 
    # curves again (-1,1).

    # read function arguments
    (servo_km1,curve,bm) = args

    # define curve slopes and offsets (left, right, middle curve) - from SteeringMapID
    # al = -0.000979158381869845
    bl = 1.69779532882438 #1.62779532882438 #1.52814338208387
    ar = -0.00110818379877912 #-0.00112818379877912 #-0.00120369154293885
    al = ar
    br = 1.79593612045077 #1.91686589764569
    am = -0.000371249151551210

    print servo
    # print (bl-br)/(ar-am)

    # curve -1:
    def dfl(servo_pwm):
        return al*servo_pwm+bl

    # curve 1:
    def dfr(servo_pwm):
        return ar*servo_pwm+br

    # curve 0:
    def dfm(servo_pwm,bm):
        return am*servo_pwm+bm

    # figure out whether servo_pwm increases or decreases
    servo_hist_current = servo - servo_km1

    if servo_hist_current >= 0 and curve == 1:
        # The steering action performed is in the steep area of the Steering Map.
        # (Turning right (not necessarily globally)) Moreover, the servo_pwm input
        # keeps increasing.
        d_f = dfr(servo)

    elif servo_hist_current <= 0 and curve == -1:
        # The steering action performed is in the steep area of the Steering Map.
        # (Turning left (not necessarily globally)) Moreover, the servo_pwm input
        # keeps decreasing.
        d_f = dfl(servo)

    elif curve == 0:
        # The steering action performed is in the area with a low slope of the 
        # Steering Map. (Transition area)

        # compute boundaries on this area (in servo_pwm space)
        UB = (br-bm)/(am-ar)
        LB = (bl-bm)/(am-al)
        # print UB, LB
        # check if boundaries are violated or not
        if servo >= LB and servo <= UB:
            # move along the curve with low gradient (curve 0)
            d_f = dfm(servo,bm)
        elif servo < LB:
            # move along the left curve (curve -1)
            d_f = dfl(servo)
            curve = -1
        else:
            # move along the right curve (curve 1)
            d_f = dfr(servo)
            curve = 1

    elif servo_hist_current < 0 and curve == 1:
        # The steering action performed is in the steep area of the Steering Map.
        # (Turning right (not necessarily globally)) Moreover, the servo_pwm input
        # starts decreasing (enter the area with a low slope from the previous 
        # servo_pwm input)

        # compute new offset for middle curve (0)
        bm = dfr(servo_km1) - am*servo_km1
        # compute boundaries on this area (curve 0) (in servo_pwm space)
        UB = (br-bm)/(am-ar)
        LB = (bl-bm)/(am-al)
        if servo >= LB and servo <= UB:
            # enter the area with low gradient
            curve = 0
            # move along curve 0
            d_f = dfm(servo,bm)
        else:
            # steering action is so big, that it "jumped" right to the other curve
            curve = -1
            d_f = dfl(servo)

    elif servo_hist_current > 0 and curve == -1:
        # The steering action performed is in the steep area of the Steering Map.
        # (Turning left (not necessarily globally)) Moreover, the servo_pwm input
        # starts increasing (enter the area with a low slope from the previous 
        # servo_pwm input)

        # compute new offset for middle curve (0)
        bm = dfl(servo_km1) - am*servo_km1
        # compute boundaries on this area (curve 0) (in servo_pwm space)
        UB = (br-bm)/(am-ar)
        LB = (bl-bm)/(am-al)
        if servo >= LB and servo <= UB:
            # enter the area with low gradient
            curve = 0
            # move along curve 0
            d_f = dfm(servo,bm)
        else:
            # steering action is so big, that it "jumped" right to the other curve
            curve = 1
            d_f = dfr(servo)

    # save argument values for next function call in the args vector
    args = (servo,curve,bm) 

    return d_f, args

