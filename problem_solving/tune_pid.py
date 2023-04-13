import math
import numpy as np
import matplotlib.pyplot as plt

def tvc_dynamics(Kp, Ki, Kd, initial, transient, edf_thrust, x_com, moi):
    #dynamic variables
    dt = 0.001   #accounts for servo-delay
    time = [0]
    thrust = []
    pitch = []
    tvc_pitch = []
    ang_vel = [0]
    ang_acc = [0]
    #tvc dynamics
    pitch.append(initial)
    tvc_pitch.append(initial)
    iteration = int(transient/dt)
    for i in range(iteration):
        thrust.append(edf_thrust*math.sin(math.radians(tvc_pitch[i])))
        ang_acc.append(math.degrees((thrust[i]*x_com)/moi))
        ang_vel.append(ang_vel[i] + ang_acc[i+1]*dt)
        pitch1 = pitch[i] + ang_vel[i+1]*dt
        limit = 30
        if(abs(pitch1) > limit):
            if(pitch1 < -limit):
                pitch1 = -limit
            if(pitch1 > limit):
                pitch1 = limit
        pitch.append(pitch1)
        tvc_pitch.append(Kp*pitch[i+1] + Kd*(pitch[i+1] - pitch[i])/dt + Ki*pitch[i+1]*dt)
        time.append(time[i] + dt)

    return time, pitch, thrust, ang_vel, ang_acc

Kp = -0.5
Ki = 0
Kd = -0.5/7

run_time = 1.5
start_pitch = 10
edf_thrust = 4*9.81
distance_from_com = 0.1
moment_of_inertia = 0.002

time, pitch, thrust, ang_vel, ang_acc = tvc_dynamics(Kp, Ki, Kd, start_pitch, run_time, edf_thrust, distance_from_com, moment_of_inertia)
plt.plot(time, pitch, color=(0.5, 0.7, 1.0))
plt.xlabel('Time (s)')
plt.ylabel('Pitch (Degrees)')
plt.legend()
plt.show()