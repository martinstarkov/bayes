import math
import numpy as np
import matplotlib.pyplot as plt

class PID_tuner:
    #defines the simulation conditions
    def __init__(self, run_time, start_pitch, edf_thrust, distance_com, moment_of_inertia):
        self.transient = run_time
        self.initial = start_pitch
        self.thrust = edf_thrust
        self.x_com = distance_com
        self.moi = moment_of_inertia

    #simulation physics
    def tvc_dynamics(self, Kp, Ki, Kd):
        #dynamic variables
        dt = 0.01   #accounts for servo-delay
        time = [0]
        thrust = []
        pitch = []
        tvc_pitch = []
        ang_vel = [0]
        ang_acc = [0]
        #tvc dynamics
        pitch.append(self.initial)
        tvc_pitch.append(self.initial)
        iteration = int(self.transient/dt)
        for i in range(iteration):
            thrust.append(self.thrust*math.sin(math.radians(tvc_pitch[i])))
            ang_acc.append(math.degrees((thrust[i]*self.x_com)/self.moi))
            ang_vel.append(ang_vel[i] + ang_acc[i+1]*dt)
            pitch.append(pitch[i] + ang_vel[i+1]*dt)
            tvc_pitch.append(Kp*pitch[i+1] + Kd*(pitch[i+1] - pitch[i])/dt + Ki*pitch[i+1]*dt)
            time.append(time[i] + dt)
        return time, pitch, thrust, ang_vel, ang_acc
    
    #A method to measure loss
    def residual(self, pitch):
        total_residual = sum(abs(theta) for theta in pitch)
        return total_residual

    def loss_function(self, Kp, Ki, Kd):
        time, pitch, thrust, ang_vel, ang_acc = self.tvc_dynamics(Kp, Ki, Kd)
        return self.residual(pitch)
    
    def gradient_function(self, Kp, Ki, Kd):
        Kp_epsilon = 0.1
        Ki_epsilon = 0.001
        Kd_epsilon = 0.01
        grad_Kp = float((self.loss_function(Kp + Kp_epsilon, Ki, Kd) - self.loss_function(Kp - Kp_epsilon, Ki, Kd))/(2*Kp_epsilon))
        grad_Ki = float((self.loss_function(Kp, Ki + Ki_epsilon, Kd) - self.loss_function(Kp, Ki - Ki_epsilon, Kd))/(2*Ki_epsilon))
        grad_Kd = float((self.loss_function(Kp, Ki, Kd + Kd_epsilon) - self.loss_function(Kp, Ki, Kd - Kp_epsilon))/(2*Kd_epsilon))
        return grad_Kp, grad_Ki, grad_Kd

    def gradient_descent(self, initial_gain, learning_rate, max_iteration):
        gain = initial_gain
        Kp, Ki, Kd = gain 
        for i in range(max_iteration):
            loss = self.loss_function(Kp, Ki, Kd)
            grad1, grad2, grad3 = self.gradient_function(Kp, Ki, Kd)
            Kp -= learning_rate*grad1
            Ki -= learning_rate*grad2
            Kd -= learning_rate*grad3
        return loss, Kp, Ki, Kd

#Please insert simulation conditions here:
run_time = 1.5
start_pitch = 2
edf_thrust = 2
distance_from_com = 0.3
moment_of_inertia = 0.02

#Simlation runs here. Please do not change these settings here:
Simulation = PID_tuner(run_time, start_pitch, edf_thrust, distance_from_com, moment_of_inertia)
loss1, Kp1, Ki1, Kd1 = Simulation.gradient_descent([0.1, 0.001, 0.01], 0.0001, 1000)
loss2, Kp2, Ki2, Kd2 = Simulation.gradient_descent([0.1, 0.001, 0.01], 0.0001, 10000)
loss3, Kp3, Ki3, Kd3 = Simulation.gradient_descent([0.1, 0.001, 0.01], 0.0001, 100000)

print(f'{loss1}, {Kp1}, {Ki1}, {Kd1}')
print(f'{loss2}, {Kp2}, {Ki2}, {Kd2}')
print(f'{loss3}, {Kp3}, {Ki3}, {Kd3}')

time1, pitch1, thrust1, ang_vel1, ang_acc1 = Simulation.tvc_dynamics(Kp1, Ki1, Kd1)
time2, pitch2, thrust2, ang_vel2, ang_acc2 = Simulation.tvc_dynamics(Kp2, Ki2, Kd2)
time3, pitch3, thrust3, ang_vel3, ang_acc3 = Simulation.tvc_dynamics(Kp3, Ki3, Kd3)

#A graph is generated here:
plt.plot(time1, pitch1, color=(0.5, 0.7, 1.0), label='1000 iterations')
plt.plot(time2, pitch2, color=(0.2, 0.4, 0.8), label='10000 iterations')
plt.plot(time3, pitch3, color=(0.0, 0.2, 0.6), label='100000 iterations')
plt.xlabel('Time (s)')
plt.ylabel('Pitch (Degrees)')
plt.legend()
plt.show()

#Most optimal value so far has been Kp = 16.6, Ki = 0.167, Kd = 0.936.