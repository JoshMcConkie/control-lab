import mujoco
import mujoco.viewer as viewer
import numpy as np
import time
import matplotlib.pyplot as plt
from math import pi
model = mujoco.MjModel.from_xml_path("CartPole/cart_pole.xml")
data = mujoco.MjData(model)
data.qpos[1] = .1

# Future PID controller parameters
Kp = 100
Ki = 1
Kd = 5

Kdx = 1.0

# # desired trajectory
# def desired_trajectory(t):
#     theta = 0
#     return theta

# def pid_control(target, current, prev_error, integral, dt):
#     error = target - current # pi-current % (2*pi)
#     integral += error*dt
#     error_dot = (error-prev_error)/dt
#     control_signal = Kp * error + Ki * integral + Kd * error_dot
#     return control_signal, error, integral


CONTROL_ENABLED = False

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    prev_error = 0
    integral = 0

    # logging
    trajectory_theta = []
    velocity_theta = []
    trajectory_x = []
    signals = []

    while viewer.is_running():
        current_time = time.time() - start_time
        dt = model.opt.timestep
        current_angle = data.qpos[1]
        current_velocity = data.qvel[1]
        x, xdot = data.qpos[0], data.qvel[0]

        trajectory_theta.append(current_angle)
        velocity_theta.append(current_velocity)
        # control signal
        if CONTROL_ENABLED:
            control_signals = []
            control_signal = Kp * current_angle + Kd * current_velocity - Kdx*xdot
            control_signals.append(control_signal)
            #  log data
            signals.append(control_signals)
            
        x = np.linspace(0, 10, 500)

        # CHANGE TO True TO TRIGGER LOGGING
        if ((True) and (len(trajectory_theta) == 500)):
            plt.plot(x, trajectory_theta)
            # plt.plot(x, trajectory)
            plt.plot(x, signals, label="Signal")
            plt.legend()
            plt.title("Signals and trajectory until over 1000 timesteps")
            plt.show()

        # apply control
        if CONTROL_ENABLED:
            data.ctrl[0] = control_signals[0] # first actuator is force on cart

        # Step
        mujoco.mj_step(model, data)

        # Update
        viewer.sync()
        time.sleep(dt)