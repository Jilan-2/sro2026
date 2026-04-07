# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# %%
# 1. Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

# %%
# 2. Start Simulation
sim.startSimulation()
print("Simulation Started")

# %%
# 3. Simple Test: Post a message to CoppeliaSim status bar
sim.addLog(1, "Hello from Python!")
p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")
p3dx = sim.getObject("/PioneerP3DX")

rw = 0.195/2
rb = 0.381/2
d = 0.05
dt = 0.01
x_dot_int = 0
y_dot_int = 0
gama_int = 0  

x_odom = []
y_odom = []

# %%
# 4. Data storage
t_data = []
wr_data = []
wl_data = []
vx_data = []
w_data = []
x_data = []
y_data = []
gamma_data = []

try:
    start_time = time.time()
    elapsed_prev = 0.0
    while (time.time() - start_time) < 45:

        # --- STUDENT CODE GOES HERE ---
        # Example: Print elapsed time
        elapsed = time.time() - start_time
        print(f"Running... {elapsed:.1f}s", end="\r")

        dt = elapsed - elapsed_prev 
        elapsed_prev = elapsed

        # Joint velocity
        wr_vel = sim.getJointTargetVelocity(p3dx_RW)
        wl_vel = sim.getJointTargetVelocity(p3dx_LW)

        # Body velocity
        vx = (wr_vel + wl_vel) * rw / 2
        w  = (wr_vel - wl_vel) * rw / rb

        #Get Orientation
        euler_angle = sim.getObjectOrientation(p3dx, -1)

        # Orientation integration
        # x_dot = vx*math.cos(euler_angle[2])  
        # y_dot = vx*math.sin(euler_angle[2]) 

        # Angular integration
        x_dot = vx*math.cos(gama_int)  
        y_dot = vx*math.sin(gama_int)

        #Pose integration
        x_dot_int = x_dot_int + x_dot * dt
        y_dot_int = y_dot_int + y_dot * dt
        gama_int = gama_int + w * dt

        # Update odometry
        x_odom.append(x_dot_int)
        y_odom.append(y_dot_int)

        # sim.addLog(1, f"RW:{wr_vel:.1f}, LW:{wl_vel:.1f}, Vx:{vx:.1f} m/s, W:{w:.1f} rad/s")
        sim.addLog(1, f" x_dot: {x_dot:.2f} m/s, y_dot: {y_dot:.2f} m/s")
        sim.addLog(1, f" x_int: {x_dot_int:.2f} m, y_int: {y_dot_int:.2f} m")
        sim.addLog(1, f" gama_int: {gama_int:.2f} rad")
        # Save data
        t_data.append(elapsed)
        wr_data.append(wr_vel)
        wl_data.append(wl_vel)
        vx_data.append(vx)
        w_data.append(w)
        x_data.append(x_dot_int)
        y_data.append(y_dot_int)
        gamma_data.append(gama_int)

        time.sleep(0.2)

finally:
    sim.stopSimulation()
    print("Simulation Stopped")

# %%
# 5. Odometry Path Plot

plt.figure(figsize=(10,6))
plt.plot(x_odom, y_odom, 'b-', label='Odometry Path')
plt.title("Odometry Path")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.legend()
plt.axis('equal')  # Equal aspect ratio
plt.grid(True)
plt.show()

