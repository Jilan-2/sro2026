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

rw = 0.195/2
rb = 0.381/2
d = 0.05

# %%
# 4. Data storage
t_data = []
wr_data = []
wl_data = []
vx_data = []
w_data = []

try:
    start_time = time.time()
    while (time.time() - start_time) < 10:

        # --- STUDENT CODE GOES HERE ---
        # Example: Print elapsed time
        elapsed = time.time() - start_time
        print(f"Running... {elapsed:.1f}s", end="\r")

        # Joint velocity
        wr_vel = sim.getJointTargetVelocity(p3dx_RW)
        wl_vel = sim.getJointTargetVelocity(p3dx_LW)

        # Body velocity
        vx = (wr_vel + wl_vel) * rw / rb
        w  = (wr_vel - wl_vel) * rw / rb

        sim.addLog(1, f"RW:{wr_vel:.1f}, LW:{wl_vel:.1f}, Vx:{vx:.1f} m/s, W:{w:.1f} rad/s")

        # Save data
        t_data.append(elapsed)
        wr_data.append(wr_vel)
        wl_data.append(wl_vel)
        vx_data.append(vx)
        w_data.append(w)

        time.sleep(0.1)

finally:
    sim.stopSimulation()
    print("Simulation Stopped")

# %%
# 5. Plot Graphs

plt.figure()

plt.subplot(2,2,1)
plt.plot(t_data, wr_data)
plt.title("Right Wheel Velocity")
plt.xlabel("Time (s)")
plt.ylabel("φR (rad/s)")

plt.subplot(2,2,2)
plt.plot(t_data, wl_data)
plt.title("Left Wheel Velocity")
plt.xlabel("Time (s)")
plt.ylabel("φL (rad/s)")

plt.subplot(2,2,3)
plt.plot(t_data, vx_data)
plt.title("Linear Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Vx (m/s)")

plt.subplot(2,2,4)
plt.plot(t_data, w_data)
plt.title("Angular Velocity")
plt.xlabel("Time (s)")
plt.ylabel("ω (rad/s)")

plt.tight_layout()
plt.show()