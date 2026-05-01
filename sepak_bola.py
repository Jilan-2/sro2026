import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Setup Connection
client = RemoteAPIClient()
sim = client.require('sim')

if sim.getSimulationState() == 0:
    sim.startSimulation()
    time.sleep(1)

# Object Handles
pemain = sim.getObject('/Robot_Pemain')
keeper = sim.getObject('/Robot_Lawan_01')
deff   = sim.getObject('/Robot_Lawan_02')
gawang = sim.getObject('/Gwang_Kuning')
bola   = sim.getObject('/Bola_Merah')
pagar  = sim.getObject('/Pagar_Lapangan')

l0 = sim.getObject('/Robot_Pemain/leftMotor')
r0 = sim.getObject('/Robot_Pemain/rightMotor')

l1 = sim.getObject('/Robot_Lawan_01/leftMotor')
r1 = sim.getObject('/Robot_Lawan_01/rightMotor')

l2 = sim.getObject('/Robot_Lawan_02/leftMotor')
r2 = sim.getObject('/Robot_Lawan_02/rightMotor')

gawang_posisi = sim.getObjectPosition(gawang, -1)
GOAL_X = gawang_posisi[0]
GOAL_Y = gawang_posisi[1]

# Ambil batas pagar
min_x, max_x, min_y, max_y = 0, 0, 0, 0
try:
    bb = sim.getObjectBoundingBox(pagar, -1) # -1 berarti relatif terhadap world
    min_x = bb[0]
    max_x = bb[1]
    min_y = bb[2]
    max_y = bb[3]
except:
    # Fallback jika fungsi bounding box spesifik tidak langsung merespons
    min_x, max_x = -5.5, 5.5
    min_y, max_y = -3.5, 3.5

# Ambil titik reset bola
init_bola_x, init_bola_y, _ = sim.getObjectPosition(bola, -1)

def pos(obj):
    p = sim.getObjectPosition(obj, -1)
    return p[0], p[1]

def yaw(obj):
    return sim.getObjectOrientation(obj, -1)[2]

def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def motor(l, r, vl, vr):
    sim.setJointTargetVelocity(l, vl)
    sim.setJointTargetVelocity(r, vr)

def goto(robot, l, r, tx, ty, speed=4.5):
    x, y = pos(robot)
    th = yaw(robot)

    target = math.atan2(ty - y, tx - x)
    err = wrap(target - th)

    turn = 3.5 * err * 0.8

    vl = speed - turn
    vr = speed + turn

    vl = max(min(vl, 8), -8)
    vr = max(min(vr, 8), -8)

    motor(l, r, vl, vr)

def stop_all():
    motor(l0, r0, 0, 0)
    motor(l1, r1, 0, 0)
    motor(l2, r2, 0, 0)

# Main Loop
try:
    while True:

        if sim.getSimulationState() == 0:
            time.sleep(0.1)
            continue

        px, py = pos(pemain)
        bx, by = pos(bola)
        gx, gy = pos(keeper)

        # Deteksi Bola Keluar
        if bx < min_x or bx > max_x or by < min_y or by > max_y:
            sim.setObjectPosition(bola, -1, [init_bola_x, init_bola_y, 0.05])

        # Jarak ke bola dan gawang
        d_bola = math.sqrt((bx - px)**2 + (by - py)**2)
        d_goal = math.sqrt((GOAL_X - px)**2 + (GOAL_Y - py)**2)

        # Hindari robot keeper
        keeper_dx = px - gx
        keeper_dy = py - gy
        keeper_dist = math.sqrt(keeper_dx**2 + keeper_dy**2)

        avoid_x = 0
        avoid_y = 0

        if keeper_dist < 1.5:
            avoid_x = keeper_dx / (keeper_dist + 0.001)
            avoid_y = keeper_dy / (keeper_dist + 0.001)

        # Arah gawang
        goal_x = GOAL_X - bx
        goal_y = GOAL_Y - by

        mag = math.sqrt(goal_x**2 + goal_y**2)
        if mag == 0:
            mag = 1

        goal_x /= mag
        goal_y /= mag

        dribble_dist = 0.25

        tx = bx - goal_x * dribble_dist + avoid_x * 0.4
        ty = by - goal_y * dribble_dist + avoid_y * 0.4

        # Menendang bola
        if d_bola > 0.3:

            speed = 3 + min(d_bola * 4, 5)
            speed = min(speed, 5.5)

            goto(pemain, l0, r0, tx, ty, speed)

        elif d_goal > 0.6:

            goto(pemain, l0, r0, GOAL_X, GOAL_Y, 3.8)

        else:

            shoot_y = GOAL_Y + (0.25 if by > GOAL_Y else -0.25)
            goto(pemain, l0, r0, GOAL_X, shoot_y, 6)

        # keeper
        keeper_x = 4.75
        keeper_y = max(min(by, 0.9), -2.1)
        goto(keeper, l1, r1, keeper_x, keeper_y, 2.5)

        # defender
        if bx > 0:
            dx = bx - 1.2
        else:
            dx = 1.5

        dy = max(min(by, 2.5), -2.5)
        goto(deff, l2, r2, dx, dy, 3)

        time.sleep(0.05)

except KeyboardInterrupt:
    pass

stop_all()
print("STOPPED")