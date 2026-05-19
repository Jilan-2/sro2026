# %%
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Inisialisasi koneksi remote API ke CoppeliaSim
client = RemoteAPIClient()
sim = client.require('sim')

# Memulai simulasi
sim.startSimulation()
print("Simulation Started")

# Fungsi pembentuk matriks transformasi homogen 4x4
def transformMat(alpha, beta, gamma, tx, ty, tz):
    # Matriks rotasi individual elemen (3x3)
    rotx = np.array([
        [1, 0, 0],
        [0, math.cos(alpha), -math.sin(alpha)],
        [0, math.sin(alpha),  math.cos(alpha)]
        ])
    roty = np.array([
        [ math.cos(beta), 0, math.sin(beta)],
        [0, 1, 0],
        [-math.sin(beta), 0, math.cos(beta)]
        ])
    rotz = np.array([
        [math.cos(gamma), -math.sin(gamma), 0],
        [math.sin(gamma),  math.cos(gamma), 0],
        [0,0,1]
        ])
    
    # Penggabungan matriks rotasi total
    rot_total = np.matmul(rotx, roty)
    rot_total = np.matmul(rot_total, rotz)
    
    # Vektor translasi (3x1)
    trans_vector = np.array([
                    [tx],
                    [ty],
                    [tz]
                    ])
    
    # Menggabungkan rotasi dan translasi menjadi matriks 3x4
    R_t_3x4  = np.hstack((rot_total, trans_vector))
    
    # Baris pelengkap elemen homogen [0 0 0 1]
    homogeneous_row = np.array([[0, 0, 0, 1]])
    
    # Stack vertikal akhir untuk membentuk matriks 4x4
    transform_matrix_4x4 = np.vstack((R_t_3x4, homogeneous_row))
    return transform_matrix_4x4


# Cek koneksi awal ke status bar CoppeliaSim
sim.addLog(1, "Hello from Python!")

# Ambil object handle dari scene simulasi
p3dx = sim.getObject("/PioneerP3DX")
p3dx_rw = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_lw = sim.getObject("/PioneerP3DX/leftMotor")
LH_Handle = sim.getObject("/LH")
perp_Handle = sim.getObject("/Perp")

# Load koordinat waypoint secara dinamis untuk mencegah error indeks terputus
path_Handle = []
i = 0
missing_counter = 0

while i < 200: 
    try:
        handle = sim.getObject(f"/p[{i}]")
        path_Handle.append(handle)
        missing_counter = 0 
    except:
        missing_counter += 1
        if missing_counter > 15: 
            break
    i += 1

JUMLAH_WAYPOINT = len(path_Handle)
print(f"Successfully loaded {JUMLAH_WAYPOINT} waypoints dynamically!")

# Konfigurasi sensor ultrasonik [0, 3, 4, 7]
sensor_handles = [
    sim.getObject("/PioneerP3DX/ultrasonicSensor[0]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[3]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[4]"),
    sim.getObject("/PioneerP3DX/ultrasonicSensor[7]")
]

# Sudut pemasangan relatif masing-masing sensor terhadap robot yaw (dalam radian)
sensor_angles = [math.radians(90), math.radians(30), math.radians(-30), math.radians(-90)]

# Spesifikasi dimensi robot Pioneer P3DX
rw = 0.195/2
rb = 0.318/2
d = 0.05

dt = 0.01
x_dot_int = 0.0
y_dot_int = 0.0
gamma_int = 0.0

# Jarak target Look-Ahead (Pure Pursuit parameter)
LH_distance = 0.6

# Buffer data penampung koordinat untuk visualisasi peta akhir
robot_path_x = []
robot_path_y = []
obstacles_x = []
obstacles_y = []

# =========================================================================
# Optimasi Rute Waypoint Menggunakan Algoritma Nearest Neighbor
# =========================================================================
raw_points = []
for i in range(len(path_Handle)):
    raw_points.append(sim.getObjectPosition(path_Handle[i], sim.handle_world))

p3dx_start_pos = sim.getObjectPosition(p3dx, sim.handle_world)
current_pos = np.array(p3dx_start_pos[:2])

ordered_points = []
unvisited = list(raw_points)

while len(unvisited) > 0:
    closest_idx = 0
    min_d = float('inf')
    for idx, pt in enumerate(unvisited):
        d_val = math.sqrt((pt[0] - current_pos[0])**2 + (pt[1] - current_pos[1])**2)
        if d_val < min_d:
            min_d = d_val
            closest_idx = idx
    
    chosen_point = unvisited.pop(closest_idx)
    ordered_points.append(chosen_point)
    current_pos = np.array(chosen_point[:2])

# Menutup loop jalur kembali ke titik awal
ordered_points.append(ordered_points[0])

# %%
try:
    # Loop Utama Kendali & Mapping
    start_time = time.time()
    elapsed_prev = 0.0
    
    # Batas durasi eksekusi simulasi (detik)
    DURASI_SIMULASI = 75 
    
    while (time.time() - start_time) < DURASI_SIMULASI:
        
        elapsed = time.time() - start_time
        dt = elapsed - elapsed_prev
        elapsed_prev = elapsed

        # Membaca posisi dan orientasi terkini robot
        p3dx_position = sim.getObjectPosition(p3dx, sim.handle_world)
        p3dx_orientation = sim.getObjectOrientation(p3dx, sim.handle_world)
        robot_x, robot_y = p3dx_position[0], p3dx_position[1]
        robot_yaw = p3dx_orientation[2]

        # Log posisi robot untuk plotting lintasan
        robot_path_x.append(robot_x)
        robot_path_y.append(robot_y)

        # Menghitung posisi koordinat global titik Look-Ahead
        LH_position_to_world = transformMat(0, 0, p3dx_orientation[2], p3dx_position[0], p3dx_position[1], p3dx_position[2]) @ np.array([[LH_distance], [0], [0], [1]])
        LH_position_to_world = LH_position_to_world[:3, :]

        path_points = ordered_points
        
        # Kalkulasi komponen vektor segmen garis rute A->B
        vec_AB = []
        for i in range(len(path_points)-1):
            A_pt = np.array(path_points[i])
            B_pt = np.array(path_points[i+1])
            vec_AB.append(B_pt - A_pt)
        
        # Kalkulasi komponen vektor target posisi A->LH
        vec_ALH = []
        for i in range(len(path_points)-1):
            A_pt = np.array(path_points[i])
            vec_ALH.append(LH_position_to_world - A_pt.reshape(3,1))
            
        # Proyeksi skalar ortogonal untuk mencari titik terdekat pada segmen rute
        scalar_proj_points = []
        for i in range(len(vec_AB)):
            ab = vec_AB[i].flatten()
            alh = vec_ALH[i].flatten()
            
            ab_norm_sq = np.dot(ab, ab)
            if ab_norm_sq == 0:
                scalar_proj = 0
            else:
                scalar_proj = np.dot(alh, ab) / ab_norm_sq
                
            # Batasi proyeksi di dalam rentang segmen garis rute aktif
            if scalar_proj < 0:
                scalar_proj = 0
            elif scalar_proj > 1:
                scalar_proj = 1
                
            A_pt = np.array(path_points[i]).reshape(3,1)
            scalar_proj_point = A_pt + scalar_proj * vec_AB[i].reshape(3,1)
            scalar_proj_points.append(scalar_proj_point)
                    
        # Evaluasi indeks koordinat proyeksi paling optimal terhadap target LH
        closest_index = 0
        min_distance = np.linalg.norm(scalar_proj_points[0] - LH_position_to_world)
        for i in range(1, len(scalar_proj_points)):
            distance = np.linalg.norm(scalar_proj_points[i] - LH_position_to_world)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        desired_position = scalar_proj_points[closest_index]

        # Transformasi balik matriks posisi target ke frame koordinat lokal robot
        T_world_robot = transformMat(0,0, p3dx_orientation[2], p3dx_position[0], p3dx_position[1], p3dx_position[2])
        desired_position_wrt_robot = np.linalg.inv(T_world_robot) @ np.append(desired_position, np.array([[1]]), axis=0)
        desired_position_wrt_robot = desired_position_wrt_robot[:3, :]

        # Perhitungan nilai galat (error) posisi dan orientasi heading
        ed = math.sqrt(desired_position_wrt_robot[0][0]**2 + desired_position_wrt_robot[1][0]**2)
        eh = math.atan2(desired_position_wrt_robot[1][0], desired_position_wrt_robot[0][0])

        # Penentuan parameter kontrol kecepatan linear dan angular dasar
        vx = 0.6 * ed
        wx = 1.2 * eh

        # =========================================================================
        # Algoritma Obstacle Avoidance & Pemetaan Metrik Lingkungan
        # =========================================================================
        obstacle_detected = False
        avoid_direction = 0.0
        JARAK_AMAN = 0.55

        for idx, s_handle in enumerate(sensor_handles):
            result, distance, _, _, _ = sim.readProximitySensor(s_handle)
            
            if result > 0 and distance < JARAK_AMAN:
                obstacle_detected = True
                
                # Kalkulasi bobot arah manuver berdasarkan pembacaan indeks sensor
                if idx == 0:    # Sisi samping kiri
                    avoid_direction -= 0.15
                elif idx == 1:  # Sisi depan kiri
                    avoid_direction -= 0.65
                elif idx == 2:  # Sisi depan kanan
                    avoid_direction += 0.65
                elif idx == 3:  # Sisi samping kanan
                    avoid_direction += 0.15

                # Menghitung proyeksi posisi absolut koordinat rintangan ke world frame
                absolute_sensor_angle = robot_yaw + sensor_angles[idx]
                obs_x = robot_x + (distance * math.cos(absolute_sensor_angle))
                obs_y = robot_y + (distance * math.sin(absolute_sensor_angle))
                
                # Registrasi titik objek ke buffer array plot
                obstacles_x.append(obs_x)
                obstacles_y.append(obs_y)

        # Modifikasi kontrol kecepatan saat mode menghindar aktif
        if obstacle_detected:
            vx = 0.12  
            wx = avoid_direction

        # Penerapan saturasi pembatas kecepatan demi stabilitas motor
        vx = max(min(vx, 0.4), -0.4)
        wx = max(min(wx, 1.0), -1.0)

        # Transformasi invers kinematik kecepatan roda kanan dan kiri
        wr_vel = (vx + (rb*wx)/2)/rw   
        wl_vel = (vx - (rb*wx)/2)/rw

        # Implementasi output kecepatan ke aktuator joint motor simulator
        sim.setJointTargetVelocity(p3dx_rw, wr_vel)
        sim.setJointTargetVelocity(p3dx_lw, wl_vel)

        # Sinkronisasi posisi visual dummy handle objek di simulator
        sim.setObjectPosition(LH_Handle, sim.handle_world, LH_position_to_world.flatten().tolist())
        sim.setObjectPosition(perp_Handle, sim.handle_world, desired_position.flatten().tolist())
        
        time.sleep(0.01)

finally:
    # Prosedur mematikan aktuator dan menghentikan simulasi dengan aman
    sim.setJointTargetVelocity(p3dx_rw, 0)
    sim.setJointTargetVelocity(p3dx_lw, 0)
    sim.stopSimulation()
    print("\nSimulation Stopped")
    
    # =========================================================================
    # Render Output Grafik Matplotlib (2D Exploration Metric Map)
    # =========================================================================
    plt.figure(figsize=(10, 7))
    
    # Plot data titik rintangan perimeter objek (Scatter)
    if len(obstacles_x) > 0:
        plt.scatter(obstacles_x, obstacles_y, color='black', s=10, label='Obstacles Detected')
    
    # Plot visualisasi garis lintasan penjelajahan robot (Line)
    if len(robot_path_x) > 0:
        plt.plot(robot_path_x, robot_path_y, color='blue', linewidth=2, label='Robot Path')
    
    # Atribut layout plot kartesius
    plt.title('B202 Exploration Map', fontsize=12)
    plt.xlabel('X Position (meters)', fontsize=10)
    plt.ylabel('Y Position (meters)', fontsize=10)
    
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.axis('equal')
    
    plt.show()
# %%