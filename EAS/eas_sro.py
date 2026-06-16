import math
import requests
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ==========================================
# 1. KONFIGURASI LLM (QWEN)
# ==========================================
QWEN_API_URL = "http://127.0.0.1:1234/v1/chat/completions" 
QWEN_API_KEY = "lm-studio"
MODEL_NAME = "qwen/qwen3-vl-4b"

def minta_qwen(perintah_teks, total_disc):
    # HARUS ADA huruf 'f' sebelum tanda kutip tiga agar variabel terbaca!
    system_prompt = f"""Kamu adalah mesin ekstraksi data. 
    Ubah perintah user menjadi format baku: jenis_misi,nilai_misi

    Pilihan jenis_misi HANYA: id, jumlah, jarak, terjauh.

    CONTOH WAJIB:
    User: "tolong temukan disc 2" -> id,2
    User: "cari 2 disc terdekat" -> jumlah,2
    User: "cari disc terdekat" -> jumlah,1
    User: "pergi ke disk paling dekat" -> jumlah,1
    User: "sapu area radius 5 meter" -> jarak,5
    User: "temukan disc paling jauh" -> terjauh,0
    User: "cari semua disc" -> jumlah,{total_disc}
    User: "ambil semua disk yang ada" -> jumlah,{total_disc}

    ATURAN MUTLAK: Jawab HANYA dengan format "kata,angka".
    """
    
    payload = {
        "model": MODEL_NAME,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": perintah_teks}
        ],
        "temperature": 0
    }
    
    headers = {"Authorization": f"Bearer {QWEN_API_KEY}", "Content-Type": "application/json"}
    response = requests.post(QWEN_API_URL, json=payload, headers=headers)
    response.raise_for_status() 
    
    teks_hasil = response.json()["choices"][0]["message"]["content"].strip()
    parts = teks_hasil.split(',')
    
    if len(parts) >= 2:
        return {"jenis_misi": parts[0].strip().lower(), "nilai_misi": float(parts[1].strip())}
    else:
        raise ValueError(f"Format salah: '{teks_hasil}'")

# ==========================================
# 2. SETUP COPPELIASIM
# ==========================================
print("Membuka koneksi ke CoppeliaSim...")
client = RemoteAPIClient()
sim = client.require('sim')

# Ambil handles untuk robot, motor, dan target
robot_handle = sim.getObject('/PioneerP3DX')
motor_kiri = sim.getObject('/PioneerP3DX/leftMotor')
motor_kanan = sim.getObject('/PioneerP3DX/rightMotor')
# Ambil handles target (Otomatis mendeteksi berapapun jumlah Disc di scene!)
disc_handles = []
idx = 0
while True:
    try:
        # Mencoba mencari Disc[0], Disc[1], dst sampai gagal (tidak ada lagi di scene)
        handle = sim.getObject(f'/Disc[{idx}]')
        disc_handles.append(handle)
        idx += 1
    except Exception:
        break # Berhenti mencari jika error (berarti disc sudah habis)
        
TOTAL_DISC = len(disc_handles)
print(f"[SYSTEM] Berhasil mendeteksi {TOTAL_DISC} objek Disc di dalam arena!")
us_sensors = [sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]') for i in range(0, 8)]

# ==========================================
# 3. COMMAND CENTER LOOP
# ==========================================
while True:
    print("\n" + "="*70)
    print("EAS SRO: Disc Navigation dengan Obstacle Avoidance terintegrasi LLM")
    print("="*70)
    perintah_user = input("Masukkan perintah misi (ketik 'exit' untuk keluar): ")
    
    if perintah_user.lower() == 'exit':
        print("Program dimatikan.")
        break
        
    # --- TRANSLASI LLM ---
    try:
        hasil = minta_qwen(perintah_user, TOTAL_DISC)
        mode_misi = hasil['jenis_misi']
        nilai_misi = hasil['nilai_misi']
        print(f"[TARGET QWEN] Mode: {mode_misi.upper()}, Nilai: {nilai_misi}")
    except Exception as e:
        print(f"[ERROR] LLM menolak/gagal: {e}\nSilakan eksekusi ulang perintah.")
        continue
        
    # --- EKSEKUSI SIMULASI ---
    sim.setStepping(True)
    sim.startSimulation()
    
    x_map, y_map = [], []
    discs_found_status = [False, False, False, False]
    misi_selesai = False
    target_terjauh_terkunci = -1
    step_count = 0
    
    try:
        while not misi_selesai:
            # Update Posisi dan Orientasi (State)
            pos_robot = sim.getObjectPosition(robot_handle, sim.handle_world)
            yaw_robot = sim.getObjectOrientation(robot_handle, sim.handle_world)[2]
            
            x_map.append(pos_robot[0])
            y_map.append(pos_robot[1])
            
            # Kalkulasi Jarak ke Semua Disc
            pos_discs = [sim.getObjectPosition(h, sim.handle_world) for h in disc_handles]
            dist_discs = [math.sqrt((p[0]-pos_robot[0])**2 + (p[1]-pos_robot[1])**2) for p in pos_discs]
            
            # Cek Status Penemuan Disc
            for i in range(4):
                if dist_discs[i] < 0.15 and not discs_found_status[i]:
                    discs_found_status[i] = True
                    print(f"\n[INFO SENSOR] -> Disc[{i}] ditemukan!")

            target_idx = -1
            
            # --- LOGIKA PENENTUAN TARGET ---
            if mode_misi == 'jumlah':
                if sum(discs_found_status) >= int(nilai_misi):
                    print(f"\n[MISI SUKSES] Kuota {int(nilai_misi)} disk terpenuhi!")
                    misi_selesai = True
                    break
                min_dist = float('inf')
                for i in range(4):
                    if not discs_found_status[i] and dist_discs[i] < min_dist:
                        min_dist, target_idx = dist_discs[i], i

            elif mode_misi == 'id':
                t_id = int(nilai_misi)
                if t_id < 0 or t_id > 3:
                    print(f"\n[WARNING] Disc[{t_id}] tidak ada! Misi dibatalkan.")
                    break
                if discs_found_status[t_id]:
                    print(f"\n[MISI SUKSES] Tiba di target utama Disc[{t_id}]!")
                    misi_selesai = True
                    break
                target_idx = t_id

            elif mode_misi == 'jarak':
                disk_tersisa = False
                min_dist = float('inf')
                for i in range(4):
                    if not discs_found_status[i] and dist_discs[i] <= nilai_misi:
                        disk_tersisa = True
                        if dist_discs[i] < min_dist:
                            min_dist, target_idx = dist_discs[i], i
                if not disk_tersisa:
                    print(f"\n[MISI SUKSES] Area radius {nilai_misi}m disapu bersih!")
                    misi_selesai = True
                    break
                    
            elif mode_misi == 'terjauh':
                # Kunci target agar tidak plin-plan di tengah jalan
                if target_terjauh_terkunci == -1:
                    max_dist = -1
                    for i in range(4):
                        if not discs_found_status[i] and dist_discs[i] > max_dist:
                            max_dist, target_terjauh_terkunci = dist_discs[i], i
                target_idx = target_terjauh_terkunci
                if target_idx != -1 and discs_found_status[target_idx]:
                    print(f"\n[MISI SUKSES] Disc paling jauh [{target_idx}] berhasil dicapai!")
                    misi_selesai = True
                    break

            if target_idx == -1 or target_idx >= len(pos_discs):
                print("\n[WARNING] Tidak ada target yang valid. Menghentikan manuver.")
                break

            # --- KALKULASI VEKTOR TARGET ---
            pos_target = pos_discs[target_idx]
            jarak_ke_target = dist_discs[target_idx] 
            sudut_ke_target = math.atan2(pos_target[1] - pos_robot[1], pos_target[0] - pos_robot[0])
            
            error_heading = sudut_ke_target - yaw_robot
            if error_heading > math.pi: error_heading -= 2 * math.pi
            if error_heading < -math.pi: error_heading += 2 * math.pi

            # --- PEMBACAAN SENSOR ULTRASONIK ---
            dist_all = []
            for i in range(8):
                res, dist, _, _, _ = sim.readProximitySensor(us_sensors[i])
                dist_all.append(dist if res > 0 else 1.0)
                
            dist_kiri = min(dist_all[0:3])   # Sayap Kiri
            dist_depan = min(dist_all[3:5])  # Moncong Depan
            dist_kanan = min(dist_all[5:8])  # Sayap Kanan

            # --- MOTOR & LOGIKA MENGHINDAR CERDAS ---
            v_base = 2.0
            
            # KONDISI 1: Terjebak sudut mati (Corner Trap) atau menabrak lurus
            if dist_depan < 0.35 or (dist_kiri < 0.4 and dist_kanan < 0.4):
                if dist_kiri > dist_kanan:
                    v_kiri, v_kanan = -1.5, 1.5 # Putar di tempat ke kiri
                else:
                    v_kiri, v_kanan = 1.5, -1.5 # Putar di tempat ke kanan
            
            # KONDISI 2: Menyusuri dinding kiri atau kanan
            elif dist_kiri < 0.6 or dist_kanan < 0.6:
                error_ruang = dist_kanan - dist_kiri
                v_kiri = v_base + (error_ruang * 2.0)
                v_kanan = v_base - (error_ruang * 2.0)
            
            # KONDISI 3: Ruang bebas, memburu target
            else:
                if abs(error_heading) < 0.1: 
                    error_heading = 0.0 # Deadband untuk menstabilkan jalan lurus
                kp_heading = 1.0
                v_kiri = v_base - (error_heading * kp_heading)
                v_kanan = v_base + (error_heading * kp_heading)

            # Batasi kecepatan putaran maksimum
            v_kiri_final = max(-2.0, min(v_kiri, 2.0))
            v_kanan_final = max(-2.0, min(v_kanan, 2.0))
            
            sim.setJointTargetVelocity(motor_kiri, v_kiri_final)
            sim.setJointTargetVelocity(motor_kanan, v_kanan_final)
            
            # --- LOG TELEMETRI (Sesuai Syarat Rubrik) ---
            if step_count % 30 == 0:
                lin_vel, ang_vel = sim.getObjectVelocity(robot_handle)
                kecepatan_aktual = math.sqrt(lin_vel[0]**2 + lin_vel[1]**2)
                sensor_report = [round(d, 2) for d in dist_all]
                
                print("\n" + "-"*45)
                print(f"MISI: Menuju Disc[{target_idx}] | Jarak: {jarak_ke_target:.2f}m")
                print(f"[STATE] Posisi Robot : X: {pos_robot[0]:.2f}, Y: {pos_robot[1]:.2f}")
                print(f"[STATE] Kecepatan    : {kecepatan_aktual:.2f} m/s")
                print(f"[STATE] Ultrasonic   : {sensor_report}")
                print(f"[ACTION] Joint Twist : Kiri {v_kiri_final:.2f} rad/s, Kanan {v_kanan_final:.2f} rad/s")
                print("-" * 45)
                
            sim.step()
            step_count += 1

    except KeyboardInterrupt:
        print("\n[STOP] Misi dibatalkan secara manual oleh user (Ctrl+C).")
        
    finally:
        # Hentikan robot
        sim.setJointTargetVelocity(motor_kiri, 0.0)
        sim.setJointTargetVelocity(motor_kanan, 0.0)
        sim.stopSimulation()
        print("Simulasi dihentikan. Mempersiapkan grafik hasil pemetaan...")
        
        # Plotting Matplotlib
        if len(x_map) > 0:
            plt.figure(figsize=(9, 6))
            plt.plot(y_map, x_map, color='purple', linewidth=2.5, label='Jalur Jelajah P3DX')
            plt.scatter(y_map[0], x_map[0], color='green', marker='o', s=150, label='Titik Start', zorder=5)
            
            warna_dasar = ['red', 'blue', 'orange', 'forestgreen', 'purple', 'cyan', 'magenta', 'yellow']
            colors = [warna_dasar[i % len(warna_dasar)] for i in range(TOTAL_DISC)]
            
            for i in range(TOTAL_DISC):
                # HANYA plot disc jika statusnya sudah ditemukan (True)
                if discs_found_status[i]:
                    plt.scatter(pos_discs[i][1], pos_discs[i][0], c=colors[i], marker='X', 
                                s=150, label=f'Disc[{i}]', zorder=4)
                        
            plt.gca().invert_xaxis()
            plt.xlabel('Nilai Koordinat Y Dunia (Meter)')
            plt.ylabel('Nilai Koordinat X Dunia (Meter)')
            plt.title(f'Hasil Misi: {mode_misi.upper()}\n(Tutup jendela ini untuk melanjutkan ke misi berikutnya!)', fontweight='bold')
            plt.legend(loc='upper right', bbox_to_anchor=(1.35, 1))
            plt.grid(True, linestyle='--', alpha=0.7)
            plt.axis('equal')
            plt.tight_layout()
            
            # Program akan tertahan di sini sampai grafik ditutup
            plt.show()
            print("\n[SISTEM] Grafik ditutup. Siap menerima instruksi selanjutnya!")