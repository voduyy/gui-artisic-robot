import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import re
import time
import numpy as np
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import global_var

# ===== Định nghĩa robot =====
L1, L2, L3, L4, L5 = 0.211, 0.15, 0.0, 0.23, 0.07
laser_extension = 0.075  # 7.5 cm
deg = np.pi / 180
robot = DHRobot([
    RevoluteDH(d=L1, a=0, alpha=-np.pi / 2, offset=np.pi / 2, qlim=[-90 * deg, 90 * deg]),
    RevoluteDH(d=0, a=L2, alpha=0, offset=-np.pi / 2, qlim=[-90 * deg, 90 * deg]),
    RevoluteDH(d=0, a=L4, alpha=np.pi / 2, offset=0, qlim=[-120 * deg, 120 * deg]),
], name='3DOF_Robot')
q = np.zeros(3)  # Khởi tạo nghiệm ban đầu là 0
# ===== Hệ số chuyển đổi góc → bước (mm) =====
STEP_CONVERT = {
    'X': 0.355555556,
    'Y': 0.7166666666666667,
    'Z': 0.2222222222222222,

}
# ===== Hàm tính G-code giữ G0/G1 và ưu tiên nghiệm gần nhất =====
def compute_gcode_line(cmd, x, y, z, q0=None, max_attempts=10):
    # Mục tiêu: Y luôn là [0, -1, 0]
    y_axis = np.array([0, -1, 0])  # Hướng Y của đầu cuối
    z_axis = np.array([0, 0, 1])
    x_axis = np.cross(y_axis, z_axis).astype(float)
    x_axis /= np.linalg.norm(x_axis)  # chuẩn hóa
    R_goal = np.column_stack((x_axis, y_axis, z_axis))
    T_goal = SE3(x, y, z)

    # Thêm backlash cho động cơ
    BACKLASH_X_DEG = 7 / 60  # ~ 0.1167 độ
    BACKLASH_Y_DEG = 3 / 60  # ~ 0.05 độ
    prev_q_deg = None  # Để lưu góc khớp trước đó

    for attempt in range(max_attempts):
        ik_result = robot.ikine_LM(T_goal, q0=q0, mask=[1, 1, 1, 0, 0, 0])
        if ik_result.success:
            T_actual = robot.fkine(ik_result.q)
            y_actual = T_actual.R[:, 1]  # vector cột thứ 2 là trục Y
            angle_with_y_axis = np.degrees(np.arccos(np.clip(np.dot(y_actual, np.array([0, 1, 0])), -1.0, 1.0)))
            print(f"🎯 Góc giữa trục Y của end-effector và trục Y toàn cục: {angle_with_y_axis:.1f}°")
        if not ik_result.success:
            continue
        q_deg = np.degrees(ik_result.q)
        # Thêm backlash vào góc khớp
        if prev_q_deg is not None:
            if np.sign(q_deg[0] - prev_q_deg[0]) != np.sign(prev_q_deg[0]):
                q_deg[0] += BACKLASH_X_DEG * np.sign(q_deg[0] - prev_q_deg[0])
            if np.sign(q_deg[1] - prev_q_deg[1]) != np.sign(prev_q_deg[1]):
                q_deg[1] += BACKLASH_Y_DEG * np.sign(q_deg[1] - prev_q_deg[1])
        prev_q_deg = q_deg.copy()

        if -90 < q_deg[0] < 90 and -120 < q_deg[1] < 120 and -120 < q_deg[2] < 120:
            x_step = -q_deg[0] * STEP_CONVERT['X']
            y_step = -q_deg[1] * STEP_CONVERT['Y']
            z_step = q_deg[2] * STEP_CONVERT['Z']
            gcode_line = f"{cmd} X{x_step:.2f} Y{y_step:.2f} Z{z_step:.2f} F2000"
            return gcode_line, q_deg, ik_result.q

    return None, None, None

def main():
    # ===== Đọc và xử lý file G-code =====
    # Lấy đường dẫn thư mục chứa file script đang chạy
    base_dir = os.path.dirname(os.path.abspath(__file__))

    # Đi lên một cấp thư mục
    parent_dir = os.path.abspath(os.path.join(base_dir, ".."))
    path_input_file = os.path.join(parent_dir, 'Image2Gcode', 'output_gcode')
    input_file = os.path.join(path_input_file, f"{global_var.index_capture_image}_gcode.nc")
    filename = f"{global_var.index_capture_image}_gcode.nc"
    print(input_file)

    pattern = re.compile(
        r"^(G0|G1)\s+.*?X([-+]?\d*\.?\d+)\s+Y([-+]?\d*\.?\d+)(?:\s+Z([-+]?\d*\.?\d+))?",
        re.IGNORECASE
    )

    gcode_lines = []
    q_list = []
    q0 = None  # Khởi tạo nghiệm ban đầu là None

    # ===== Giai đoạn 1: Đọc và tính min/max G-code =====
    x_gcode_vals = []
    y_gcode_vals = []
    z_gcode_vals = []

    with open(input_file, "r") as f:
        gcode_raw_lines = f.readlines()
    for line in gcode_raw_lines:
        match = pattern.search(line.strip())
        if match:
            x_gcode_vals.append(float(match.group(2)))
            y_gcode_vals.append(float(match.group(3)))
            if match.group(4):
                z_gcode_vals.append(float(match.group(4)))
            else:
                z_gcode_vals.append(0)  # Nếu không có Z thì mặc định là 0

    # ===== Tính tâm và kích thước gốc =====
    x_min, x_max = min(x_gcode_vals), max(x_gcode_vals)
    y_min, y_max = min(y_gcode_vals), max(y_gcode_vals)
    z_min, z_max = min(z_gcode_vals), max(z_gcode_vals)

    # Tính tọa độ tâm
    x_center = (x_min + x_max) / 2
    y_center = (y_min + y_max) / 2
    z_center = (z_min + z_max) / 2
    gcode_width = max(x_max - x_min, y_max - y_min)
    print(f"🧠 GCODE_WIDTH: {gcode_width:.1f} mm | Tâm gốc: ({x_center:.1f}, {y_center:.1f}, {z_center:.1f})")

    # ===== Giai đoạn 2: Biến đổi và sinh G-code ====
    DRAW_WIDTH = 0.1  # mét
    SCALE = DRAW_WIDTH / gcode_width

    def set_axes_equal(ax):
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        y_range = abs(y_limits[1] - y_limits[0])
        z_range = abs(z_limits[1] - z_limits[0])

        max_range = max([x_range, y_range, z_range])

        x_middle = np.mean(x_limits)
        y_middle = np.mean(y_limits)
        z_middle = np.mean(z_limits)

        ax.set_xlim3d([x_middle - max_range / 2, x_middle + max_range / 2])
        ax.set_ylim3d([y_middle - max_range / 2, y_middle + max_range / 2])
        ax.set_zlim3d([z_middle - max_range / 2, z_middle + max_range / 2])

    x_offset = None  # Khởi tạo offset X là None
    z_offset = None  # Khởi tạo offset X là None
    # ===== Di chuyển đến điểm tâm ban đầu =====
    x_init, y_init, z_init = 0, 0.275, 0.12
    init_line, q_deg_init, q_rad_init = compute_gcode_line("G1", x_init, y_init, z_init)
    if init_line:
        print(f"🚀 Di chuyển đến tâm: {init_line}")
        print("🔧 Góc khớp (deg): q1 = {:.2f}, q2 = {:.2f}, q3 = {:.2f}".format(*q_deg_init))
        gcode_lines.append(init_line)
        q_list.append(q_deg_init)
        q0 = q_rad_init  # Cập nhật nghiệm gần nhất
    else:
        print("❌ Không thể di chuyển đến điểm tâm đầu (0, 0.2, 0.15)")
    # ===== Đọc lại G-code và xử lý từng dòng =====
    for line in gcode_raw_lines:
        line = line.strip()
        match = pattern.search(line)
        if not match:
            if line.startswith(("M3", "M5", "G28")):
                gcode_lines.append(line)
            continue

        cmd = match.group(1).upper()
        x_gcode = float(match.group(2))
        y_gcode = float(match.group(3))

        # Scale và dịch tâm
        x = (x_gcode - x_center) * SCALE
        z = (y_gcode - y_center) * SCALE + 0.12  # đặt tâm tại Z = 0.12
        y = 0.275  # chiều cao cố định

        gcode_line, q_deg, q_rad = compute_gcode_line(cmd, x, y, z, q0=q0)
        if q_rad is None:
            print(f"❌ IK thất bại tại điểm ({x:.3f}, {y:.3f}, {z:.3f})")
        else:
            print(f"✅ {gcode_line}")
            print("🔧 Góc khớp (deg): q1 = {:.2f}, q2 = {:.2f}, q3 = {:.2f}".format(*q_deg))
            print(f"🔧 Thành công tại điểm: ({x:.3f}, {y:.3f}, {z:.3f})")
            gcode_lines.append(gcode_line)
            q_list.append(q_deg)
            q0 = q_rad

    # ===== Ghi file kết quả G-code =====
    output_file = f"final_gcode/face_gcode/{global_var.index_capture_image}.txt"
    print(output_file)
    with open(output_file, "w") as f:
        for line in gcode_lines:
            f.write(line + "\n")
    print(f"\n✅ Đã lưu {len(gcode_lines)} dòng vào '{output_file}'")
if __name__ == "__main__":
    start_time = time.time()
    main()