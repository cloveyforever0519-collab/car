import socket
import json
import time
import math
import numpy as np

class SpeedAndDistanceController:
    def __init__(self):
        # --- 参数配置 ---
        self.target_v_max = 60.0 / 3.6  # 目标车速 (16.67 m/s)
        self.switch_distance = 150.0    # 触发减速的距离 (200米)
        self.decel_rate = 0.15          # 减速平滑度 (数值越大减速越快)

        # --- 内部状态 ---
        self.state = "ACCEL"            # 初始状态：加速
        self.current_target_v = self.target_v_max
        self.start_pose = None
        self.is_finished = False

        # --- 网络配置 ---
        self.telem_addr = ("127.0.0.1", 5000)
        self.ctrl_addr = ("127.0.0.1", 5001)
        self.telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telem_sock.bind(self.telem_addr)
        self.telem_sock.setblocking(False)
        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.count = 0
        print(f">>> 控制器启动：目标 {self.target_v_max*3.6:.0f}km/h，行驶满 {self.switch_distance}m 自动减速。")

    def send_control(self, throttle, steer, brake):
        """发送控制指令"""
        msg = json.dumps({
            "throttle": round(float(throttle), 3),
            "steer": round(float(steer), 3),
            "brake": round(float(brake), 3)
        }).encode()
        self.ctrl_sock.sendto(msg, self.ctrl_addr)

    def control_step(self):
        if self.is_finished:
            return

        try:
            raw_data, _ = self.telem_sock.recvfrom(8192)
            while True:
                try:
                    raw_data, _ = self.telem_sock.recvfrom(8192)
                except BlockingIOError:
                    break
            telem = json.loads(raw_data.decode())

            # 1. 解析数据（这里使用了更健壮的查找方式）
            kin_node = next(v for k, v in telem.items() if "1_" in k)
            
            # 坐标
            pos_xyz = next(v for k, v in kin_node.items() if "1_" in k)
            curr_x, curr_y = pos_xyz[0], pos_xyz[1]

            # 速度
            vel_xyz = next(v for k, v in kin_node.items() if "3_" in k)
            curr_v = math.sqrt(sum(a ** 2 for a in vel_xyz))

            # 记录起点
            if self.start_pose is None:
                self.start_pose = {'x': curr_x, 'y': curr_y}
                print(f"起点已记录: X={curr_x:.2f}, Y={curr_y:.2f}")
                return

        except BlockingIOError:
            return
        except Exception as e:
            # 如果不触发，请看这里是否报错
            if self.count % 100 == 0:
                print(f"数据解析异常: {e}")
            return

        # 2. 计算行驶距离 (欧几里得距离，最稳妥)
        dx = curr_x - self.start_pose['x']
        dy = curr_y - self.start_pose['y']
        traveled_dist = math.sqrt(dx**2 + dy**2)

        throttle, brake = 0.0, 0.0

        # 3. 核心逻辑控制
        if self.state == "ACCEL":
            # 优先判定距离是否到达
            if traveled_dist >= self.switch_distance:
                self.state = "DECEL"
                self.current_target_v = curr_v  # 从当前实际速度开始减速
                print(f"\n[触发] 到达 {traveled_dist:.1f}m，进入减速模式。")
            
            # 速度维持逻辑
            else:
                v_err = self.target_v_max - curr_v
                if v_err > 0.1:
                    throttle = np.clip(0.6 * v_err, 0.0, 0.8)
                    brake = 0.0
                else:
                    # 速度先到了也会进入减速逻辑
                    self.state = "DECEL"
                    self.current_target_v = curr_v
                    print(f"\n[触发] 达到目标速度，开始减速。当前距离: {traveled_dist:.1f}m")

        elif self.state == "DECEL":
            # 线性下调目标速度
            if self.current_target_v > 0:
                self.current_target_v -= self.decel_rate
            else:
                self.current_target_v = 0

            v_err = self.current_target_v - curr_v

            if v_err > 0:
                throttle = 0.0
                brake = 0.05  # 微量刹车模拟阻力
            else:
                throttle = 0.0
                brake = np.clip(abs(v_err) * 0.6, 0.0, 0.5)

            # 彻底停止判定
            if self.current_target_v <= 0 and curr_v < 0.2:
                brake = 1.0
                self.is_finished = True
                print(f"\n>>> 任务完成：车辆已稳稳停下。总行驶里程: {traveled_dist:.2f}m")

        # 4. 执行发送
        self.send_control(throttle, 0.0, brake)

        # 5. 状态打印
        self.count += 1
        if self.count % 20 == 0:
            print(f"模式: {self.state} | 距离: {traveled_dist:.1f}/{self.switch_distance}m | 速度: {curr_v*3.6:.1f}km/h", end='\r')


if __name__ == "__main__":
    controller = SpeedAndDistanceController()
    try:
        while not controller.is_finished:
            controller.control_step()
            time.sleep(0.02)

        # 停止后持续锁死刹车
        print(">>> 正在保持制动...")
        for _ in range(50):
            controller.send_control(0.0, 0.0, 1.0)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n用户手动停止。")
