import cvxpy as cp
import carla
import numpy as np
from typing import List, Tuple


class VehicleModel:
    def __init__(self, vehicle: carla.Vehicle):
        self.vehicle = vehicle
        self.wheelbase = 2.9  # 轴距(m)
        self.max_steer_angle = np.deg2rad(45.0)  # 最大转向角(rad)
        self.max_throttle = 1.0  # 最大油门
        self.max_brake = 1.0  # 最大刹车
        self.max_acceleration = 1.0  # 最大加速度(m/s^2)
        self.max_deceleration = -1.0  # 最大减速度(m/s^2)
        self.lf = 1.5  # 前轮到车辆质心的距离(m)
        self.lr = 1.4  # 后轮到车辆质心的距离(m)
        self.nx = 6  # 状态向量维度
        self.nu = 2  # 控制输入维度
        self.previous_u = np.zeros(self.nu)  # 上一时刻的控制输入
        self.dt = 0.1  # 时间步长(s)

    def update_state(self, dt: float) -> Tuple[float, float, float]:
        """
        更新车辆状态
        :param dt: 时间间隔(s)
        :return: (x, y, yaw) 车辆位置和航向角
        """
        # 获取当前车辆状态
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()

        # 车辆当前位置和航向角
        x = transform.location.x
        y = transform.location.y
        yaw = transform.rotation.yaw

        # 车辆当前速度大小和方向
        speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
        direction = np.array(
            [np.cos(np.deg2rad(yaw)), np.sin(np.deg2rad(yaw))])

        # 更新车辆位置
        x += speed * direction[0] * dt
        y += speed * direction[1] * dt

        return x, y, yaw

    def apply_control(self, throttle: float, steer: float, brake: float):
        """
        应用控制指令
        :param throttle: 油门(0~1)
        :param steer: 转向角(-1~1)
        :param brake: 刹车(0~1)
        """
        # 转向角转换
        steer_angle = steer * self.max_steer_angle

        # 油门和刹车转换
        throttle_control = np.clip(throttle, 0, self.max_throttle)
        brake_control = np.clip(brake, 0, self.max_brake)

        # 应用控制指令
        self.vehicle.apply_control(carla.VehicleControl(
            throttle=throttle_control,
            steer=steer_angle,
            brake=brake_control
        ))

    def objective_function(self, x: np.ndarray, u: np.ndarray, target_speed: float, target_waypoints: List[Tuple[float, float]]) -> float:
        """
        目标函数
        :param x: 车辆状态向量 (x, y, yaw, v)
        :param u: 控制输入向量 (a, delta)
        :param target_speed: 目标速度
        :param target_waypoints: 目标路点列表,每个路点为(x, y)元组
        :return: 目标函数值
        """
        # 状态追踪误差权重
        Q_x = 1.0  # 横向位置误差权重
        Q_y = 1.0  # 纵向位置误差权重
        Q_yaw = 1.0  # 航向角误差权重
        Q_v = 0.5  # 速度误差权重

        # 控制输入变化率权重
        R_a = 0.1  # 加速度变化率权重
        R_delta = 0.1  # 转向角变化率权重

        # 舒适性约束权重
        Q_acc = 1.0  # 加速度约束权重
        Q_jerk = 1.0  # 加加速度约束权重

        # 计算状态追踪误差
        x_error = x[0] - target_waypoints[0][0]
        y_error = x[1] - target_waypoints[0][1]

        # 计算航向角误差
        if len(target_waypoints) > 1:
            yaw_error = x[2] - np.arctan2(target_waypoints[1][1] - target_waypoints[0]
                                          [1], target_waypoints[1][0] - target_waypoints[0][0])
        else:
            yaw_error = 0

        v_error = x[3] - target_speed

        # 计算控制输入变化率
        a_diff = u[0] - self.previous_u[0]
        delta_diff = u[1] - self.previous_u[1]

        # 计算加速度和加加速度
        acc = u[0]
        jerk = (u[0] - self.previous_u[0]) / self.dt

        # 目标函数值
        cost = Q_x * x_error**2 + Q_y * y_error**2 + Q_yaw * yaw_error**2 + Q_v * v_error**2 + \
            R_a * a_diff**2 + R_delta * delta_diff**2 + \
            Q_acc * acc**2 + Q_jerk * jerk**2

        return cost

    def velocity_constraints(self, x: np.ndarray, v_min: float, v_max: float) -> Tuple[np.ndarray, np.ndarray]:
        v = x[3]  # 当前速度
        v_lower = np.maximum(v_min, v - self.max_deceleration * self.dt)
        v_upper = np.minimum(v_max, v + self.max_acceleration * self.dt)
        return v_lower, v_upper

    def acceleration_constraints(self, u_min: float, u_max: float) -> Tuple[np.ndarray, np.ndarray]:
        a_lower = np.ones(self.N) * u_min
        a_upper = np.ones(self.N) * u_max
        return a_lower, a_upper

    def steering_constraints(self, u_min: float, u_max: float) -> Tuple[np.ndarray, np.ndarray]:
        delta_lower = np.ones(self.N) * u_min
        delta_upper = np.ones(self.N) * u_max
        return delta_lower, delta_upper

    def lateral_acceleration_constraints(self, x: np.ndarray, u: np.ndarray, ay_max: float) -> Tuple[np.ndarray, np.ndarray]:
        v = x[3]  # 当前速度
        delta = u[1]  # 转向角
        ay = v**2 * np.tan(delta) / self.wheelbase  # 横向加速度
        ay_lower = np.ones(self.N) * (-ay_max)
        ay_upper = np.ones(self.N) * ay_max
        return ay_lower, ay_upper

    def tire_slip_angle_constraints(self, x: np.ndarray, u: np.ndarray, alpha_max: float) -> Tuple[np.ndarray, np.ndarray]:
        v = x[3]  # 当前速度
        delta = u[1]  # 转向角
        alpha_f = np.arctan2(v * np.sin(delta), v *
                             np.cos(delta) + self.lf * delta) - delta  # 前轮侧偏角
        alpha_r = np.arctan2(v * np.sin(delta), v *
                             np.cos(delta) - self.lr * delta)  # 后轮侧偏角
        alpha_lower = np.ones(self.N) * (-alpha_max)
        alpha_upper = np.ones(self.N) * alpha_max
        return alpha_lower, alpha_upper


class MPCController:
    def __init__(self, model, N=10, dt=0.03):
        self.model = model
        self.N = N  # 预测时域步数
        self.dt = dt  # 时间步长

    def solve(self, x0, target_speed, target_waypoints):
        """
        求解MPC问题
        :param x0: 初始状态向量
        :param target_speed: 目标速度
        :param target_waypoints: 目标路点列表
        :return: 最优控制序列的第一个元素
        """
        # 定义优化变量
        x = cp.Variable((self.model.nx, self.N + 1))
        u = cp.Variable((self.model.nu, self.N))

        # 定义目标函数
        cost = 0
        for k in range(self.N):
            cost += self.model.objective_function(
                x[:, k], u[:, k], target_speed, target_waypoints[:2])

        # 定义约束条件
        constraints = []
        for k in range(self.N):
            # 动力学约束
            constraints += [x[:, k + 1] ==
                            self.model.dynamics(x[:, k], u[:, k])]

            # 状态约束
            v_min, v_max = self.model.velocity_constraints(x[:, k], 0, 20)
            constraints += [v_min <= x[3, k], x[3, k] <= v_max]

            # 控制输入约束
            a_min, a_max = self.model.acceleration_constraints(-1, 1)
            delta_min, delta_max = self.model.steering_constraints(
                -np.pi / 4, np.pi / 4)
            constraints += [a_min <= u[0, k], u[0, k] <= a_max]
            constraints += [delta_min <= u[1, k], u[1, k] <= delta_max]

            # 横向加速度约束
            ay_min, ay_max = self.model.lateral_acceleration_constraints(
                x[:, k], u[:, k], 5)
            constraints += [ay_min <= x[4, k], x[4, k] <= ay_max]

            # 轮胎侧偏角约束
            alpha_min, alpha_max = self.model.tire_slip_angle_constraints(
                x[:, k], u[:, k], np.deg2rad(5))
            constraints += [alpha_min <= x[5, k], x[5, k] <= alpha_max]

        # 初始状态约束
        constraints += [x[:, 0] == x0]

        # 求解优化问题
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        # 返回最优控制序列的第一个元素
        if problem.status == cp.OPTIMAL or problem.status == cp.OPTIMAL_INACCURATE:
            u_opt = u[:, 0].value
        else:
            u_opt = np.zeros(self.model.nu)

        return u_opt
