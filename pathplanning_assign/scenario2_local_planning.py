"""
시나리오 2: Local Path Planning (DWA - Dynamic Window Approach)
이동 장애물이 있는 환경에서 지역 경로 계획

학생들이 파라미터를 조정하며 테스트할 수 있는 예제
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import time
import sys
import os
import warnings

# 한글 폰트 경고 무시
warnings.filterwarnings('ignore', category=UserWarning, message='.*Glyph.*missing from.*')

# matplotlib 한글 폰트 설정
plt.rcParams['axes.unicode_minus'] = False  # 마이너스 기호 깨짐 방지

# 상위 디렉토리의 common 모듈 import
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import Robot, Environment, RectangleObstacle, CircleObstacle, MovingObstacle

# 충돌 체크를 위한 alias
Circle = CircleObstacle
Rectangle = CircleObstacle  # 간단화를 위해 CircleObstacle 사용


class DWA:
    """Dynamic Window Approach 알고리즘"""
    
    def __init__(self, robot: Robot, config: dict, environment: Environment = None):
        """
        Args:
            robot: 로봇 객체
            config: DWA 설정 딕셔너리
            environment: 환경 객체 (정적 장애물 포함)
        """
        self.robot = robot
        self.config = config
        self.env = environment
        
    def calc_dynamic_window(self):
        """동적 윈도우 계산"""
        # 로봇의 동역학적 제약 (후진 허용)
        vs = [-self.robot.max_v * 0.5,  # 후진 가능 (최대 속도의 50%)
              self.robot.max_v,
              -self.robot.max_w,
              self.robot.max_w]
        
        # 현재 속도 기준 가속도 제약
        vd = [self.robot.v - self.config['max_accel'] * self.config['dt'],
              self.robot.v + self.config['max_accel'] * self.config['dt'],
              self.robot.w - self.config['max_delta_yaw_rate'] * self.config['dt'],
              self.robot.w + self.config['max_delta_yaw_rate'] * self.config['dt']]
        
        # 교집합
        dw = [max(vs[0], vd[0]),
              min(vs[1], vd[1]),
              max(vs[2], vd[2]),
              min(vs[3], vd[3])]
        
        return dw
        return dw
    
    def predict_trajectory(self, v: float, w: float):
        """주어진 속도로 궤적 예측
        
        Returns:
            trajectory: 예측된 궤적 [(x, y), ...]
        """
        x, y, theta = self.robot.x, self.robot.y, self.robot.theta
        trajectory = [(x, y)]
        time = 0
        
        while time <= self.config['predict_time']:
            x += v * np.cos(theta) * self.config['dt']
            y += v * np.sin(theta) * self.config['dt']
            theta += w * self.config['dt']
            trajectory.append((x, y))
            time += self.config['dt']
            
        return trajectory
    
    def calc_obstacle_cost(self, trajectory, environment):
        """
        장애물과의 거리에 따른 비용 계산
        """
        min_distance = float('inf')
        safety_margin = 0.8  # 안전 마진 증가 (로봇 반경 0.5m + 여유 0.3m)
        
        # 현재 시간과 예측 시간
        current_time = 0.0
        time_step = self.config['dt']
        
        for point in trajectory:
            x, y = point  # trajectory는 (x, y) 튜플의 리스트
            
            # 정적 장애물 확인
            if self.env:
                for obs in self.env.static_obstacles:
                    dist = float('inf')  # 초기화
                    if isinstance(obs, CircleObstacle):
                        dist = np.hypot(x - obs.x, y - obs.y) - obs.radius - self.robot.radius - safety_margin
                    elif isinstance(obs, RectangleObstacle):
                        # 사각형 장애물과의 최단 거리 계산
                        dx = max(obs.x - x, 0, x - (obs.x + obs.width))
                        dy = max(obs.y - y, 0, y - (obs.y + obs.height))
                        dist = np.hypot(dx, dy) - self.robot.radius - safety_margin
                    
                    if dist < min_distance:
                        min_distance = dist
                    
            # 동적 장애물 확인 - 미래 위치 예측
            if environment and hasattr(environment, 'dynamic_obstacles'):
                for obs in environment.dynamic_obstacles:
                    # 현재 위치와의 거리
                    obs_x, obs_y = obs.x, obs.y
                    
                    # 미래 위치 예측 (장애물의 현재 속도 사용)
                    future_x = obs_x + obs.vx * current_time
                    future_y = obs_y + obs.vy * current_time
                    
                    dist = np.hypot(x - future_x, y - future_y) - obs.radius - self.robot.radius - safety_margin
                    
                    if dist < min_distance:
                        min_distance = dist
            
            current_time += time_step
        
        # 충돌 또는 너무 가까우면 무한대 비용
        if min_distance <= 0:
            return float('inf')
        
        # 거리에 반비례하는 비용 (더 완만한 곡선)
        return 1.0 / (min_distance + 0.1)
    
    def calc_to_goal_cost(self, trajectory, goal):
        """목표 지향 비용 계산 (낮을수록 좋음)"""
        # 궤적의 끝점에서 목표까지의 거리
        last_pos = np.array(trajectory[-1])
        goal_pos = np.array(goal)
        
        dx = goal_pos[0] - last_pos[0]
        dy = goal_pos[1] - last_pos[1]
        dist = np.sqrt(dx**2 + dy**2)
        
        # 목표 방향과의 각도 차이도 고려
        goal_theta = np.arctan2(dy, dx)
        
        # 궤적의 마지막 방향
        if len(trajectory) >= 2:
            dx_traj = trajectory[-1][0] - trajectory[-2][0]
            dy_traj = trajectory[-1][1] - trajectory[-2][1]
            traj_theta = np.arctan2(dy_traj, dx_traj)
            angle_diff = abs(goal_theta - traj_theta)
            angle_diff = min(angle_diff, 2 * np.pi - angle_diff)
        else:
            angle_diff = 0
        
        return dist + angle_diff * 0.5
    
    def calc_velocity_cost(self, v: float):
        """속도 비용 (빠를수록 낮은 비용)"""
        return self.robot.max_v - v
    
    def plan(self, goal, environment: Environment):
        """DWA 알고리즘으로 최적 속도 계산
        
        Returns:
            best_v, best_w: 최적 선속도, 각속도
            best_trajectory: 최적 궤적
        """
        dw = self.calc_dynamic_window()
        
        best_v, best_w = 0.0, 0.0
        best_cost = float('inf')
        best_trajectory = None
        
        # 속도 공간 샘플링
        v_samples = np.linspace(dw[0], dw[1], self.config['v_reso'])
        w_samples = np.linspace(dw[2], dw[3], self.config['w_reso'])
        
        trajectories = []  # 시각화용
        valid_count = 0   # 유효한 궤적 수
        
        for v in v_samples:
            for w in w_samples:
                # 궤적 예측
                trajectory = self.predict_trajectory(v, w)
                
                # 비용 계산
                obstacle_cost = self.calc_obstacle_cost(trajectory, environment)
                
                # 충돌 궤적은 제외
                if obstacle_cost == float('inf'):
                    continue
                
                valid_count += 1
                goal_cost = self.calc_to_goal_cost(trajectory, goal)
                velocity_cost = self.calc_velocity_cost(v)
                
                # 가중치 적용
                total_cost = (self.config['to_goal_cost_gain'] * goal_cost +
                            self.config['obstacle_cost_gain'] * obstacle_cost +
                            self.config['speed_cost_gain'] * velocity_cost)
                
                trajectories.append(trajectory)
                
                # 최적 궤적 선택
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_v = v
                    best_w = w
                    best_trajectory = trajectory
        
        # 유효한 궤적이 없으면 회피 행동
        if valid_count == 0:
            print(f"⚠️  No valid trajectory! Emergency maneuver.")
            # 제자리 회전 시도
            best_v = 0.0
            best_w = self.robot.max_w * 0.5  # 최대 각속도의 50%로 회전
            best_trajectory = [(self.robot.x, self.robot.y)]
        elif valid_count < 3:  # 선택지가 매우 적으면
            print(f"⚠️  Limited options ({valid_count}). Cautious mode.")
        
        return best_v, best_w, best_trajectory, trajectories


class DWASimulator:
    """DWA 시뮬레이터"""
    
    def __init__(self, robot: Robot, environment: Environment, 
                 goal, dwa_config: dict):
        self.robot = robot
        self.env = environment
        self.goal = np.array(goal)
        self.dt = dwa_config['dt']
        self.dwa = DWA(robot, dwa_config, environment)
        self.completed = False
        self.best_trajectory = None
        self.all_trajectories = []
        
    def step(self):
        """시뮬레이션 한 스텝"""
        if self.completed:
            return True
        
        # 환경 업데이트 (이동 장애물)
        self.env.update(self.dt)
        
        # 충돌 검사 (먼저 확인)
        if self.env.check_collision(self.robot.get_position(), self.robot.radius):
            print("\n✗ Collision detected!")
            print(f"  - Position: ({self.robot.x:.2f}, {self.robot.y:.2f})")
            print(f"  - Distance traveled: {self.robot.total_distance:.2f}m")
            self.completed = True
            self.robot.v = 0.0
            self.robot.w = 0.0
            return True
        
        # 목표 도달 확인
        dist_to_goal = np.linalg.norm(self.robot.get_position() - self.goal)
        if dist_to_goal < 1.0:
            self.completed = True
            self.robot.v = 0.0
            self.robot.w = 0.0
            return True
        
        # DWA로 최적 속도 계산 (환경 전체 전달)
        v, w, best_traj, all_trajs = self.dwa.plan(self.goal, self.env)
        
        self.best_trajectory = best_traj
        self.all_trajectories = all_trajs
        
        # 로봇 상태 업데이트
        self.robot.update(v, w, self.dt)
        
        return False


def visualize_dwa(simulator: DWASimulator):
    """DWA 시뮬레이션 시각화"""
    
    fig, ax = plt.subplots(figsize=(12, 10))
    
    robot = simulator.robot
    env = simulator.env
    goal = simulator.goal
    
    def init():
        ax.clear()
        ax.set_xlim(0, env.width)
        ax.set_ylim(0, env.height)
        ax.set_aspect('equal')
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('DWA - Local Path Planning (Dynamic Obstacles)', fontsize=16, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # 고정 장애물
        for obs in env.static_obstacles:
            if isinstance(obs, CircleObstacle):
                circle = patches.Circle((obs.x, obs.y), obs.radius,
                                       facecolor='gray', edgecolor='black', linewidth=2)
                ax.add_patch(circle)
        
        # 목표점
        ax.scatter(goal[0], goal[1], c='red', s=300, marker='*', 
                  edgecolors='darkred', linewidths=2, label='Goal', zorder=10)
        
        return []
    
    # init 호출 전에 초기화
    init()
    
    # 그래픽 요소 (init 이후에 추가)
    robot_circle = patches.Circle((robot.x, robot.y), robot.radius,
                                 facecolor='blue', edgecolor='darkblue', linewidth=2, alpha=0.8)
    robot_direction = patches.FancyArrow(robot.x, robot.y,
                                        robot.radius * np.cos(robot.theta),
                                        robot.radius * np.sin(robot.theta),
                                        width=0.3, head_width=0.5, head_length=0.3,
                                        facecolor='yellow', edgecolor='orange', linewidth=1.5)
    trajectory_line, = ax.plot([], [], 'r-', linewidth=2, label='Trajectory', alpha=0.8)
    
    # 이동 장애물 그래픽 (주황색 원)
    dynamic_obs_patches = []
    for obs in env.dynamic_obstacles:
        circle = patches.Circle((obs.x, obs.y), obs.radius,
                               facecolor='orange', edgecolor='red', linewidth=2, alpha=0.7)
        dynamic_obs_patches.append(circle)
        ax.add_patch(circle)
    
    # 로봇 요소 추가
    ax.add_patch(robot_circle)
    ax.add_patch(robot_direction)
    ax.legend(loc='upper right', fontsize=9)
    
    def update(frame):
        # 시뮬레이션 스텝
        completed = simulator.step()
        
        # 로봇 업데이트
        robot_circle.center = (robot.x, robot.y)
        robot_direction.set_data(x=robot.x, y=robot.y,
                                dx=robot.radius * np.cos(robot.theta),
                                dy=robot.radius * np.sin(robot.theta))
        
        # 궤적 업데이트
        if len(robot.trajectory) > 1:
            traj = np.array(robot.trajectory)
            trajectory_line.set_data(traj[:, 0], traj[:, 1])
        
        # 이동 장애물 업데이트
        for i, obs in enumerate(env.dynamic_obstacles):
            dynamic_obs_patches[i].center = (obs.x, obs.y)
        
        # 예측 궤적 그리기 (가장 최근 것만)
        for line in ax.lines[1:]:  # 첫 번째는 Trajectory
            line.remove()
        
        # 모든 고려된 궤적 (연한 색)
        if simulator.all_trajectories:
            for traj in simulator.all_trajectories[-20:]:  # 최근 20개만
                if traj:
                    traj_array = np.array(traj)
                    ax.plot(traj_array[:, 0], traj_array[:, 1], 'c-', alpha=0.1, linewidth=0.5)
        
        # 선택된 최적 궤적 (진한 색)
        if simulator.best_trajectory:
            traj_array = np.array(simulator.best_trajectory)
            ax.plot(traj_array[:, 0], traj_array[:, 1], 'g-', alpha=0.6, linewidth=2)
        
        # 정보 텍스트
        info_text = f'Time: {frame * simulator.dt:.1f}s | Distance: {robot.total_distance:.2f}m\n'
        info_text += f'Velocity: {robot.v:.2f}m/s | Angular: {robot.w:.2f}rad/s'
        if hasattr(update, 'text'):
            update.text.remove()
        update.text = ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
                            fontsize=10, verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        if completed and not hasattr(update, 'completed'):
            update.completed = True
            if not simulator.env.check_collision(robot.get_position(), robot.radius):
                print(f"\n✓ Goal reached!")
                print(f"  - Total distance: {robot.total_distance:.2f}m")
                print(f"  - Total time: {frame * simulator.dt:.2f}s")
            # 애니메이션 정지
            anim.event_source.stop()
        
        return [robot_circle, robot_direction, trajectory_line, update.text] + dynamic_obs_patches
    
    anim = FuncAnimation(fig, update, frames=1000, interval=50, blit=False, repeat=False)
    
    # 애니메이션 객체를 figure에 저장하여 가비지 컬렉션 방지
    fig._animation = anim
    
    plt.tight_layout()
    plt.show()


def create_dynamic_environment():
    """이동 장애물이 있는 환경 생성 (랜덤)"""
    np.random.seed()  # 매번 다른 시드 사용
    env = Environment(width=50.0, height=50.0)
    
    # 정적 장애물 제거 (DWA 순수 테스트)
    
    # 이동 장애물들 (랜덤 생성)
    num_dynamic = np.random.randint(4, 7)  # 4~6개의 동적 장애물
    for _ in range(num_dynamic):
        x = np.random.uniform(10, 40)
        y = np.random.uniform(10, 40)
        radius = np.random.uniform(1.5, 2.5)
        vx = np.random.uniform(-1.5, 1.5)
        vy = np.random.uniform(-1.5, 1.5)
        # 시작점과 목표점 근처는 피하기
        if np.hypot(x - 5, y - 5) > 5 and np.hypot(x - 45, y - 45) > 5:
            env.add_dynamic_obstacle(MovingObstacle(x, y, radius, vx=vx, vy=vy))
    
    print(f"\n[환경 생성]")
    print(f"  - 고정 장애물: {len(env.static_obstacles)}개")
    print(f"  - 동적 장애물: {len(env.dynamic_obstacles)}개")
    
    return env


def main():
    """메인 함수"""
    print("=" * 60)
    print("시나리오 2: Local Path Planning (DWA)")
    print("=" * 60)
    
    # ============================================================
    # 학생들이 조정할 수 있는 DWA 파라미터
    # ============================================================
    DWA_CONFIG = {
        'max_accel': 0.8,              # 최대 가속도 (m/s^2) - 증가
        'max_delta_yaw_rate': 90.0 * np.pi / 180.0,  # 최대 각가속도 (rad/s^2) - 매우 증가
        'v_reso': 10,                  # 속도 해상도 (샘플 개수)
        'w_reso': 20,                  # 각속도 해상도 (샘플 개수)
        'dt': 0.1,                     # 시간 간격 (s)
        'predict_time': 2.5,           # 예측 시간 (s) - 증가 (동적 장애물 예측)
        'to_goal_cost_gain': 2.0,      # 목표 지향 비용 가중치 - 증가
        'obstacle_cost_gain': 2.0,     # 장애물 회피 비용 가중치 - 감소 (더 공격적)
        'speed_cost_gain': 0.01,       # 속도 비용 가중치
    }
    
    ROBOT_MAX_SPEED = 2.5         # 로봇 최대 속도 (m/s)
    ROBOT_MAX_OMEGA = np.pi       # 로봇 최대 각속도 (rad/s)
    ROBOT_RADIUS = 0.8            # 로봇 반경 (m)
    
    START_POS = (5, 5)            # 시작 위치
    GOAL_POS = (45, 45)           # 목표 위치
    # ============================================================
    
    print("\n[DWA 파라미터]")
    print(f"  - 예측 시간: {DWA_CONFIG['predict_time']}s")
    print(f"  - 속도 해상도: {DWA_CONFIG['v_reso']}")
    print(f"  - 각속도 해상도: {DWA_CONFIG['w_reso']}")
    print(f"  - 목표 지향 가중치: {DWA_CONFIG['to_goal_cost_gain']}")
    print(f"  - 장애물 회피 가중치: {DWA_CONFIG['obstacle_cost_gain']}")
    print(f"  - 속도 가중치: {DWA_CONFIG['speed_cost_gain']}")
    
    # 환경 생성
    env = create_dynamic_environment()
    
    # 로봇 생성
    robot = Robot(START_POS[0], START_POS[1], 0,
                 max_v=ROBOT_MAX_SPEED, max_w=ROBOT_MAX_OMEGA,
                 radius=ROBOT_RADIUS)
    
    # 시뮬레이터 생성
    simulator = DWASimulator(robot, env, GOAL_POS, DWA_CONFIG)
    
    # 시각화
    print("\n시뮬레이션 시작...")
    print("(이동 장애물들이 움직이는 것을 관찰하세요)")
    visualize_dwa(simulator)


if __name__ == "__main__":
    main()
