"""
시나리오 3: Global + Local Path Planning 통합 (학생 과제)

이 파일은 학생들이 구현해야 할 과제 파일입니다.
TODO 표시된 부분을 완성하세요.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import time
import sys
import sys
import os
import json
from datetime import datetime
import warnings

# 한글 폰트 경고 무시
warnings.filterwarnings('ignore', category=UserWarning, message='.*Glyph.*missing from.*')

# 상위 디렉토리의 common 모듈 import
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import Robot, Environment, RectangleObstacle, CircleObstacle, MovingObstacle

# 시나리오 1, 2의 알고리즘 import
from scenario1_global_planning import AStarPlanner, DijkstraPlanner
from scenario2_local_planning import DWA


class HybridPlanner:
    """
    Global + Local Path Planning 통합 플래너 (학생 구현)
    """
    
    def __init__(self, environment: Environment, robot: Robot, goal, 
                 global_planner_type='astar', dwa_config=None):
        """
        초기화 함수
        
        Args:
            environment: 환경 객체
            robot: 로봇 객체
            goal: 최종 목표 위치 (x, y)
            global_planner_type: 'astar' 또는 'dijkstra'
            dwa_config: DWA 설정 딕셔너리
        """
        # TODO: 구현하세요
        pass
        
    def plan_global_path(self, start):
        """
        전역 경로 계획
        
        Args:
            start: 시작 위치 (x, y)
            
        Returns:
            success: 경로 계획 성공 여부
        """
        # TODO: 구현하세요
        pass
        
    def get_local_goal(self, look_ahead_distance=5.0):
        """
        전역 경로에서 지역 목표점 선택
        
        Args:
            look_ahead_distance: 전방 주시 거리
            
        Returns:
            local_goal: 지역 목표 위치 (x, y) 또는 None
        """
        # TODO: 구현하세요
        pass
        
    def execute_step(self, dt):
        """
        한 스텝 실행: DWA로 제어 명령 계산 및 로봇 제어
        
        Args:
            dt: 시간 간격
            
        Returns:
            completed: 목표 도달 여부
            collision: 충돌 여부
        """
        # TODO: 구현하세요
        pass


class ChallengeEnvironment:
    """과제용 복잡한 환경"""
    
    @staticmethod
    def create():
        """고정 및 이동 장애물이 있는 복잡한 환경 생성"""
        env = Environment(width=50.0, height=50.0)
        
        # 고정 장애물 (미로 형태 - 단순화)
        env.add_static_obstacle(RectangleObstacle(10, 5, 2, 15))
        env.add_static_obstacle(RectangleObstacle(20, 15, 2, 20))
        env.add_static_obstacle(RectangleObstacle(30, 5, 2, 15))
        env.add_static_obstacle(RectangleObstacle(15, 30, 15, 2))
        
        env.add_static_obstacle(CircleObstacle(5, 10, 2))
        env.add_static_obstacle(CircleObstacle(25, 8, 2))
        env.add_static_obstacle(CircleObstacle(35, 25, 2))
        env.add_static_obstacle(CircleObstacle(40, 40, 2.5))
        
        # 이동 장애물 (크기 증가!)
        env.add_dynamic_obstacle(MovingObstacle(8, 22, 2.0, vx=0.6, vy=0.3))
        env.add_dynamic_obstacle(MovingObstacle(28, 8, 2.0, vx=-0.5, vy=0.5))
        env.add_dynamic_obstacle(MovingObstacle(14, 40, 2.0, vx=0.5, vy=-0.4))
        env.add_dynamic_obstacle(MovingObstacle(42, 35, 2.0, vx=-0.4, vy=-0.5))
        
        return env


class PerformanceEvaluator:
    """성능 평가 시스템"""
    
    def __init__(self, save_file='rankings.json'):
        self.save_file = save_file
        self.rankings = self.load_rankings()
        
    def load_rankings(self):
        """기존 랭킹 로드"""
        try:
            with open(self.save_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except FileNotFoundError:
            return []
    
    def save_rankings(self):
        """랭킹 저장"""
        with open(self.save_file, 'w', encoding='utf-8') as f:
            json.dump(self.rankings, f, indent=2, ensure_ascii=False)
    
    def add_result(self, student_name, distance, time_taken, collision, algorithm):
        """결과 추가"""
        score = self.calculate_score(distance, time_taken, collision)
        
        result = {
            'name': student_name,
            'distance': distance,
            'time': time_taken,
            'collision': collision,
            'algorithm': algorithm,
            'score': score,
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
        
        self.rankings.append(result)
        self.rankings.sort(key=lambda x: x['score'], reverse=True)
        self.save_rankings()
        
        return score
    
    def calculate_score(self, distance, time_taken, collision):
        """점수 계산 (높을수록 좋음)"""
        if collision:
            return 0  # 충돌 시 0점
        
        # 기본 점수: 1000점에서 거리와 시간에 따라 감점
        score = 1000
        score -= distance * 2  # 거리 1m당 2점 감점
        score -= time_taken * 5  # 시간 1초당 5점 감점
        
        return max(0, score)
    
    def print_rankings(self, top_n=10):
        """랭킹 출력"""
        print("\n" + "=" * 80)
        print("🏆 랭킹 TOP {}".format(top_n))
        print("=" * 80)
        print(f"{'순위':<6} {'이름':<15} {'점수':<8} {'거리(m)':<10} {'시간(s)':<10} {'알고리즘':<12} {'날짜'}")
        print("-" * 80)
        
        for i, result in enumerate(self.rankings[:top_n], 1):
            collision_mark = " ⚠️충돌" if result['collision'] else ""
            print(f"{i:<6} {result['name']:<15} {result['score']:<8.1f} {result['distance']:<10.2f} "
                  f"{result['time']:<10.2f} {result['algorithm']:<12} {result['timestamp']}{collision_mark}")


class Simulator:
    """시뮬레이터"""
    
    def __init__(self, planner: HybridPlanner, dt=0.1):
        self.planner = planner
        self.dt = dt
        self.completed = False
        self.collision = False
        self.start_time = None
        self.elapsed_time = 0
        
    def run(self, max_steps=2000):
        """시뮬레이션 실행 (시각화 없이)"""
        self.start_time = time.time()
        
        for step in range(max_steps):
            # 환경 업데이트 (이동 장애물)
            self.planner.env.update(self.dt)
            
            # 한 스텝 실행
            completed, collision = self.planner.execute_step(self.dt)
            
            if collision:
                self.collision = True
                print(f"\n✗ 충돌 발생! (스텝: {step})")
                break
            
            if completed:
                self.completed = True
                print(f"\n✓ 목표 도달! (스텝: {step})")
                break
        
        self.elapsed_time = time.time() - self.start_time
        
        return self.completed, self.collision


def visualize_simulation(planner: HybridPlanner, dt=0.1):
    """시뮬레이션 시각화"""
    
    fig, ax = plt.subplots(figsize=(14, 12))
    
    robot = planner.robot
    env = planner.env
    goal = planner.goal
    
    completed = False
    collision = False
    
    def init():
        ax.clear()
        ax.set_xlim(0, env.width)
        ax.set_ylim(0, env.height)
        ax.set_aspect('equal')
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('시나리오 3: Global + Local Path Planning (학생 과제)', fontsize=16, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # 고정 장애물
        for obs in env.static_obstacles:
            if isinstance(obs, RectangleObstacle):
                rect = patches.Rectangle((obs.x, obs.y), obs.width, obs.height,
                                        facecolor='gray', edgecolor='black', linewidth=2)
                ax.add_patch(rect)
            elif isinstance(obs, CircleObstacle):
                circle = patches.Circle((obs.x, obs.y), obs.radius,
                                       facecolor='gray', edgecolor='black', linewidth=2)
                ax.add_patch(circle)
        
        # 동적 장애물 (여기서 추가!)
        for obs in env.dynamic_obstacles:
            circle = patches.Circle((obs.x, obs.y), obs.radius,
                                   facecolor='orange', edgecolor='red', linewidth=3, alpha=0.8)
            ax.add_patch(circle)
            dynamic_obs_patches.append(circle)
        
        # 전역 경로
        if planner.global_path is not None:
            path_array = np.array(planner.global_path)
            ax.plot(path_array[:, 0], path_array[:, 1], 'b--', linewidth=2, 
                   label='Global Path', alpha=0.5)
        
        # 시작점과 목표점
        start_pos = planner.global_path[0] if planner.global_path is not None else robot.get_position()
        ax.scatter(start_pos[0], start_pos[1], c='green', s=300, marker='*',
                  edgecolors='darkgreen', linewidths=2, label='Start', zorder=10)
        ax.scatter(goal[0], goal[1], c='red', s=300, marker='*',
                  edgecolors='darkred', linewidths=2, label='Goal', zorder=10)
        
        return []
    
    # 그래픽 요소
    robot_circle = patches.Circle((robot.x, robot.y), robot.radius,
                                 facecolor='blue', edgecolor='darkblue', linewidth=2, alpha=0.8)
    robot_direction = patches.FancyArrow(robot.x, robot.y,
                                        robot.radius * np.cos(robot.theta),
                                        robot.radius * np.sin(robot.theta),
                                        width=0.3, head_width=0.5, head_length=0.3,
                                        facecolor='yellow', edgecolor='orange', linewidth=1.5)
    trajectory_line, = ax.plot([], [], 'r-', linewidth=2, label='Actual Trajectory', alpha=0.8)
    
    # 동적 장애물 패치 리스트 (init에서 채워짐)
    dynamic_obs_patches = []
    
    # 지역 목표점 표시
    local_goal_marker, = ax.plot([], [], 'go', markersize=10, label='Local Goal')
    
    def update(frame):
        nonlocal completed, collision
        
        if completed or collision:
            return [robot_circle, robot_direction, trajectory_line, local_goal_marker] + dynamic_obs_patches
        
        # 환경 업데이트
        env.update(dt)
        
        # 시뮬레이션 스텝
        completed, collision = planner.execute_step(dt)
        
        # 로봇 업데이트
        robot_circle.center = (robot.x, robot.y)
        robot_direction.set_data(x=robot.x, y=robot.y,
                                dx=robot.radius * np.cos(robot.theta),
                                dy=robot.radius * np.sin(robot.theta))
        
        # 궤적 업데이트
        if len(robot.trajectory) > 1:
            traj = np.array(robot.trajectory)
            trajectory_line.set_data(traj[:, 0], traj[:, 1])
        
        # 지역 목표 표시
        local_goal = planner.get_local_goal()
        if local_goal is not None:
            local_goal_marker.set_data([local_goal[0]], [local_goal[1]])
        
        # 이동 장애물 업데이트
        for i, obs in enumerate(env.dynamic_obstacles):
            dynamic_obs_patches[i].center = (obs.x, obs.y)
        
        # 정보 텍스트
        info_text = f'Time: {frame * dt:.1f}s | Distance: {robot.total_distance:.2f}m | Speed: {robot.v:.2f}m/s\n'
        info_text += f'Waypoint: {planner.current_waypoint_idx}/{len(planner.global_path) if planner.global_path is not None else 0}'
        if hasattr(update, 'text'):
            update.text.remove()
        update.text = ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
                            fontsize=10, verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        if completed and not hasattr(update, 'completed'):
            update.completed = True
            print(f"\n✓ Goal reached!")
            print(f"  - Total distance: {robot.total_distance:.2f}m")
            print(f"  - Total time: {frame * dt:.2f}s")
            # 애니메이션 정지
            anim.event_source.stop()
        
        if collision and not hasattr(update, 'collision'):
            update.collision = True
            print(f"\n✗ Collision!")
            # 애니메이션 정지
            anim.event_source.stop()
        
        return [robot_circle, robot_direction, trajectory_line, update.text, local_goal_marker] + dynamic_obs_patches
    
    init()
    ax.add_patch(robot_circle)
    ax.add_patch(robot_direction)
    ax.legend(loc='upper right', fontsize=9)
    
    anim = FuncAnimation(fig, update, frames=2000, interval=20, blit=False, repeat=False)  # 50→20ms로 빠르게
    
    # 애니메이션 객체를 figure에 저장하여 가비지 컬렉션 방지
    fig._animation = anim
    
    plt.tight_layout()
    plt.show()


def main():
    """메인 함수"""
    print("=" * 80)
    print("시나리오 3: Global + Local Path Planning 통합 (학생 과제)")
    print("=" * 80)
    
    # 학생 이름 입력
    student_name = input("\n학생 이름을 입력하세요: ").strip()
    if not student_name:
        student_name = "학생"
    
    # 알고리즘 선택
    print("\nGlobal Planner를 선택하세요:")
    print("1. A* 알고리즘")
    print("2. Dijkstra 알고리즘")
    choice = input("선택 (1 or 2, Enter=1): ").strip()
    if not choice:
        choice = '1'
    global_planner_type = 'astar' if choice == '1' else 'dijkstra'
    
    # DWA 설정 (속도 최적화 + 장애물 회피 시 속도 향상)
    DWA_CONFIG = {
        'max_accel': 0.8,
        'max_delta_yaw_rate': 60.0 * np.pi / 180.0,
        'v_reso': 8,
        'w_reso': 16,
        'dt': 0.1,
        'predict_time': 2.5,
        'to_goal_cost_gain': 1.0,
        'obstacle_cost_gain': 5.0,
        'speed_cost_gain': 0.05,
    }
    
    # 환경 및 로봇 생성
    START_POS = (3, 3)
    GOAL_POS = (47, 47)
    
    env = ChallengeEnvironment.create()
    robot = Robot(START_POS[0], START_POS[1], np.pi/4, 
                 max_v=2.5, max_w=np.pi*1.2, radius=0.8)
    
    # 플래너 생성
    print("\n플래너 초기화 중...")
    planner = HybridPlanner(env, robot, GOAL_POS, global_planner_type, DWA_CONFIG)
    
    # 전역 경로 계획
    print("\n전역 경로 계획 중...")
    start_time = time.time()
    success = planner.plan_global_path(START_POS)
    planning_time = time.time() - start_time
    
    if not success:
        print("❌ 전역 경로 계획 실패!")
        return
    
    print(f"✓ 전역 경로 계획 완료 (소요 시간: {planning_time:.3f}초)")
    
    # 시각화 선택
    print("\n시각화 옵션:")
    print("1. 실시간 애니메이션 (느림)")
    print("2. 빠른 실행 (시각화 없음)")
    viz_choice = input("선택 (1 or 2, Enter=1): ").strip()
    if not viz_choice:
        viz_choice = '1'
    
    if viz_choice == '1':
        # 실시간 애니메이션
        visualize_simulation(planner)
        
        completed = np.linalg.norm(robot.get_position() - planner.goal) < 1.0
        collision = env.check_collision(robot.get_position(), robot.radius)
        elapsed_time = len(robot.trajectory) * 0.1
    else:
        # 빠른 실행
        print("\n시뮬레이션 실행 중...")
        simulator = Simulator(planner)
        completed, collision = simulator.run()
        elapsed_time = simulator.elapsed_time
    
    # 결과 출력
    print("\n" + "=" * 80)
    print("결과")
    print("=" * 80)
    print(f"학생 이름: {student_name}")
    print(f"알고리즘: {global_planner_type.upper()}")
    print(f"주행 거리: {robot.total_distance:.2f} m")
    print(f"소요 시간: {elapsed_time:.2f} s")
    print(f"목표 도달: {'✓ 성공' if completed else '✗ 실패'}")
    print(f"충돌 여부: {'✗ 충돌' if collision else '✓ 안전'}")
    
    # 성능 평가 및 랭킹 업데이트
    evaluator = PerformanceEvaluator()
    score = evaluator.add_result(student_name, robot.total_distance, 
                                 elapsed_time, collision, global_planner_type.upper())
    
    print(f"\n최종 점수: {score:.1f}점")
    
    # 랭킹 출력
    evaluator.print_rankings(top_n=10)


if __name__ == "__main__":
    main()
