"""
시나리오 1: Global Path Planning (A*, Dijkstra)
고정 장애물이 있는 환경에서 전역 경로 계획

학생들이 파라미터를 조정하며 테스트할 수 있는 예제
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import heapq
import time
import sys
import os
import warnings

# 한글 폰트 경고 무시
warnings.filterwarnings('ignore', category=UserWarning, message='.*Glyph.*missing from.*')

# 상위 디렉토리의 common 모듈 import
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import Robot, Environment, RectangleObstacle, CircleObstacle


class GridBasedPlanner:
    """그리드 기반 경로 계획 클래스"""
    
    def __init__(self, environment: Environment, grid_resolution: float = 1.0):
        """
        Args:
            environment: 환경 객체
            grid_resolution: 그리드 해상도 (m)
        """
        self.env = environment
        self.resolution = grid_resolution
        self.grid_width = int(environment.width / grid_resolution)
        self.grid_height = int(environment.height / grid_resolution)
        
        # 그리드 맵 생성 (0: 자유공간, 1: 장애물)
        self.grid_map = self._create_grid_map()
        self.visited_nodes = []
        
    def _create_grid_map(self):
        """환경을 그리드 맵으로 변환"""
        grid = np.zeros((self.grid_height, self.grid_width), dtype=int)
        
        # 로봇 반경 + 안전 마진 (너무 크면 경로를 못 찾음)
        robot_radius = 0.5  # 로봇 반경
        safety_padding = 0.3  # 안전 마진
        total_clearance = robot_radius + safety_padding
        
        for i in range(self.grid_height):
            for j in range(self.grid_width):
                # 그리드 중심점의 실제 좌표
                x = (j + 0.5) * self.resolution
                y = (i + 0.5) * self.resolution
                pos = np.array([x, y])
                
                # 충돌 검사
                if self.env.check_collision(pos, total_clearance):
                    grid[i, j] = 1
                    
        return grid
    
    def world_to_grid(self, x: float, y: float):
        """실제 좌표를 그리드 좌표로 변환"""
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int):
        """그리드 좌표를 실제 좌표로 변환"""
        x = (grid_x + 0.5) * self.resolution
        y = (grid_y + 0.5) * self.resolution
        return x, y
    
    def is_valid(self, grid_x: int, grid_y: int):
        """그리드 좌표가 유효한지 확인"""
        if grid_x < 0 or grid_x >= self.grid_width:
            return False
        if grid_y < 0 or grid_y >= self.grid_height:
            return False
        if self.grid_map[grid_y, grid_x] == 1:
            return False
        return True
    
    def get_neighbors(self, grid_x: int, grid_y: int):
        """8방향 이웃 노드 반환"""
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            nx, ny = grid_x + dx, grid_y + dy
            if self.is_valid(nx, ny):
                # 대각선 이동 비용: √2, 직선 이동 비용: 1
                cost = np.sqrt(2) if abs(dx) + abs(dy) == 2 else 1.0
                neighbors.append(((nx, ny), cost))
        return neighbors
    
    def heuristic(self, pos1, pos2):
        """휴리스틱 함수 (유클리드 거리)"""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)


class AStarPlanner(GridBasedPlanner):
    """A* 알고리즘 구현"""
    
    def plan(self, start, goal):
        """A* 알고리즘으로 경로 계획
        
        Args:
            start: 시작 위치 (world coordinates)
            goal: 목표 위치 (world coordinates)
            
        Returns:
            path: 경로 리스트 (world coordinates)
        """
        # 좌표 변환
        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])
        
        if not self.is_valid(*start_grid) or not self.is_valid(*goal_grid):
            print("시작점 또는 목표점이 유효하지 않습니다.")
            return None
        
        # 초기화
        self.visited_nodes = []
        open_list = []
        heapq.heappush(open_list, (0, start_grid))
        
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}
        
        while open_list:
            _, current = heapq.heappop(open_list)
            
            self.visited_nodes.append(current)
            
            if current == goal_grid:
                # 경로 재구성
                path = []
                while current in came_from:
                    x, y = self.grid_to_world(current[0], current[1])
                    path.append((x, y))
                    current = came_from[current]
                x, y = self.grid_to_world(start_grid[0], start_grid[1])
                path.append((x, y))
                return path[::-1]
            
            for neighbor, move_cost in self.get_neighbors(current[0], current[1]):
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
        
        print("경로를 찾을 수 없습니다.")
        return None


class DijkstraPlanner(GridBasedPlanner):
    """Dijkstra 알고리즘 구현"""
    
    def plan(self, start, goal):
        """Dijkstra 알고리즘으로 경로 계획
        
        Args:
            start: 시작 위치 (world coordinates)
            goal: 목표 위치 (world coordinates)
            
        Returns:
            path: 경로 리스트 (world coordinates)
        """
        # 좌표 변환
        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])
        
        if not self.is_valid(*start_grid) or not self.is_valid(*goal_grid):
            print("시작점 또는 목표점이 유효하지 않습니다.")
            return None
        
        # 초기화
        self.visited_nodes = []
        open_list = []
        heapq.heappush(open_list, (0, start_grid))
        
        came_from = {}
        dist = {start_grid: 0}
        
        while open_list:
            current_dist, current = heapq.heappop(open_list)
            
            if current in self.visited_nodes:
                continue
                
            self.visited_nodes.append(current)
            
            if current == goal_grid:
                # 경로 재구성
                path = []
                while current in came_from:
                    x, y = self.grid_to_world(current[0], current[1])
                    path.append((x, y))
                    current = came_from[current]
                x, y = self.grid_to_world(start_grid[0], start_grid[1])
                path.append((x, y))
                return path[::-1]
            
            for neighbor, move_cost in self.get_neighbors(current[0], current[1]):
                new_dist = dist[current] + move_cost
                
                if neighbor not in dist or new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    came_from[neighbor] = current
                    heapq.heappush(open_list, (new_dist, neighbor))
        
        print("경로를 찾을 수 없습니다.")
        return None


class Simulator:
    """시뮬레이션 클래스"""
    
    def __init__(self, environment: Environment, robot: Robot, path: list):
        self.env = environment
        self.robot = robot
        self.path = np.array(path)
        self.current_target_idx = 0
        self.dt = 0.1  # 시간 간격
        self.completed = False
        
    def get_control(self):
        """Pure Pursuit 제어기"""
        if self.current_target_idx >= len(self.path):
            return 0.0, 0.0
        
        # Look-ahead distance
        look_ahead = 2.0
        
        # 가장 가까운 경로 포인트 찾기
        robot_pos = self.robot.get_position()
        distances = np.linalg.norm(self.path - robot_pos, axis=1)
        closest_idx = np.argmin(distances)
        
        # Look-ahead 포인트 찾기
        target_idx = closest_idx
        for i in range(closest_idx, len(self.path)):
            dist = np.linalg.norm(self.path[i] - robot_pos)
            if dist >= look_ahead:
                target_idx = i
                break
        else:
            target_idx = len(self.path) - 1
        
        self.current_target_idx = target_idx
        target = self.path[target_idx]
        
        # 디버깅: 주기적으로 정보 출력
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 50 == 0:
            final_target = self.path[-1]
            distance_to_goal = np.linalg.norm(final_target - robot_pos)
            print(f"[DEBUG] Robot: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}), Target idx: {target_idx}/{len(self.path)}, Dist to goal: {distance_to_goal:.2f}")
        
        # 목표까지의 거리와 각도 계산
        dx = target[0] - self.robot.x
        dy = target[1] - self.robot.y
        distance_to_target = np.sqrt(dx**2 + dy**2)
        target_angle = np.arctan2(dy, dx)
        
        # 각도 차이 계산
        angle_diff = target_angle - self.robot.theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
        
        # 제어 입력 계산
        v = self.robot.max_v * 0.7  # 최대 속도의 70%
        w = 2.0 * angle_diff
        
        # 최종 목표 도달 확인 (마지막 포인트에 가까이 가면 종료)
        final_target = self.path[-1]
        distance_to_goal = np.linalg.norm(final_target - robot_pos)
        
        if distance_to_goal < 0.5:
            self.completed = True
            v = 0.0
            w = 0.0
        
        return v, w
    
    def step(self):
        """시뮬레이션 한 스텝 진행"""
        if not self.completed:
            v, w = self.get_control()
            self.robot.update(v, w, self.dt)
        return self.completed


def visualize_planning(environment, planner, path, robot, algorithm_name):
    """경로 계획 결과 시각화 (애니메이션)"""
    
    fig, ax = plt.subplots(figsize=(12, 10))
    
    def init():
        ax.clear()
        ax.set_xlim(0, environment.width)
        ax.set_ylim(0, environment.height)
        ax.set_aspect('equal')
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(f'{algorithm_name} - Global Path Planning', fontsize=16, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # 장애물 그리기
        for obs in environment.static_obstacles:
            if isinstance(obs, RectangleObstacle):
                rect = patches.Rectangle((obs.x, obs.y), obs.width, obs.height,
                                        facecolor='gray', edgecolor='black', linewidth=2)
                ax.add_patch(rect)
            elif isinstance(obs, CircleObstacle):
                circle = patches.Circle((obs.x, obs.y), obs.radius,
                                       facecolor='gray', edgecolor='black', linewidth=2)
                ax.add_patch(circle)
        
        # 방문 노드 그리기
        if planner.visited_nodes:
            visited_world = [planner.grid_to_world(gx, gy) for gx, gy in planner.visited_nodes]
            visited_x = [p[0] for p in visited_world]
            visited_y = [p[1] for p in visited_world]
            ax.scatter(visited_x, visited_y, c='lightblue', s=10, alpha=0.3, label='Visited Nodes')
        
        # 계획된 경로 그리기
        if path is not None:
            path_array = np.array(path)
            ax.plot(path_array[:, 0], path_array[:, 1], 'b--', linewidth=2, label='Planned Path', alpha=0.7)
            ax.scatter(path[0][0], path[0][1], c='green', s=200, marker='*', 
                      edgecolors='darkgreen', linewidths=2, label='Start', zorder=10)
            ax.scatter(path[-1][0], path[-1][1], c='red', s=200, marker='*', 
                      edgecolors='darkred', linewidths=2, label='Goal', zorder=10)
        
        return []
    
    # 시뮬레이터 생성
    sim = Simulator(environment, robot, path)
    
    # 로봇 그래픽 요소
    robot_circle = patches.Circle((robot.x, robot.y), robot.radius, 
                                 facecolor='blue', edgecolor='darkblue', linewidth=2, alpha=0.8)
    robot_direction = patches.FancyArrow(robot.x, robot.y, 
                                        robot.radius * np.cos(robot.theta),
                                        robot.radius * np.sin(robot.theta),
                                        width=0.3, head_width=0.5, head_length=0.3,
                                        facecolor='yellow', edgecolor='orange', linewidth=1.5)
    trajectory_line, = ax.plot([], [], 'r-', linewidth=2, label='Trajectory')
    
    def update(frame):
        # 시뮬레이션 스텝
        completed = sim.step()
        
        # 로봇 위치 업데이트
        robot_circle.center = (robot.x, robot.y)
        
        # 방향 화살표 업데이트
        robot_direction.set_data(x=robot.x, y=robot.y,
                                dx=robot.radius * np.cos(robot.theta),
                                dy=robot.radius * np.sin(robot.theta))
        
        # 궤적 업데이트
        if len(robot.trajectory) > 1:
            traj = np.array(robot.trajectory)
            trajectory_line.set_data(traj[:, 0], traj[:, 1])
        
        # 정보 텍스트
        info_text = f'Time: {frame * sim.dt:.1f}s | Distance: {robot.total_distance:.2f}m | Speed: {robot.v:.2f}m/s'
        if hasattr(update, 'text'):
            update.text.remove()
        update.text = ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
                            fontsize=10, verticalalignment='top',
                            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        if completed and not hasattr(update, 'completed'):
            update.completed = True
            print(f"\n✓ Goal reached!")
            print(f"  - Total distance: {robot.total_distance:.2f}m")
            print(f"  - Total time: {frame * sim.dt:.2f}s")
            # 애니메이션 정지
            anim.event_source.stop()
        
        return [robot_circle, robot_direction, trajectory_line, update.text]
    
    # 초기화 함수 호출하고 패치 추가
    init()
    ax.add_patch(robot_circle)
    ax.add_patch(robot_direction)
    ax.legend(loc='upper right', fontsize=9)
    
    # 애니메이션 생성 - 충분한 프레임 수 확보 (1000프레임 = 100초)
    anim = FuncAnimation(fig, update, frames=1000, interval=50, blit=False, repeat=False)
    
    # 애니메이션 객체를 figure에 저장하여 가비지 컬렉션 방지
    fig._animation = anim
    
    plt.tight_layout()
    plt.show()
    
    return anim  # 애니메이션 객체 반환


def create_test_environment():
    """테스트 환경 생성"""
    env = Environment(width=50.0, height=50.0)
    
    # 장애물 배치
    env.add_static_obstacle(RectangleObstacle(10, 5, 2, 20))
    env.add_static_obstacle(RectangleObstacle(20, 15, 15, 2))
    env.add_static_obstacle(RectangleObstacle(30, 25, 2, 15))
    env.add_static_obstacle(RectangleObstacle(15, 35, 20, 2))
    
    env.add_static_obstacle(CircleObstacle(5, 10, 2))
    env.add_static_obstacle(CircleObstacle(40, 10, 3))
    env.add_static_obstacle(CircleObstacle(25, 5, 2))
    
    return env


def main():
    """메인 함수"""
    print("=" * 60)
    print("시나리오 1: Global Path Planning")
    print("=" * 60)
    
    # 알고리즘 선택
    print("\n알고리즘을 선택하세요:")
    print("1. A* 알고리즘")
    print("2. Dijkstra 알고리즘")
    choice = input("선택 (1 or 2): ").strip()
    
    # ============================================================
    # 학생들이 조정할 수 있는 파라미터
    # ============================================================
    GRID_RESOLUTION = 1.0    # 그리드 해상도 (m) - 작을수록 정밀하지만 느림
    ROBOT_MAX_SPEED = 1.0    # 로봇 최대 속도 (m/s)
    ROBOT_MAX_OMEGA = np.pi/2  # 로봇 최대 각속도 (rad/s)
    ROBOT_RADIUS = 0.5       # 로봇 반경 (m)
    
    START_POS = (5, 5)       # 시작 위치
    GOAL_POS = (45, 45)      # 목표 위치
    # ============================================================
    
    # 환경 생성
    env = create_test_environment()
    
    # 로봇 생성
    robot = Robot(START_POS[0], START_POS[1], 0, 
                 max_v=ROBOT_MAX_SPEED, max_w=ROBOT_MAX_OMEGA, 
                 radius=ROBOT_RADIUS)
    
    # 경로 계획기 생성
    if choice == '1':
        print("\n>>> A* 알고리즘 사용")
        planner = AStarPlanner(env, grid_resolution=GRID_RESOLUTION)
        algorithm_name = "A*"
    else:
        print("\n>>> Dijkstra 알고리즘 사용")
        planner = DijkstraPlanner(env, grid_resolution=GRID_RESOLUTION)
        algorithm_name = "Dijkstra"
    
    # 경로 계획
    print(f"\n경로 계획 중... (해상도: {GRID_RESOLUTION}m)")
    start_time = time.time()
    path = planner.plan(START_POS, GOAL_POS)
    planning_time = time.time() - start_time
    
    if path is None:
        print("경로를 찾을 수 없습니다.")
        return
    
    # 경로 정보 출력
    path_length = sum(np.linalg.norm(np.array(path[i+1]) - np.array(path[i])) 
                     for i in range(len(path) - 1))
    
    print(f"\n✓ 경로 계획 완료!")
    print(f"  - 계획 시간: {planning_time:.3f}초")
    print(f"  - 경로 길이: {path_length:.2f}m")
    print(f"  - 경로 노드 수: {len(path)}")
    print(f"  - 탐색한 노드 수: {len(planner.visited_nodes)}")
    
    # 시각화
    print("\n시뮬레이션 시작...")
    anim = visualize_planning(env, planner, path, robot, algorithm_name)


if __name__ == "__main__":
    main()
