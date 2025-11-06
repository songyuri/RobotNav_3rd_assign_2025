"""
환경 및 장애물 클래스
"""
import numpy as np
from typing import List, Tuple


class Obstacle:
    """장애물 기본 클래스"""
    
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        
    def get_position(self):
        return np.array([self.x, self.y])
    
    def is_collision(self, pos: np.ndarray, robot_radius: float) -> bool:
        """충돌 검사"""
        raise NotImplementedError


class CircleObstacle(Obstacle):
    """원형 장애물"""
    
    def __init__(self, x: float, y: float, radius: float):
        super().__init__(x, y)
        self.radius = radius
        
    def is_collision(self, pos: np.ndarray, robot_radius: float) -> bool:
        distance = np.linalg.norm(pos - self.get_position())
        return distance < (self.radius + robot_radius)


class RectangleObstacle(Obstacle):
    """직사각형 장애물"""
    
    def __init__(self, x: float, y: float, width: float, height: float):
        super().__init__(x, y)
        self.width = width
        self.height = height
        
    def is_collision(self, pos: np.ndarray, robot_radius: float) -> bool:
        # AABB 충돌 검사
        closest_x = np.clip(pos[0], self.x, self.x + self.width)
        closest_y = np.clip(pos[1], self.y, self.y + self.height)
        distance = np.linalg.norm(pos - np.array([closest_x, closest_y]))
        return distance < robot_radius


class MovingObstacle(CircleObstacle):
    """이동하는 원형 장애물"""
    
    def __init__(self, x: float, y: float, radius: float, 
                 vx: float = 0.0, vy: float = 0.0):
        super().__init__(x, y, radius)
        self.vx = vx
        self.vy = vy
        self.initial_x = x
        self.initial_y = y
        
    def update(self, dt: float, bounds: Tuple[float, float, float, float]):
        """장애물 위치 업데이트
        
        Args:
            dt: 시간 간격
            bounds: (x_min, x_max, y_min, y_max) 경계
        """
        self.x += self.vx * dt
        self.y += self.vy * dt
        
        # 경계에서 반사
        x_min, x_max, y_min, y_max = bounds
        if self.x - self.radius < x_min or self.x + self.radius > x_max:
            self.vx = -self.vx
            self.x = np.clip(self.x, x_min + self.radius, x_max - self.radius)
        if self.y - self.radius < y_min or self.y + self.radius > y_max:
            self.vy = -self.vy
            self.y = np.clip(self.y, y_min + self.radius, y_max - self.radius)
    
    def reset(self):
        """초기 위치로 리셋"""
        self.x = self.initial_x
        self.y = self.initial_y


class Environment:
    """시뮬레이션 환경"""
    
    def __init__(self, width: float = 50.0, height: float = 50.0):
        self.width = width
        self.height = height
        self.static_obstacles: List[Obstacle] = []
        self.dynamic_obstacles: List[MovingObstacle] = []
        
    def add_static_obstacle(self, obstacle: Obstacle):
        """고정 장애물 추가"""
        self.static_obstacles.append(obstacle)
        
    def add_dynamic_obstacle(self, obstacle: MovingObstacle):
        """이동 장애물 추가"""
        self.dynamic_obstacles.append(obstacle)
        
    def update(self, dt: float):
        """환경 업데이트 (이동 장애물)"""
        bounds = (0, self.width, 0, self.height)
        for obs in self.dynamic_obstacles:
            obs.update(dt, bounds)
            
    def check_collision(self, pos: np.ndarray, robot_radius: float) -> bool:
        """충돌 검사"""
        # 경계 검사
        if (pos[0] < robot_radius or pos[0] > self.width - robot_radius or
            pos[1] < robot_radius or pos[1] > self.height - robot_radius):
            return True
            
        # 고정 장애물 검사
        for obs in self.static_obstacles:
            if obs.is_collision(pos, robot_radius):
                return True
                
        # 이동 장애물 검사
        for obs in self.dynamic_obstacles:
            if obs.is_collision(pos, robot_radius):
                return True
                
        return False
    
    def reset(self):
        """환경 초기화"""
        for obs in self.dynamic_obstacles:
            obs.reset()
