"""
로봇 클래스 정의
"""
import numpy as np


class Robot:
    """로봇 상태를 나타내는 클래스"""
    
    def __init__(self, x: float, y: float, theta: float, 
                 max_v: float = 1.0, max_w: float = np.pi/2,
                 radius: float = 0.5):
        """
        Args:
            x: x 좌표
            y: y 좌표
            theta: 방향 (라디안)
            max_v: 최대 선속도 (m/s)
            max_w: 최대 각속도 (rad/s)
            radius: 로봇 반경 (m)
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0  # 현재 선속도
        self.w = 0.0  # 현재 각속도
        
        self.max_v = max_v
        self.max_w = max_w
        self.radius = radius
        
        # 주행 기록
        self.trajectory = [(x, y)]
        self.total_distance = 0.0
        
    def update(self, v: float, w: float, dt: float):
        """로봇 상태 업데이트 (운동학 모델)
        
        Args:
            v: 선속도 명령 (양수: 전진, 음수: 후진)
            w: 각속도 명령
            dt: 시간 간격
        """
        # 속도 제한 (후진 허용)
        v = np.clip(v, -self.max_v, self.max_v)
        w = np.clip(w, -self.max_w, self.max_w)
        
        # 이전 위치 저장
        prev_x, prev_y = self.x, self.y
        
        # 운동학 모델 (differential drive)
        self.theta += w * dt
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        
        self.v = v
        self.w = w
        
        # 주행 거리 계산 (절대값으로 계산)
        distance = np.sqrt((self.x - prev_x)**2 + (self.y - prev_y)**2)
        self.total_distance += distance
        
        # 궤적 저장
        self.trajectory.append((self.x, self.y))
        
    def get_position(self):
        """현재 위치 반환"""
        return np.array([self.x, self.y])
    
    def get_state(self):
        """현재 상태 반환"""
        return np.array([self.x, self.y, self.theta, self.v, self.w])
    
    def reset(self, x: float, y: float, theta: float):
        """로봇 상태 초기화"""
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.w = 0.0
        self.trajectory = [(x, y)]
        self.total_distance = 0.0
