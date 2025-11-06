# Robot Navigation 과제

## 📋 과제 개요

로봇 네비게이션의 Global Path Planning과 Local Path Planning을 학습하고 구현하는 과제입니다.

### 시나리오 구성

1. **시나리오 1**: Global Path Planning (정적 장애물)
   - A* / Dijkstra 알고리즘
   - 그리드 기반 경로 계획

2. **시나리오 2**: Local Path Planning (동적 장애물)
   - DWA (Dynamic Window Approach)
   - 실시간 장애물 회피

3. **시나리오 3**: 통합 과제 ⭐
   - Global + Local Planning 통합 구현
   - 정적 + 동적 장애물 동시 회피
   - 성능 평가 및 랭킹

---

## 🚀 빠른 시작

### 환경 설정

```bash
cd /home/syr/pathplanning_assign
source .venv/bin/activate  # Python 가상환경
```

### 시나리오 실행

```bash
# 시나리오 1: Global Planning
python scenario1_global_planning.py

# 시나리오 2: Local Planning  
python scenario2_local_planning.py

# 시나리오 3: 통합 과제
python scenario3_challenge.py
```

---

## 📚 시나리오 1: Global Path Planning

### 개요
정적 장애물 환경에서 A* 또는 Dijkstra로 전역 경로를 계획합니다.
예시 코드가 이미 작성되어 있으며, 시뮬레이션을 실행해보면서 주요 파라미터값을 바꿔보며 시도해보시고, 보고서에 작성해주세요.

### 실행
```bash
python scenario1_global_planning.py
```

**선택 사항:**
- 1: A* 알고리즘 (휴리스틱 사용)
- 2: Dijkstra 알고리즘 (균일 탐색)

### 학습 목표
- A*와 Dijkstra의 차이 이해
- 그리드 해상도 영향 파악
- 휴리스틱 함수의 역할

### 주요 파라미터
```python
GRID_RESOLUTION = 1.0    # 그리드 해상도
ROBOT_MAX_SPEED = 1.0    # 최대 속도
START_POS = (5, 5)       # 시작점
GOAL_POS = (45, 45)      # 목표점
```

---

## 🎯 시나리오 2: Local Path Planning

### 개요
동적 장애물 환경에서 DWA로 실시간 경로를 계획합니다.
예시 코드가 이미 작성되어 있으며, 시뮬레이션을 실행해보면서 주요 파라미터값을 바꿔보며 시도해보시고, 보고서에 작성해주세요.

### 실행
```bash
python scenario2_local_planning.py
```

**특징:**
- 랜덤 이동 장애물 (4-6개)
- 실시간 장애물 회피
- 예측 궤적 시각화

### 학습 목표
- DWA 알고리즘 이해
- 비용 함수 가중치 조정
- 동적 환경 대응

### DWA 파라미터
```python
DWA_CONFIG = {
    'predict_time': 2.5,           # 예측 시간
    'v_reso': 10,                  # 속도 샘플
    'w_reso': 20,                  # 각속도 샘플
    'to_goal_cost_gain': 2.0,      # 목표 지향
    'obstacle_cost_gain': 2.0,     # 장애물 회피
}
```

---

## ⭐ 시나리오 3: 통합 과제

### 핵심 개념

**시나리오 1 + 시나리오 2 = 시나리오 3**

1. **Global Planning**: A*/Dijkstra로 정적 장애물 피하는 전체 경로 생성
2. **Local Planning**: DWA로 경로 추종하며 동적 장애물 실시간 회피

### 구현 파일

`scenario3_challenge.py`

### 실행
```bash
python scenario3_challenge.py
```

**입력:**
1. 학생 이름
2. 알고리즘 선택 (A* / Dijkstra)
3. 시각화 옵션 (실시간 / 빠른 실행)

### 구현해야 할 것

`HybridPlanner` 클래스의 4개 메서드:

#### 1. `__init__`: 초기화
```python
def __init__(self, environment, robot, goal, global_planner_type, dwa_config):
    # TODO: 
    # 1. self.env, self.robot, self.goal 저장
    # 2. AStarPlanner 또는 DijkstraPlanner 생성
    # 3. DWA 객체 생성
    # 4. self.global_path = None 초기화
```

#### 2. `plan_global_path`: 전역 경로 계획
```python
def plan_global_path(self, start):
    # TODO:
    # 1. self.global_planner.plan(start, goal) 호출
    # 2. 결과를 self.global_path에 저장
    # 3. 성공 여부 반환
```

#### 3. `get_local_goal`: 지역 목표 선택
```python
def get_local_goal(self, look_ahead_distance=5.0):
    # TODO:
    # 1. 로봇에서 가장 가까운 경로 포인트 찾기
    # 2. look_ahead 거리만큼 앞의 포인트 선택
    # 3. 경로 끝이면 최종 목표 반환
```

#### 4. `execute_step`: 실행 스텝
```python
def execute_step(self, dt):
    # TODO:
    # 1. get_local_goal()로 지역 목표 얻기
    # 2. dwa.plan(local_goal, env)로 속도 계산
    # 3. robot.update(v, w, dt)로 로봇 업데이트
    # 4. 목표 도달 및 충돌 확인
```

---

## 📊 평가 시스템

### 점수 계산
```
점수 = 1000 - (거리 × 2) - (시간 × 5)
충돌 시 = 0점
```

### 목표
- ✅ 충돌 없이 목표 도달
- 📏 주행 거리 최소화
- ⏱️ 소요 시간 최소화

### 랭킹
- `rankings.json`에 자동 저장
- 이름, 점수, 거리, 시간, 알고리즘 기록

---

## 📁 파일 구조

```
pathplanning_assign/
├── common/                          # 공통 모듈
│   ├── __init__.py
│   ├── robot.py                    # Robot 클래스
│   └── environment.py              # Environment, Obstacle 클래스
├── scenario1_global_planning.py    # A*, Dijkstra
├── scenario2_local_planning.py     # DWA
├── scenario3_solution.py           # 통합 과제 (학생 구현)
├── rankings.json                   # 랭킹 데이터
└── README.md                       # 이 파일
```

---

## 🎓 과제 제출

### 제출 파일
깃허브에서 다운로드 받은 폴더 전체를 zip으로 압축하여 제출.
용량 초과로 제출 안될 시, thddbfl20217@gmail.com으로 제출

### 보고서 내용
1. 시나리오 1, 2 파라미터 튜닝 설명
2. 시나리오3 구현 설명
3. 성능 분석
4. 최적화 전략
5. 결과 스크린샷


---

## 📞 문의

과제 관련 문의: 담당 조교

Happy Coding! 🤖🚀

---
