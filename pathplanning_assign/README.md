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
python scenario3_solution.py
```

---

## 📚 시나리오 1: Global Path Planning

### 개요
정적 장애물 환경에서 A* 또는 Dijkstra로 전역 경로를 계획합니다.

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

`scenario3_solution.py`

### 실행
```bash
python scenario3_solution.py
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

### 구현 힌트

**핵심 아이디어:**
```python
# Global path 생성
from scenario1_global_planning import AStarPlanner
planner = AStarPlanner(env, grid_resolution=1.0)
global_path = planner.plan(START, GOAL)

# Local goal 선택 (전방 주시)
robot_pos = robot.get_position()
distances = np.linalg.norm(global_path - robot_pos, axis=1)
closest_idx = np.argmin(distances)
local_goal = global_path[closest_idx + look_ahead]

# DWA로 추종
from scenario2_local_planning import DWA
dwa = DWA(robot, dwa_config, env)
v, w, _, _ = dwa.plan(local_goal, env)
robot.update(v, w, dt)
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
- TOP 10 랭킹 표시
- 이름, 점수, 거리, 시간, 알고리즘 기록

---

## 🔧 최적화 팁

### Global Planning
- 그리드 해상도 조정
- 경로 스무딩
- 장애물 마진 설정

### Local Planning
- DWA 가중치 튜닝
- 예측 시간 조정
- Look-ahead 거리 최적화

### 통합
- 적응형 속도 제어
- 동적 재계획
- 하이브리드 전략

---

## 🐛 디버깅 가이드

**로봇이 안 움직임**
→ `v, w` 값 확인, `get_local_goal()` 반환값 체크

**장애물 충돌**
→ `obstacle_cost_gain` 높이기 (5.0 이상)

**목표 미도달**
→ `to_goal_cost_gain` 높이기, look-ahead 조정

**속도 느림**
→ `v_reso`, `w_reso` 줄이기 (5, 10 등)

---

## 📖 알고리즘 설명

### A* 알고리즘
```
f(n) = g(n) + h(n)
g: 시작점에서 현재까지 비용
h: 현재에서 목표까지 추정 비용 (휴리스틱)
```

### Dijkstra 알고리즘
```
A*에서 h(n) = 0인 경우
모든 방향 균일하게 탐색
```

### DWA (Dynamic Window Approach)
```
1. 로봇의 동역학 제약으로 속도 공간 제한
2. 가능한 (v, w) 조합 샘플링
3. 각 조합의 궤적 예측
4. 비용 함수로 최적 속도 선택
```

### 로봇 운동학
```
x' = v * cos(θ)
y' = v * sin(θ)  
θ' = ω
```

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
1. `scenario3_solution.py` (구현 완료)
2. `rankings.json` (실행 결과)
3. `report.pdf` (분석 보고서)

### 보고서 내용
1. 구현 설명
2. 파라미터 튜닝 과정
3. 성능 분석
4. 최적화 전략
5. 결과 스크린샷

---

## 💡 FAQ

**Q: Global path가 장애물 통과**  
A: 그리드 해상도 낮추기 (0.5)

**Q: DWA가 지역 최적해에 갇힘**  
A: `to_goal_cost_gain` 높이기

**Q: 동적 장애물 충돌**  
A: `predict_time` 늘리고 `obstacle_cost_gain` 높이기

**Q: 랭킹 미등록**  
A: 충돌하면 0점, 목표 도달 필수

---

## 📞 문의

과제 관련 문의: 담당 조교

Happy Coding! 🤖🚀

---

## 📊 평가 기준

### 점수 계산 방식

```
기본 점수: 1000점
감점: 주행 거리 × 2 + 소요 시간 × 5
충돌 시: 0점
```

**목표:**
- 주행 거리를 최소화
- 소요 시간을 최소화
- 충돌 없이 목표 도달

### 랭킹 시스템

- 모든 실행 결과는 `rankings.json`에 자동 저장
- 실행할 때마다 TOP 10 랭킹이 표시됨
- 학생 이름, 점수, 거리, 시간, 알고리즘이 기록됨

---

## 🔧 고급 최적화 팁

### 1. Global Path Planning 최적화

- **그리드 해상도**: 장애물이 많은 영역은 세밀하게, 빈 공간은 거칠게
- **휴리스틱 함수**: 유클리드 거리 대신 맨해튼 거리 시도
- **경로 스무딩**: 계획된 경로를 부드럽게 만들어 주행 거리 단축

### 2. Local Path Planning 최적화

- **동적 look-ahead**: 속도에 따라 전방 주시 거리 조정
- **비용 함수 튜닝**: 환경에 따라 가중치 동적 조정
- **예측 시간 조정**: 장애물이 많을 때는 짧게, 적을 때는 길게

### 3. 통합 최적화

- **경로 재계획**: 이동 장애물이 전역 경로를 막으면 재계획
- **적응형 속도**: 장애물 밀도에 따라 최대 속도 조정
- **하이브리드 전략**: 목표 근처에서는 순수 DWA 사용

---

## 📈 디버깅 가이드

### 로봇이 움직이지 않는 경우

1. `execute_step` 함수에서 `v, w` 값을 print로 확인
2. `get_local_goal`이 None을 반환하는지 확인
3. DWA의 dynamic window가 올바른지 확인

### 로봇이 장애물에 충돌하는 경우

1. `obstacle_cost_gain` 가중치를 높이기 (3.0 이상)
2. 로봇 반경(`ROBOT_RADIUS`)을 크게 설정
3. DWA의 `predict_time`을 늘리기

### 로봇이 목표에 도달하지 못하는 경우

1. `to_goal_cost_gain` 가중치를 높이기
2. `get_local_goal`의 look-ahead 거리 조정
3. Global path가 올바르게 생성되었는지 확인

### 실행 속도가 느린 경우

1. `v_reso`, `w_reso` 값을 줄이기 (5, 10 등)
2. `GRID_RESOLUTION`을 크게 (2.0 이상)
3. 시각화 없는 모드로 실행

---

## 📚 참고 자료

### 알고리즘 설명

- **A* 알고리즘**: f(n) = g(n) + h(n) 최소화 (g: 실제 비용, h: 휴리스틱)
- **Dijkstra 알고리즘**: A*에서 h(n) = 0인 특수 케이스
- **DWA**: 동적 제약 하에서 속도 공간을 샘플링하여 최적 경로 선택

### 로봇 운동학

```
x' = v * cos(θ)
y' = v * sin(θ)
θ' = ω
```

- v: 선속도 (m/s)
- ω: 각속도 (rad/s)
- θ: 방향 (rad)

---

## 🎯 과제 제출

### 제출 파일

1. `scenario3_student_challenge.py` (구현 완료된 코드)
2. `rankings.json` (실행 결과)
3. `report.pdf` (실험 결과 및 분석 보고서)

### 보고서 포함 내용

1. 구현 설명
2. 파라미터 튜닝 과정
3. 성능 분석 (거리, 시간, 점수)
4. 최적화 전략
5. 결과 스크린샷

---

## 💡 FAQ

**Q: Global path가 장애물을 통과합니다.**
A: 그리드 해상도를 낮추거나(0.5), 장애물 주변에 마진을 추가하세요.

**Q: DWA가 지역 최적해에 갇힙니다.**
A: Global path를 따라가도록 `to_goal_cost_gain`을 높이세요.

**Q: 이동 장애물과 자주 충돌합니다.**
A: `predict_time`을 늘리고, `obstacle_cost_gain`을 높이세요.

**Q: 랭킹에 등록되지 않습니다.**
A: 목표에 도달해야 하며, 충돌하면 0점 처리됩니다.

---

## 👨‍🏫 교수자용 정보

### 채점 기준

- 코드 구현 완성도: 40%
- 성능 (랭킹 점수): 30%
- 보고서 품질: 20%
- 창의적 최적화: 10%

### 환경 변경

`ChallengeEnvironment.create()` 함수에서 장애물 배치를 변경하여 난이도 조절 가능.

### 파라미터 잠금

학생들이 수정하면 안 되는 부분을 명시하려면 코드에 주석 추가:
```python
# ===== 수정 금지 구역 시작 =====
# ===== 수정 금지 구역 끝 =====
```

---

## 📞 문의

과제 관련 문의사항은 담당 조교에게 연락하세요.

Happy Coding! 🤖🚀
