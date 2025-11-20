# Robot Navigation 과제 설치 가이드

## 📋 필수 요구사항

- **Python 3.8 이상** (Python 3.10 이상 권장)
- **필수 패키지**: numpy, matplotlib
- 가상환경은 **선택사항**입니다.
---

## 🚀 설치 방법

### 방법 1: 가상환경 사용

```bash
# 1. 과제 디렉토리로 이동
cd pathplanning_assign

# 2. Python 가상환경 생성
python3 -m venv .venv

# 3. 가상환경 활성화
source .venv/bin/activate  # Linux/Mac
# 또는
.venv\Scripts\activate     # Windows

# 4. 필요한 패키지 설치
pip install -r requirements.txt

# 5. 시나리오 실행 테스트
python scenario1_global_planning.py
```

### 방법 2: 시스템 Python 사용

```bash
# 1. 과제 디렉토리로 이동
cd pathplanning_assign

# 2. 필요한 패키지 설치
pip install numpy matplotlib

# 3. 시나리오 실행 테스트
python scenario1_global_planning.py
```

---

## 🔍 설치 확인

```bash
# Python 버전 확인
python --version  # 3.8 이상이어야 함

# 필수 패키지 확인
python -c "import numpy; print('numpy:', numpy.__version__)"
python -c "import matplotlib; print('matplotlib:', matplotlib.__version__)"
```

**정상 출력 예시:**
```
numpy: 1.24.3
matplotlib: 3.7.1
```

---

## ⚠️ 문제 해결

### Q1: `python3: command not found`
```bash
# python3 대신 python 사용
python -m venv .venv
python -c "import sys; print(sys.version)"
```

### Q2: `pip: command not found`
```bash
# pip 설치
sudo apt install python3-pip  # Ubuntu/Debian
# 또는
python -m ensurepip --upgrade
```

### Q3: `Permission denied` (Linux/Mac)
```bash
# pip install 시 --user 옵션 사용
pip install --user numpy matplotlib
```

### Q4: 가상환경을 만들고 싶지 않아요
```bash
# 시스템 Python에 직접 설치 (가상환경 없이)
pip install numpy matplotlib

# 바로 실행
python scenario1_global_planning.py
```

### Q5: matplotlib 창이 안 뜨는 경우
```bash
# tkinter 설치 (Ubuntu/Debian)
sudo apt install python3-tk

# matplotlib backend 확인
python -c "import matplotlib; print(matplotlib.get_backend())"
```

---

## 📁 디렉토리 구조 확인

```bash
pathplanning_assign/
├── common/                    # 필수!
│   ├── __init__.py
│   ├── robot.py
│   └── environment.py
├── scenario1_global_planning.py
├── scenario2_local_planning.py
├── scenario3_challenge.py     # 학생 과제 파일
├── scenario3_solution.py      # 정답 참고용
├── requirements.txt           # 이 파일!
├── INSTALL.md                 # 설치 가이드
└── README.md
```

---

## ✅ 빠른 시작 (요약)

```bash
# 1단계: 가상환경 생성 및 활성화 (선택사항)
python3 -m venv .venv
source .venv/bin/activate

# 2단계: 패키지 설치
pip install -r requirements.txt

# 3단계: 시나리오 실행
python scenario1_global_planning.py  # 시나리오 1
python scenario2_local_planning.py   # 시나리오 2
python scenario3_challenge.py        # 시나리오 3 (과제)
```

---

## 💡 추가 정보

### Anaconda/Miniconda 사용자

```bash
# conda 환경 생성
conda create -n pathplanning python=3.10
conda activate pathplanning

# 패키지 설치
conda install numpy matplotlib

# 실행
python scenario1_global_planning.py
```


