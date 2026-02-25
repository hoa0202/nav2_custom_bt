# cmd_vel 각속도 스무딩 방안

## 문제 상황
- 각속도에 최소값(0.85 rad/s)을 boost하고 있음
- 방향 전환 시 급격한 변화 발생 (예: +0.95 → -0.95)
- 로봇이 "탁탁" 거리며 움직임

## 현재 로직
```python
if wz != 0:
    out_wz = wz + sign(wz) * 0.85
```

---

## 방안 1: Rate Limiter (변화율 제한)

### 개념
한 스텝에 변할 수 있는 최대 각속도 변화량을 제한

### 파라미터
- `max_angular_rate`: 스텝당 최대 변화량 (예: 0.5 rad/s)

### 동작 예시
```
현재 출력: +0.95
새 목표:   -0.95
max_rate:  0.5 rad/s per step

Step 1: +0.95 → +0.45
Step 2: +0.45 → -0.05
Step 3: -0.05 → -0.55
Step 4: -0.55 → -0.95 (목표 도달)
```

### 코드
```python
def apply_rate_limit(self, target, current, max_rate):
    diff = target - current
    if abs(diff) > max_rate:
        return current + math.copysign(max_rate, diff)
    return target
```

### 장점
- 구현 간단
- 반응성 유지
- 예측 가능한 동작

### 단점
- 급격한 변화 시 여러 스텝 필요
- max_rate 튜닝 필요

---

## 방안 2: Low-pass Filter (저역 통과 필터)

### 개념
이전 출력값과 현재 목표값의 가중 평균

### 파라미터
- `alpha`: 스무딩 계수 (0~1, 낮을수록 부드러움)

### 공식
```
out = alpha * target + (1 - alpha) * previous_out
```

### 동작 예시 (alpha = 0.3)
```
현재 출력: +0.95
새 목표:   -0.95

Step 1: 0.3 * (-0.95) + 0.7 * (+0.95) = +0.38
Step 2: 0.3 * (-0.95) + 0.7 * (+0.38) = -0.02
Step 3: 0.3 * (-0.95) + 0.7 * (-0.02) = -0.30
Step 4: 0.3 * (-0.95) + 0.7 * (-0.30) = -0.50
...
```

### 코드
```python
def apply_lowpass(self, target, previous, alpha):
    return alpha * target + (1 - alpha) * previous
```

### 장점
- 매우 부드러운 전환
- 노이즈 제거 효과

### 단점
- 반응 지연 발생
- 목표값에 완전히 도달하지 못할 수 있음
- alpha 튜닝 필요

---

## 방안 3: Rate Limiter + 방향 전환 특별 처리

### 개념
- 기본: Rate Limiter 적용
- 방향 전환 시: **반드시 0을 거쳐서** 반대 방향으로 가속

### 파라미터
- `max_angular_rate`: 스텝당 최대 변화량
- `zero_crossing_rate`: 0 통과 시 변화량 (더 느리게 설정 가능)

### 동작 예시
```
현재 출력: +0.95
새 목표:   -0.95
max_rate:  0.5 rad/s

Phase 1 (감속 to 0):
  +0.95 → +0.45 → 0.0

Phase 2 (가속 to target):
  0.0 → -0.50 → -0.95
```

### 코드
```python
def apply_rate_limit_with_zero_crossing(self, target, current, max_rate):
    # 방향 전환 감지 (부호가 다르고 둘 다 0이 아님)
    if current * target < 0:
        # 먼저 0으로 감속
        if abs(current) > max_rate:
            return current - math.copysign(max_rate, current)
        else:
            return 0.0  # 0 도달
    else:
        # 일반 rate limit
        diff = target - current
        if abs(diff) > max_rate:
            return current + math.copysign(max_rate, diff)
        return target
```

### 장점
- 가장 자연스러운 방향 전환
- 기계적 스트레스 최소화
- 물리적으로 합리적인 동작

### 단점
- 구현 복잡
- 방향 전환 시간 더 길어짐

---

## 방안 4: Exponential Smoothing (지수 스무딩)

### 개념
목표값과 현재값의 차이에 비례하여 이동

### 파라미터
- `smoothing_factor`: 스무딩 강도 (0~1)

### 공식
```
out = previous + smoothing_factor * (target - previous)
```

### 동작 예시 (factor = 0.2)
```
현재: +0.95, 목표: -0.95

Step 1: +0.95 + 0.2 * (-1.90) = +0.57
Step 2: +0.57 + 0.2 * (-1.52) = +0.27
Step 3: +0.27 + 0.2 * (-1.22) = +0.03
Step 4: +0.03 + 0.2 * (-0.98) = -0.17
...
```

### 코드
```python
def apply_exponential_smoothing(self, target, previous, factor):
    return previous + factor * (target - previous)
```

### 장점
- 목표에 가까울수록 변화량 감소 (자연스러움)
- 부드러운 정지

### 단점
- 목표에 정확히 도달하는 데 시간 걸림
- Low-pass와 비슷한 지연 문제

---

## 비교 요약

| 방안 | 부드러움 | 반응성 | 구현 복잡도 | 추천 용도 |
|------|----------|--------|-------------|-----------|
| 1. Rate Limiter | ★★★☆☆ | ★★★★☆ | ★☆☆☆☆ | 일반적인 경우 |
| 2. Low-pass | ★★★★★ | ★★☆☆☆ | ★☆☆☆☆ | 노이즈 많은 경우 |
| 3. Rate + Zero | ★★★★☆ | ★★★☆☆ | ★★★☆☆ | 방향 전환 많은 경우 |
| 4. Exponential | ★★★★☆ | ★★★☆☆ | ★★☆☆☆ | 정밀 제어 필요 시 |

---

## 테스트 순서 제안

1. **방안 1 (Rate Limiter)** - 가장 간단, 기본 효과 확인
2. **방안 3 (Rate + Zero)** - 방향 전환 개선 확인
3. **방안 2 or 4** - 추가 부드러움 필요 시

---

## 테스트 방법

```bash
# 1. 파일 수정 후 빌드
colcon build --packages-select nav2_custom_bt

# 2. Nav2 재시작
ros2 launch nav2_custom_bt navigation_launch.py

# 3. cmd_vel 모니터링
ros2 topic echo /cmd_vel

# 4. 방향 전환 테스트 (목표점 변경하며 관찰)
```
