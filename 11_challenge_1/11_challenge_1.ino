#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low (LOW=ON, HIGH=OFF)
#define PIN_TRIG  12
#define PIN_ECHO  13
#define PIN_SERVO 10

// sonar params
#define SND_VEL 346.0
#define INTERVAL 25            // ms, 주기
#define PULSE_DURATION 10      // us: TRIG 펄스 길이

// ===== 거리(단위 mm) 관련 상수 =====
#define _DIST_MIN 180.0        // 18 cm (하한)
#define _DIST_MAX 360.0        // 36 cm (상한)

// pulseIn timeout 계산
#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)  // pulse us -> mm 변환 상수

// ===== EMA 설정 =====
// α(알파): 새 샘플 가중치.
// 이 값을 바꾸면 반응성/부드러움이 달라짐.
#define _EMA_ALPHA 0.3   // <-- 알파 튜닝은 여기 숫자만 수정하면 됨

// servo duty 범위 (네 서보에 맞춰서 보정)
#define _DUTY_MIN 500   // us, 0°
#define _DUTY_MAX 2400   // us, 180°

float  dist_ema;                 // EMA 결과 (mm)
float  dist_prev = _DIST_MAX;    // 마지막 유효 raw값
bool   ema_initialized = false;  // EMA 초기화 플래그

unsigned long last_sampling_time; // 주기 관리(ms)

Servo myservo;


// 거리(mm) -> 서보 각도(0~180도)
float distanceToAngle(float d_mm) {
  if (d_mm <= _DIST_MIN) {
    return 0.0;
  } else if (d_mm >= _DIST_MAX) {
    return 180.0;
  } else {
    float ratio = (d_mm - _DIST_MIN) / (_DIST_MAX - _DIST_MIN); // 0~1
    return ratio * 180.0; // 0~180도
  }
}

// 각도(0~180도) -> 서보 펄스us
int angleToDuty(float deg){
  float duty = _DUTY_MIN + (deg / 180.0f) * (_DUTY_MAX - _DUTY_MIN);
  return (int)duty;
}


// 초음파 센서로 거리(mm) 측정
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // mm
}


void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_MIN); // 초기 위치

  Serial.begin(57600);

  last_sampling_time = millis();
}


void loop() {
  // 주기 제어 (언더플로우 안전 방식)
  if (millis() - last_sampling_time < INTERVAL) return;
  last_sampling_time = millis();

  // 1) 초음파 raw(mm)
  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 2) 범위 필터 (18~36cm 밖이면 이전값 유지)
  float dist_filtered;
  if (dist_raw == 0.0 || dist_raw < _DIST_MIN || dist_raw > _DIST_MAX) {
    dist_filtered = dist_prev;
  } else {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
  }

  // 3) EMA 필터
  if (!ema_initialized) {
    dist_ema = dist_filtered;
    ema_initialized = true;
  } else {
    dist_ema = _EMA_ALPHA * dist_filtered + (1.0f - _EMA_ALPHA) * dist_ema;
  }

  // 4) 거리 -> 각도 -> 펄스
  float servo_deg = distanceToAngle(dist_ema);
  int duty_us = angleToDuty(servo_deg);

  myservo.writeMicroseconds(duty_us);

  // (옵션) LED: 너무 가까우면 경고등 켜기 예시
  if (dist_ema <= 200.0) {          // 20cm 이하일 때 LED ON
    digitalWrite(PIN_LED, LOW);     // active-low -> ON
  } else {
    digitalWrite(PIN_LED, HIGH);    // OFF
  }

  // 5) 플로터용 시리얼 출력 (네가 준 포맷 그대로)
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",dist:");  Serial.print(dist_raw);
  Serial.print(",ema:");   Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(myservo.read()); // 서보 각도 추정치(Servo 라이브러리 기준)
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");
}
