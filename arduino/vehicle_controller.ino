/*
 * MS3 Vehicle Controller – Team 23 (FINAL TUNED)
 * Autonomous Ackermann Vehicle (MCTR1002)
 *
 * Serial: Raspberry Pi USB-A -> Arduino USB-B (/dev/ttyUSB0)
 * Command format: SPD:<float>,STR:<float>\n
 *
 * ════════════════════════════════════════════════════════════════════════
 *  WIRING (VERIFIED & TESTED 2026-04-28)
 * ════════════════════════════════════════════════════════════════════════
 *
 *  Arduino D6  (PWM/Timer0) ──> L298N ENA   << Timer0, no Servo conflict
 *  Arduino D8               ──> L298N IN1
 *  Arduino D7               ──> L298N IN2
 *  Arduino D9  (PWM/Timer1) ──> MG995 servo signal (Timer1 via Servo.h)
 *  Arduino D2  (INT0)       ──> Encoder yellow wire (ch A)
 *  Arduino D3  (INT1)       ──> Encoder green  wire (ch B)
 *  Arduino GND              ──> common GND rail
 *
 *  L298N 12V/Vs  ──> 12V battery rail
 *  L298N GND     ──> common GND rail
 *  L298N 5V pin  ──> LM2596 5V rail  (both jumpers REMOVED)
 *  L298N OUT1    ──> Motor red wire
 *  L298N OUT2    ──> Motor white wire
 *
 *  Forward:  IN1=HIGH, IN2=LOW,  ENA=PWM
 *  Reverse:  IN1=LOW,  IN2=HIGH, ENA=PWM
 *
 * ════════════════════════════════════════════════════════════════════════
 *  CALIBRATION (measured on hardware 2026-04-28)
 *    Encoder ticks/rev  = 748  (25GA370 @ 34:1, 11 PPR, both edges)
 *    Direction sign     = +1.0 (forward = positive ticks)
 *    Speed @ PWM 150    = 0.569 m/s
 *    Speed @ PWM 255    ≈ 0.97  m/s  → SPEED_MAX set to 1.0 m/s
 * ════════════════════════════════════════════════════════════════════════
 */

#include <Servo.h>

// ── Pins (TESTED – do not change) ─────────────────────────────────────
const int MOTOR_ENA = 6;    // Timer0 PWM – no conflict with Servo on Timer1
const int MOTOR_IN1 = 8;
const int MOTOR_IN2 = 7;
const int SERVO_PIN = 9;    // Timer1 via Servo.h
const int ENCODER_A = 2;    // INT0
const int ENCODER_B = 3;    // INT1

// ── Encoder calibration ────────────────────────────────────────────────
const float ENCODER_TICKS_PER_WHEEL_REV = 748.0;
const float WHEEL_DIAMETER_M            = 0.065;
const float ENCODER_DIRECTION_SIGN      = 1.0;   // +1 = forward is positive
const float SPEED_FILTER_ALPHA          = 0.30;  // Slightly more smoothing

// ── Speed P controller ─────────────────────────────────────────────────
// Feedforward dominates: PWM = (cmd/SPEED_MAX)*255
// KP corrects load/slope disturbances.
// Tuned: KP=150 gives stable response; raise to 200 if sluggish under load.
const float SPEED_KP = 150.0;

// ── Motor limits ───────────────────────────────────────────────────────
const int   MOTOR_PWM_MIN  = 75;    // Below this: motor stalls. Measured safe minimum.
const int   MOTOR_PWM_MAX  = 255;
const float SPEED_MAX      = 1.0;   // m/s  – raised from 0.5; hardware confirmed ~0.97 m/s at full PWM
const float SPEED_DEADBAND = 0.005; // m/s  – below this = stop

// ── Steering ───────────────────────────────────────────────────────────
// SERVO_CENTER: trim this on hardware if wheels aren't perfectly straight.
// Increase from 90 if car drifts left, decrease if it drifts right.
const int   SERVO_CENTER  = 90;    // degrees – calibrate on physical car
const int   SERVO_RANGE   = 40;    // degrees each side – raised for more authority
const float STEERING_MAX  = 0.5;   // rad – must match teleop max_turn_rate

// ── Timing ─────────────────────────────────────────────────────────────
const unsigned long CMD_TIMEOUT_MS    = 2000;  // stop if no cmd for 2 s
const unsigned long CONTROL_PERIOD_MS = 20;    // 50 Hz control loop

// ── State ──────────────────────────────────────────────────────────────
Servo steeringServo;

float cmd_speed    = 0.0;
float cmd_steering = 0.0;

volatile long encoder_ticks      = 0;
long          prev_encoder_ticks = 0;
float         measured_speed_mps = 0.0;
float         filtered_speed_mps = 0.0;
float         last_signed_pwm    = 0.0;

unsigned long last_cmd_time     = 0;
unsigned long last_control_time = 0;

char buf[64];
int  buf_idx = 0;

// ── Prototypes ─────────────────────────────────────────────────────────
void encoderAInterrupt();
void parseCommand(const char* line);
void updateMeasuredSpeed(float dt_s);
void applyMotorClosedLoop();
void applySteering();

// ══════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // Motor
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  analogWrite(MOTOR_ENA, 0);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  // Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderAInterrupt, CHANGE);

  // Servo – attach after encoder ISR is set, center immediately
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);

  last_cmd_time     = millis();
  last_control_time = millis();

  Serial.println(F("MS3 Team 23 ready"));
  Serial.print(F("ENA=D")); Serial.print(MOTOR_ENA);
  Serial.print(F(" IN1=D")); Serial.print(MOTOR_IN1);
  Serial.print(F(" IN2=D")); Serial.print(MOTOR_IN2);
  Serial.print(F(" SERVO=D")); Serial.print(SERVO_PIN);
  Serial.print(F(" ENC=D2/D3 SPEED_MAX="));
  Serial.print(SPEED_MAX, 1);
  Serial.print(F(" SERVO_RANGE=+-"));
  Serial.println(SERVO_RANGE);
}

// ══════════════════════════════════════════════════════════════════════
void loop() {
  // Read serial
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[buf_idx] = '\0';
      if (buf_idx > 0) {
        parseCommand(buf);
        last_cmd_time = millis();
      }
      buf_idx = 0;
    } else if (buf_idx < (int)sizeof(buf) - 1) {
      buf[buf_idx++] = c;
    }
  }

  // Safety timeout
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    cmd_speed    = 0.0;
    cmd_steering = 0.0;
  }

  // 50 Hz control
  unsigned long now = millis();
  if (now - last_control_time >= CONTROL_PERIOD_MS) {
    float dt_s = (now - last_control_time) / 1000.0f;
    last_control_time = now;
    updateMeasuredSpeed(dt_s);
    applyMotorClosedLoop();
    applySteering();
  }
}

// ══════════════════════════════════════════════════════════════════════
void parseCommand(const char* line) {
  const char* spd_ptr = strstr(line, "SPD:");
  const char* str_ptr = strstr(line, "STR:");
  if (spd_ptr) cmd_speed    = atof(spd_ptr + 4);
  if (str_ptr) cmd_steering = atof(str_ptr + 4);

  // Telemetry: ACK SPD:x.xxx STR:x.xxx MEAS:x.xxx PWM:xxx.x
  Serial.print(F("ACK SPD:"));  Serial.print(cmd_speed,          3);
  Serial.print(F(" STR:"));     Serial.print(cmd_steering,       3);
  Serial.print(F(" MEAS:"));    Serial.print(filtered_speed_mps, 3);
  Serial.print(F(" PWM:"));     Serial.println(last_signed_pwm,  1);
}

// ══════════════════════════════════════════════════════════════════════
void updateMeasuredSpeed(float dt_s) {
  if (dt_s <= 0.0f) return;

  long ticks_now;
  noInterrupts();
  ticks_now = encoder_ticks;
  interrupts();

  long delta = ticks_now - prev_encoder_ticks;
  prev_encoder_ticks = ticks_now;

  float mpt     = (PI * WHEEL_DIAMETER_M) / ENCODER_TICKS_PER_WHEEL_REV;
  float instant = (ENCODER_DIRECTION_SIGN * delta * mpt) / dt_s;

  measured_speed_mps = instant;
  filtered_speed_mps = SPEED_FILTER_ALPHA * instant
                     + (1.0f - SPEED_FILTER_ALPHA) * filtered_speed_mps;
}

// ══════════════════════════════════════════════════════════════════════
void applyMotorClosedLoop() {
  float target = constrain(cmd_speed, -SPEED_MAX, SPEED_MAX);

  if (abs(target) <= SPEED_DEADBAND) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, 0);
    last_signed_pwm = 0.0f;
    return;
  }

  // Feedforward: scales command directly to PWM range
  // P term: corrects remaining error from load/slope
  float error  = target - filtered_speed_mps;
  float ff     = (target / SPEED_MAX) * MOTOR_PWM_MAX;
  float p      = SPEED_KP * error;
  float result = constrain(ff + p, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);

  int pwm = (int)abs(result);
  if (pwm > 0 && pwm < MOTOR_PWM_MIN) pwm = MOTOR_PWM_MIN;

  // L298N direction
  if (result > 0.5f) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  } else if (result < -0.5f) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    pwm = 0;
  }

  analogWrite(MOTOR_ENA, pwm);
  last_signed_pwm = result;
}

// ══════════════════════════════════════════════════════════════════════
void applySteering() {
  // Positive cmd_steering = left (ROS convention)
  // Negate: left command → servo angle < center
  float ratio = constrain(-cmd_steering / STEERING_MAX, -1.0f, 1.0f);
  int angle   = SERVO_CENTER + (int)(ratio * SERVO_RANGE);
  angle = constrain(angle, SERVO_CENTER - SERVO_RANGE, SERVO_CENTER + SERVO_RANGE);
  steeringServo.write(angle);
}

// ══════════════════════════════════════════════════════════════════════
void encoderAInterrupt() {
  bool a = digitalRead(ENCODER_A);
  bool b = digitalRead(ENCODER_B);
  if (a == b) encoder_ticks++;
  else        encoder_ticks--;
}
