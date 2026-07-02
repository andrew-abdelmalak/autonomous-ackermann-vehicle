/*
 * MS4 Vehicle Controller – Team 23 (Raspberry Pi Pico Version)
 *
 * Serial command format from Pi:
 *   SPD:<float>,STR:<float>\n
 *
 * Telemetry format back to Pi:
 *   VEL:<float>,THETA:<float>,TICKS:<int>\n
 *
 * VEL   = wheel speed in m/s from encoder
 * THETA = heading in degrees from MPU-6050 gyro integration
 *
 * This version has been adapted for the Raspberry Pi Pico.
 */

#include <Servo.h>
#include <Wire.h>
#include <mbed.h>

// ── Pico Pin Assignments ──────────────────────────────────────
const int MOTOR_ENA_PIN = 16;  // GP16 (handled by mbed::PwmOut)
mbed::PwmOut motorENA(digitalPinToPinName(16));
const int MOTOR_IN1 = 15;  // GP15
const int MOTOR_IN2 = 14;  // GP14
const int SERVO_PIN = 17;  // GP17
const int ENCODER_A = 2;   // GP2 (Interrupt)
const int ENCODER_B = 3;   // GP3
// I2C0 pins: SDA = GP4, SCL = GP5

// ── MPU-6050 registers ────────────────────────────────────────
const uint8_t MPU_ADDR = 0x68;
const uint8_t REG_PWR_MGMT1 = 0x6B;
const uint8_t REG_ACCEL_CFG = 0x1C;
const uint8_t REG_GYRO_CFG = 0x1B;
const uint8_t REG_ACCEL_XOUT = 0x3B;

// ── IMU scales ────────────────────────────────────────────────
const float ACCEL_SCALE = 16384.0;  // +/-2 g
const float GYRO_SCALE = 131.0;     // +/-250 deg/s
const int CALIB_SAMPLES = 300;

// ── Encoder calibration ───────────────────────────────────────
const float ENCODER_TICKS_PER_METER = 7000.0;
const float ENCODER_DIRECTION_SIGN = 1.0;
const float SPEED_FILTER_ALPHA = 0.30;

// ── Speed controller ──────────────────────────────────────────
const float SPEED_KP = 150.0;
const int MOTOR_PWM_MIN = 75;
const int MOTOR_PWM_MAX = 255;
const float SPEED_MAX = 1.0;
const float SPEED_DEADBAND = 0.005;

// ── Steering ──────────────────────────────────────────────────
const int SERVO_CENTER = 90;
const int SERVO_RANGE = 40;
const float STEERING_MAX = 0.5;
const float STEERING_KP = 1.5;
const float STEERING_KD = 0.8;
float target_heading_rad = 0.0;
bool heading_locked = true;
float current_gz_deg_s = 0.0;

// ── Timing ────────────────────────────────────────────────────
const unsigned long CMD_TIMEOUT_MS = 2000;
const unsigned long CONTROL_PERIOD_MS = 20;   // 50 Hz
const unsigned long TELEMETRY_PERIOD_MS = 50; // 20 Hz

Servo steeringServo;

float cmd_speed = 0.0;
float cmd_steering = 0.0;

volatile long encoder_ticks = 0;
long prev_encoder_ticks = 0;
float measured_speed_mps = 0.0;
float filtered_speed_mps = 0.0;
float last_signed_pwm = 0.0;

float gx_off = 0.0;
float gy_off = 0.0;
float gz_off = 0.0;
float heading_rad = 0.0;

unsigned long last_cmd_time = 0;
unsigned long last_control_time = 0;
unsigned long last_telemetry_time = 0;

char buf[64];
int buf_idx = 0;

void encoderAInterrupt();
void parseCommand(const char* line);
void updateMeasuredSpeed(float dt_s);
void applyMotorClosedLoop();
void applySteering();
void publishTelemetry();
void initializeImu();
void calibrateImu();
void updateHeading(float dt_s);
void writeByte(uint8_t reg, uint8_t value);
void readAll(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

void setup() {
  Serial.begin(115200);

  // Set PWM slice to 50Hz (20ms) for Servo compatibility
  motorENA.period_ms(20);
  motorENA.write(0.0f);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderAInterrupt, CHANGE);

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);

  initializeImu();

  last_cmd_time = millis();
  last_control_time = millis();
  last_telemetry_time = millis();

  Serial.println(F("MS4 Team 23 ready (Raspberry Pi Pico)"));
  Serial.println(F("Streaming VEL/THETA telemetry"));
}

void loop() {
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

  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    cmd_speed = 0.0;
    cmd_steering = 0.0;
  }

  unsigned long now = millis();
  if (now - last_control_time >= CONTROL_PERIOD_MS) {
    float dt_s = (now - last_control_time) / 1000.0f;
    last_control_time = now;

    updateMeasuredSpeed(dt_s);
    updateHeading(dt_s);
    applyMotorClosedLoop();
    applySteering();
  }

  if (now - last_telemetry_time >= TELEMETRY_PERIOD_MS) {
    last_telemetry_time = now;
    publishTelemetry();
  }
}

void parseCommand(const char* line) {
  if (line[0] == 'r' || line[0] == 'R') {
    heading_rad = 0.0;
    target_heading_rad = 0.0;
    heading_locked = true;
    noInterrupts();
    encoder_ticks = 0;
    interrupts();
    prev_encoder_ticks = 0;
    measured_speed_mps = 0.0;
    filtered_speed_mps = 0.0;
    return;
  }

  const char* spd_ptr = strstr(line, "SPD:");
  const char* str_ptr = strstr(line, "STR:");
  if (spd_ptr) {
    cmd_speed = atof(spd_ptr + 4);
  }
  if (str_ptr) {
    cmd_steering = atof(str_ptr + 4);
  }
}

void updateMeasuredSpeed(float dt_s) {
  if (dt_s <= 0.0f) {
    return;
  }

  long ticks_now;
  noInterrupts();
  ticks_now = encoder_ticks;
  interrupts();

  long delta = ticks_now - prev_encoder_ticks;
  prev_encoder_ticks = ticks_now;

  float meters_per_tick = 1.0f / ENCODER_TICKS_PER_METER;
  float instant = (ENCODER_DIRECTION_SIGN * delta * meters_per_tick) / dt_s;

  measured_speed_mps = instant;
  filtered_speed_mps = SPEED_FILTER_ALPHA * instant
                     + (1.0f - SPEED_FILTER_ALPHA) * filtered_speed_mps;
}

void applyMotorClosedLoop() {
  float target = constrain(cmd_speed, -SPEED_MAX, SPEED_MAX);

  if (abs(target) <= SPEED_DEADBAND) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    motorENA.write(0.0f);
    last_signed_pwm = 0.0f;
    return;
  }

  float error = target - filtered_speed_mps;
  float ff = (target / SPEED_MAX) * MOTOR_PWM_MAX;
  float p = SPEED_KP * error;
  float result = constrain(ff + p, -MOTOR_PWM_MAX, MOTOR_PWM_MAX);

  int pwm = (int)abs(result);
  if (pwm > 0 && pwm < MOTOR_PWM_MIN) {
    pwm = MOTOR_PWM_MIN;
  }

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

  motorENA.write(pwm / 255.0f);
  last_signed_pwm = result;
}

void applySteering() {
  if (abs(cmd_steering) > 0.001f) {
    // Manual steering overrides heading lock
    heading_locked = false;
    float ratio = constrain(-cmd_steering / STEERING_MAX, -1.0f, 1.0f);
    int angle = SERVO_CENTER - (int)(ratio * SERVO_RANGE);
    angle = constrain(angle, SERVO_CENTER - SERVO_RANGE, SERVO_CENTER + SERVO_RANGE);
    steeringServo.write(angle);
  } else {
    // Autonomous heading lock using PD controller
    if (!heading_locked) {
      target_heading_rad = heading_rad;
      heading_locked = true;
    }
    
    float error_deg = (target_heading_rad - heading_rad) * (180.0f / PI);
    float correction = STEERING_KP * error_deg - STEERING_KD * current_gz_deg_s;
    
    int angle = SERVO_CENTER - (int)correction;
    angle = constrain(angle, SERVO_CENTER - SERVO_RANGE, SERVO_CENTER + SERVO_RANGE);
    steeringServo.write(angle);
  }
}

void publishTelemetry() {
  long ticks_now;
  noInterrupts();
  ticks_now = encoder_ticks;
  interrupts();

  Serial.print(F("VEL:"));
  Serial.print(filtered_speed_mps, 3);
  Serial.print(F(",THETA:"));
  Serial.print(heading_rad * 180.0f / PI, 2);
  Serial.print(F(",TICKS:"));
  Serial.println(ticks_now);
}

void initializeImu() {
  // Wire defaults to SDA=GP4, SCL=GP5 in the Mbed core
  Wire.begin();
  Wire.setClock(400000);

  writeByte(REG_PWR_MGMT1, 0x00);
  writeByte(REG_ACCEL_CFG, 0x00);
  writeByte(REG_GYRO_CFG, 0x00);
  delay(100);

  calibrateImu();
}

void calibrateImu() {
  float sax = 0.0f;
  float say = 0.0f;
  float saz = 0.0f;
  float sgx = 0.0f;
  float sgy = 0.0f;
  float sgz = 0.0f;
  float ax, ay, az, gx, gy, gz;

  Serial.println(F("Calibrating MPU-6050. Keep the car still..."));
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    readAll(ax, ay, az, gx, gy, gz);
    sax += ax;
    say += ay;
    saz += az;
    sgx += gx;
    sgy += gy;
    sgz += gz;
    delay(5);
  }

  gx_off = sgx / CALIB_SAMPLES;
  gy_off = sgy / CALIB_SAMPLES;
  gz_off = sgz / CALIB_SAMPLES;

  (void)sax;
  (void)say;
  (void)saz;
  heading_rad = 0.0f;
  Serial.println(F("MPU-6050 calibration complete"));
}

void updateHeading(float dt_s) {
  float ax, ay, az, gx, gy, gz;
  readAll(ax, ay, az, gx, gy, gz);

  gx -= gx_off;
  gy -= gy_off;
  gz -= gz_off;

  current_gz_deg_s = gz;
  heading_rad += gz * (PI / 180.0f) * dt_s;

  (void)ax;
  (void)ay;
  (void)az;
  (void)gx;
  (void)gy;
}

void writeByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

void readAll(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  int16_t rawAx = Wire.read() << 8 | Wire.read();
  int16_t rawAy = Wire.read() << 8 | Wire.read();
  int16_t rawAz = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();
  int16_t rawGx = Wire.read() << 8 | Wire.read();
  int16_t rawGy = Wire.read() << 8 | Wire.read();
  int16_t rawGz = Wire.read() << 8 | Wire.read();

  ax = rawAx / ACCEL_SCALE;
  ay = rawAy / ACCEL_SCALE;
  az = rawAz / ACCEL_SCALE;
  gx = rawGx / GYRO_SCALE;
  gy = rawGy / GYRO_SCALE;
  gz = rawGz / GYRO_SCALE;
}

void encoderAInterrupt() {
  bool a = digitalRead(ENCODER_A);
  bool b = digitalRead(ENCODER_B);
  if (a == b) {
    encoder_ticks++;
  } else {
    encoder_ticks--;
  }
}
