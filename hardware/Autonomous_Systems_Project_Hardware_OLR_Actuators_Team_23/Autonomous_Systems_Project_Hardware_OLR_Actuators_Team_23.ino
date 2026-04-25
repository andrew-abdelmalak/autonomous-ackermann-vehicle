/*
 * MS2/MS3 Vehicle Controller – Team 23
 * Updated with PID Closed-Loop Control
 */

#include <Servo.h>

// ── Pin Configuration (Arduino Uno) ──
const int MOTOR_ENA = 9;   // PWM
const int MOTOR_IN1 = 8;   // Dir 1
const int MOTOR_IN2 = 7;   // Dir 2
const int SERVO_PIN = 10;  // Steering
const int ENCODER_A = 2;   // Encoder Phase A (Interrupt Pin)
const int ENCODER_B = 3;   // Encoder Phase B (Direction Sensing)

// ── PID Tuning Parameters ──
// These values usually require manual tuning (Ziegler-Nichols or trial/error)
float Kp = 150.0; 
float Ki = 80.0;
float Kd = 2.0;

// ── Physical Constants ──
const int   MOTOR_PWM_MIN   = 80;    // Lowered slightly as PID helps overcome friction
const int   MOTOR_PWM_MAX   = 255;
const float SPEED_MAX       = 0.5;   // m/s
const int   PULSES_PER_REV  = 440;   // Update based on JGA25-370 gear ratio (e.g., 11ppr * 40:1)
const float WHEEL_DIAMETER  = 0.065; // meters (example: 65mm wheel)
const float PI_VAL          = 3.14159;

// ── Steering Constants ──
const int   SERVO_CENTER    = 90;
const int   SERVO_RANGE     = 35;
const float STEERING_MAX    = 0.5;

// ── Variables ──
volatile long encoderTicks = 0;
float actual_speed = 0.0;
float cmd_speed    = 0.0;
float cmd_steering = 0.0;
float error_integral = 0.0;
float last_error = 0.0;

unsigned long last_cmd_time = 0;
unsigned long last_pid_time = 0;
const int PID_INTERVAL_MS = 50; // 20Hz PID loop

Servo steeringServo;
char buf[64];
int  buf_idx = 0;

// ── Encoder ISR ──
void handleEncoder() {
  // Check Phase B to determine direction
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  
  // Setup Encoders
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoder, RISING);

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);

  last_cmd_time = millis();
  last_pid_time = millis();

  Serial.println("MS2 PID Controller ready");
}

void loop() {
  // 1. Read Serial Commands from Pi
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

  // 2. Safety Timeout
  if (millis() - last_cmd_time > 500) {
    cmd_speed = 0.0;
    error_integral = 0; // Reset integral on stop
  }

  // 3. PID Control Loop (Timed)
  unsigned long now = millis();
  if (now - last_pid_time >= PID_INTERVAL_MS) {
    float dt = (now - last_pid_time) / 1000.0;
    
    // Calculate actual velocity (m/s)
    // Speed = (Ticks / Pulses_per_Rev) * (Wheel_Circumference) / dt
    actual_speed = ((float)encoderTicks / PULSES_PER_REV) * (WHEEL_DIAMETER * PI_VAL) / dt;
    encoderTicks = 0; // Reset for next interval

    computePID(dt);
    applySteering();
    
    last_pid_time = now;
  }
}

void parseCommand(const char* line) {
  const char* spd_ptr = strstr(line, "SPD:");
  const char* str_ptr = strstr(line, "STR:");
  if (spd_ptr) cmd_speed = atof(spd_ptr + 4);
  if (str_ptr) cmd_steering = atof(str_ptr + 4);
}

void computePID(float dt) {
  // Error calculation
  float error = cmd_speed - actual_speed;
  
  // Proportional term
  float P_out = Kp * error;
  
  // Integral term (with anti-windup clamping)
  error_integral += error * dt;
  error_integral = constrain(error_integral, -50, 50); 
  float I_out = Ki * error_integral;
  
  // Derivative term
  float D_out = Kd * (error - last_error) / dt;
  last_error = error;

  // Total Output
  float total_output = P_out + I_out + D_out;
  
  // Map PID output to PWM and Direction
  int pwm = (int)constrain(abs(total_output), 0, MOTOR_PWM_MAX);
  
  // Direction Logic
  if (cmd_speed > 0.01) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
  } else if (cmd_speed < -0.01) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    pwm = 0;
    error_integral = 0; 
  }

  // Minimum PWM to overcome initial friction
  if (pwm > 0 && pwm < MOTOR_PWM_MIN) pwm = MOTOR_PWM_MIN;
  
  analogWrite(MOTOR_ENA, pwm);
}

void applySteering() {
  float ratio = constrain(-cmd_steering / STEERING_MAX, -1.0, 1.0);
  int angle = SERVO_CENTER + (int)(ratio * SERVO_RANGE);
  steeringServo.write(constrain(angle, SERVO_CENTER - SERVO_RANGE, SERVO_CENTER + SERVO_RANGE));
}