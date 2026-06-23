#include <Wire.h>
#include <Kalman.h>
#include <esp_now.h>
#include <WiFi.h>

// ======================= MOTOR PINS =======================
const int stepPin1 = 2;
const int stepPin2 = 4;
const int dirPin1  = 15;
const int dirPin2  = 0;

// ======================= PWM =======================
int stepChannel;
const int resolutionBits = 1;
int currentSpeed = 0;
double currentMaxSpeed = 4000.0;

// ======================= MPU6050 + KALMAN =======================
Kalman kalmanX;
double kalAngleX = 0;
double correctedAngle = 0;
uint32_t lastMicros = 0;
int16_t gyroBias = 0;

// ======================= PID VARIABLES =======================
double setpoint = 1;          // تم تعديلها إلى 1.2 كما في تجربتك
double Kp = 8.0;
double Ki = 0.5;                // بدأت معها
double Kd = 0.006;
double error = 0;
double prevError = 0;
double integral = 0;
double derivative = 0;
double pidOutput = 0;
double deadZone = 1.2;          // تم رفعها إلى 1.5 لتقليل الاهتزاز
const double outputMax = 255.0;
const double outputMin = -255.0;
double currentDt = 0.005;

// ======================= ROBOT STATE =======================
bool robotEnabled = true;

// ======================= FALL PROTECTION =======================
const double FALL_ANGLE = 45.0;
const double HYSTERESIS = 2.0;
bool fallen = false;

// ======================= ESP-NOW COMMANDS (استقبال) =======================
#define CMD_STOP        1
#define CMD_RUN         2
#define CMD_SETPOINT    3
#define CMD_MAXSPEED    4

typedef struct {
  float kp;
  float ki;
  float kd;
} PIDvalues;

typedef struct {
  uint8_t cmd;
  float value;
} CommandMsg;

// ======================= هيكل بيانات الإرسال (موسع بالكامل) =======================
typedef struct {
  float angle;
  float error;
  float output;
  float setpoint;
  float kp;
  float ki;
  float kd;
  float speed;
  float deadZone;
  float integral;
  float derivative;
  float dt;
  bool enabled;
  bool fallen;
} RobotStatus;

RobotStatus statusData;

// ======================= عنوان MAC لجهاز التحكم =======================
uint8_t controllerMac[] = {0x00, 0x4B, 0x12, 0x2C, 0xAE, 0x74}; // ⚠️ غيّره

// ======================= دوال =======================
void setupMPU();
void calibrateGyroBias();
bool updateAngle();
void setupMotorPWM();
void setMotorSpeed(int stepsPerSecond);
void computePID();
void updateMotorControl();
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);
void sendStatus();

void setup() {
  pinMode(stepPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  digitalWrite(stepPin1, LOW);
  digitalWrite(stepPin2, LOW);

  setupMotorPWM();
  setupMPU();
  calibrateGyroBias();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) while (1);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, controllerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) while (1);
}

void loop() {
  static unsigned long lastPIDtime = 0;
  static unsigned long lastSendTime = 0;
  unsigned long now = millis();

  if (updateAngle()) correctedAngle = kalAngleX;

  if (now - lastPIDtime >= 5) {
    lastPIDtime = now;
    computePID();
    updateMotorControl();
  }

  if (now - lastSendTime >= 50) {
    lastSendTime = now;
    sendStatus();
  }
}

// ======================= إرسال جميع البيانات =======================
void sendStatus() {
  statusData.angle      = (float)correctedAngle;
  statusData.error      = (float)error;
  statusData.output     = (float)pidOutput;
  statusData.setpoint   = (float)setpoint;
  statusData.kp         = (float)Kp;
  statusData.ki         = (float)Ki;
  statusData.kd         = (float)Kd;
  statusData.speed      = (float)currentSpeed;
  statusData.deadZone   = (float)deadZone;
  statusData.integral   = (float)integral;
  statusData.derivative = (float)derivative;
  statusData.dt         = (float)currentDt;
  statusData.enabled    = robotEnabled;
  statusData.fallen     = fallen;

  esp_now_send(controllerMac, (uint8_t*)&statusData, sizeof(statusData));
}

// ======================= ESP-NOW RECEIVER =======================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(PIDvalues)) {
    PIDvalues pid;
    memcpy(&pid, incomingData, sizeof(pid));
    Kp = pid.kp;
    Ki = pid.ki;
    Kd = pid.kd;
    integral = 0;
    prevError = 0;
    derivative = 0;
    return;
  }
  else if (len == sizeof(CommandMsg)) {
    CommandMsg cmd;
    memcpy(&cmd, incomingData, sizeof(cmd));
    switch (cmd.cmd) {
      case CMD_STOP:
        robotEnabled = false;
        pidOutput = 0;
        setMotorSpeed(0);
        break;
      case CMD_RUN:
        if (!fallen) robotEnabled = true;
        break;
      case CMD_SETPOINT:
        setpoint = cmd.value;
        prevError = 0;
        break;
      case CMD_MAXSPEED:
        if (cmd.value >= 100 && cmd.value <= 8000)
          currentMaxSpeed = cmd.value;
        break;
    }
  }
}

// ======================= PID COMPUTATION =======================
void computePID() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;
  if (dt <= 0.0) dt = 0.001;
  lastTime = now;
  currentDt = dt;

  if (!robotEnabled || fallen) {
    pidOutput = 0;
    integral = 0;
    prevError = 0;
    return;
  }

  error = setpoint - correctedAngle;

  if (fabs(error) < deadZone) {
    pidOutput = 0;
    prevError = error;
    return;
  }

  integral += Ki * error * dt;
  if (integral > outputMax) integral = outputMax;
  if (integral < outputMin) integral = outputMin;

  derivative = Kd * (error - prevError) / dt;
  prevError = error;

  pidOutput = Kp * error + integral + derivative;
  if (pidOutput > outputMax) pidOutput = outputMax;
  if (pidOutput < outputMin) pidOutput = outputMin;
}

// ======================= MOTOR CONTROL =======================
void updateMotorControl() {
  if (correctedAngle > (FALL_ANGLE + HYSTERESIS) || correctedAngle < -(FALL_ANGLE + HYSTERESIS)) {
    if (!fallen) {
      fallen = true;
      setMotorSpeed(0);
      pidOutput = 0;
      integral = 0;
      prevError = 0;
    }
    setMotorSpeed(0);
    return;
  }
  else if (correctedAngle < (FALL_ANGLE - HYSTERESIS) && correctedAngle > -(FALL_ANGLE - HYSTERESIS)) {
    if (fallen) fallen = false;
  }

  if (!robotEnabled) {
    setMotorSpeed(0);
    return;
  }

  if (pidOutput > 0) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
  } else {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
  }

  double absOut = fabs(pidOutput);
  double desiredSpeed = (absOut / outputMax) * currentMaxSpeed;
  if (desiredSpeed < 10.0) desiredSpeed = 0;
  currentSpeed = (int)desiredSpeed;
  setMotorSpeed(currentSpeed);
}

// ======================= دوال المحركات و MPU (بدون تغيير) =======================
void setMotorSpeed(int stepsPerSecond) {
  if (stepsPerSecond == 0) {
    ledcWrite(stepPin1, 0);
    ledcWrite(stepPin2, 0);
  } else {
    ledcChangeFrequency(stepPin1, stepsPerSecond, resolutionBits);
    ledcChangeFrequency(stepPin2, stepsPerSecond, resolutionBits);
    ledcWrite(stepPin1, 1);
    ledcWrite(stepPin2, 1);
  }
}

void setupMotorPWM() {
  stepChannel = ledcAttach(stepPin1, 1000, resolutionBits);
  ledcAttachChannel(stepPin2, 1000, resolutionBits, stepChannel);
  ledcWrite(stepPin1, 0);
  ledcWrite(stepPin2, 0);
}

void setupMPU() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x01);
  if (Wire.endTransmission() != 0) while (1);
  delay(100);
  uint8_t data[6];
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6);
  for (int i = 0; i < 6; i++) data[i] = Wire.read();
  int16_t accY = (int16_t)((data[2] << 8) | data[3]);
  int16_t accZ = (int16_t)((data[4] << 8) | data[5]);
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  kalmanX.setAngle(roll);
  kalmanX.setQangle(0.01);
  kalmanX.setQbias(0.003);
  kalmanX.setRmeasure(0.5);
  lastMicros = micros();
}

void calibrateGyroBias() {
  long sum = 0;
  const int samples = 1000;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 2);
    int16_t raw = Wire.read() << 8 | Wire.read();
    sum += raw;
    delay(1);
  }
  gyroBias = sum / samples;
}

bool updateAngle() {
  uint8_t i2cData[14];
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(0x68, 14) != 14) return false;
  for (int i = 0; i < 14; i++) i2cData[i] = Wire.read();
  int16_t accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  int16_t accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  int16_t gyroRaw = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  double gyroX = (gyroRaw - gyroBias) / 131.0;
  double rawRollNew = atan2(accY, accZ) * RAD_TO_DEG;
  uint32_t now = micros();
  double dt_mpu = (now - lastMicros) / 1000000.0;
  if (dt_mpu < 0.001 || dt_mpu > 0.5) dt_mpu = 0.005;
  lastMicros = now;
  kalAngleX = kalmanX.getAngle(rawRollNew, gyroX, dt_mpu);
  return true;
}