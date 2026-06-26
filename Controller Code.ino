#include <esp_now.h>
#include <WiFi.h>

#define CMD_STOP        1
#define CMD_RUN         2
#define CMD_SETPOINT    3
#define CMD_MAXSPEED    4
#define CMD_SET_DEADZONE 5

typedef struct {
  uint8_t cmd;
  float value;
} CommandMsg;

typedef struct {
  float kp;
  float ki;
  float kd;
} PIDvalues;

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

uint8_t robotMac[] = {0xB4, 0xBF, 0xE9, 0x09, 0xF0, 0x60}; 

bool sendSuccess = false;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(RobotStatus)) {
    RobotStatus status;
    memcpy(&status, incomingData, sizeof(status));

    Serial.print("Angle=");
    Serial.print(status.angle, 6);
    Serial.print(" | Err=");
    Serial.print(status.error, 6);
    Serial.print(" | out=");
    Serial.print(status.output, 6);
    Serial.print(" | Speed=");
    Serial.print(status.speed, 0);
    Serial.print(" | Kp=");
    Serial.print(status.kp, 6);
    Serial.print(" Ki=");
    Serial.print(status.ki, 6);
    Serial.print(" Kd=");
    Serial.print(status.kd, 6);
    Serial.print(" | SP=");
    Serial.print(status.setpoint, 6);
    Serial.print(" | DZ=");
    Serial.print(status.deadZone, 6);
    Serial.print(" | I=");
    Serial.print(status.integral, 6);
    Serial.print(" | D=");
    Serial.print(status.derivative, 6);
    Serial.print(" | dt=");
    Serial.print(status.dt, 6);
    Serial.print(" | En:");
    Serial.print(status.enabled ? "1" : "0");
    Serial.print(" | Fall:");
    Serial.println(status.fallen ? "1" : "0");
  }
}

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  sendSuccess = (status == ESP_NOW_SEND_SUCCESS);
}

void sendCommand(uint8_t cmd, float value) {
  CommandMsg msg;
  msg.cmd = cmd;
  msg.value = value;
  esp_err_t result = esp_now_send(robotMac, (uint8_t*)&msg, sizeof(msg));
  if (result == ESP_OK) Serial.println("Command sent");
  else Serial.println("Error sending command");
}

void sendPID(float kp, float ki, float kd) {
  PIDvalues pid;
  pid.kp = kp;
  pid.ki = ki;
  pid.kd = kd;
  esp_err_t result = esp_now_send(robotMac, (uint8_t*)&pid, sizeof(pid));
  if (result == ESP_OK) Serial.println("PID sent");
  else Serial.println("Error sending PID");
}

void sendDeadZone(float dz) {
  sendCommand(CMD_SET_DEADZONE, dz);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP-NOW Transmitter starting...");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, robotMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1);
  }

  Serial.println("Ready. Commands:");
  Serial.println("  stop, run");
  Serial.println("  setpoint <value>");
  Serial.println("  maxspeed <value>");
  Serial.println("  pid:<kp>,<ki>,<kd>   (e.g. pid:8,0.5,0.006)");
  Serial.println("  deadzone <value>     (e.g. deadzone 1.2)");
  Serial.println("  help");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    if (input.equalsIgnoreCase("stop")) {
      sendCommand(CMD_STOP, 0);
    }
    else if (input.equalsIgnoreCase("run")) {
      sendCommand(CMD_RUN, 0);
    }
    else if (input.equalsIgnoreCase("help")) {
      Serial.println("Commands: stop, run, setpoint X, maxspeed X, pid:kp,ki,kd, deadzone X");
    }

    // ---- setpoint ----
    else if (input.startsWith("setpoint")) {
      float val = input.substring(9).toFloat();
      sendCommand(CMD_SETPOINT, val);
      Serial.print("Setpoint set to: ");
      Serial.println(val, 6);
    }

    // ---- maxspeed ----
    else if (input.startsWith("maxspeed")) {
      float val = input.substring(9).toFloat();
      if (val >= 100 && val <= 8000) {
        sendCommand(CMD_MAXSPEED, val);
        Serial.print("Max speed set to: ");
        Serial.println(val, 0);
      } else {
        Serial.println("Max speed must be between 100 and 8000");
      }
    }

    // ---- deadzone ----
    else if (input.startsWith("deadzone")) {
      float val = input.substring(9).toFloat();
      if (val >= 0.0 && val <= 10.0) {
        sendDeadZone(val);
        Serial.print("DeadZone set to: ");
        Serial.println(val, 6);
      } else {
        Serial.println("DeadZone must be between 0 and 10");
      }
    }

    else if (input.startsWith("pid")) {
      String rest = input.substring(3);   
      rest.trim();

      
      if (rest.startsWith(":")) {
        rest = rest.substring(1);
        rest.trim();
      }

      int firstComma = rest.indexOf(',');
      int secondComma = rest.indexOf(',', firstComma + 1);
      if (firstComma != -1 && secondComma != -1) {
        String kpStr = rest.substring(0, firstComma);
        String kiStr = rest.substring(firstComma + 1, secondComma);
        String kdStr = rest.substring(secondComma + 1);

        kpStr.trim(); kiStr.trim(); kdStr.trim(); 

        float kp = kpStr.toFloat();
        float ki = kiStr.toFloat();
        float kd = kdStr.toFloat();

        sendPID(kp, ki, kd);
        Serial.print("PID updated: Kp=");
        Serial.print(kp, 6);
        Serial.print(", Ki=");
        Serial.print(ki, 6);
        Serial.print(", Kd=");
        Serial.println(kd, 6);
      } else {
        Serial.println("Invalid format. Use: pid:<kp>,<ki>,<kd>   (e.g. pid:8,0.5,0.006)");
      }
    }

    else {
      Serial.println("Unknown command. Type 'help'");
    }
  }
}
double Ki = 0.15;
double Kd = 0.06;
