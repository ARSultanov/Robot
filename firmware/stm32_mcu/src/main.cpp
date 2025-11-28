#include <Arduino.h>

// Пины для драйвера моторов
#define MOTOR_LEFT_ENABLE    PA0
#define MOTOR_LEFT_IN1       PA1  // PC3  PA3
#define MOTOR_LEFT_IN2       PA2
#define MOTOR_RIGHT_ENABLE   PA3
#define MOTOR_RIGHT_IN1      PA4
#define MOTOR_RIGHT_IN2      PA5

char command_buffer[32];
uint8_t buffer_index = 0;
#define ACCELERATION_STEP 5;

void setup() {
  // Настройка пинов моторов
  pinMode(MOTOR_LEFT_ENABLE, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_ENABLE, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  
  // Остановка моторов
  analogWrite(MOTOR_LEFT_ENABLE, 0);
  analogWrite(MOTOR_RIGHT_ENABLE, 0);
  
  Serial.begin(115200);
  Serial.println("Motor Controller READY for Line Following");
}

struct MotorSpeeds
{
  int current_left;
  int current_right;
  int target_left;
  int target_right;
};

MotorSpeeds motors_speed {0,0,0,0};

void change_motor_speeds(int left_speed, int right_speed) {
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);

  motors_speed.target_left = left_speed;
  motors_speed.target_right = right_speed;

  // изменение скорости левый мотор
  if (motors_speed.current_left < motors_speed.target_left) {
    motors_speed.current_left += ACCELERATION_STEP;
    if (motors_speed.current_left > motors_speed.target_left) {
      motors_speed.current_left = motors_speed.target_left;
    }
  }
  else if (motors_speed.current_left > motors_speed.target_left) {
     motors_speed.current_left -= ACCELERATION_STEP;
    if (motors_speed.current_left < motors_speed.target_left) {
      motors_speed.current_left = motors_speed.target_left;
    }
  }

  // изменение скорости правый мотор
  if (motors_speed.current_right < motors_speed.target_right) {
    motors_speed.current_right += ACCELERATION_STEP;
    if (motors_speed.current_right > motors_speed.target_right) {
      motors_speed.current_right = motors_speed.target_right;
    }
  }
  else if (motors_speed.current_right > motors_speed.target_right) {
     motors_speed.current_right -= ACCELERATION_STEP;
    if (motors_speed.current_right < motors_speed.target_right) {
      motors_speed.current_right = motors_speed.target_right;
    }
  }
  // применение скорости к моторам
  // левый мотор
  digitalWrite(MOTOR_LEFT_IN1, motors_speed.current_left > 0 ? HIGH : LOW); 
  digitalWrite(MOTOR_LEFT_IN2, motors_speed.current_left > 0 ? LOW : HIGH);
  digitalWrite(MOTOR_LEFT_ENABLE, motors_speed.current_left);
  // правый мотор
  digitalWrite(MOTOR_LEFT_IN1, motors_speed.current_right > 0 ? HIGH : LOW); 
  digitalWrite(MOTOR_LEFT_IN2, motors_speed.current_right > 0 ? LOW : HIGH);
  digitalWrite(MOTOR_LEFT_ENABLE, motors_speed.current_right);
}

void process_command(const char* command) {
  if (strncmp(command, "MOTO:", 5) == 0) {
    int left, right;
    if (sscanf(command + 5, "%d,%d", &left, &right) == 2) {
      change_motor_speeds(left, right);
    }
  } 
  else if (strcmp(command, "STOP") == 0) {
    change_motor_speeds(0, 0);
    Serial.println("STOPPED");
  }
}

void loop() {
  // Чтение команд от Raspberry Pi
  if (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n') {
      command_buffer[buffer_index] = '\0';
      process_command(command_buffer);
      buffer_index = 0;
    } else if (buffer_index < sizeof(command_buffer) - 1) {
      command_buffer[buffer_index++] = c;
    }
  }
}