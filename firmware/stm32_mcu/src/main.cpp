#include <Arduino.h>

// Пины для драйвера моторов
#define MOTOR_LEFT_ENABLE    PA0
#define MOTOR_LEFT_IN1       PA1  
#define MOTOR_LEFT_IN2       PA2
#define MOTOR_RIGHT_ENABLE   PA3
#define MOTOR_RIGHT_IN1      PA4
#define MOTOR_RIGHT_IN2      PA5

char command_buffer[32];
uint8_t buffer_index = 0;

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

void set_motor_speeds(int left_speed, int right_speed) {
  // Левый мотор
  if (left_speed > 0) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  } else if (left_speed < 0) {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    left_speed = -left_speed;
  } else {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  }
  analogWrite(MOTOR_LEFT_ENABLE, left_speed);
  
  // Правый мотор
  if (right_speed > 0) {
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  } else if (right_speed < 0) {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    right_speed = -right_speed;
  } else {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  }
  analogWrite(MOTOR_RIGHT_ENABLE, right_speed);
}

void process_command(const char* command) {
  if (strncmp(command, "MOTO:", 5) == 0) {
    int left, right;
    if (sscanf(command + 5, "%d,%d", &left, &right) == 2) {
      set_motor_speeds(left, right);
    }
  } 
  else if (strcmp(command, "STOP") == 0) {
    set_motor_speeds(0, 0);
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
  set_motor_speeds(250, 250);
  delay(1000);
  set_motor_speeds(250, 0);
  delay(1000);
}