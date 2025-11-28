#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import serial
import time

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        # Подключение к STM32
        try:
            self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info("✅ Connected to STM32")
        except Exception as e:
            self.get_logger().error(f"❌ STM32 connection failed: {e}")
            return
        
        # Настройка камеры
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Параметры для желтой линии
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([30, 255, 255])
        
        # PID параметры
        self.last_error = 0
        self.kp = 0.8
        self.kd = 0.3
        
        # Таймер для обработки кадров
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 FPS
        
        self.get_logger().info("🚀 Line Follower Started!")
    
    def detect_yellow_line(self, frame):
        """Обнаружение желтой линии"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # Улучшение маски
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask
    
    def find_line_center(self, mask):
        """Нахождение центра линии"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Берем самый большой контур
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Находим центр масс
        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None
            
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        
        return (cx, cy)
    
    def send_motor_command(self, left_speed, right_speed):
        """Отправка команды моторам на STM32"""
        try:
            command = f"MOTO:{left_speed},{right_speed}\n"
            self.serial.write(command.encode())
        except Exception as e:
            self.get_logger().error(f"❌ Motor command failed: {e}")
    
    def process_frame(self):
        """Обработка одного кадра"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("❌ Cannot read from camera")
            return
        
        # Обнаружение линии
        mask = self.detect_yellow_line(frame)
        line_center = self.find_line_center(mask)
        
        # Если линия не найдена
        if line_center is None:
            self.get_logger().warn("⚠️ Line lost - searching...")
            self.send_motor_command(0, 100)  # Поворот на месте
            return
        
        # ПИД регулятор
        frame_center_x = 320  # Центр кадра
        error = frame_center_x - line_center[0]
        
        # Пропорционально-дифференциальный регулятор
        angular_correction = self.kp * error + self.kd * (error - self.last_error)
        self.last_error = error
        
        # Преобразование в скорости моторов
        base_speed = 150
        left_speed = int(base_speed - angular_correction)
        right_speed = int(base_speed + angular_correction)
        
        # Ограничение скоростей
        left_speed = max(min(left_speed, 255), -255)
        right_speed = max(min(right_speed, 255), -255)
        
        # Отправка команд на STM32
        self.send_motor_command(left_speed, right_speed)
        
        # Логирование
        self.get_logger().info(f"🎯 Line at: {line_center[0]}, Motors: L={left_speed}, R={right_speed}")
        
        # Отладочная визуализация (опционально)
        self.debug_display(frame, mask, line_center, left_speed, right_speed)
    
    def debug_display(self, frame, mask, line_center, left_speed, right_speed):
        """Визуализация для отладки"""
        if line_center:
            cv2.circle(frame, line_center, 10, (0, 255, 0), -1)
            cv2.line(frame, (320, 480), line_center, (255, 0, 0), 2)
        
        # Добавляем информацию о скоростях
        cv2.putText(frame, f"L: {left_speed} R: {right_speed}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('Line Following', frame)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)
    
    def stop_motors(self):
        """Остановка моторов при завершении"""
        try:
            self.serial.write(b"STOP\n")
            self.get_logger().info("🛑 Motors stopped")
        except:
            pass

def main():
    rclpy.init()
    node = LineFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Line follower stopped by user")
    finally:
        node.stop_motors()
        if hasattr(node, 'cap'):
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()