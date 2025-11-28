#!/usr/bin/env python3
import cv2

def test_camera():
    # Открываем камеру (0 - первая камера)
    cap = cv2.VideoCapture(0)
    
    # Устанавливаем разрешение
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Проверяем, открылась ли камера
    if not cap.isOpened():
        print("Ошибка: Не могу открыть камеру!")
        return
    
    print("Камера подключена успешно!")
    print("Для выхода нажмите 'q' в окне с изображением")
    
    while True:
        # Читаем кадр с камеры
        ret, frame = cap.read()
        
        if not ret:
            print("Ошибка: Не могу получить кадр с камеры!")
            break
        
        # Показываем изображение в окне
        cv2.imshow('Raspberry Pi Camera', frame)
        
        # Выход по нажатию 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Освобождаем ресурсы
    cap.release()
    cv2.destroyAllWindows()
    print("Камера закрыта")

if __name__ == '__main__':
    test_camera()