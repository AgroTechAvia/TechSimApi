# Библиотеки для работы со временем
import time
from datetime import datetime


# Импортируем класс PID
from agrotechsimapi import PID
from agrotechsimapi import HighLevelSimClient
import cv2

# Задаем IP и порт для подключения к дрону
ip = "127.0.0.1"
port = "1233"

# Переменная для хранения времени последнего сообщения
last_time = None

def find_marker(speed_roll):
    while True:
        error = client.getArucos()
        image = client.getArucosImage()
        cv2.imshow("blob", image)
        cv2.waitKey(1)
        client.setVelXYYaw(0, speed_roll, 0)
        if error:
            client.setDiod(0, 255, 0)
            print('маркер найден')
            client.setVelXYYaw(0, 0, 0)
            time.sleep(1)
            break

# Функция посадки дрона на аруко-маркер
def Aruco_boarding():
    # Время начала отсчёта
    time_begin = datetime.now()
    # Инициализируем переменную для текущего времени
    time_now = time_begin
    # Находим их разность
    delta_time = time_now - time_begin
    # Инициализируем переменную для текущей высоты
    current_height = client.getHeightRange()
    # Выводим текущую высоту
    print(f"Started Height is: {current_height}")

    # Инициализация PID-контроллеров для крена и тангажа
    pid_pitch = PID(0.4, 0.0008, 0.15)
    pid_roll = PID(0.4, 0.0008, 0.15)
    # Инициализируем переменные ошибок
    pitch_error = 0.0
    roll_error = 0.0
    
    # Устанавливаем время выполнения, по истечению которого дрон совершит посадку 
    while delta_time.total_seconds() < 200:
        if delta_time.total_seconds == 200:
            print('время')
        try:
            # Обновляем текущее время и рассчитываем время выполнения
            time_now = datetime.now()
            delta_time = time_now - time_begin
            # С помощью сервиса getArucos получаем координаты маркера относительно дрона - это и есть ошибки
            errors = client.getArucos()
            
            # Если камера не видит аруко-маркеров
            if errors == []:  # Потерян маркер
                print('I dont see Aruco')
                # Вызываем функцию регулировки
                ArucoRegulation(pitch_error, roll_error, pid_pitch, pid_roll)
                # Продолжаем выполнения цикла
                continue
            if errors:
                image = client.getArucosImage()
                cv2.imshow("blob", image)
                cv2.waitKey(1)
                # Если камера видит аруко-маркеры, записываем в переменные ошибки
                roll_error = errors[0]['pose']['position']['x']
                pitch_error = errors[0]['pose']['position']['y'] - 0.09
                # Вызываем функцию регулировки
                if current_height < 0.4:
                    pid_pitch = PID(0.4, 0.00085, 0.2)
                    pid_roll = PID(0.4, 0.00085, 0.2)

                ArucoRegulation(pitch_error, roll_error, pid_pitch, pid_roll)
                # Если ошибки меньше 20 по x и y - снижаем высоту 
                if abs(pitch_error) <= 0.1 and abs(roll_error) <= 0.1:
                    current_height *= 0.75
                    print("Снижение")
                    # Посылаем в сервис setHeight новую высоту
                    client.setVelXYYaw(0, 0, 0)
                    time.sleep(0.5)
                    client.setHeight(current_height)
                    print(f"Current Height is: {current_height}")
                if current_height < 0.3:
                    # Останавливаем дрон и завершаем посадку
                    client.setVelXYYaw(0, 0, 0)
                    time.sleep(0.5)
                    print("Landing")
                    client.boarding()
                    break
        except KeyboardInterrupt:
            # Обрабатываем прерывание программы с клавиатуры
            print("KeyboardInterrupt detected, landing the drone...")
            break

# Функция регулировки положения дрона относительно аруко-маркера
def ArucoRegulation(pitch_error, roll_error, pid_pitch: PID, pid_roll: PID):
    print(f"Adjusting position: pitch={pitch_error}, roll={roll_error}")

    # Обновление контроллеров по осям
    pid_pitch.update_control(pitch_error)
    pid_roll.update_control(roll_error)

    PID_PITCH = -pid_pitch.get_control()
    PID_ROLL = pid_roll.get_control()  

    # Ограничение управляющих сигналов
    PID_PITCH = constrain(PID_PITCH, 0.3)
    PID_ROLL = constrain(PID_ROLL, 0.3)

    print(f"PID Control: PITCH={PID_PITCH}, ROLL={PID_ROLL}")

    # Устанавливаем скорости для дрона
    client.setVelXYYaw(PID_PITCH, PID_ROLL, 0)
    return (PID_PITCH, PID_ROLL)

# Функция для ограничения значений
def constrain(value, threshold):
    if value > threshold:
        value = threshold
    if value < -threshold:
        value = -threshold
    return value

# Функция обработки входящих сообщений


# Создаем объект клиента для взаимодействия с дроном
client = HighLevelSimClient()

# Подключаемся к дрону и выполняем команды
print("connected?", client.connect(ip, port), "\n")
print("VelCorrect", client.setVelXY(0, 0), "\n")
print("takeoff?", client.takeoff(), "\n")
time.sleep(7)
print("Aruco Boarding: ", find_marker(0.25), "\n")
print("Aruco Boarding: ", Aruco_boarding(), "\n")
print("boarding?", client.boarding(), "\n")
print("disconnected?", client.disconnect(), "\n")