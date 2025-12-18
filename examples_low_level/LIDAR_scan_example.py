# Импорт основных модулей для работы с API симулятора
from agrotechsimapi import SimClient
import time
import numpy as np
import matplotlib.pyplot as plt

# Флаг для управления основным циклом программы
is_loop = True

# Включение интерактивного режима matplotlib для обновления графиков в реальном времени
plt.ion() 

def plot_lidar_data(distances):
    """Функция для визуализации данных лидара"""
    
    # Создание массива углов от -π до π с количеством точек равным длине массива дистанций
    angles = np.linspace(-np.pi, np.pi, num=len(distances), endpoint=False)
    
    # Преобразование полярных координат (угол, дистанция) в декартовы (x, y)
    x = distances * -np.cos(angles + np.pi/2)
    y = distances * np.sin(angles + np.pi/2)
    
    # Очистка предыдущего графика
    plt.clf()  
    
    # Отображение точек лидарного сканирования
    plt.scatter(x, y, s=5)  
    
    # Установка пределов осей (-12 до 12 метров)
    plt.ylim(-12, 12)  
    plt.xlim(-12, 12) 
    
    # Настройка заголовка и подписей осей
    plt.title("Lidar Scan Data")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True)
    
    # Обновление графика с небольшой паузой
    plt.pause(0.1) 

def main():
    """Основная функция программы"""
    
    # Создание клиента для подключения к симулятору
    # Подключение к локальному серверу симулятора на порту 8080
    client = SimClient(address="127.0.0.1", port=8080)

    # Создание фигуры matplotlib для отображения данных
    plt.figure()
    
    # Основной цикл получения и отображения данных лидара
    while is_loop:
        # Получение данных лазерного сканирования
        # angle_min, angle_max - диапазон углов сканирования (от -π до π)
        # range_max - максимальная дальность сканирования (10 метров)
        # num_ranges - количество лучей лидара (360 для полного круга)
        # range_error - погрешность измерения дальности (0.1)
        # is_clear - очистка предыдущих данных
        result = client.get_laser_scan(angle_min=-np.pi, angle_max=np.pi, range_max=10, num_ranges=360, range_error=0.1, is_clear=True)
        
        # Отображение полученных данных на графике
        plot_lidar_data(result)

        # Задержка между сканированиями (15 Hz)
        time.sleep(1/15)
    

# Запуск основной функции
main()