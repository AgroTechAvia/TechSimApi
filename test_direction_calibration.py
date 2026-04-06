"""
Скрипт для тестирования и настройки коэффициентов направления дрона.

Запустите этот скрипт, чтобы определить правильные значения для:
- _roll_direction (направление влево/вправо)
- _pitch_direction (направление вперед/назад)
- _yaw_direction (направление по/против часовой стрелки)

Инструкция по использованию:
1. Запустите симулятор и подключитесь
2. Запустите этот скрипт
3. Следуйте инструкциям в консоли
4. Запоминайте результаты и используйте их в основной программе
"""

import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_direction_coefficients(client):
    """
    Пошаговый тест для определения правильных коэффициентов направления.
    
    Args:
        client: Экземпляр HighLevelSimClient
    """
    print("\n" + "="*60)
    print("ТЕСТ КОЭФФИЦИЕНТОВ НАПРАВЛЕНИЯ")
    print("="*60)
    
    # Взлетаем
    print("\n1. Взлетаем...")
    client.takeoff()
    time.sleep(2)
    
    # Текущие коэффициенты
    coeffs = client.get_direction_coefficients()
    print(f"\nТекущие коэффициенты: {coeffs}")
    
    # ===== ТЕСТ PITCH (вперед/назад) =====
    print("\n" + "-"*60)
    print("ТЕСТ PITCH (движение вперед/назад)")
    print("-"*60)
    print("Сейчас дрон попытается лететь ВПЕРЕД в течение 2 секунд")
    print("Наблюдайте за направлением движения")
    input("Нажмите Enter для начала теста...")
    
    client.set_velocity_xy(0.3, 0.0, frame="base_link")
    time.sleep(2)
    client.set_velocity_xy(0.0, 0.0)
    time.sleep(1)
    
    direction = input("\nДрон двигался ВПЕРЕД? (д/н): ").strip().lower()
    if direction == "н":
        print("→ Инвертируем pitch_direction")
        client.invert_pitch()
        print(f"✓ pitch_direction = {client._pitch_direction}")
    else:
        print("✓ pitch_direction корректен")
    
    time.sleep(1)
    
    # ===== ТЕСТ ROLL (влево/вправо) =====
    print("\n" + "-"*60)
    print("ТЕСТ ROLL (движение влево/вправо)")
    print("-"*60)
    print("Сейчас дрон попытается лететь ВПРАВО в течение 2 секунд")
    print("Наблюдайте за направлением движения")
    input("Нажмите Enter для начала теста...")
    
    client.set_velocity_xy(0.0, 0.3, frame="base_link")
    time.sleep(2)
    client.set_velocity_xy(0.0, 0.0)
    time.sleep(1)
    
    direction = input("\nДрон двигался ВПРАВО? (д/н): ").strip().lower()
    if direction == "н":
        print("→ Инвертируем roll_direction")
        client.invert_roll()
        print(f"✓ roll_direction = {client._roll_direction}")
    else:
        print("✓ roll_direction корректен")
    
    time.sleep(1)
    
    # ===== ТЕСТ YAW (вращение) =====
    print("\n" + "-"*60)
    print("ТЕСТ YAW (вращение по часовой стрелке)")
    print("-"*60)
    print("Сейчас дрон попытается повернуться ПО ЧАСОВОЙ СТРЕЛКЕ")
    print("Наблюдайте за направлением вращения")
    input("Нажмите Enter для начала теста...")
    
    current_yaw = client._get_yaw_cw()
    target_yaw = current_yaw + 1.57  # +90 градусов
    client.setYaw(target_yaw)
    time.sleep(1)
    
    direction = input("\nДрон вращался ПО ЧАСОВОЙ СТРЕЛКЕ? (д/н): ").strip().lower()
    if direction == "н":
        print("→ Инвертируем yaw_direction")
        client.invert_yaw()
        print(f"✓ yaw_direction = {client._yaw_direction}")
    else:
        print("✓ yaw_direction корректен")
    
    # Возвращаем yaw в исходное положение
    client.setYaw(current_yaw)
    time.sleep(1)
    
    # ===== ИТОГОВЫЕ РЕЗУЛЬТАТЫ =====
    print("\n" + "="*60)
    print("ИТОГОВЫЕ КОЭФФИЦИЕНТЫ")
    print("="*60)
    coeffs = client.get_direction_coefficients()
    print(f"\nroll:   {coeffs['roll']}")
    print(f"pitch:  {coeffs['pitch']}")
    print(f"yaw:    {coeffs['yaw']}")
    
    print("\n" + "="*60)
    print("Скопируйте эти значения в свой код:")
    print("="*60)
    print(f"client.set_direction_coefficients(")
    print(f"    roll={coeffs['roll']},")
    print(f"    pitch={coeffs['pitch']},")
    print(f"    yaw={coeffs['yaw']}")
    print(f")")
    print("="*60)
    
    # Посадка
    print("\nЗавершаем тест...")
    client.boarding()
    print("✓ Тест завершен!")


def quick_calibration(client):
    """
    Быстрая калибровка с автоматическим определением.
    Выполняет короткие импульсы и ожидает обратную связь от пользователя.
    """
    print("\n" + "="*60)
    print("БЫСТРАЯ КАЛИБРОВКА")
    print("="*60)
    
    print("\n1. Взлетаем...")
    client.takeoff()
    time.sleep(2)
    
    # Сбрасываем все в +1
    client.set_direction_coefficients(roll=1, pitch=1, yaw=1)
    
    # Тест pitch
    print("\n--- Калибровка PITCH ---")
    client.set_velocity_xy(0.2, 0.0, frame="base_link")
    time.sleep(0.5)
    client.set_velocity_xy(0.0, 0.0)
    time.sleep(1)
    
    resp = input("Дрон двигался ВПЕРЕД? (д/н): ").lower()
    if resp != "д":
        client._pitch_direction = -1
        print("  → Установлено pitch = -1")
    
    # Тест roll
    print("\n--- Калибровка ROLL ---")
    client.set_velocity_xy(0.0, 0.2, frame="base_link")
    time.sleep(0.5)
    client.set_velocity_xy(0.0, 0.0)
    time.sleep(1)
    
    resp = input("Дрон двигался ВПРАВО? (д/н): ").lower()
    if resp != "д":
        client._roll_direction = -1
        print("  → Установлено roll = -1")
    
    # Тест yaw
    print("\n--- Калибровка YAW ---")
    current_yaw = client._get_yaw_cw()
    client.setYaw(current_yaw + 0.785)  # +45 градусов
    time.sleep(1)
    client.setYaw(current_yaw)
    
    resp = input("Дрон вращался ПО ЧАСОВОЙ? (д/н): ").lower()
    if resp != "д":
        client._yaw_direction = -1
        print("  → Установлено yaw = -1")
    
    # Итог
    coeffs = client.get_direction_coefficients()
    print("\n" + "="*60)
    print("РЕЗУЛЬТАТ КАЛИБРОВКИ:")
    print("="*60)
    print(f"client.set_direction_coefficients(roll={coeffs['roll']}, pitch={coeffs['pitch']}, yaw={coeffs['yaw']})")
    print("="*60)
    
    client.boarding()


if __name__ == "__main__":
    # Пример использования:
    # from agrotechsimapi.high_level_client import HighLevelSimClient
    
    # client = HighLevelSimClient()
    # client.connect("127.0.0.1", 5762)
    
    # Вариант 1: Пошаговый тест
    # test_direction_coefficients(client)
    
    # Вариант 2: Быстрая калибровка
    # quick_calibration(client)
    
    print("Подключите ваш клиент и вызовите одну из функций:")
    print("  - test_direction_coefficients(client)")
    print("  - quick_calibration(client)")
