"""
Полный тест системы управления от позиции до PWM.
Проверяет всю цепочку: позиция → скорость → PWM
"""
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from agrotechsimapi.pid import PID
from agrotechsimapi.utils.utils import vel_to_rc_signal


def test_full_control_chain(yaw_deg, target_rel_x, target_rel_y, current_x=0.0, current_y=0.0, current_vx=0.0, current_vy=0.0):
    """
    Полный тест цепочки управления.
    
    Args:
        yaw_deg: Угол поворота дрона (по часовой, в градусах)
        target_rel_x: Целевая точка относительно дрона (X дрона)
        target_rel_y: Целевая точка относительно дрона (Y дрона)
        current_x: Текущая позиция дрона в мировой СК (X мира)
        current_y: Текущая позиция дрона в мировой СК (Y мира)
        current_vx: Текущая скорость дрона в мировой СК (X мира)
        current_vy: Текущая скорость дрона в мировой СК (Y мира)
    """
    yaw = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    
    print(f"\n{'='*60}")
    print(f"ТЕСТ: yaw={yaw_deg}°, цель отн. дрона=({target_rel_x}, {target_rel_y})")
    print(f"{'='*60}")
    
    # ШАГ 1: Преобразование из СК дрона в мировую СК (go_to_xy base_link)
    print(f"\n[ШАГ 1] Преобразование base_link → odom:")
    target_x_world = current_x + cos_yaw * target_rel_x - sin_yaw * target_rel_y
    target_y_world = current_y + sin_yaw * target_rel_x + cos_yaw * target_rel_y
    print(f"  Цель в мировой СК: ({target_x_world:.3f}, {target_y_world:.3f})")
    
    # ШАГ 2: position_callback - расчет ошибки позиции в мировой СК
    print(f"\n[ШАГ 2] position_callback (ошибка позиции → скорость в мировой СК):")
    pos_error_x = target_x_world - current_x
    pos_error_y = target_y_world - current_y
    print(f"  Ошибка позиции (мир): ({pos_error_x:.3f}, {pos_error_y:.3f})")
    
    # PID позиции (упрощенно, только kp)
    kp_pos = 0.33
    max_velocity = 0.33
    tvx_world = max(-max_velocity, min(max_velocity, kp_pos * pos_error_x))
    tvy_world = max(-max_velocity, min(max_velocity, kp_pos * pos_error_y))
    print(f"  Целевая скорость (мир): ({tvx_world:.3f}, {tvy_world:.3f})")
    
    # ШАГ 3: velocity_callback - преобразование в СК дрона
    print(f"\n[ШАГ 3] velocity_callback (преобразование в СК дрона):")
    
    # Целевая скорость в СК дрона
    tvx_body = tvx_world * cos_yaw + tvy_world * sin_yaw
    tvy_body = -tvx_world * sin_yaw + tvy_world * cos_yaw
    print(f"  Целевая скорость (дрон): ({tvx_body:.3f}, {tvy_body:.3f})")
    
    # Текущая скорость в СК дрона
    vx_body = current_vx * cos_yaw + current_vy * sin_yaw
    vy_body = -current_vx * sin_yaw + current_vy * cos_yaw
    print(f"  Текущая скорость (дрон): ({vx_body:.3f}, {vy_body:.3f})")
    
    # Ошибка скорости в СК дрона
    vel_error_x = tvx_body - vx_body
    vel_error_y = tvy_body - vy_body
    print(f"  Ошибка скорости (дрон): ({vel_error_x:.3f}, {vel_error_y:.3f})")
    
    # ШАГ 4: PID скорости → PWM
    print(f"\n[ШАГ 4] PID скорости → PWM:")
    kp_vel = 5.0
    kd_vel = 12.0
    
    # Упрощенно: derivative = error (первый вызов)
    pid_vel_x_output = kp_vel * vel_error_x + kd_vel * vel_error_x
    pid_vel_y_output = kp_vel * vel_error_y + kd_vel * vel_error_y
    
    # processing_func (сигмоида)
    def sigmoid(x):
        return ((2/(1+(2.7**(-x * 6)))) - 1) * 1.65
    
    pid_x_final = sigmoid(pid_vel_x_output)
    pid_y_final = sigmoid(pid_vel_y_output)
    
    print(f"  PID X (до sigmoid): {pid_vel_x_output:.3f}")
    print(f"  PID Y (до sigmoid): {pid_vel_y_output:.3f}")
    print(f"  PID X (после sigmoid): {pid_x_final:.3f}")
    print(f"  PID Y (после sigmoid): {pid_y_final:.3f}")
    
    # ШАГ 5: Преобразование в RC сигнал
    print(f"\n[ШАГ 5] RC сигнал:")
    pitch_pwm = int(vel_to_rc_signal(pid_x_final))
    roll_pwm = int(vel_to_rc_signal(pid_y_final))
    
    print(f"  pitch_pwm: {pitch_pwm} (1500=нейтраль, >1500=вперед)")
    print(f"  roll_pwm: {roll_pwm} (1500=нейтраль, >1500=вправо)")
    
    # ШАГ 6: Проверка корректности
    print(f"\n[ШАГ 6] Проверка корректности:")
    
    # Определяем ожидаемое направление
    if target_rel_x > 0:
        expected_pitch = "вперед (>1500)"
        pitch_ok = pitch_pwm > 1500
    elif target_rel_x < 0:
        expected_pitch = "назад (<1500)"
        pitch_ok = pitch_pwm < 1500
    else:
        expected_pitch = "нейтраль (1500)"
        pitch_ok = abs(pitch_pwm - 1500) < 50
    
    if target_rel_y > 0:
        expected_roll = "вправо (>1500)"
        roll_ok = roll_pwm > 1500
    elif target_rel_y < 0:
        expected_roll = "влево (<1500)"
        roll_ok = roll_pwm < 1500
    else:
        expected_roll = "нейтраль (1500)"
        roll_ok = abs(roll_pwm - 1500) < 50
    
    print(f"  Ожидается pitch: {expected_pitch} → {pitch_pwm} {'✅' if pitch_ok else '❌'}")
    print(f"  Ожидается roll: {expected_roll} → {roll_pwm} {'✅' if roll_ok else '❌'}")
    
    return pitch_ok and roll_ok


if __name__ == '__main__':
    print("\n" + "="*60)
    print("ТЕСТИРОВАНИЕ ПОЛНОЙ ЦЕПОЧКИ УПРАВЛЕНИЯ")
    print("="*60)
    
    results = []
    
    # Тест 1: yaw=0°, цель (1, -1) → вперед и вправо
    results.append(("yaw=0°, цель (1, -1)", test_full_control_chain(
        yaw_deg=0, target_rel_x=1.0, target_rel_y=-1.0
    )))
    
    # Тест 2: yaw=90°, цель (1, -1) → вперед и влево
    results.append(("yaw=90°, цель (1, -1)", test_full_control_chain(
        yaw_deg=90, target_rel_x=1.0, target_rel_y=-1.0
    )))
    
    # Тест 3: yaw=45°, цель (1, -1) → вперед
    results.append(("yaw=45°, цель (1, -1)", test_full_control_chain(
        yaw_deg=45, target_rel_x=1.0, target_rel_y=-1.0
    )))
    
    # Тест 4: yaw=-90°, цель (1, -1) → назад и вправо
    results.append(("yaw=-90°, цель (1, -1)", test_full_control_chain(
        yaw_deg=-90, target_rel_x=1.0, target_rel_y=-1.0
    )))
    
    # Итоговый отчет
    print(f"\n{'='*60}")
    print("ИТОГОВЫЙ ОТЧЕТ")
    print(f"{'='*60}")
    
    all_passed = True
    for name, passed in results:
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"  {name}: {status}")
        if not passed:
            all_passed = False
    
    print(f"\n{'='*60}")
    if all_passed:
        print("ВСЕ ТЕСТЫ ПРОЙДЕНЫ ✅")
    else:
        print("ЕСТЬ ОШИБКИ ❌")
    print(f"{'='*60}\n")
