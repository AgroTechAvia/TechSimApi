# Коэффициенты направления (Direction Coefficients)

## Проблема

В симуляторе TechSim направление управления может отличаться от ожидаемого:
- При команде "вперед" дрон может лететь назад
- При команде "вправо" дрон может лететь влево
- При команде "по часовой" дрон может вращаться против часовой

Это происходит из-за особенностей реализации симулятора и преобразования координат.

## Решение

Добавлены три коэффициента направления, которые можно настраивать:

```python
self._roll_direction = 1.0    # Направление roll (влево/вправо)
self._pitchDirection = 1.0    # Направление pitch (вперед/назад)
self._yawDirection = 1.0      # Направление yaw (по/против часовой)
```

- `+1`: PWM растет при положительной команде
- `-1`: PWM падает при положительной команде (инверсия)

## Использование

### Базовая настройка

```python
from agrotechsimapi.high_level_client import HighLevelSimClient

client = HighLevelSimClient()
client.connect("127.0.0.1", 5762)

# Установить коэффициенты направления
client.set_direction_coefficients(
    roll=1.0,    # или -1.0 если летит не в ту сторону
    pitch=1.0,   # или -1.0 если летит не в ту сторону
    yaw=1.0      # или -1.0 если вращается не в ту сторону
)
```

### Быстрая инверсия

Если дрон летит не в ту сторону, можно быстро инвертировать:

```python
# Дрон летит назад вместо вперед
client.invert_pitch()

# Дрон летит влево вместо вправо
client.invert_roll()

# Дрон вращается против часовой вместо по часовой
client.invert_yaw()
```

### Проверка текущих коэффициентов

```python
coeffs = client.get_direction_coefficients()
print(coeffs)  # {'roll': 1.0, 'pitch': -1.0, 'yaw': 1.0}
```

## Калибровка

### Автоматическая калибровка

Запустите скрипт для пошаговой калибровки:

```bash
python deps/TechSimApi/test_direction_calibration.py
```

### Ручная калибровка

1. **Взлетите** и переведите дрон в режим velocity:
   ```python
   client.takeoff()
   client.set_velocity_xy(0, 0)
   ```

2. **Проверьте pitch** (вперед/назад):
   ```python
   client.set_velocity_xy(0.3, 0.0, frame="base_link")
   # Дрон должен лететь ВПЕРЕД (по носу)
   # Если летит назад:
   client.invert_pitch()
   ```

3. **Проверьте roll** (влево/вправо):
   ```python
   client.set_velocity_xy(0.0, 0.3, frame="base_link")
   # Дрон должен лететь ВПРАВО (по правому борту)
   # Если летит влево:
   client.invert_roll()
   ```

4. **Проверьте yaw** (вращение):
   ```python
   current_yaw = client._get_yaw_cw()
   client.setYaw(current_yaw + 1.57)  # +90 градусов
   # Дрон должен вращаться ПО ЧАСОВОЙ СТРЕЛКЕ
   # Если вращается против:
   client.invert_yaw()
   ```

## Как это работает

Коэффициенты применяются в `velocity_callback` и `yaw_callback`:

```python
# В velocity_callback:
pitch_pwm = int(vel_to_rc_signal(self._pid_vel_x.get_control() * self._pitchDirection))
roll_pwm = int(vel_to_rc_signal(self._pid_vel_y.get_control() * self._rollDirection))

# В yaw_callback:
yaw_pwm = vel_to_rc_signal(yaw_rate * self._yawDirection)
```

## Типичные значения

Для большинства симуляторов TechSim:
```python
client.set_direction_coefficients(roll=1.0, pitch=1.0, yaw=1.0)
```

Если что-то работает наоборот, измените соответствующий коэффициент на `-1.0`.

## Пример полного использования

```python
from agrotechsimapi.high_level_client import HighLevelSimClient

client = HighLevelSimClient()
client.connect("127.0.0.1", 5762)

# Настраиваем направление (подберите для вашего симулятора)
client.set_direction_coefficients(
    roll=-1.0,   # Инвертировать roll
    pitch=1.0,   # Нормально
    yaw=-1.0     # Инвертировать yaw
)

# Теперь дрон управляется корректно
client.takeoff()
client.go_to_xy("odom", 5.0, 0.0)  # Летит вдоль мировой X
client.setYaw(1.57)                 # Поворачивается на 90 градусов
client.boarding()
```
