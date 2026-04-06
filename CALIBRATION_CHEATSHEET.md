# Быстрая шпаргалка по настройке направления

## Симптомы и решения

| Проблема | Решение |
|----------|---------|
| Дрон летит **назад** вместо **вперед** | `client.invert_pitch()` |
| Дрон летит **влево** вместо **вправо** | `client.invert_roll()` |
| Дрон вращается **против часовой** вместо **по часовой** | `client.invert_yaw()` |

## Быстрый старт

```python
# После подключения и взлета
client.takeoff()

# 1. Проверяем pitch (вперед/назад)
client.set_velocity_xy(0.2, 0.0, frame="base_link")
# Если летит не туда:
client.invert_pitch()

# 2. Проверяем roll (влево/вправо)  
client.set_velocity_xy(0.0, 0.2, frame="base_link")
# Если летит не туда:
client.invert_roll()

# 3. Проверяем yaw (вращение)
client.setYaw(client._get_yaw_cw() + 1.57)
# Если вращается не туда:
client.invert_yaw()
```

## Сохраняем результат

После настройки узнайте текущие значения:

```python
print(client.get_direction_coefficients())
# {'roll': 1.0, 'pitch': -1.0, 'yaw': 1.0}
```

И используйте их в следующий раз:

```python
client.set_direction_coefficients(roll=1.0, pitch=-1.0, yaw=1.0)
```

## Полная программа калибровки

```bash
python deps/TechSimApi/test_direction_calibration.py
```
