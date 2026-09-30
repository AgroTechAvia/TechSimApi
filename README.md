# AgroTechSim API

Python API для симулятора AgroTechSim. Пакет предоставляет низкоуровневый
доступ к сенсорам и событиям симулятора, а также высокоуровневое автономное
управление с каскадом **положение → скорость → ускорение → PWM**.

- Python 3.10+
- установка из PyPI или исходников
- Windows и Linux
- именованные PID-калибровки с импортом и экспортом
- встроенная автоматическая калибровка и автономные HTML-отчёты

## Установка

Из PyPI:

```bash
python -m pip install agrotechsimapi
```

Из локального репозитория:

```bash
python -m pip install .
```

Для разработки без повторной установки после каждого изменения:

```bash
python -m pip install -e .
```

## Низкоуровневый клиент

`SimClient` работает с RPC-интерфейсом симулятора: камерами, кинематикой,
лидаром, радаром, дальномером, LED и событиями.

```python
from agrotechsimapi import SimClient

client = SimClient(address="127.0.0.1", port=8080)
kinematics = client.get_kinametics_data()

print("position:", kinematics["location"])
print("velocity:", kinematics["linear_velocity"])

client.close_connection()
```

Примеры находятся в [`examples_low_level`](examples_low_level/).

## Высокоуровневый клиент

`HighLevelSimClient` и `HighLevelClient` являются именами одного класса. Клиент
использует MSP TCP на порту `5762` и RPC симулятора на порту `8080`.

```python
import math
from agrotechsimapi import HighLevelSimClient

client = HighLevelSimClient(calibration="edu")
client.connect("127.0.0.1", 5762, sim_port=8080)

client.armDrone()
client.altholdOn()
client.takeoff()

client.gotoXYdrone(2.0, 0.0)
client.setYaw(math.radians(90))  # setYaw принимает радианы
client.gotoXYdrone(1.0, 0.0)

client.boarding()
client.disarmDrone()
client.disconnect()
```

Примеры находятся в [`examples_high_level`](examples_high_level/).

## Встроенные калибровки

Пакет поставляется с тремя пресетами только для чтения:

| Имя | Назначение |
|---|---|
| `edu-ext` | пресет по умолчанию |
| `edu-constructor` | стандартный пресет EDU Constructor |
| `edu` | стандартный пресет EDU |

Старое имя `default` поддерживается как скрытый псевдоним `edu-ext`.

```python
from agrotechsimapi import HighLevelClient

default_drone = HighLevelClient()  # edu-ext
edu_drone = HighLevelClient(calibration="edu")
custom_drone = HighLevelClient(calibration="my_drone")
file_drone = HighLevelClient(calibration_path="exported_calibration.json")
```

Посмотреть доступные калибровки:

```bash
python -m agrotechsimapi calibrations list
python -m agrotechsimapi calibrations show edu
```

## Автоматическая калибровка

Создайте конфигурацию, проверьте план и затем запустите полёт:

```bash
python -m agrotechsimapi calibrations init-config --output calibration.json
python -m agrotechsimapi calibrate --name my_drone --config calibration.json
python -m agrotechsimapi calibrate --name my_drone --config calibration.json --fly
```

Калибровка последовательно настраивает высоту, yaw, ускорение XY, скорость XY и
положение XY. Готовый профиль сохраняется в пользовательском каталоге данных и
становится доступен по имени в `HighLevelClient`.

Полная практическая инструкция:

- [Запуск калибровки, графики и перенос профилей](docs/calibration_guide.md)
- [Техническое описание алгоритмов и хранилища](docs/calibration.md)

## Графики

Графики завершённой калибровки:

```bash
python -m agrotechsimapi plot --calibration my_drone --open
```

Одноразовый снимок текущего запуска:

```bash
python -m agrotechsimapi plot --current --last 8 --open
```

Отчёт является автономным HTML-файлом с вкладками высоты, yaw, ускорения,
скорости и положения, а также переключателем русского и английского языка.

## Импорт и экспорт

```bash
python -m agrotechsimapi calibrations export my_drone --output my_drone.json
python -m agrotechsimapi calibrations import my_drone.json --name imported_drone
```

Пользовательское хранилище расположено в `%LOCALAPPDATA%\agrotechsimapi` на
Windows, `$XDG_DATA_HOME/agrotechsimapi` или `~/.local/share/agrotechsimapi` на
Linux и `~/Library/Application Support/agrotechsimapi` на macOS.

## Проверка исходников

```bash
python -m pytest -q tests utils/test_auto_calibration_tool.py
python -m pip wheel . --no-deps
```

## English

AgroTechSim API provides low-level simulator access and calibrated autonomous
flight control for Python 3.10+. Install it with:

```bash
python -m pip install agrotechsimapi
```

The high-level client uses a position → velocity → acceleration → PWM control
cascade. Select one of the bundled profiles with
`HighLevelClient(calibration="edu")`, or run a named calibration with:

```bash
python -m agrotechsimapi calibrations init-config --output calibration.json
python -m agrotechsimapi calibrate --name my_drone --config calibration.json --fly
python -m agrotechsimapi plot --calibration my_drone --lang en --open
```

See the [calibration guide](docs/calibration_guide.md) and the
[technical calibration reference](docs/calibration.md) for details.

## License

Apache-2.0.
