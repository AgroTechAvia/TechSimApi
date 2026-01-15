# AgroTechSim API

<div align="center">
  <img src="img/header.png" alt="AgroTechSim API" width="800">
  
  [Русский](#русский) | [English](#english)
</div>

---

## Русский

### 🚀 Описание

**AgroTechSim API** — это Python-библиотека для работы с симулятором AgroTechSim, предназначенная для получения телеметрии с дронов, обработки сенсорных данных и реализации систем автономного управления. Библиотека предоставляет как низкоуровневый доступ к сырым данным сенсоров, так и высокоуровневые абстракции для упрощённого управления дроном.

Модуль позволяет разработчикам создавать сложные алгоритмы компьютерного зрения, навигации и управления без необходимости работы с реальным оборудованием, используя полнофункциональный симулятор сельскохозяйственных дронов.

### 📊 Совместимость

| Компонент | Версия | Статус |
|-----------|--------|--------|
| **AgroTechSim API** | 1.0.0 | ✅ Актуальная |
| **Симулятор AgroTechSim** | 1.0.3 | ✅ Рекомендуемая |
| **Python** | 3.10+ | ✅ Рекомендуемая |
| **InavMSPApi** | 1.1.0 | ✅ Зависимость |

### ⚡ Установка

#### Вариант 1: Установка из PyPi (рекомендуется)

```bash
pip install agrotechsimapi
```

Этот способ автоматически установит все зависимости, включая `inavmspapi` требуемой версии.

#### Вариант 2: Установка из исходников

1. Клонируйте репозиторий:
```bash
git clone https://github.com/AgroTechAvia/agrotechsimapi.git
cd agrotechsimapi
```

2. Запустите скрипт установки:
```bash
python setup_by_source.py
```

Скрипт автоматически установит все зависимости и настроит окружение.

#### Вариант 3: Ручная установка (для разработчиков)

```bash
# 1. Создайте виртуальное окружение
python -m venv .venv

# 2. Активируйте окружение
# Windows:
.venv\Scripts\activate
# Linux/Mac:
source .venv/bin/activate

# 3. Установите inavmspapi
git clone https://github.com/AgroTechAvia/InavMSPApi.git
cd InavMSPApi
git checkout v1.1.0
pip install .

# 4. Установите agrotechsimapi
cd ..
git clone https://github.com/AgroTechAvia/agrotechsimapi.git
cd agrotechsimapi
pip install .
```

### 🎯 Функционал

#### Низкоуровневый API (`SimClient`)
- 📷 **Камеры**: Получение цветных, тепловизионных, глубинных и спектральных изображений
- 📡 **Лидар**: 360° сканирование окружения с настраиваемыми параметрами
- 🎯 **Радар**: Обнаружение ближайших объектов с угловыми координатами
- 📏 **Дальномер**: Точечное измерение расстояния
- 💡 **LED управление**: Контроль подсветки дрона
- 📊 **Кинематика**: Получение позиции, ориентации и скорости дрона

#### Высокоуровневый API (`HighLevelSimClient`)
- 🚁 **Управление полётом**: Взлёт, посадка, удержание высоты
- 🧭 **Навигация**: Движение к координатам в разных системах отсчёта
- 🎯 **Стабилизация**: PID-регуляторы для автоматического удержания позиции
- 👁️ **Обработка изображений**: Детектирование ArUco маркеров и blob-объектов
- 🔄 **Одометрия**: Отслеживание перемещения относительно начальной точки

### 📁 Примеры использования

В репозитории представлены готовые примеры:

#### `examples_low_level/` - Низкоуровневые примеры:

#### `examples_high_level/` - Высокоуровневые примеры:


### 🚀 Быстрый старт

```python
from agrotechsimapi import SimClient, CaptureType
import cv2

# Подключение к симулятору
client = SimClient(address="127.0.0.1", port=8080)

# Получение изображения с камеры
image = client.get_camera_capture(camera_id=0, type=CaptureType.color)

# Отображение изображения
cv2.imshow("Drone Camera", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

### 🧪 Тестирование работоспособности

Для проверки корректной работы модуля с симулятором используйте интеграционные тесты. Файл `test_sim_client_real.py` содержит тесты, которые проверяют подключение к симулятору и работу всех основных функций.

#### 📋 Предварительные требования:
1. **Запущенный симулятор AgroTechSim**
2. **Дрон появился** в симуляторе
3. **Установлен Python 3.10+** и необходимые зависимости

#### 🚀 Запуск тестов:

```bash
# Перейдите в директорию проекта
cd agrotechsimapi

# Запустите тесты с симулятором
pytest test/test_sim_client_real.py --with-simulator -v
```

#### 📊 Что проверяют тесты:

Тест `test_sim_client_real.py` последовательно проверяет:

1. **✅ Подключение к симулятору** - проверка соединения
2. **✅ Получение изображений с камер** - цветная, тепловизионная, глубинная камеры
3. **✅ Кинематические данные** - позиция и ориентация дрона
4. **✅ Данные дальномера** - измерение расстояния
5. **✅ Сканирование лидаром** - 360° сканирование окружения
6. **✅ Радар** - обнаружение ближайших объектов
7. **✅ Управление LED** - включение/выключение подсветки
8. **✅ Обработка изображений** - добавление шума и артефактов
9. **✅ Несколько камер** - работа с разными ID камер
10. **✅ Обработка ошибок** - корректная реакция на невалидные параметры
11. **✅ Производительность** - время отклика основных функций

#### ⚠️ Важные замечания по тестированию:

- **Симулятор должен быть запущен** перед запуском тестов
- **Тесты могут пропускаться**, если определенный сенсор не доступен в текущей конфигурации симулятора
- **Первые измерения лидара и радара** могут быть некорректными (особенность симулятора)
- **Для ручной проверки** можно использовать скрипт:


#### 🔧 Пример вывода успешного тестирования:

```
============================================================
Running SimClient Integration Tests with Real Simulator
============================================================

test_connection: ✓ PASSED
test_get_camera_capture: ✓ PASSED  
test_get_kinematics_data: ✓ PASSED
test_get_range_data: ✓ PASSED
test_get_laser_scan: ✓ PASSED
test_get_radar_point: ⚠ SKIPPED (Radar not available)
test_led_control: ✓ PASSED
test_image_processing_methods: ✓ PASSED
test_multiple_camera_ids: ✓ PASSED
test_call_event_action: ✓ PASSED
test_error_handling: ✓ PASSED
test_performance: ✓ PASSED

✅ Все основные функции работают корректно!
```

#### 🐛 Если тесты не проходят:

1. **Проверьте подключение к симулятору:**
```python
from agrotechsimapi import SimClient
client = SimClient()
print(f"Connected: {client.is_connected()}")
```

2. **Убедитесь, что порт 8080 открыт:**
```bash
# Linux/Mac
nc -z localhost 8080
# Windows
Test-NetConnection -ComputerName localhost -Port 8080
```

3. **Проверьте, что дрон заспавнен в симуляторе**

Тестирование позволяет убедиться, что все компоненты модуля работают корректно с текущей версией симулятора.

---

## English

### 🚀 Description

**AgroTechSim API** is a Python library for working with the AgroTechSim simulator, designed for obtaining drone telemetry, processing sensor data, and implementing autonomous control systems. The library provides both low-level access to raw sensor data and high-level abstractions for simplified drone control.

The module enables developers to create complex computer vision, navigation, and control algorithms without the need for real hardware, using a full-featured agricultural drone simulator.

### 📊 Compatibility

| Component | Version | Status |
|-----------|---------|--------|
| **AgroTechSim API** | 1.0.0 | ✅ Current |
| **AgroTechSim Simulator** | 1.0.3 | ✅ Recommended |
| **Python** | 3.10+ | ✅ Recommended |
| **InavMSPApi** | 1.1.0 | ✅ Dependency |

### ⚡ Installation

#### Option 1: Install from PyPi (recommended)

```bash
pip install agrotechsimapi
```

This method will automatically install all dependencies, including the required version of `inavmspapi`.

#### Option 2: Install from source

1. Clone the repository:
```bash
git clone https://github.com/AgroTechAvia/agrotechsimapi.git
cd agrotechsimapi
```

2. Run the installation script:
```bash
python setup_by_source.py
```

The script will automatically install all dependencies and set up the environment.

#### Option 3: Manual installation (for developers)

```bash
# 1. Create a virtual environment
python -m venv .venv

# 2. Activate the environment
# Windows:
.venv\Scripts\activate
# Linux/Mac:
source .venv/bin/activate

# 3. Install inavmspapi
git clone https://github.com/AgroTechAvia/InavMSPApi.git
cd InavMSPApi
git checkout v1.1.0
pip install .

# 4. Install agrotechsimapi
cd ..
git clone https://github.com/AgroTechAvia/agrotechsimapi.git
cd agrotechsimapi
pip install .
```

### 🎯 Features

#### Low-Level API (`SimClient`)
- 📷 **Cameras**: Capture color, thermal, depth, and spectral images
- 📡 **Lidar**: 360° environment scanning with configurable parameters
- 🎯 **Radar**: Nearest object detection with angular coordinates
- 📏 **Rangefinder**: Point distance measurement
- 💡 **LED Control**: Drone lighting control
- 📊 **Kinematics**: Get drone position, orientation, and velocity

#### High-Level API (`HighLevelSimClient`)
- 🚁 **Flight Control**: Takeoff, landing, altitude hold
- 🧭 **Navigation**: Movement to coordinates in different reference frames
- 🎯 **Stabilization**: PID controllers for automatic position hold
- 👁️ **Image Processing**: ArUco marker and blob object detection
- 🔄 **Odometry**: Track movement relative to starting point

### 📁 Usage Examples

The repository includes ready-to-use examples:

#### `examples_low_level/` - Low-level examples:
- `camera_capture.py` - Working with drone cameras
- `lidar_scan.py` - Environment scanning with lidar
- `sensor_fusion.py` - Combined use of different sensors
- `led_control.py` - LED lighting control

#### `examples_high_level/` - High-level examples:
- `autonomous_flight.py` - Autonomous route flight
- `object_tracking.py` - Object tracking using camera
- `precision_landing.py` - Precision landing using visual markers
- `mission_planner.py` - Mission planning and execution

### 🚀 Quick Start

```python
from agrotechsimapi import SimClient, CaptureType
import cv2

# Connect to simulator
client = SimClient(address="127.0.0.1", port=8080)

# Get camera image
image = client.get_camera_capture(camera_id=0, type=CaptureType.color)

# Display image
cv2.imshow("Drone Camera", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

### 🧪 Functionality Testing

To verify that the module works correctly with the simulator, use the integration tests. The `test_sim_client_real.py` file contains tests that check the connection to the simulator and the operation of all main functions.

#### 📋 Prerequisites:
1. **Running AgroTechSim simulator** on `localhost:8080`
2. **Drone spawned** in the simulator
3. **Python 3.10+ installed** with necessary dependencies

#### 🚀 Running tests:

```bash
# Go to the project directory
cd agrotechsimapi

# Run tests with simulator
pytest tests/integration/test_sim_client_real.py --with-simulator -v
```

#### 📊 What the tests check:

The `test_sim_client_real.py` test sequentially verifies:

1. **✅ Connection to simulator** - connection check
2. **✅ Camera image capture** - color, thermal, depth cameras
3. **✅ Kinematic data** - drone position and orientation
4. **✅ Rangefinder data** - distance measurement
5. **✅ Lidar scanning** - 360° environment scanning
6. **✅ Radar** - nearest object detection
7. **✅ LED control** - turning lights on/off
8. **✅ Image processing** - adding noise and artifacts
9. **✅ Multiple cameras** - working with different camera IDs
10. **✅ Error handling** - correct response to invalid parameters
11. **✅ Performance** - response time of main functions

#### ⚠️ Important testing notes:

- **Simulator must be running** before starting tests
- **Tests may be skipped** if a particular sensor is not available in the current simulator configuration
- **First lidar and radar measurements** may be incorrect (simulator feature)
- **For manual checking** you can use the script:

```bash
# Quick connection check
python tests/integration/simple_test.py
```

#### 🔧 Example of successful test output:

```
============================================================
Running SimClient Integration Tests with Real Simulator
============================================================

test_connection: ✓ PASSED
test_get_camera_capture: ✓ PASSED  
test_get_kinematics_data: ✓ PASSED
test_get_range_data: ✓ PASSED
test_get_laser_scan: ✓ PASSED
test_get_radar_point: ⚠ SKIPPED (Radar not available)
test_led_control: ✓ PASSED
test_image_processing_methods: ✓ PASSED
test_multiple_camera_ids: ✓ PASSED
test_call_event_action: ✓ PASSED
test_error_handling: ✓ PASSED
test_performance: ✓ PASSED

✅ All main functions work correctly!
```

#### 🐛 If tests fail:

1. **Check connection to simulator:**
```python
from agrotechsimapi import SimClient
client = SimClient()
print(f"Connected: {client.is_connected()}")
```

2. **Make sure port 8080 is open:**
```bash
# Linux/Mac
nc -z localhost 8080
# Windows
Test-NetConnection -ComputerName localhost -Port 8080
```

3. **Verify that drone is spawned in simulator**

Testing allows you to ensure that all module components work correctly with the current simulator version.

---

<div align="center">
  <sub>Built with ❤️ by AgroTechAvia</sub>
</div>