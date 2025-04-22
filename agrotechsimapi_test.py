import unittest
import numpy as np
import cv2
from agrotechsimapi import SimClient
from agrotechsimapi.client import post_process
import time

class TestSimClient(unittest.TestCase):

    def setUp(self):
        # Создаем реальный клиент, который будет подключаться к серверу
        self.client = SimClient(address="127.0.0.1", port=8080)

    def tearDown(self):
        self.client.close_connection()
        return super().tearDown()

    def test_post_process(self):
        # Тестируем функцию post_process
        image = np.random.randint(0, 255, (360, 480, 4), dtype=np.uint8)
        processed_image = post_process(image, gamma=1.8, new_size=(640, 480), saturation=1.05, contrast=1)
        self.assertEqual(processed_image.shape, (640, 480, 3))

    def test_add_noise(self):
        # Тестируем функцию add_noise
        image = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        noisy_image = self.client.add_noise(image)
        self.assertEqual(noisy_image.shape, image.shape)

    def test_add_artifacts(self):
        # Тестируем функцию add_artifacts
        image = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        artifact_image = self.client.add_artifacts(image)
        self.assertEqual(artifact_image.shape, image.shape)

    def test_is_connected(self):
        # Тестируем функцию is_connected
        is_connected = self.client.is_connected()
        self.assertTrue(is_connected)  # Ожидаем, что соединение установлено

    def test_get_camera_capture(self):
        # Тестируем функцию get_camera_capture
        image = self.client.get_camera_capture(camera_id=0, is_clear=True)
        self.assertIsNotNone(image)  # Проверяем, что изображение получено
        self.assertEqual(image.shape, (640, 480, 3))  # Проверяем размер изображения

        image_with_noise = self.client.get_camera_capture(camera_id=0, is_clear=False)
        self.assertIsNotNone(image_with_noise)  # Проверяем, что изображение с шумом получено
        self.assertEqual(image_with_noise.shape, (640, 480, 3))  # Проверяем размер изображения

    def test_get_laser_scan(self):
        # Тестируем функцию get_laser_scan
        scan_data = self.client.get_laser_scan(is_clear=True)
        self.assertIsNotNone(scan_data)  # Проверяем, что данные получены
        self.assertGreater(len(scan_data), 0)  # Проверяем, что данные не пустые

        scan_data_with_noise = self.client.get_laser_scan(is_clear=False)
        self.assertIsNotNone(scan_data_with_noise)  # Проверяем, что данные с шумом получены
        self.assertGreater(len(scan_data_with_noise), 0)  # Проверяем, что данные не пустые

    def test_get_radar_point(self):
        # Тестируем функцию get_radar_point
        radar_point = self.client.get_radar_point(is_clear=True)
        self.assertIsNotNone(radar_point)  # Проверяем, что данные получены
        self.assertEqual(len(radar_point), 3)  # Проверяем, что возвращается три значения (расстояние и два угла)

        radar_point_with_noise = self.client.get_radar_point(is_clear=False)
        self.assertIsNotNone(radar_point_with_noise)  # Проверяем, что данные с шумом получены
        self.assertEqual(len(radar_point_with_noise), 3)  # Проверяем, что возвращается три значения

    def test_get_range_data(self):
        # Тестируем функцию get_range_data
        range_data = self.client.get_range_data(is_clear=True)
        self.assertIsNotNone(range_data)  # Проверяем, что данные получены
        self.assertIsInstance(range_data, float)  # Проверяем, что возвращается число

        range_data_with_noise = self.client.get_range_data(is_clear=False)
        self.assertIsNotNone(range_data_with_noise)  # Проверяем, что данные с шумом получены
        self.assertIsInstance(range_data_with_noise, float)  # Проверяем, что возвращается число

    def test_set_led_intensity(self):
        # Тестируем функцию set_led_intensity
        self.client.set_led_intensity(led_id=0, new_intensity=0.7)
        # Проверяем, что функция выполнилась без ошибок (прямой проверки результата нет, так как это side effect)

    def test_set_led_state(self):
        # Тестируем функцию set_led_state
        self.client.set_led_state(led_id=0, new_state=False)
        # Проверяем, что функция выполнилась без ошибок (прямой проверки результата нет, так как это side effect)

    def test_get_kinematics_data(self):
        # Тестируем функцию get_kinematics_data
        kinematics_data = self.client.get_kinametics_data()
        self.assertIsNotNone(kinematics_data)  # Проверяем, что данные получены
        self.assertIn('orientation', kinematics_data)  # Проверяем, что в данных есть позиция
        self.assertIn('location', kinematics_data)  # Проверяем, что в данных есть позиция

        self.assertIn('linear_velocity', kinematics_data)  # Проверяем, что в данных есть скорость
        self.assertIn('angular_velocity', kinematics_data)  # Проверяем, что в данных есть скорость


    def test_led_intensity_cycle(self):
        # Проверяем цикл изменения интенсивности светодиода
        intensities = [0.2, 0.5, 0.8, 1.0]
        for intensity in intensities:
            self.client.set_led_intensity(led_id=0, new_intensity=intensity)
            # Здесь можно добавить проверку состояния светодиода, если есть такая возможность
            # Например, если есть функция get_led_intensity, можно вызвать её и проверить значение
            # current_intensity = self.client.get_led_intensity(led_id=0)
            # self.assertAlmostEqual(current_intensity, intensity, delta=0.01)

    def test_camera_and_lidar_simultaneous(self):
        # Проверяем одновременное использование камеры и лидара
        for _ in range(30):  # Повторяем несколько раз
            image = self.client.get_camera_capture(camera_id=0, is_clear=True)
            self.assertIsNotNone(image)  # Проверяем, что изображение получено
            self.assertEqual(image.shape, (640, 480, 3))  # Проверяем размер изображения

            lidar_data = self.client.get_laser_scan(is_clear=True)
            self.assertIsNotNone(lidar_data)  # Проверяем, что данные лидара получены
            self.assertGreater(len(lidar_data), 0)  # Проверяем, что данные не пустые

    def test_led_state_and_radar_cycle(self):
    # Проверяем цикл изменения состояния светодиода и получение данных с радара
        for _ in range(30):  # Повторяем несколько раз
            self.client.set_led_state(led_id=0, new_state=True)
            radar_data = self.client.get_radar_point(is_clear=True)
            self.assertIsNotNone(radar_data)  # Проверяем, что данные радара получены
            self.assertEqual(len(radar_data), 3)  # Проверяем, что возвращается три значения

            self.client.set_led_state(led_id=0, new_state=False)
            radar_data = self.client.get_radar_point(is_clear=True)
            self.assertIsNotNone(radar_data)  # Проверяем, что данные радара получены
            self.assertEqual(len(radar_data), 3)  # Проверяем, что возвращается три значения
    
    def test_camera_lidar_radar_simultaneous(self):
        # Проверяем одновременное использование камеры, лидара и радара
        for _ in range(30):  # Повторяем несколько раз
            image = self.client.get_camera_capture(camera_id=0, is_clear=True)
            self.assertIsNotNone(image)  # Проверяем, что изображение получено
            self.assertEqual(image.shape, (640, 480, 3))  # Проверяем размер изображения

            lidar_data = self.client.get_laser_scan(is_clear=True)
            self.assertIsNotNone(lidar_data)  # Проверяем, что данные лидара получены
            self.assertGreater(len(lidar_data), 0)  # Проверяем, что данные не пустые

            radar_data = self.client.get_radar_point(is_clear=True)
            self.assertIsNotNone(radar_data)  # Проверяем, что данные радара получены
            self.assertEqual(len(radar_data), 3)  # Проверяем, что возвращается три значения
    
    def test_kinematics_after_commands(self):
        # Выполняем несколько команд
        for _ in range(30):
            self.client.set_led_state(led_id=0, new_state=True)
            image = self.client.get_camera_capture(camera_id=0, is_clear=True)
            self.assertIsNotNone(image)  # Проверяем, что изображение получено

            # Получаем данные кинематики
            kinematics_data = self.client.get_kinametics_data()
            self.assertIsNotNone(kinematics_data)  # Проверяем, что данные получены
            self.assertIn('orientation', kinematics_data)  # Проверяем, что в данных есть позиция
            self.assertIn('location', kinematics_data)  # Проверяем, что в данных есть позиция

            self.assertIn('linear_velocity', kinematics_data)  # Проверяем, что в данных есть скорость
            self.assertIn('angular_velocity', kinematics_data)  # Проверяем, что в данных есть скорость

    def test_noise_and_artifacts_cycle(self):
        # Проверяем цикл получения изображений с шумом и артефактами
        for _ in range(30):  # Повторяем несколько раз
            image = self.client.get_camera_capture(camera_id=0, is_clear=False)
            self.assertIsNotNone(image)  # Проверяем, что изображение получено
            self.assertEqual(image.shape, (640, 480, 3))  # Проверяем размер изображения

    def test_multiple_cameras(self):
        # Проверяем работу с несколькими камерами
        for _ in range(300):
            for camera_id in range(3):  # Предположим, что у нас есть 2 камеры
                image = self.client.get_camera_capture(camera_id=camera_id, is_clear=True)
                self.assertIsNotNone(image)  # Проверяем, что изображение получено
                self.assertEqual(image.shape, (480, 640, 3))  # Проверяем размер изображения
            time.sleep(1/30)

    def test_camera_lidar_radar_with_noise(self):
        # Проверяем одновременное использование камеры, лидара и радара с шумом
        for _ in range(30):  # Повторяем несколько раз
            # Получаем изображение с шумом
            image = self.client.get_camera_capture(camera_id=0, is_clear=False)
            self.assertIsNotNone(image)  # Проверяем, что изображение получено
            self.assertEqual(image.shape, (640, 480, 3))  # Проверяем размер изображения

            # Получаем данные лидара с шумом
            lidar_data = self.client.get_laser_scan(is_clear=False)
            self.assertIsNotNone(lidar_data)  # Проверяем, что данные лидара получены
            self.assertGreater(len(lidar_data), 0)  # Проверяем, что данные не пустые

            # Получаем данные радара с шумом
            radar_data = self.client.get_radar_point(is_clear=False)
            self.assertIsNotNone(radar_data)  # Проверяем, что данные радара получены
            self.assertEqual(len(radar_data), 3)  # Проверяем, что возвращается три значения

    def test_led_intensity_and_camera(self):
        # Проверяем изменение интенсивности светодиода и получение данных с камеры
        intensities = [0.2, 0.5, 0.8, 1.0]
        for intensity in intensities:
            # Устанавливаем интенсивность светодиода
            self.client.set_led_intensity(led_id=0, new_intensity=intensity)

            # Получаем изображение с камеры
            image = self.client.get_camera_capture(camera_id=0, is_clear=True)
            self.assertIsNotNone(image)  # Проверяем, что изображение получено
            self.assertEqual(image.shape, (640, 480, 3))  # Проверяем размер изображения

    def test_multiple_cameras_and_lidar(self):
        # Проверяем работу с несколькими камерами и лидаром
        for camera_id in range(3):  # Предположим, что у нас есть 3 камеры
            # Получаем изображение с камеры
            image = self.client.get_camera_capture(camera_id=camera_id, is_clear=True)
            self.assertIsNotNone(image)  # Проверяем, что изображение получено
            self.assertEqual(image.shape, (640, 480, 3))  # Проверяем размер изображения

            # Получаем данные лидара
            lidar_data = self.client.get_laser_scan(is_clear=True)
            self.assertIsNotNone(lidar_data)  # Проверяем, что данные лидара получены
            self.assertGreater(len(lidar_data), 0)  # Проверяем, что данные не пустые

    def test_radar_and_rangefinder(self):
        # Проверяем одновременное использование радара и дальномера
        for _ in range(30):  # Повторяем несколько раз
            # Получаем данные радара
            radar_data = self.client.get_radar_point(is_clear=True)
            self.assertIsNotNone(radar_data)  # Проверяем, что данные радара получены
            self.assertEqual(len(radar_data), 3)  # Проверяем, что возвращается три значения

            # Получаем данные дальномера
            range_data = self.client.get_range_data(is_clear=True)
            self.assertIsNotNone(range_data)  # Проверяем, что данные дальномера получены
            self.assertIsInstance(range_data, float)  # Проверяем, что возвращается число

    def test_kinematics_after_multiple_commands(self):
        # Выполняем несколько команд
        self.client.set_led_state(led_id=0, new_state=True)
        self.client.set_led_intensity(led_id=0, new_intensity=0.7)
        image = self.client.get_camera_capture(camera_id=0, is_clear=True)
        self.assertIsNotNone(image)  # Проверяем, что изображение получено

        # Получаем данные кинематики
        kinematics_data = self.client.get_kinametics_data()
        self.assertIsNotNone(kinematics_data)  # Проверяем, что данные получены
        self.assertIn('orientation', kinematics_data)  # Проверяем, что в данных есть позиция
        self.assertIn('location', kinematics_data)  # Проверяем, что в данных есть позиция

        self.assertIn('linear_velocity', kinematics_data)  # Проверяем, что в данных есть скорость
        self.assertIn('angular_velocity', kinematics_data)  # Проверяем, что в данных есть скорость


    def test_camera_and_lidar_with_noise_cycle(self):
        # Проверяем цикл получения данных с камеры и лидара с шумом
        for _ in range(30):  # Повторяем несколько раз
            # Получаем изображение с шумом
            image = self.client.get_camera_capture(camera_id=0, is_clear=False)
            self.assertIsNotNone(image)  # Проверяем, что изображение получено
            self.assertEqual(image.shape, (640, 480, 3))  # Проверяем размер изображения

            # Получаем данные лидара с шумом
            lidar_data = self.client.get_laser_scan(is_clear=False)
            self.assertIsNotNone(lidar_data)  # Проверяем, что данные лидара получены
            self.assertGreater(len(lidar_data), 0)  # Проверяем, что данные не пустые


    def test_camera_lidar_radar_cycle(self):
        # Проверяем цикл получения данных с камеры, лидара и радара
        for _ in range(30):  # Повторяем несколько раз
            # Получаем изображение с камеры
            image = self.client.get_camera_capture(camera_id=0, is_clear=True)
            self.assertIsNotNone(image)  # Проверяем, что изображение получено
            self.assertEqual(image.shape, (640, 480, 3))  # Проверяем размер изображения

            # Получаем данные лидара
            lidar_data = self.client.get_laser_scan(is_clear=True)
            self.assertIsNotNone(lidar_data)  # Проверяем, что данные лидара получены
            self.assertGreater(len(lidar_data), 0)  # Проверяем, что данные не пустые

            # Получаем данные радара
            radar_data = self.client.get_radar_point(is_clear=True)
            self.assertIsNotNone(radar_data)  # Проверяем, что данные радара получены
            self.assertEqual(len(radar_data), 3)  # Проверяем, что возвращается три значения

    def test_camera_and_rangefinder_cycle(self):
        # Проверяем цикл получения данных с камеры и дальномера
        for _ in range(30):  # Повторяем несколько раз
            # Получаем изображение с камеры
            image = self.client.get_camera_capture(camera_id=0, is_clear=True)
            self.assertIsNotNone(image)  # Проверяем, что изображение получено
            self.assertEqual(image.shape, (640, 480, 3))  # Проверяем размер изображения

            # Получаем данные дальномера
            range_data = self.client.get_range_data(is_clear=True)
            self.assertIsNotNone(range_data)  # Проверяем, что данные дальномера получены
            self.assertIsInstance(range_data, float)  # Проверяем, что возвращается число

if __name__ == '__main__':
    unittest.main()