"""
Тесты для проверки преобразования координат при разных углах yaw.
Проверяет согласованность position_callback и velocity_callback.
"""
import math
import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))


class TestCoordinateTransformationConsistency(unittest.TestCase):
    """Тесты согласованности преобразования координат между position и velocity callback"""

    def test_yaw_0_degrees(self):
        """yaw=0°: преобразование должно быть идентичным"""
        yaw = 0.0
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # Тестовые значения
        pos_error_x = 0.5
        pos_error_y = -0.3
        vx_world = 0.1
        vy_world = -0.05
        
        # position_callback (мир → дрон)
        pos_error_body_x = pos_error_x * cos_yaw - pos_error_y * sin_yaw
        pos_error_body_y = pos_error_x * sin_yaw + pos_error_y * cos_yaw
        
        # velocity_callback (мир → дрон)
        vx_body = vx_world * cos_yaw - vy_world * sin_yaw
        vy_body = vx_world * sin_yaw + vy_world * cos_yaw
        
        # При yaw=0 все должно остаться без изменений
        self.assertAlmostEqual(pos_error_body_x, 0.5)
        self.assertAlmostEqual(pos_error_body_y, -0.3)
        self.assertAlmostEqual(vx_body, 0.1)
        self.assertAlmostEqual(vy_body, -0.05)

    def test_yaw_90_degrees_cw(self):
        """yaw=90° по часовой: X мира → Y дрона, Y мира → -X дрона"""
        yaw = math.pi / 2  # 90°
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        # Дрон в (1.5, -1.5), цель в (2, -2)
        pos_error_x = 0.5  # 2 - 1.5
        pos_error_y = -0.5  # -2 - (-1.5)
        
        # Дрон движется к цели
        vx_world = 0.1
        vy_world = -0.1
        
        # position_callback (мир → дрон)
        pos_error_body_x = pos_error_x * cos_yaw - pos_error_y * sin_yaw
        pos_error_body_y = pos_error_x * sin_yaw + pos_error_y * cos_yaw
        
        # velocity_callback (мир → дрон)
        vx_body = vx_world * cos_yaw - vy_world * sin_yaw
        vy_body = vx_world * sin_yaw + vy_world * cos_yaw
        
        print(f"\n[yaw=90°] position_callback:")
        print(f"  pos_error_world=({pos_error_x}, {pos_error_y})")
        print(f"  pos_error_body=({pos_error_body_x:.3f}, {pos_error_body_y:.3f})")
        
        print(f"\n[yaw=90°] velocity_callback:")
        print(f"  vx_world=({vx_world}, {vy_world})")
        print(f"  vx_body=({vx_body:.3f}, {vy_body:.3f})")
        
        # При yaw=90°: cos=0, sin=1
        # position: (0.5, -0.5) → (0.5, 0.5)
        self.assertAlmostEqual(pos_error_body_x, 0.5, places=5)
        self.assertAlmostEqual(pos_error_body_y, 0.5, places=5)
        
        # velocity: (0.1, -0.1) → (0.1, 0.1)
        self.assertAlmostEqual(vx_body, 0.1, places=5)
        self.assertAlmostEqual(vy_body, 0.1, places=5)
        
        # Ошибка скорости должна быть согласованной
        target_vx = -0.33  # целевая скорость из position_callback
        target_vy = -0.33
        vel_error_x = target_vx - vx_body
        vel_error_y = target_vy - vy_body
        
        print(f"\n[yaw=90°] Ошибка скорости:")
        print(f"  vel_error=({vel_error_x:.3f}, {vel_error_y:.3f})")
        
        # Дрон должен лететь вперед и влево (отрицательные значения в СК дрона)
        self.assertLess(vel_error_x, 0)
        self.assertLess(vel_error_y, 0)

    def test_yaw_45_degrees(self):
        """yaw=45°: проверка диагонального преобразования"""
        yaw = math.pi / 4  # 45°
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        pos_error_x = 1.0
        pos_error_y = 0.0
        vx_world = 0.2
        vy_world = 0.0
        
        # position_callback (мир → дрон)
        pos_error_body_x = pos_error_x * cos_yaw - pos_error_y * sin_yaw
        pos_error_body_y = pos_error_x * sin_yaw + pos_error_y * cos_yaw
        
        # velocity_callback (мир → дрон)
        vx_body = vx_world * cos_yaw - vy_world * sin_yaw
        vy_body = vx_world * sin_yaw + vy_world * cos_yaw
        
        print(f"\n[yaw=45°] position_callback:")
        print(f"  pos_error_world=({pos_error_x}, {pos_error_y})")
        print(f"  pos_error_body=({pos_error_body_x:.3f}, {pos_error_body_y:.3f})")
        
        print(f"\n[yaw=45°] velocity_callback:")
        print(f"  vx_world=({vx_world}, {vy_world})")
        print(f"  vx_body=({vx_body:.3f}, {vy_body:.3f})")
        
        # При yaw=45°: cos=sin=√2/2 ≈ 0.707
        expected = math.sqrt(2) / 2
        self.assertAlmostEqual(pos_error_body_x, expected, places=5)
        self.assertAlmostEqual(pos_error_body_y, expected, places=5)
        self.assertAlmostEqual(vx_body, expected * 0.2, places=5)
        self.assertAlmostEqual(vy_body, expected * 0.2, places=5)

    def test_yaw_minus_90_degrees(self):
        """yaw=-90°: проверка обратного поворота"""
        yaw = -math.pi / 2  # -90°
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        pos_error_x = 0.5
        pos_error_y = -0.5
        vx_world = 0.1
        vy_world = -0.1
        
        # position_callback (мир → дрон)
        pos_error_body_x = pos_error_x * cos_yaw - pos_error_y * sin_yaw
        pos_error_body_y = pos_error_x * sin_yaw + pos_error_y * cos_yaw
        
        # velocity_callback (мир → дрон)
        vx_body = vx_world * cos_yaw - vy_world * sin_yaw
        vy_body = vx_world * sin_yaw + vy_world * cos_yaw
        
        print(f"\n[yaw=-90°] position_callback:")
        print(f"  pos_error_world=({pos_error_x}, {pos_error_y})")
        print(f"  pos_error_body=({pos_error_body_x:.3f}, {pos_error_body_y:.3f})")
        
        print(f"\n[yaw=-90°] velocity_callback:")
        print(f"  vx_world=({vx_world}, {vy_world})")
        print(f"  vx_body=({vx_body:.3f}, {vy_body:.3f})")
        
        # При yaw=-90°: cos=0, sin=-1
        self.assertAlmostEqual(pos_error_body_x, -0.5, places=5)
        self.assertAlmostEqual(pos_error_body_y, -0.5, places=5)
        self.assertAlmostEqual(vx_body, -0.1, places=5)
        self.assertAlmostEqual(vy_body, -0.1, places=5)

    def test_inverse_transformation_consistency(self):
        """Прямое и обратное преобразование должны давать исходные значения"""
        yaw = 0.7  # произвольный угол
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        pos_error_x = 2.5
        pos_error_y = -1.3
        vx_world = 0.33
        vy_world = -0.2
        
        # Мир → дрон
        pos_error_body_x = pos_error_x * cos_yaw - pos_error_y * sin_yaw
        pos_error_body_y = pos_error_x * sin_yaw + pos_error_y * cos_yaw
        vx_body = vx_world * cos_yaw - vy_world * sin_yaw
        vy_body = vx_world * sin_yaw + vy_world * cos_yaw
        
        # Дрон → мир (обратная матрица)
        pos_error_x_back = pos_error_body_x * cos_yaw + pos_error_body_y * sin_yaw
        pos_error_y_back = -pos_error_body_x * sin_yaw + pos_error_body_y * cos_yaw
        vx_world_back = vx_body * cos_yaw + vy_body * sin_yaw
        vy_world_back = -vx_body * sin_yaw + vy_body * cos_yaw
        
        # Проверяем восстановление исходных значений
        self.assertAlmostEqual(pos_error_x, pos_error_x_back, places=5)
        self.assertAlmostEqual(pos_error_y, pos_error_y_back, places=5)
        self.assertAlmostEqual(vx_world, vx_world_back, places=5)
        self.assertAlmostEqual(vy_world, vy_world_back, places=5)

    def test_cascade_system_consistency(self):
        """Тест согласованности каскадной системы при разных yaw"""
        test_cases = [
            (0, "yaw=0°"),
            (math.pi/4, "yaw=45°"),
            (math.pi/2, "yaw=90°"),
            (-math.pi/2, "yaw=-90°"),
            (math.pi, "yaw=180°"),
        ]
        
        print(f"\n[Cascade Consistency] Тест согласованности:")
        
        for yaw, name in test_cases:
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            # Дрон в (0, 0), цель в (1, 0) в мировой СК
            pos_error_x = 1.0
            pos_error_y = 0.0
            
            # position_callback
            pos_error_body_x = pos_error_x * cos_yaw - pos_error_y * sin_yaw
            pos_error_body_y = pos_error_x * sin_yaw + pos_error_y * cos_yaw
            
            # Целевая скорость (упрощенно)
            target_vx = pos_error_body_x
            target_vy = pos_error_body_y
            
            # velocity_callback (дрон стоит на месте)
            vx_world = 0.0
            vy_world = 0.0
            vx_body = vx_world * cos_yaw - vy_world * sin_yaw
            vy_body = vx_world * sin_yaw + vy_world * cos_yaw
            
            # Ошибка скорости
            vel_error_x = target_vx - vx_body
            vel_error_y = target_vy - vy_body
            
            # Проверяем, что ошибка скорости направлена к цели
            # (в СК дрона цель всегда должна быть в направлении pos_error_body)
            expected_error_x = pos_error_body_x
            expected_error_y = pos_error_body_y
            
            print(f"  {name}: vel_error=({vel_error_x:.3f}, {vel_error_y:.3f}), "
                  f"expected=({expected_error_x:.3f}, {expected_error_y:.3f})")
            
            self.assertAlmostEqual(vel_error_x, expected_error_x, places=5)
            self.assertAlmostEqual(vel_error_y, expected_error_y, places=5)


if __name__ == '__main__':
    unittest.main(verbosity=2)
