from inavmspapi import MultirotorControl  
from inavmspapi.transmitter import TCPTransmitter  
from inavmspapi.msp_codes import MSPCodes
from agrotechsimapi.client import SimClient

from agrotechsimapi.pid import PID, AdaptivePID
from typing import Iterable

from agrotechsimapi.utils.utils import LoopingTimer, sim_to_api_distance, vel_to_rc_signal
from agrotechsimapi.utils.vision import process_aruco, process_blob, resolution_changes

from transforms3d.euler import quat2euler

import time
import math
import threading  # Добавлен импорт


def main():
    pass


if __name__ == "__main__":
    main()


class HighLevelSimClient:
    def __init__(self): 

        self.__alt_pid = PID(3, 0.015, 8)
        
        self._odom = (0.0, 0.0)

        self._altitude = 0.0

        self._target_height = 0.0

        self._armed_flag = False
        self._poshold_flag = False

        self._sim_img = None 
        self._blob_img = None 
        self._aruco_img = None 
        self._aruco_data = [] 
        self._camera_pose_aruco_data = [] 
        self._blob_data = []

        self._odom0_xy = (0.0, 0.0)   # (x0, y0) в мировой СК на момент «сброса одометрии»

        self._sim_kinematics = None

        self._sim_ultrasonic = None
        
        # Блокировка для безопасного доступа к клиенту из нескольких потоков
        self._client_lock = threading.Lock()
        
        print("process started")

    def connect(self, ip, port):
        self.__HOST = ip
        self.__SIM_PORT = 8080
        self.__TCP_PORT = 5762
        self.__TCP_ADDRESS = (self.__HOST, self.__TCP_PORT)

        

        self.__tcp_transmitter = TCPTransmitter(self.__TCP_ADDRESS)
        self.__tcp_transmitter.connect()
        self._control = MultirotorControl(self.__tcp_transmitter)
        time.sleep(2)

        # Инициализируем клиент с блокировкой
        with self._client_lock:
            self._client = SimClient(address=self.__HOST, port=self.__SIM_PORT)

        self._height_timer = LoopingTimer(interval=1/50, callback=self.calculate_height_rc, name="height_timer")
        self._rc_timer = LoopingTimer(interval=1/50, callback=self.transmit_rc_to_sim, name="rc_timer")
        self._sim_kinematics_timer = LoopingTimer(interval=1/50, callback=self.sim_kinematics_callback, name="sim_kinematics_timer")
        self._image_processing_timer = LoopingTimer(interval=1/10, callback=self.image_processing_callback, name="image_processing_timer")   #new

        self.sim_kinematics_callback()

        self.initDrone()

        self._sim_kinematics_timer.start()
        self._height_timer.start()
        self._rc_timer.start()
        self._image_processing_timer.start()

        time.sleep(1)
        self._armed_flag = True
        self._arm_data = 2000
        time.sleep(1)
        self._poshold_flag = True
        self._nav_mode = 1500


    def disconnect(self):
        self._sim_kinematics_timer.stop()
        self._height_timer.stop()
        self._rc_timer.stop()
        self._image_processing_timer.stop() #new


    def _world_xy(self, kin=None) -> tuple[float, float]:
        """Мировые координаты XY в твоей API-конвенции."""
        if kin is None:
            kin = self.get_sim_kinematics()
        x_w = sim_to_api_distance(kin["location"][0])
        y_w = sim_to_api_distance(kin["location"][1])
        return x_w, y_w

    def _odom_xy(self, kin=None) -> tuple[float, float]:
        """Одометрия (XY), отн. сохранённого нуля. Если ноль не задан — возвращаем мировые XY."""
        x_w, y_w = self._world_xy(kin)
        if self._odom0_xy == (0.0, 0.0):
            return x_w, y_w
        x0, y0 = self._odom0_xy
        return x_w - x0, y_w - y0
    
    def getHeightRange(self):
        return self._altitude
    
    def getHeightBarometer(self):
        return self._altitude
    
    def getArm(self):
        return self._armed_flag
    
    def setZeroOdomOpticflow(self) -> bool:
        """
        Запомнить текущие мировые координаты (x0,y0) как ноль одометрии.
        Дальше frame='odom' трактуется как координаты относительно этого нуля.
        """
        kin = self.get_sim_kinematics()
        if kin is None:
            raise RuntimeError("Нет кэша кинематики: setZeroOdomOpticflow() вызван слишком рано")
        self._odom0_xy = self._world_xy(kin)
        return True
    
    def getUltrasonic(self):
        return self._sim_ultrasonic


        
    def calculate_height_rc(self):
        self._altitude = sim_to_api_distance(self.get_sim_kinematics()["location"][2])

        error = self._target_height - self._altitude
        # print(f"target height: {self._target_height}, error: {error}")

        self.__alt_pid.update_control(error)
        alt_rc_output = self.__alt_pid.get_control()

        throttle_output = self.clamp_rc(1500 + alt_rc_output * 100)

        if self._armed_flag and self._poshold_flag:
            self._throttle_data = throttle_output

    def getRPY(self):
        kin = self.get_sim_kinematics()
        qx, qy, qz, qw = kin["orientation"]
        roll, pitch, yaw = quat2euler((qw, qx, qy, qz), axes='sxyz')
        return [roll, pitch, yaw]

    
    def go_to_xy(self, frame: str, x: float, y: float) -> bool:
        # ========= FRAME =========
        if frame == "odom":
            use_body_shift = False
            use_odom_frame = True
        elif frame == "base_link":
            use_body_shift = True
            use_odom_frame = False
        else:
            raise ValueError("frame must be 'odom' or 'base_link'")

        # ========= PARAMETERS =========
        PERIOD = 1 / 50.0

        POS_THR = 0.1
        VEL_THR = 0.004

        V_MAX = 1.34 #1.325
        A_MAX = 1.45   #1.4       

        T_MIN = 1.0
        T_MAX = 30.0

        # --- Position loop (SOFT) ---
        KP_POS = 1.26        

        # --- Velocity loop ---
        KP_VEL = 4.2 #3.69
        KI_VEL = 0.938 #0.935
        KD_VEL = 8.5 # 6.0

        pid_vx = PID(KP_VEL, KI_VEL, KD_VEL,
                    max_control=V_MAX,
                    i_limit=0.3)

        pid_vy = PID(KP_VEL, KI_VEL, KD_VEL,
                    max_control=V_MAX,
                    i_limit=0.3)

        pid_vx.reset()
        pid_vy.reset()

        mono = time.monotonic
        next_t = mono()

        kin = self.get_sim_kinematics()
        cx, cy = self._world_xy(kin)

        qx, qy, qz, qw = kin["orientation"]
        _, _, yaw = quat2euler((qw, qx, qy, qz), axes="sxyz")

        # ========= TARGET =========
        if not use_body_shift:
            if use_odom_frame and self._odom0_xy is not None:
                x0, y0 = self._odom0_xy
                tgt_x, tgt_y = x0 + x, y0 + y
            else:
                tgt_x, tgt_y = x, y
        else:
            tgt_x = cx + math.cos(yaw) * x - math.sin(yaw) * y
            tgt_y = cy + math.sin(yaw) * x + math.cos(yaw) * y

        dx0 = tgt_x - cx
        dy0 = tgt_y - cy
        dist0 = math.hypot(dx0, dy0)
        timeout = max(T_MIN, min((dist0 / 0.1) * 2.0, T_MAX))
        deadline = mono() + timeout

        x_prev, y_prev = cx, cy
        v_fwd_target_prev = 0.0
        v_left_target_prev = 0.0

    
        try:
            while True:
                kin = self.get_sim_kinematics()
                x_w = sim_to_api_distance(kin["location"][0])
                y_w = sim_to_api_distance(kin["location"][1])

                qx, qy, qz, qw = kin["orientation"]
                _, _, yaw = quat2euler((qw, qx, qy, qz), axes="sxyz")

                # ========= POSITION ERROR =========
                dx = tgt_x - x_w
                dy = tgt_y - y_w

                err_x =  math.cos(yaw) * dx + math.sin(yaw) * dy
                err_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

                # ========= POSITION → VELOCITY =========
                v_fwd_target  = KP_POS * err_x
                v_left_target = KP_POS * err_y

                # accel limiting
                dv_fwd  = max(-A_MAX * PERIOD, min(v_fwd_target  - v_fwd_target_prev,  A_MAX * PERIOD))
                dv_left = max(-A_MAX * PERIOD, min(v_left_target - v_left_target_prev, A_MAX * PERIOD))

                v_fwd_target  = v_fwd_target_prev  + dv_fwd
                v_left_target = v_left_target_prev + dv_left

                v_fwd_target  = max(-V_MAX, min(v_fwd_target,  V_MAX))
                v_left_target = max(-V_MAX, min(v_left_target, V_MAX))

                v_fwd_target_prev  = v_fwd_target
                v_left_target_prev = v_left_target

                # ========= CURRENT VELOCITY =========
                vx_w = (x_w - x_prev) / PERIOD
                vy_w = (y_w - y_prev) / PERIOD

                v_fwd_current  =  math.cos(yaw) * vx_w + math.sin(yaw) * vy_w
                v_left_current = -math.sin(yaw) * vx_w + math.cos(yaw) * vy_w

                x_prev, y_prev = x_w, y_w

                # ========= VELOCITY LOOP =========
                err_v_fwd  = v_fwd_target  - v_fwd_current
                err_v_left = v_left_target - v_left_current

                pid_vx.update_control(err_v_fwd)
                pid_vy.update_control(err_v_left)

                u_fwd  = pid_vx.get_control()
                u_left = pid_vy.get_control()

                pitch_pwm = vel_to_rc_signal(u_fwd)
                roll_pwm  = vel_to_rc_signal(-u_left)  # <-- ЛЕВАЯ СК, СОХРАНЕНО

                self._rpy_vel_data = (roll_pwm, pitch_pwm, 1500)

                # ========= EXIT =========
                if (
                    math.sqrt(pow(abs(err_x),2)+pow(abs(err_y),2)) < POS_THR and
                    math.sqrt(pow(abs(vx_w),2)+pow(abs(vy_w),2)) < VEL_THR
                ):
                    self._rpy_vel_data = (1500, 1500, 1500)
                    return True

                if mono() > deadline:
                    self._rpy_vel_data = (1500, 1500, 1500)
                    return False

                next_t += PERIOD
                sleep_time = max(0.0, next_t - mono())
                if sleep_time > 0:
                    time.sleep(sleep_time)

        finally:
            self._rpy_vel_data = (1500, 1500, 1500)


    
    def _flush_log_buffer(self, log_file, buffer):
        """Записывает буфер логов в файл"""
        for entry in buffer:
            self._log_pid_data(log_file, *entry)
        log_file.flush()  # Гарантируем запись на диск

    def getOdomOpticflow(self):
        kin = self.get_sim_kinematics()
        x, y = self._world_xy(kin)
        last_x, last_y = self._odom0_xy
        odom_x = x - last_x
        odom_y = y - last_y
        return [odom_x, odom_y, self._altitude]

    def gotoXYdrone(self, x, y):
        return self.go_to_xy("base_link", x, y)

    def gotoXYodom(self, x, y):
        return self.go_to_xy("odom", x, y)

    
    @staticmethod
    def _wrap_pi(a: float) -> float:
        """Нормализация угла в интервал [-pi, pi)."""
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _get_yaw_cw(self) -> float:
        """
        Текущий курс в РАДИАНАХ с положительным направлением ПО ЧАСОВОЙ.
        IMU/сим даёт yaw CCW -> инвертируем знак.
        """
        kin = self.get_sim_kinematics()  # ДОЛЖЕН быть кэш (см. твой sim_kinematics_callback)
        qx, qy, qz, qw = kin["orientation"]  # (x,y,z,w)
        _, _, yaw_ccw = quat2euler((qw, qx, qy, qz), axes='sxyz')
        return -yaw_ccw  # переводим в CW

    def setYaw(self, yaw: float) -> bool:
        """
        БЛОКИРУЮЩИЙ поворот до абсолютного угла в ГЛОБАЛЬНОЙ СК.
        Вход: yaw — радианы, ПО ЧАСОВОЙ (CW), нормализуется в [-pi, pi).
        Возврат: True (достиг в допуске) / False (таймаут).
        """
        # ===== жёсткие настройки =====
        PERIOD     = 1/20.0     # 20 Гц
        TOL        = 0.025       # рад, допуск по углу
        MAX_TIME   = 10.0       # сек, таймаут
        MAX_RATE   = 0.75        # рад/с, ограничение выхода PID
        KP, KI, KD = 3.0, 0.001, 0.30  # PID по курсу
        I_LIMIT    = None       # можно поставить число (напр. 600) для ограничения интеграла
        # =============================

        pid = PID(KP, KI, KD, max_control=MAX_RATE, i_limit=I_LIMIT)
        pid.reset()

        mono = time.monotonic
        next_t = mono()
        deadline = mono() + MAX_TIME

        # цель сразу нормализуем в CW
        goal = self._wrap_pi(yaw)

        try:
            while True:
                curr = self._get_yaw_cw()                    # текущее CW
                err  = self._wrap_pi(goal - curr)            # кратчайшая дуга

                # завершение
                if abs(err) < TOL:
                    r, p, _ = self._rpy_vel_data
                    self._rpy_vel_data = (r, p, 1500)            # отпустить руддер
                    return True
                if mono() > deadline:
                    r, p, _ = self._rpy_vel_data
                    self._rpy_vel_data = (r, p, 1500)
                    return False

                # PID -> угловая скорость (рад/с), внутри уже есть сатурация MAX_RATE
                pid.update_control(err)
                rate_cw = pid.get_control()

                # рад/с -> PWM (если «вправо» у тебя даёт yaw<1500 — просто поставь минус)
                yaw_pwm = vel_to_rc_signal(rate_cw)  # или vel_to_rc_signal(-rate_cw), если знак канала другой

                # пишем только yaw
                r, p, _ = self._rpy_vel_data
                self._rpy_vel_data = (r, p, yaw_pwm)  # оставь именно (r, p, yaw_pwm)

                next_t += PERIOD
                time.sleep(max(0.0, next_t - mono()))
        finally:
            # гарантированно нейтраль по yaw
            r, p, _ = self._rpy_vel_data
            self._rpy_vel_data = (r, p, 1500)

    def _wrap(self, a: float) -> float:  # на будущее (не обязателен)
        return (a + math.pi) % (2*math.pi) - math.pi

    def _get_height(self) -> float:
        # Текущая высота из твоего кэша (calculate_height_rc её обновляет)
        return float(self._altitude)

    def _clamp_h(self, h: float, lo: float, hi: float) -> float:
        return max(lo, min(h, hi))

    def _sleep_until(self, t_deadline: float, period: float) -> bool:
        # true -> живём; false -> вышел таймаут
        now = time.monotonic()
        if now >= t_deadline:
            return False
        time.sleep(max(0.0, period - (time.monotonic() - now)))
        return True

    def takeoff(self) -> bool:
        """
        ВЗЛЁТ: мгновенно кидаем целевую высоту takeoff_h и ждём достижения.
        Блокирующий. True/False = достиг/таймаут.
        """
        # ---- жёсткие настройки ----
        PERIOD        = 0.05      # 20 Гц
        MIN_H         = 0.00
        TAKEOFF_H     = 1.00
        MAX_H         = 5.00
        REACH_COEF    = 0.70      # считаем «достигли», если >= 70% цели
        TIMEOUT_COEF  = 10.0      # с/м
        # ---------------------------

        h_now = self._get_height()
        tgt   = self._clamp_h(TAKEOFF_H, MIN_H, MAX_H)

        # Мгновенно ставим финальный сетпоинт
        self.set_target_height(tgt)

        # Таймаут пропорционален требуемому подъёму (как в твоём ROS-коде)
        climb = max(0.0, tgt - h_now)
        deadline = time.monotonic() + (TIMEOUT_COEF * climb if climb > 0.0 else TIMEOUT_COEF)

        while True:
            h = self._get_height()
            if h >= REACH_COEF * tgt:
                return True
            if time.monotonic() >= deadline:
                return False
            if not self._sleep_until(deadline, PERIOD):
                return False

    def boarding(self) -> bool:
        """
        ПОСАДКА: плавно снижаем целевой сетпоинт до MIN_H лесенкой.
        Блокирующий. True/False = завершил/таймаут.
        """
        # ---- жёсткие настройки ----
        PERIOD        = 0.05      # 20 Гц
        MIN_H         = 0.00
        MAX_H         = 5.00
        STEP          = 0.01      # м за шаг сетпоинта
        TIMEOUT_MIN   = 15.0      # базовый таймаут
        TIMEOUT_COEF  = 10.0      # доп. таймаут пропорционален высоте
        # ---------------------------

        # Стартуем от текущего командного сетпоинта (если он меньше факта — ок)
        curr_cmd = float(getattr(self, "_target_height", 0.0))
        curr_cmd = self._clamp_h(curr_cmd, MIN_H, MAX_H)

        total_drop = max(0.0, curr_cmd - MIN_H)
        deadline = time.monotonic() + TIMEOUT_MIN + TIMEOUT_COEF * total_drop

        while curr_cmd > MIN_H:
            curr_cmd = max(MIN_H, curr_cmd - STEP)
            self.set_target_height(curr_cmd)
            if time.monotonic() >= deadline:
                return False
            if not self._sleep_until(deadline, PERIOD):
                return False

        # На земле — успех (ждать фактического нуля не обязательно: контур дотянет сам)
        return True

    def setHeight(self, target_height: float) -> bool:
        """
        Установить высоту.
        - Если цель ВЫШЕ текущей фактической высоты → сразу ставим конечный сетпоинт и ждём достижения.
        - Если цель НИЖЕ текущей фактической высоты → плавно опускаем сетпоинт лесенкой до цели.
        Блокирующий. True/False = достиг/таймаут/пределы.
        """
        # ---- жёсткие настройки ----
        PERIOD        = 0.05
        MIN_H         = 0.00
        MAX_H         = 5.00
        STEP          = 0.01
        REACH_COEF    = 0.70
        TIMEOUT_MIN   = 15.0
        TIMEOUT_COEF  = 10.0
        # ---------------------------

        tgt = self._clamp_h(float(target_height), MIN_H, MAX_H)
        h0  = self._get_height()

        # ВВЕРХ: сразу кидаем финальный сетпоинт
        if tgt >= h0:
            self.set_target_height(tgt)
            deadline = time.monotonic() + TIMEOUT_MIN + TIMEOUT_COEF * max(0.0, tgt - h0)
            while True:
                h = self._get_height()
                if h >= REACH_COEF * tgt:
                    return True
                if time.monotonic() >= deadline:
                    return False
                if not self._sleep_until(deadline, PERIOD):
                    return False

        # ВНИЗ: плавная лесенка от текущего командного
        curr_cmd = float(getattr(self, "_target_height", h0))
        curr_cmd = self._clamp_h(curr_cmd, MIN_H, MAX_H)
        deadline = time.monotonic() + TIMEOUT_MIN + TIMEOUT_COEF * max(0.0, curr_cmd - tgt)

        while curr_cmd > tgt:
            curr_cmd = max(tgt, curr_cmd - STEP)
            self.set_target_height(curr_cmd)
            if time.monotonic() >= deadline:
                return False
            if not self._sleep_until(deadline, PERIOD):
                return False

        return True


    def sim_kinematics_callback(self):
        # Используем блокировку при обращении к клиенту
        with self._client_lock:
            self._sim_kinematics = self._client.get_kinametics_data()
            self._sim_img = self._client.get_camera_capture(camera_id=1) #выбор камеры 0-передняя 1-нижняя
            self._sim_ultrasonic = self._client.get_range_data(rangefinder_id = 0, range_min = 0.15, range_max = 4, is_clear = True, range_error = 0.0003) * 100


    def get_sim_kinematics(self):
        return self._sim_kinematics
    

    def transmit_rc_to_sim(self):
        roll, pitch, yaw = self._rpy_vel_data
        raw_rc = [roll, pitch, self._throttle_data, yaw, self._arm_data, self._fliyng_mode, self._nav_mode]
        raw_rc = self.clamp_rc_list(raw_rc)
        msg = self._control.send_RAW_RC(raw_rc)
        data_handler = self._control.receive_msg()

    def setVelXY(self, x, y):
        rpy = self._rpy_vel_data
        rc_x = vel_to_rc_signal(x)
        rc_y = vel_to_rc_signal(y)
        self._rpy_vel_data = (rc_y, rc_x, rpy[2])
    
    def setVelXYYaw(self, x, y, yaw):
        rc_x = vel_to_rc_signal(x)
        rc_y = vel_to_rc_signal(y)
        rc_yaw = vel_to_rc_signal(yaw)
        self._rpy_vel_data = (rc_y, rc_x, rc_yaw)

    def armDrone(self):
        self._armed_flag = True
        self._arm_data = 2000

    def disarmDrone(self):
        self._armed_flag = False
        self._arm_data = 1000

    def initDrone(self):
        self._rpy_vel_data = (1500, 1500, 1500)
        self._throttle_data = 1000
        self._arm_data = 1000
        self._fliyng_mode = 1000
        self._nav_mode = 1000

    def posholdOn(self):
        self._poshold_flag = True
        self._nav_mode = 1500

    def posholdOff(self):
        self._poshold_flag = False
        self._nav_mode = 1000

    def clamp_rc(self, data):
        return max(min(data, 2000), 1000)
    
    def clamp_rc_list(self, data: Iterable):
        return [self.clamp_rc(rc) for rc in data]
    
    def round_data(self, iterable):
        return map(lambda x: round(x, 3), iterable)
    
    def set_target_height(self, height):
        self._target_height = height
        self.__alt_pid.reset()

    #new
    def getImage(self): 
        return self._sim_img
    
    def getArucos(self): 
        return self._aruco_data
    
    def getCameraPoseAruco(self): 
        return self._camera_pose_aruco_data
    
    def getBlobs(self): 
        return self._blob_data
    
    def getBlobsImage(self): 
        return self._blob_img
    
    def getArucosImage(self): 
        return self._aruco_img
    
    def image_processing_callback(self): 
        # Обработка изображения не требует доступа к клиенту, 
        # используем уже полученное изображение из кэша

        sim_image = self._sim_img.copy() if self._sim_img is not None else None
        camera_img = resolution_changes(sim_image, (320, 240))

        img_aruco = camera_img.copy() if self._sim_img is not None else None
        img_blob = camera_img.copy() if self._sim_img is not None else None
        
        if img_aruco is not None:
            self._aruco_data, self._camera_pose_aruco_data, aruco_img = process_aruco(img_aruco)

            if aruco_img is None:
                self._aruco_img = sim_image
            else:
                self._aruco_img = resolution_changes(aruco_img, (640, 480))


        if img_blob is not None:
            self._blob_data, blob_img = process_blob(img_blob)
            if blob_img is None:
                self._blob_img = sim_image
            else:
                self._blob_img = resolution_changes(blob_img, (640, 480))

    def setDiod(self, r, g, b):
        # Используем блокировку при обращении к клиенту
        with self._client_lock:
            return self._client.set_Diod(float(r), float(g), float(b))
        
    def setShoot(self, time):
        with self._client_lock:
            return self._client.call_event_action()


