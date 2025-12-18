import time
import threading
from typing import Callable


def sim_to_api_distance(distance):
    return distance * 1

def vel_to_rc_signal(vel):
    return 1500 + vel * 100

def round_data(iterable):
    return map(lambda x: round(x, 3), iterable)


class LoopingTimer:
    def __init__(self, interval: float, callback: Callable, name: str="", *args, **kwargs):
        """
        interval  – период вызова в секундах
        callback  – функция, которую надо вызывать
        *args, **kwargs – аргументы для callback
        """
        self.interval = interval
        self.callback = callback
        self.args = args
        self.kwargs = kwargs
        self.name = name

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop_event.is_set():
            start = time.time()
            try:
                self.callback(*self.args, **self.kwargs)
            except Exception as e:
                print(f"Ошибка в коллбэке {self.name}: {e.with_traceback()}")
            # ждём остаток интервала (чтобы цикл был равномерный)
            elapsed = time.time() - start
            time.sleep(max(0, self.interval - elapsed))

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()