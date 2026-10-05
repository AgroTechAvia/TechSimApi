"""Показ камеры AgroTechSim в браузере через MJPEG."""
from __future__ import annotations

import argparse
import threading
import time

import cv2
try:
    from flask import Flask, Response, jsonify, render_template_string, url_for
except ModuleNotFoundError as exc:
    raise SystemExit(
        'Для MJPEG-примера нужен Flask. Установите его командой '
        '`python -m pip install .` из репозитория.'
    ) from exc

from agrotechsimapi import SimClient


app = Flask(__name__)
sim_client: SimClient | None = None
camera_id = 0
capture_lock = threading.Lock()
stream_state = {"last_frame_at": None, "error": "Поток ещё не запущен"}

PAGE = """<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AgroTechSim — камера {{ camera_id }}</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    body { margin: 0; background: #111827; color: #f3f4f6; }
    main { width: min(1100px, calc(100% - 32px)); margin: 32px auto; }
    h1 { margin-bottom: 8px; }
    p { color: #cbd5e1; }
    .viewer { margin-top: 24px; padding: 12px; background: #020617;
              border: 1px solid #334155; border-radius: 12px; }
    img { display: block; width: 100%; height: auto; min-height: 240px;
          object-fit: contain; background: #000; border-radius: 7px; }
    code { color: #7dd3fc; }
    #error { display: none; margin-top: 12px; color: #fca5a5; }
  </style>
</head>
<body>
  <main>
    <h1>AgroTechSim MJPEG Stream</h1>
    <p>Камера {{ camera_id }} · прямой адрес потока:
       <a href="{{ feed_url }}"><code>{{ feed_url }}</code></a></p>
    <div class="viewer">
      <img src="{{ feed_url }}" alt="Поток камеры {{ camera_id }}"
           onerror="document.getElementById('error').style.display='block'">
      <p id="error">Кадр пока не получен. Проверьте симулятор и сообщение в терминале.</p>
    </div>
  </main>
</body>
</html>
"""


def _capture_frame():
    if sim_client is None:
        raise RuntimeError("SimClient не инициализирован")
    # msgpack-rpc client is shared by all browser connections and is not safe
    # for simultaneous calls.
    with capture_lock:
        return sim_client.get_camera_capture(camera_id=camera_id)


def generate_frames():
    """Continuously yield JPEG frames using the MJPEG multipart format."""
    reported_error = None
    while True:
        try:
            frame = _capture_frame()
            if frame is None or getattr(frame, "size", 0) == 0:
                raise RuntimeError(f"камера {camera_id} вернула пустой кадр")
            if frame.ndim == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            encoded, buffer = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
            )
            if not encoded:
                raise RuntimeError("OpenCV не смог закодировать кадр в JPEG")

            stream_state["last_frame_at"] = time.time()
            stream_state["error"] = None
            reported_error = None
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Cache-Control: no-cache\r\n\r\n"
                + buffer.tobytes()
                + b"\r\n"
            )
        except GeneratorExit:
            return
        except Exception as exc:  # keep the stream alive if the simulator recovers
            message = f"{type(exc).__name__}: {exc}"
            stream_state["error"] = message
            if message != reported_error:
                print(f"[ERROR] Не удалось получить кадр: {message}")
                reported_error = message
            time.sleep(0.5)


@app.route("/video_feed")
def video_feed():
    """Return the raw MJPEG stream for browsers and media players."""
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


@app.route("/health")
def health():
    """Expose enough state to diagnose a blank viewer."""
    return jsonify(
        camera_id=camera_id,
        streaming=stream_state["last_frame_at"] is not None,
        last_frame_at=stream_state["last_frame_at"],
        error=stream_state["error"],
    )


@app.route("/")
def index():
    """Render a page that embeds the MJPEG feed."""
    return render_template_string(
        PAGE, camera_id=camera_id, feed_url=url_for("video_feed")
    )


def main(args) -> int:
    global sim_client, camera_id

    camera_id = args.camera_num
    sim_client = SimClient(address=args.host, port=args.sim_port)
    if not sim_client.is_connected():
        print(f"[ERROR] Симулятор недоступен по адресу {args.host}:{args.sim_port}")
        return 1

    try:
        frame = _capture_frame()
        if frame is None or getattr(frame, "size", 0) == 0:
            print(f"[WARN] Камера #{camera_id} пока возвращает пустой кадр; сервер продолжит попытки")
        else:
            height, width = frame.shape[:2]
            print(f"[OK] Получен тестовый кадр {width}x{height} с камеры #{camera_id}")
    except Exception as exc:
        print(f"[WARN] Первый кадр получить не удалось: {type(exc).__name__}: {exc}")
        print("[INFO] Сервер запустится и продолжит попытки получения изображения")

    page_url = f"http://127.0.0.1:{args.port}/"
    feed_url = f"http://127.0.0.1:{args.port}/video_feed"
    print(f"[INFO] Страница с изображением: {page_url}")
    print(f"[INFO] Прямой MJPEG-поток для VLC: {feed_url}")
    print("[INFO] Для остановки нажмите Ctrl+C")

    try:
        app.run(host=args.bind, port=args.port, threaded=True, use_reloader=False)
    finally:
        sim_client.close_connection()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stream an AgroTechSim camera over MJPEG")
    parser.add_argument(
        "--camera-num", "--camera_num", dest="camera_num", type=int,
        choices=(0, 1, 2), default=0,
        help="camera: 0=front, 1=bottom, 2=back (default: 0)",
    )
    parser.add_argument("--host", default="127.0.0.1", help="simulator RPC host")
    parser.add_argument("--sim-port", type=int, default=8080, help="simulator RPC port")
    parser.add_argument("--bind", default="127.0.0.1", help="HTTP bind address")
    parser.add_argument("--port", type=int, default=5000, help="HTTP server port")
    return parser


if __name__ == "__main__":
    raise SystemExit(main(build_parser().parse_args()))
