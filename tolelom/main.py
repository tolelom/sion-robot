import logging
import time
import threading
import queue
from datetime import datetime
from typing import Optional, Any, Dict, Tuple

logger = logging.getLogger(__name__)

from tolelom.web_socket_client import AGVWebSocketClient, resolve_backend_ws_url

ws_client: Optional[AGVWebSocketClient] = None

# (type, payload) 형태로 전송 작업을 쌓아두는 큐
outbox: "queue.Queue[Tuple[str, Dict[str, Any]]]" = queue.Queue()


def init_websocket_client(server_url: Optional[str] = None):
    global ws_client
    ws_client = AGVWebSocketClient(resolve_backend_ws_url(server_url))
    ws_client.start()
    return ws_client


# ====== 실제 시스템에서는 여기만 하드웨어/알고리즘 값으로 교체 ======
cnt = 0.0
def get_position() -> Tuple[float, float, float]:
    # TODO: localization/odometry에서 읽기
    global cnt
    cnt += 0.1
    return cnt, cnt, 0.0

def get_status() -> Dict[str, Any]:
    # TODO: 내부 상태머신/센서에서 읽기
    return {
        "mode": "auto",
        "state": "moving",
        "moving": True,
        "target_cell": [15, 25],
        "speed": 1.2,
        "battery": 85,
        "detected_enemies": [],
        "sensors": {
            "front_distance": 150.0,
            "left_distance": 200.0,
            "right_distance": 180.0,
            "camera_active": True,
            "objects_detected": 3
        }
    }

def get_path_points() -> list[dict]:
    # TODO: planner에서 최신 경로 가져오기
    now = datetime.now().isoformat()
    return [
        {"x": 10.5, "y": 20.3, "angle": 0.0, "timestamp": now},
        {"x": 12.0, "y": 22.0, "angle": 0.0, "timestamp": now},
        {"x": 15.0, "y": 25.0, "angle": 0.0, "timestamp": now},
    ]
# =====================================================================


def enqueue_log(msg: str, level: str = "info"):
    outbox.put(("log", {"message": msg, "level": level}))

def enqueue_target_found(enemy: Dict[str, Any]):
    outbox.put(("target_found", {"enemy": enemy}))


def send_loop(stop_evt: threading.Event,
              pos_period: float = 0.1,     # 10Hz
              status_period: float = 0.5,  # 2Hz
              path_period: float = 2.0     # 0.5Hz
              ):
    global ws_client

    next_pos = time.monotonic()
    next_status = time.monotonic()
    next_path = time.monotonic()

    while not stop_evt.is_set():
        # 연결 안 되어 있으면 잠깐 쉬고 재시도(재연결 로직은 AGVWebSocketClient 정책에 맞춰 확장)
        if not ws_client or not ws_client.connected:
            stop_evt.wait(0.2)
            continue

        now = time.monotonic()

        # 1) 주기 전송: position
        if now >= next_pos:
            x, y, angle = get_position()
            ws_client.send_position(x, y, angle)
            next_pos += pos_period

        # 2) 주기 전송: status
        if now >= next_status:
            ws_client.send_status(get_status())
            next_status += status_period

        # 3) 주기 전송: path
        if now >= next_path:
            ws_client.send_path_update(get_path_points(), "a_star")
            next_path += path_period

        # 4) 비주기 이벤트(로그/타겟발견 등) 처리: 한 루프에서 몰아서 전송
        try:
            while True:
                typ, payload = outbox.get_nowait()
                if typ == "log":
                    ws_client.send_log(payload["message"], payload["level"])
                elif typ == "target_found":
                    ws_client.send_target_found(payload["enemy"])
        except queue.Empty:
            pass

        # 너무 바쁘게 돌지 않게 짧게 sleep
        stop_evt.wait(0.02)


if __name__ == "__main__":
    client = init_websocket_client()

    stop_evt = threading.Event()
    th = threading.Thread(target=send_loop, args=(stop_evt,), daemon=True)
    th.start()

    enqueue_log("AGV 주기 전송 시작", "info")

    try:
        logger.info("주기 전송 중... (Ctrl+C로 종료)")
        while True:
            time.sleep(1)
            # 예시: 이벤트성 메시지
            # enqueue_target_found({...})
    except KeyboardInterrupt:
        logger.info("종료 중...")
    finally:
        stop_evt.set()
        th.join(timeout=2.0)
        client.close()
