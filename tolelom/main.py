"""WebSocket 주기 송신 데모. 저장소 루트에서 `python -m tolelom.main`으로 실행한다.

get_position·get_status·get_path_points는 가짜 값이며, 제어 루프와 연결할 때 교체한다.
"""
import logging
import queue
import threading
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from tolelom.web_socket_client import AGVWebSocketClient, resolve_backend_ws_url

logger = logging.getLogger(__name__)

POSITION_PERIOD_SEC = 0.1
STATUS_PERIOD_SEC = 0.5
PATH_PERIOD_SEC = 2.0
DISCONNECTED_POLL_SEC = 0.2
LOOP_IDLE_SEC = 0.02
SHUTDOWN_JOIN_SEC = 2.0

OutboxItem = Tuple[str, Dict[str, Any]]
outbox: "queue.Queue[OutboxItem]" = queue.Queue()


def init_websocket_client(server_url: Optional[str] = None) -> AGVWebSocketClient:
    client = AGVWebSocketClient(resolve_backend_ws_url(server_url))
    client.start()
    return client


_fake_pos = 0.0


def get_position() -> Tuple[float, float, float]:
    global _fake_pos
    _fake_pos += 0.1
    return _fake_pos, _fake_pos, 0.0


def get_status() -> Dict[str, Any]:
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
            "objects_detected": 3,
        },
    }


def get_path_points() -> List[Dict[str, Any]]:
    now = datetime.now().isoformat()
    return [
        {"x": 10.5, "y": 20.3, "angle": 0.0, "timestamp": now},
        {"x": 12.0, "y": 22.0, "angle": 0.0, "timestamp": now},
        {"x": 15.0, "y": 25.0, "angle": 0.0, "timestamp": now},
    ]


def enqueue_log(msg: str, level: str = "info") -> None:
    outbox.put(("log", {"message": msg, "level": level}))


def enqueue_target_found(enemy: Dict[str, Any]) -> None:
    outbox.put(("target_found", {"enemy": enemy}))


def drain_outbox(client: AGVWebSocketClient, box: "queue.Queue[OutboxItem]") -> int:
    """보낸 개수를 반환한다."""
    sent = 0
    while True:
        try:
            typ, payload = box.get_nowait()
        except queue.Empty:
            return sent
        if typ == "log":
            client.send_log(payload["message"], payload["level"])
        elif typ == "target_found":
            client.send_target_found(payload["enemy"])
        else:
            logger.warning("알 수 없는 outbox 항목 type=%s — 버린다", typ)
            continue
        sent += 1


def send_loop(
    client: AGVWebSocketClient,
    stop_evt: threading.Event,
    pos_period: float = POSITION_PERIOD_SEC,
    status_period: float = STATUS_PERIOD_SEC,
    path_period: float = PATH_PERIOD_SEC,
    box: "queue.Queue[OutboxItem]" = outbox,
) -> None:
    next_pos = next_status = next_path = time.monotonic()
    was_connected = False

    while not stop_evt.is_set():
        if not client.connected:
            was_connected = False
            stop_evt.wait(DISCONNECTED_POLL_SEC)
            continue

        # 끊긴 동안 밀린 주기를 재연결 직후 한꺼번에 보내지 않도록 일정을 지금으로 되돌린다.
        if not was_connected:
            next_pos = next_status = next_path = time.monotonic()
            was_connected = True

        now = time.monotonic()

        # 예외 한 번에 송신 스레드가 죽으면 프로세스는 살아 있는데 아무것도 안 보내게 된다.
        try:
            if now >= next_pos:
                next_pos += pos_period
                x, y, angle = get_position()
                client.send_position(x, y, angle)

            if now >= next_status:
                next_status += status_period
                client.send_status(get_status())

            if now >= next_path:
                next_path += path_period
                client.send_path_update(get_path_points(), "a_star")

            drain_outbox(client, box)
        except Exception:
            logger.exception("send_loop 사이클 실패 — 다음 주기에 계속")

        stop_evt.wait(LOOP_IDLE_SEC)


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    client = init_websocket_client()

    stop_evt = threading.Event()
    th = threading.Thread(target=send_loop, args=(client, stop_evt), daemon=True)
    th.start()

    enqueue_log("AGV 주기 전송 시작", "info")

    try:
        logger.info("주기 전송 중... (Ctrl+C로 종료)")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("종료 중...")
    finally:
        stop_evt.set()
        th.join(timeout=SHUTDOWN_JOIN_SEC)
        client.close()


if __name__ == "__main__":
    main()
