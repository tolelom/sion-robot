import asyncio
import json
import logging
import os
import threading
import time
from datetime import datetime

import websockets

from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)

DEFAULT_BACKEND_WS_URL = "ws://localhost:3000/websocket/agv"
DEFAULT_START_TIMEOUT_SEC = 5.0
DEFAULT_SEND_TIMEOUT_SEC = 1.0


def resolve_backend_ws_url(explicit: Optional[str] = None) -> str:
    """server_url 결정: 명시 인자 > SION_BACKEND_WS_URL 환경변수 > localhost 기본값."""
    if explicit:
        return explicit
    return os.environ.get("SION_BACKEND_WS_URL", DEFAULT_BACKEND_WS_URL)


class AGVWebSocketClient:
    """AGV WebSocket 클라이언트.

    - 연결 상태는 threading.Event(_connected_event)로 추적. plain bool은 멀티스레드에서 명시성이 떨어진다.
    - start()는 timeout 안에 연결되지 않으면 False를 반환한다(이전엔 무한 busy wait였다).
    - _send_message는 모든 예외를 잡아 로그만 남긴다. 한 호출 실패가 send_loop를 죽이지 않게.
    """

    def __init__(self, server_url: Optional[str] = None):
        self.server_url = resolve_backend_ws_url(server_url)
        self.websocket: Optional[websockets.WebSocketClientProtocol] = None
        self._connected_event = threading.Event()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.receive_thread: Optional[threading.Thread] = None

        # AGV 상태 저장
        self.agv_id = "sion-001"
        self.agv_name = "Sion AGV"
        self.current_position = {"x": 0.0, "y": 0.0, "angle": 0.0}
        self.battery = 100
        self.speed = 0.0

    @property
    def connected(self) -> bool:
        return self._connected_event.is_set()

    def start(self, timeout: float = DEFAULT_START_TIMEOUT_SEC) -> bool:
        """별도 스레드에서 WebSocket 연결을 시작하고 timeout 안에 연결 완료를 대기.

        Returns:
            True: timeout 안에 연결 성공.
            False: timeout 초과. 호출자가 폴백을 결정한다(메모: "통신 안되면 그냥 이동 -> 영상").
        """
        self.receive_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.receive_thread.start()

        if not self._connected_event.wait(timeout=timeout):
            logger.warning("Web Socket 연결 timeout(%ss): %s", timeout, self.server_url)
            return False

        logger.info("Web Socket 연결 완료: %s", self.server_url)
        return True

    def _run_async_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._connect_and_listen())

    async def _connect_and_listen(self):
        try:
            async with websockets.connect(self.server_url) as websocket:
                self.websocket = websocket
                self._connected_event.set()
                logger.info("Web Socket 연결 시도: %s", self.server_url)

                async for message in websocket:
                    await self._handle_message(message)

        except websockets.exceptions.ConnectionClosed as e:
            logger.warning("WebSocket 연결 종료: code=%s reason=%s", e.code, e.reason)
        except (websockets.exceptions.WebSocketException, OSError) as e:
            logger.warning("WebSocket 네트워크 오류: %s", e)
        except Exception:
            logger.exception("WebSocket 예기치 못한 오류")
        finally:
            self._connected_event.clear()
            self.websocket = None

    async def _handle_message(self, message: str):
        try:
            data = json.loads(message)
        except json.JSONDecodeError as e:
            logger.error("JSON 파싱 오류: %s", e)
            return

        msg_type = data.get("type")
        msg_data = data.get("data")
        logger.debug("서버 메시지 수신: %s", msg_type)

        # data 페이로드가 없거나 dict 형식이 아니면 명령으로 해석할 수 없다.
        if not isinstance(msg_data, dict):
            logger.warning("메시지 페이로드 형식 오류 (type=%s, data=%r)", msg_type, msg_data)
            return

        if msg_type == "command":
            target_x = msg_data.get("target_x")
            target_y = msg_data.get("target_y")
            mode = msg_data.get("mode")
            logger.info("이동 명령 도착: (%s, %s), 모드: %s", target_x, target_y, mode)
            await self.execute_move_command(target_x, target_y, mode)

        elif msg_type == "mode_change":
            new_mode = msg_data.get("mode")
            logger.info("모드 변경: %s", new_mode)
            await self.change_mode(new_mode)

        elif msg_type == "emergency_stop":
            reason = msg_data.get("reason", "알 수 없음")
            logger.info("긴급 정지: %s", reason)
            await self.execute_emergency_stop(reason)

    def _send_message(self, msg_type: str, data: Dict[Any, Any]):
        """동기 호출자에서 asyncio loop으로 send를 위탁한다.

        future.result()는 timeout/연결 종료/직렬화 등 다양한 사유로 raise할 수 있는데,
        한 호출 실패가 send_loop 스레드를 죽이지 않게 모두 잡아 로그만 남긴다.
        """
        if not self.connected or not self.websocket or not self.loop:
            logger.warning("Web Socket이 연결되지 않았습니다.")
            return

        message = {
            "type": msg_type,
            "data": data,
            "timestamp": int(time.time() * 1000),
        }

        try:
            future = asyncio.run_coroutine_threadsafe(
                self.websocket.send(json.dumps(message)),
                self.loop,
            )
            future.result(timeout=DEFAULT_SEND_TIMEOUT_SEC)
        except Exception as e:
            logger.warning("메시지 전송 실패 (type=%s): %s", msg_type, e)

    def send_position(self, x: float, y: float, angle: float):
        self.current_position = {"x": x, "y": y, "angle": angle}

        data = {
            "x": x,
            "y": y,
            "angle": angle,
            "timestamp": datetime.now().isoformat(),
        }

        self._send_message("position", data)
        logger.debug("위치 전송: x=%.2f, y=%.2f, angle=%.2f", x, y, angle)

    def send_status(self, status: dict):
        agv_status = {
            "id": self.agv_id,
            "name": self.agv_name,
            "connected": True,
            "last_update": datetime.now().isoformat(),
            "position": {
                "x": self.current_position["x"],
                "y": self.current_position["y"],
                "angle": self.current_position["angle"],
                "timestamp": datetime.now().isoformat(),
            },
            "mode": status.get("mode", "auto"),
            "state": self._convert_state(status),
            "speed": status.get("speed", self.speed),
            "battery": status.get("battery", self.battery),
            "current_path": status.get("current_path"),
            "target_pos": self._convert_target_pos(status.get("target_cell")),
            "target_enemy": status.get("target_enemy"),
            "detected_enemies": status.get("detected_enemies", []),
            "sensors": status.get("sensors", self._get_default_sensors()),
        }

        self._send_message("status", agv_status)
        logger.debug("상태 전송: mode=%s, state=%s", agv_status['mode'], agv_status['state'])

    def send_log(self, message: str, level: str = "info"):
        data = {"message": message, "level": level}
        self._send_message("log", data)
        logger.debug("로그 전송: %s", message)

    def send_target_found(self, enemy: dict):
        data = {"enemy": enemy}
        self._send_message("target_found", data)
        logger.info("타겟 발견: %s", enemy.get('name', 'Unknown'))

    def send_path_update(self, points: list, algorithm: str = "a_star"):
        total_length = 0.0
        for i in range(len(points) - 1):
            dx = points[i + 1]["x"] - points[i]["x"]
            dy = points[i + 1]["y"] - points[i]["y"]
            total_length += (dx ** 2 + dy ** 2) ** 0.5

        data = {
            "points": points,
            "length": total_length,
            "algorithm": algorithm,
            "created_at": datetime.now().isoformat(),
        }
        self._send_message("path_update", data)
        logger.debug("경로 전송: %d개 포인트, 길이=%.2fm", len(points), total_length)

    def _convert_state(self, status: dict) -> str:
        if status.get("moving"):
            return "moving"
        elif status.get("charging"):
            return "charging"
        elif status.get("searching"):
            return "searching"
        else:
            return "idle"

    def _convert_target_pos(self, target_cell) -> Optional[dict]:
        if target_cell is None:
            return None
        if isinstance(target_cell, (list, tuple)) and len(target_cell) >= 2:
            return {
                "x": float(target_cell[0]),
                "y": float(target_cell[1]),
                "angle": 0.0,
                "timestamp": datetime.now().isoformat(),
            }
        return None

    def _get_default_sensors(self) -> dict:
        """status에 sensors 필드가 누락된 경우의 기본값. 호출자가 매번 같은 dict를 만들지 않게 한다."""
        return {
            "front_distance": 0.0,
            "left_distance": 0.0,
            "right_distance": 0.0,
            "accel_x": 0.0,
            "accel_y": 0.0,
            "accel_z": 0.0,
            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,
            "camera_active": False,
            "objects_detected": 0,
        }

    def update_position(self, x: float, y: float, angle: float):
        self.current_position = {"x": x, "y": y, "angle": angle}

    def update_battery(self, battery: int):
        self.battery = battery

    def update_speed(self, speed: float):
        self.speed = speed

    def close(self):
        if self.websocket and self.loop:
            asyncio.run_coroutine_threadsafe(
                self.websocket.close(),
                self.loop,
            )
        self._connected_event.clear()
        logger.info("WebSocket 연결 종료")

    async def execute_move_command(self, target_x, target_y, mode):
        logger.info("이동: (%s, %s)", target_x, target_y)
        # TODO: 실제 이동 로직

    async def change_mode(self, new_mode):
        logger.info("모드 변경: %s", new_mode)
        # TODO: 모드 변경 로직

    async def execute_emergency_stop(self, reason):
        logger.info("긴급 정지: %s", reason)
        # TODO: 긴급 정지 로직
