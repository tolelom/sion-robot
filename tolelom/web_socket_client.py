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


def resolve_backend_ws_url(explicit: Optional[str] = None) -> str:
    """server_url 결정: 명시 인자 > SION_BACKEND_WS_URL 환경변수 > localhost 기본값."""
    if explicit:
        return explicit
    return os.environ.get("SION_BACKEND_WS_URL", DEFAULT_BACKEND_WS_URL)

"""
AGV WebSocket 클라이언트

LLM 후순위 
지금 통신 -> 이동 -> 영상 찍는게 1순위
통신 안되면 그냥 이동 -> 영상 

"""


class AGVWebSocketClient:
    def __init__(self, server_url: Optional[str] = None):
        self.server_url = resolve_backend_ws_url(server_url)
        self.websocket: Optional[websockets.WebSocketClientProtocol] = None
        self.connected = False
        self.loop = None  # ?
        self.receive_thread = None  # ?

        # AGV 상태 저장
        self.agv_id = "sion-001"
        self.agv_name = "Sion AGV"
        self.current_position = {"x": 0.0, "y": 0.0, "angle": 0.0}
        self.battery = 100
        self.speed = 0.0

    def start(self):
        # WebSocket 연결 시작 (별도 스레드에서 실행됨)
        self.receive_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.receive_thread.start()

        while not self.connected:
            time.sleep(0.1)
        logger.info("Web Socket 연결 완료: %s", self.server_url)

    # 비동기 이벤트 루프 실행
    def _run_async_loop(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._connect_and_listen())

    # Web Socket 연결 및 메시지 수신
    async def _connect_and_listen(self):
        try:
            async with websockets.connect(self.server_url) as websocket:
                self.websocket = websocket
                self.connected = True
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
            self.connected = False
            self.websocket = None

    # 서버에서 받은 메시지 처리 함수
    async def _handle_message(self, message: str):
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            msg_data = data.get("data")
            logger.debug("서버 메시지 수신: %s", msg_type)

            if msg_type == "command":
                target_x = msg_data.get("target_x")
                target_y = msg_data.get("target_y")
                mode = msg_data.get("mode")
                logger.info("이동 명령 도착: (%s, %s), 모드: %s", target_x, target_y, mode)

                # TODO: 실제 이동 로직을 여기에 추가하세요
                await self.execute_move_command(target_x, target_y, mode)

            elif msg_type == "mode_change":
                new_mode = msg_data.get("mode")
                logger.info("모드 변경: %s", new_mode)

                # TODO: 모드 변경 로직을 여기에 추가하세요
                await self.change_mode(new_mode)

            elif msg_type == "emergency_stop":
                reason = msg_data.get("reason", "알 수 없음")  # 이 데이터가 백엔드에서 보내는 지 확인할 것
                logger.info("긴급 정지: %s", reason)

                # TODO: 긴급 정지 로직을 여기에 추가하세요
                await self.execute_emergency_stop(reason)

        except json.JSONDecodeError as e:
            logger.error("JSON 파싱 오류: %s", e)

    # 메시지 전송 함수 (동기 함수에서 호출 가능)
    def _send_message(self, msg_type: str, data: Dict[Any, Any]):
        if not self.connected or not self.websocket:
            logger.warning("Web Socket이 연결되지 않았습니다.")
            return

        message = {
            "type": msg_type,
            "data": data,
            "timestamp": int(time.time() * 1000)
        }

        # 비동기 전송을 동기로 래핑?
        future = asyncio.run_coroutine_threadsafe(
            self.websocket.send(json.dumps(message)),
            self.loop
        )
        future.result(timeout=1.0)

    def send_position(self, x: float, y: float, angle: float):
        self.current_position = {"x": x, "y": y, "angle": angle}

        data = {
            "x": x,
            "y": y,
            "angle": angle,
            "timestamp": datetime.now().isoformat()
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
                "timestamp": datetime.now().isoformat()
            },
            "mode": status.get("mode", "auto"),
            "state": self._convert_state(status),
            "speed": status.get("speed", self.speed),
            "battery": status.get("battery", self.battery),
            "current_path": status.get("current_path"),
            "target_pos": self._convert_target_pos(status.get("target_cell")),
            "target_enemy": status.get("target_enemy"),
            "detected_enemies": status.get("detected_enemies", []),
            "sensors": status.get("sensors", self._get_default_sensors())
        }

        self._send_message("status", agv_status)
        logger.debug("상태 전송: mode=%s, state=%s", agv_status['mode'], agv_status['state'])

    def send_log(self, message: str, level: str = "info"):
        data = {
            "message": message,
            "level": level
        }

        self._send_message("log", data)
        logger.debug("로그 전송: %s", message)

    def send_target_found(self, enemy: dict):
        """타겟 발견 알림 전송"""
        data = {
            "enemy": enemy
        }
        self._send_message("target_found", data)
        logger.info("타겟 발견: %s", enemy.get('name', 'Unknown'))

    def send_path_update(self, points: list, algorithm: str = "a_star"):
        """경로 업데이트 전송"""
        # 경로 길이 계산
        total_length = 0.0
        for i in range(len(points) - 1):
            dx = points[i + 1]["x"] - points[i]["x"]
            dy = points[i + 1]["y"] - points[i]["y"]
            total_length += (dx ** 2 + dy ** 2) ** 0.5

        data = {
            "points": points,
            "length": total_length,
            "algorithm": algorithm,
            "created_at": datetime.now().isoformat()
        }
        self._send_message("path_update", data)
        logger.debug("경로 전송: %d개 포인트, 길이=%.2fm", len(points), total_length)

    def _convert_state(self, status: dict) -> str:
        """상태 변환 (moving -> state)"""
        if status.get("moving"):
            return "moving"
        elif status.get("charging"):
            return "charging"
        elif status.get("searching"):
            return "searching"
        else:
            return "idle"

    def _convert_target_pos(self, target_cell) -> Optional[dict]:
        """목표 셀을 위치 데이터로 변환"""
        if target_cell is None:
            return None

        # target_cell이 [gx, gy] 형식이라고 가정
        # 실제 좌표로 변환 필요 (그리드 -> 미터)
        if isinstance(target_cell, (list, tuple)) and len(target_cell) >= 2:
            return {
                "x": float(target_cell[0]),
                "y": float(target_cell[1]),
                "angle": 0.0,
                "timestamp": datetime.now().isoformat()
            }
        return None

    # 아마 안쓸 것 같은 함수
    def _get_default_sensors(self) -> dict:
        """기본 센서 데이터"""
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
            "objects_detected": 0
        }

    def update_position(self, x: float, y: float, angle: float):
        """내부 위치 업데이트"""
        self.current_position = {"x": x, "y": y, "angle": angle}

    def update_battery(self, battery: int):
        """배터리 업데이트"""
        self.battery = battery

    def update_speed(self, speed: float):
        """속도 업데이트"""
        self.speed = speed

    def close(self):
        """연결 종료"""
        if self.websocket:
            asyncio.run_coroutine_threadsafe(
                self.websocket.close(),
                self.loop
            )
        self.connected = False
        logger.info("WebSocket 연결 종료")

    async def execute_move_command(self, target_x, target_y, mode):
        """이동 명령 실행"""
        logger.info("이동: (%s, %s)", target_x, target_y)
        # TODO: 실제 이동 로직

    async def change_mode(self, new_mode):
        """모드 변경"""
        logger.info("모드 변경: %s", new_mode)
        # TODO: 모드 변경 로직

    async def execute_emergency_stop(self, reason):
        """긴급 정지"""
        logger.info("긴급 정지: %s", reason)
        # TODO: 긴급 정지 로직

