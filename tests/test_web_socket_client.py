"""tolelom.web_socket_client 단위 테스트.

실제 네트워크는 쓰지 않는다. 전송은 `_send_message`를 가로채거나 가짜 websocket을 끼워
검증하고, 수신은 `_handle_message`에 JSON 문자열을 직접 넣어 검증한다.
"""
import asyncio
import json
import threading
from typing import Any, Dict, List, Tuple
from unittest.mock import AsyncMock

import pytest

from tolelom import web_socket_client as wsc
from tolelom.web_socket_client import AGVWebSocketClient, resolve_backend_ws_url


@pytest.fixture
def client() -> AGVWebSocketClient:
    return AGVWebSocketClient("ws://test.invalid/websocket/agv")


@pytest.fixture
def sent(client, monkeypatch) -> List[Tuple[str, Dict[str, Any]]]:
    """client._send_message 호출을 (type, data) 리스트로 기록한다."""
    records: List[Tuple[str, Dict[str, Any]]] = []
    monkeypatch.setattr(client, "_send_message", lambda t, d: records.append((t, d)))
    return records


class _LoopThread:
    """백그라운드 스레드에서 asyncio loop를 돌린다(run_coroutine_threadsafe 검증용)."""

    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self._th = threading.Thread(target=self.loop.run_forever, daemon=True)
        self._th.start()

    def stop(self):
        self.loop.call_soon_threadsafe(self.loop.stop)
        self._th.join(timeout=2.0)
        self.loop.close()


@pytest.fixture
def loop_thread():
    lt = _LoopThread()
    yield lt
    lt.stop()


class _FakeWebSocket:
    def __init__(self):
        self.sent: List[str] = []
        self.closed = False

    async def send(self, message: str) -> None:
        self.sent.append(message)

    async def close(self) -> None:
        self.closed = True


# ---------------------------------------------------------------------------
# URL 결정
# ---------------------------------------------------------------------------

class TestResolveBackendWsUrl:
    def test_explicit_wins(self, monkeypatch):
        monkeypatch.setenv("SION_BACKEND_WS_URL", "ws://env/agv")
        assert resolve_backend_ws_url("ws://explicit/agv") == "ws://explicit/agv"

    def test_env_used_when_no_explicit(self, monkeypatch):
        monkeypatch.setenv("SION_BACKEND_WS_URL", "ws://env/agv")
        assert resolve_backend_ws_url() == "ws://env/agv"

    def test_default_when_nothing(self, monkeypatch):
        monkeypatch.delenv("SION_BACKEND_WS_URL", raising=False)
        assert resolve_backend_ws_url() == wsc.DEFAULT_BACKEND_WS_URL

    def test_client_uses_resolved_url(self, monkeypatch):
        monkeypatch.setenv("SION_BACKEND_WS_URL", "ws://env/agv")
        assert AGVWebSocketClient().server_url == "ws://env/agv"


# ---------------------------------------------------------------------------
# 송신 메시지 형식 (백엔드와의 계약)
# ---------------------------------------------------------------------------

class TestOutgoingPayloads:
    def test_send_position(self, client, sent):
        client.send_position(1.0, 2.0, 0.5)
        (typ, data), = sent
        assert typ == "position"
        assert set(data) == {"x", "y", "angle", "timestamp"}
        assert (data["x"], data["y"], data["angle"]) == (1.0, 2.0, 0.5)
        assert client.current_position == {"x": 1.0, "y": 2.0, "angle": 0.5}

    def test_send_status_fields(self, client, sent):
        client.update_position(3.0, 4.0, 1.0)
        client.send_status({"mode": "manual", "moving": True, "target_cell": [5, 6]})
        (typ, data), = sent
        assert typ == "status"
        assert set(data) == {
            "id", "name", "connected", "last_update", "position", "mode", "state",
            "speed", "battery", "current_path", "target_pos", "target_enemy",
            "detected_enemies", "sensors",
        }
        assert data["id"] == client.agv_id
        assert data["connected"] is True
        assert data["mode"] == "manual"
        assert data["state"] == "moving"
        assert data["position"]["x"] == 3.0 and data["position"]["y"] == 4.0
        assert data["target_pos"]["x"] == 5.0 and data["target_pos"]["y"] == 6.0
        assert data["detected_enemies"] == []

    def test_send_status_defaults(self, client, sent):
        client.update_battery(42)
        client.update_speed(0.3)
        client.send_status({})
        _, data = sent[0]
        assert data["mode"] == "auto"
        assert data["state"] == "idle"
        assert data["battery"] == 42
        assert data["speed"] == 0.3
        assert data["target_pos"] is None
        assert data["sensors"]["camera_active"] is False

    @pytest.mark.parametrize("status,expected", [
        ({"moving": True, "charging": True}, "moving"),
        ({"charging": True, "searching": True}, "charging"),
        ({"searching": True}, "searching"),
        ({}, "idle"),
    ])
    def test_state_priority(self, client, sent, status, expected):
        client.send_status(status)
        assert sent[0][1]["state"] == expected

    @pytest.mark.parametrize("target", ["bad", [1], 7])
    def test_invalid_target_cell_becomes_none(self, client, sent, target):
        client.send_status({"target_cell": target})
        assert sent[0][1]["target_pos"] is None

    def test_send_log(self, client, sent):
        client.send_log("hello", "warn")
        assert sent == [("log", {"message": "hello", "level": "warn"})]

    def test_send_target_found(self, client, sent):
        client.send_target_found({"name": "dummy"})
        assert sent == [("target_found", {"enemy": {"name": "dummy"}})]

    def test_send_path_update_length(self, client, sent):
        pts = [{"x": 0.0, "y": 0.0}, {"x": 3.0, "y": 4.0}, {"x": 3.0, "y": 5.0}]
        client.send_path_update(pts)
        (typ, data), = sent
        assert typ == "path_update"
        assert data["points"] is pts
        assert data["length"] == pytest.approx(6.0)
        assert data["algorithm"] == "a_star"
        assert "created_at" in data

    def test_send_path_update_empty(self, client, sent):
        client.send_path_update([])
        assert sent[0][1]["length"] == 0.0


# ---------------------------------------------------------------------------
# _send_message: 연결 상태 가드와 envelope
# ---------------------------------------------------------------------------

class TestSendMessage:
    def test_not_connected_is_noop(self, client):
        # 연결 전 호출이 예외 없이 무시되어야 한다.
        client._send_message("log", {"message": "x"})

    def test_envelope_sent_through_loop(self, client, loop_thread):
        ws = _FakeWebSocket()
        client.websocket = ws
        client.loop = loop_thread.loop
        client._connected_event.set()

        client._send_message("log", {"message": "hi", "level": "info"})

        assert len(ws.sent) == 1
        msg = json.loads(ws.sent[0])
        assert msg["type"] == "log"
        assert msg["data"] == {"message": "hi", "level": "info"}
        assert isinstance(msg["timestamp"], int)

    def test_send_failure_is_swallowed(self, client, loop_thread):
        ws = _FakeWebSocket()

        async def boom(_msg):
            raise RuntimeError("send failed")

        ws.send = boom
        client.websocket = ws
        client.loop = loop_thread.loop
        client._connected_event.set()

        client._send_message("log", {"message": "hi"})  # raise하지 않아야 한다


# ---------------------------------------------------------------------------
# 수신 메시지 처리
# ---------------------------------------------------------------------------

def _handle(client: AGVWebSocketClient, payload) -> None:
    raw = payload if isinstance(payload, str) else json.dumps(payload)
    asyncio.run(client._handle_message(raw))


class TestHandleMessage:
    @pytest.fixture(autouse=True)
    def _mock_handlers(self, client, monkeypatch):
        for name in ("execute_move_command", "change_mode", "execute_emergency_stop"):
            monkeypatch.setattr(client, name, AsyncMock())

    def test_command(self, client):
        _handle(client, {"type": "command", "data": {"target_x": 1, "target_y": 2, "mode": "manual"}})
        client.execute_move_command.assert_awaited_once_with(1, 2, "manual")

    def test_mode_change(self, client):
        _handle(client, {"type": "mode_change", "data": {"mode": "auto"}})
        client.change_mode.assert_awaited_once_with("auto")

    def test_emergency_stop_default_reason(self, client):
        _handle(client, {"type": "emergency_stop", "data": {}})
        client.execute_emergency_stop.assert_awaited_once_with("알 수 없음")

    def test_invalid_json_ignored(self, client):
        _handle(client, "{not json")
        client.execute_move_command.assert_not_awaited()

    @pytest.mark.parametrize("data", [None, "str", [1, 2]])
    def test_non_dict_data_ignored(self, client, data):
        _handle(client, {"type": "command", "data": data})
        client.execute_move_command.assert_not_awaited()

    def test_unknown_type_ignored(self, client):
        _handle(client, {"type": "whatever", "data": {}})
        client.execute_move_command.assert_not_awaited()
        client.change_mode.assert_not_awaited()
        client.execute_emergency_stop.assert_not_awaited()


# ---------------------------------------------------------------------------
# start(): 연결 timeout
# ---------------------------------------------------------------------------

class TestStart:
    def test_timeout_returns_false(self, client, monkeypatch):
        monkeypatch.setattr(client, "_run_async_loop", lambda: None)
        assert client.start(timeout=0.01) is False
        assert client.connected is False

    def test_connected_returns_true(self, client, monkeypatch):
        monkeypatch.setattr(client, "_run_async_loop", client._connected_event.set)
        assert client.start(timeout=1.0) is True
        assert client.connected is True


# ---------------------------------------------------------------------------
# 회귀: 잘못된 메시지 한 건이 연결을 끊으면 안 된다
# ---------------------------------------------------------------------------

class TestMalformedMessageDoesNotBreakConnection:
    @pytest.mark.parametrize("raw", ["[1, 2]", "42", "\"text\"", "null"])
    def test_non_object_json_ignored(self, client, raw):
        # 이전에는 data.get에서 AttributeError가 나 수신 루프가 깨지고 재연결했다.
        asyncio.run(client._handle_message(raw))

    def test_handler_exception_isolated(self, client, monkeypatch):
        monkeypatch.setattr(client, "execute_move_command", AsyncMock(side_effect=RuntimeError("boom")))
        raw = json.dumps({"type": "command", "data": {"target_x": 1, "target_y": 2}})
        asyncio.run(client._handle_message_safely(raw))  # raise하지 않아야 한다
        client.execute_move_command.assert_awaited_once()


# ---------------------------------------------------------------------------
# 재연결 루프와 close()
# ---------------------------------------------------------------------------

class TestReconnectAndClose:
    def test_close_interrupts_backoff(self, client, monkeypatch):
        attempts = []

        def failing_connect(url):
            attempts.append(url)
            raise OSError("connection refused")

        monkeypatch.setattr(wsc.websockets, "connect", failing_connect)
        # 백오프를 길게 잡아, close()가 대기를 깨우지 못하면 테스트가 확실히 실패하게 한다.
        monkeypatch.setattr(wsc, "INITIAL_RECONNECT_BACKOFF_SEC", 60.0)

        assert client.start(timeout=0.2) is False
        assert attempts, "최소 한 번은 연결을 시도해야 한다"

        client.close(timeout=2.0)
        assert not client.receive_thread.is_alive()
        assert client.loop is None
        assert len(attempts) == 1

    def test_close_before_start_is_safe(self, client):
        client.close()
        assert client.connected is False

    def test_start_twice_does_not_spawn_second_thread(self, client, monkeypatch):
        gate = threading.Event()
        monkeypatch.setattr(client, "_run_async_loop", lambda: gate.wait(2.0))
        client.start(timeout=0.01)
        first = client.receive_thread
        client.start(timeout=0.01)
        assert client.receive_thread is first
        gate.set()


class _FakeConnection:
    """websockets.connect(...)가 돌려주는 async context manager 흉내."""

    def __init__(self, incoming: List[str]):
        self._incoming = list(incoming)
        self._closed = None  # loop 안에서 생성
        self.sent: List[str] = []

    async def __aenter__(self):
        self._closed = asyncio.Event()
        return self

    async def __aexit__(self, *exc):
        return False

    def __aiter__(self):
        return self._iter()

    async def _iter(self):
        for m in self._incoming:
            yield m
        await self._closed.wait()  # 서버가 메시지를 더 안 보내고 연결만 유지하는 상태

    async def send(self, message: str) -> None:
        self.sent.append(message)

    async def close(self) -> None:
        self._closed.set()


class TestConnectedLifecycle:
    def test_receive_send_close(self, client, monkeypatch):
        conn = _FakeConnection([json.dumps({"type": "mode_change", "data": {"mode": "manual"}})])
        monkeypatch.setattr(wsc.websockets, "connect", lambda url: conn)
        received = threading.Event()

        async def on_mode(mode):
            assert mode == "manual"
            received.set()

        monkeypatch.setattr(client, "change_mode", on_mode)

        assert client.start(timeout=1.0) is True
        assert received.wait(1.0)

        client.send_log("hi")
        assert json.loads(conn.sent[0])["type"] == "log"

        client.close(timeout=2.0)
        assert not client.receive_thread.is_alive()
        assert client.connected is False
