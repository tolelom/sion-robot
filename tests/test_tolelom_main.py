"""tolelom.main 송신 루프 단위 테스트 (가짜 클라이언트, 네트워크 없음)."""
import queue
import threading
import time
from typing import Any, List, Tuple

import pytest

from tolelom import main as ws_main


class FakeClient:
    def __init__(self, connected: bool = True):
        self.connected = connected
        self.calls: List[Tuple[str, Any]] = []

    def send_position(self, x, y, angle):
        self.calls.append(("position", (x, y, angle)))

    def send_status(self, status):
        self.calls.append(("status", status))

    def send_path_update(self, points, algorithm="a_star"):
        self.calls.append(("path_update", (points, algorithm)))

    def send_log(self, message, level="info"):
        self.calls.append(("log", (message, level)))

    def send_target_found(self, enemy):
        self.calls.append(("target_found", enemy))

    def types(self) -> List[str]:
        return [t for t, _ in self.calls]


def _run_loop_for(client, seconds: float, **kwargs) -> None:
    stop = threading.Event()
    th = threading.Thread(target=ws_main.send_loop, args=(client, stop), kwargs=kwargs, daemon=True)
    th.start()
    time.sleep(seconds)
    stop.set()
    th.join(timeout=2.0)
    assert not th.is_alive()


class TestDrainOutbox:
    def test_sends_all_in_order(self):
        box: queue.Queue = queue.Queue()
        box.put(("log", {"message": "a", "level": "info"}))
        box.put(("target_found", {"enemy": {"name": "e"}}))
        client = FakeClient()
        assert ws_main.drain_outbox(client, box) == 2
        assert client.calls == [("log", ("a", "info")), ("target_found", {"name": "e"})]
        assert box.empty()

    def test_unknown_type_dropped(self):
        box: queue.Queue = queue.Queue()
        box.put(("bogus", {}))
        client = FakeClient()
        assert ws_main.drain_outbox(client, box) == 0
        assert client.calls == []

    def test_enqueue_helpers_use_module_outbox(self, monkeypatch):
        box: queue.Queue = queue.Queue()
        monkeypatch.setattr(ws_main, "outbox", box)
        ws_main.enqueue_log("hello", "warn")
        ws_main.enqueue_target_found({"name": "x"})
        assert box.get_nowait() == ("log", {"message": "hello", "level": "warn"})
        assert box.get_nowait() == ("target_found", {"enemy": {"name": "x"}})


class TestSendLoop:
    def test_disconnected_sends_nothing(self):
        client = FakeClient(connected=False)
        _run_loop_for(client, 0.1)
        assert client.calls == []

    def test_connected_sends_periodic_and_outbox(self):
        box: queue.Queue = queue.Queue()
        box.put(("log", {"message": "hi", "level": "info"}))
        client = FakeClient()
        _run_loop_for(client, 0.3, pos_period=0.02, status_period=10.0, path_period=10.0, box=box)
        types = client.types()
        # 첫 사이클에 세 종류가 모두 한 번 나가고, 이후 position만 주기적으로 나간다.
        assert types.count("status") == 1
        assert types.count("path_update") == 1
        assert types.count("position") >= 3
        assert ("log", ("hi", "info")) in client.calls

    @pytest.mark.slow
    def test_no_burst_after_reconnect(self, monkeypatch):
        monkeypatch.setattr(ws_main, "DISCONNECTED_POLL_SEC", 0.005)
        client = FakeClient(connected=False)
        stop = threading.Event()
        th = threading.Thread(
            target=ws_main.send_loop, args=(client, stop),
            kwargs=dict(pos_period=0.2, status_period=0.2, path_period=0.2, box=queue.Queue()),
            daemon=True,
        )
        th.start()
        time.sleep(1.0)  # 끊긴 동안 다섯 주기가 지나간다
        client.connected = True
        time.sleep(0.08)  # 재연결 직후, 한 주기(0.2s)보다 짧게
        stop.set()
        th.join(timeout=2.0)
        # 밀린 주기를 몰아서 보내지 않고 각 종류당 한 번만 보낸다.
        assert client.types().count("position") == 1
        assert client.types().count("status") == 1


@pytest.mark.parametrize("fn", [ws_main.get_status, ws_main.get_path_points])
def test_placeholder_sources_return_data(fn):
    assert fn()


def test_send_loop_survives_source_exception(monkeypatch):
    calls = {"n": 0}

    def flaky_position():
        calls["n"] += 1
        if calls["n"] == 1:
            raise RuntimeError("odometry not ready")
        return 1.0, 2.0, 0.0

    monkeypatch.setattr(ws_main, "get_position", flaky_position)
    client = FakeClient()
    _run_loop_for(client, 0.2, pos_period=0.02, status_period=10.0, path_period=10.0, box=queue.Queue())
    # 첫 호출이 실패해도 루프가 살아남아 이후 position을 보낸다.
    assert calls["n"] >= 2
    assert ("position", (1.0, 2.0, 0.0)) in client.calls
