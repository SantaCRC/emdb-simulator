"""Self-check for SceneLoader's sim-thread queue.

env.reset()/env.step()/env.render() must only ever run on the thread that
created the MuJoCo/GLFW render context (see run_sim_loop() and
_run_on_sim_thread() in scene_loader.py, and
docs/source/howto/run_simulator.md). This exercises just that hand-off --
job submitted from a "ROS callback" thread, executed on the "sim" thread,
result/exception propagated back -- without needing rclpy/robosuite
installed, since _run_on_sim_thread()/run_sim_loop() only touch
self._sim_queue.
"""
import threading
import types

import pytest

pytest.importorskip("rclpy")
from emdb_simulator.core.scene_loader import SceneLoader  # noqa: E402


def _fake_node():
    node = types.SimpleNamespace()
    node._sim_queue = __import__("queue").Queue()
    node._shutdown = False
    node._run_on_sim_thread = SceneLoader._run_on_sim_thread.__get__(node)
    return node


def _rclpy_ok_stub(node):
    return not node._shutdown


def test_job_runs_on_sim_thread_and_returns_result(monkeypatch):
    node = _fake_node()
    monkeypatch.setattr(
        "emdb_simulator.core.scene_loader.rclpy.ok", lambda: _rclpy_ok_stub(node)
    )

    sim_thread_id = []
    sim_thread = threading.Thread(
        target=lambda: SceneLoader.run_sim_loop(node), daemon=True
    )
    sim_thread.start()

    result = node._run_on_sim_thread(lambda: (sim_thread_id.append(threading.get_ident()), 42)[1])

    assert result == 42
    assert sim_thread_id == [sim_thread.ident]
    assert threading.get_ident() != sim_thread.ident

    node._shutdown = True
    sim_thread.join(timeout=2)
    assert not sim_thread.is_alive()


def test_exception_propagates_to_caller(monkeypatch):
    node = _fake_node()
    monkeypatch.setattr(
        "emdb_simulator.core.scene_loader.rclpy.ok", lambda: _rclpy_ok_stub(node)
    )
    sim_thread = threading.Thread(
        target=lambda: SceneLoader.run_sim_loop(node), daemon=True
    )
    sim_thread.start()

    def boom():
        raise RuntimeError("env.reset() failed")

    with pytest.raises(RuntimeError, match="env.reset\\(\\) failed"):
        node._run_on_sim_thread(boom)

    node._shutdown = True
    sim_thread.join(timeout=2)


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))
