import pytest
import rclpy
from rclpy._rclpy_pybind11 import RCLError

from go2_common import node as node_mod


class Dummy:
    destroyed = False

    def destroy_node(self):
        Dummy.destroyed = True


def run_spin(monkeypatch, ok):
    Dummy.destroyed = False
    monkeypatch.setattr(rclpy, 'init', lambda: None)
    monkeypatch.setattr(rclpy, 'ok', lambda: ok)
    monkeypatch.setattr(rclpy, 'shutdown', lambda: None)

    def boom(node):
        raise RCLError('Failed to publish: publisher\'s context is invalid')
    monkeypatch.setattr(rclpy, 'spin', boom)
    node_mod.spin(Dummy)


def test_publish_error_after_shutdown_is_quiet_and_the_node_is_still_destroyed(monkeypatch):
    run_spin(monkeypatch, ok=False)
    assert Dummy.destroyed


def test_the_same_error_while_the_context_is_alive_is_not_swallowed(monkeypatch):
    with pytest.raises(RCLError):
        run_spin(monkeypatch, ok=True)
    assert Dummy.destroyed
