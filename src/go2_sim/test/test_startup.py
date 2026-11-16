"""Check controller dependency/failure progression without a second control owner."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from go2_sim.startup import SimStartup, CONTROLLERS


def adapter(tmp_path, monkeypatch):
    monkeypatch.setenv('GO2_RUN_DIR',str(tmp_path))
    obj=object.__new__(SimStartup)
    obj.index=0;obj.configuring=False;obj.future=None
    obj.started_at=100.;obj.failed=obj.finished=False
    obj.get_logger=Mock(return_value=Mock())
    obj.load=Mock();obj.configure=Mock()
    obj.load.service_is_ready.return_value=True
    obj.configure.service_is_ready.return_value=True
    for client in (obj.load,obj.configure):
        client.call_async.return_value=Mock(done=lambda:True,result=lambda:NS(ok=True))
    return obj


def test_controllers_are_loaded_then_configured_in_order(tmp_path,monkeypatch):
    obj=adapter(tmp_path,monkeypatch)
    calls=[]
    obj.load.call_async.side_effect=lambda req:(calls.append(('load',req.name)) or Mock(done=lambda:True,result=lambda:NS(ok=True)))
    obj.configure.call_async.side_effect=lambda req:(calls.append(('configure',req.name)) or Mock(done=lambda:True,result=lambda:NS(ok=True)))
    with patch('go2_sim.startup.time.monotonic',return_value=101.):
        for _ in range(12):obj.tick()
    assert calls==[(op,name) for name,_ in CONTROLLERS for op in ('load','configure')]
    assert obj.index==3 and not obj.failed
    import json
    rows=[json.loads(line) for line in (tmp_path/'startup.jsonl').read_text().splitlines()]
    assert [r['stage'] for r in rows]==[stage for _,stage in CONTROLLERS]


def test_rejection_stops_without_next_controller_or_standing(tmp_path,monkeypatch):
    obj=adapter(tmp_path,monkeypatch)
    obj.load.call_async.return_value=Mock(done=lambda:True,result=lambda:NS(ok=False))
    with patch('go2_sim.startup.time.monotonic',return_value=101.):
        obj.tick();obj.tick()
    assert obj.failed and obj.finished and obj.index==0
    obj.configure.call_async.assert_not_called()
    assert not (tmp_path/'startup.jsonl').exists()


def test_setup_deadline_fails_without_dispatch(tmp_path,monkeypatch):
    obj=adapter(tmp_path,monkeypatch)
    with patch('go2_sim.startup.time.monotonic',return_value=131.):obj.tick()
    assert obj.failed and obj.finished
    obj.load.call_async.assert_not_called()
