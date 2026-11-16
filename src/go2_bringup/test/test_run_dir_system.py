import os

from go2_bringup import run_dir


def test_system_snapshot_has_cores_and_no_dds_values(monkeypatch):
    monkeypatch.setenv('GO2_DDS_CONFIG', '/secret/path/cyclonedds.xml')
    monkeypatch.setenv('CYCLONEDDS_URI', '<CycloneDDS><Security>KEY</Security></CycloneDDS>')
    text = run_dir.system_snapshot()
    assert 'cpus_online:' in text and 'loadavg:' in text and 'kernel:' in text
    assert 'dds env set: GO2_DDS_CONFIG,CYCLONEDDS_URI' in text
    assert 'secret' not in text and 'KEY' not in text
    assert text.endswith('\n') and os.cpu_count() >= 1
