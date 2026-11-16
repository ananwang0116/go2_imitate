import os
from pathlib import Path
import shutil
import subprocess

import pytest


def test_rotated_view_pointer_controls_and_pose_coordinates():
    node = os.environ.get('GO2_TEST_NODE') or shutil.which('node')
    if not node:
        pytest.skip('Node required for JavaScript interaction checks (also verified in the desktop browser)')
    root = Path(__file__).resolve().parents[1]
    subprocess.run([node, str(Path(__file__).with_suffix('.cjs')), str(root / 'web/index.html')], check=True, timeout=10)
