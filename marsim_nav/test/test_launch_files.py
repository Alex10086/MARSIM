"""启动文件必须能被加载并构建出 LaunchDescription。

这是一条**回归防线**，为一个真实事故补的：Task 7 给
sim_navigation.launch.py 加了 Node(...) 却漏了 `from launch_ros.actions
import Node`，Task 8 又引用了不存在的 map_path —— 于是 `ros2 launch`
直接报 NameError，整个系统起不来。而当时的验证只查「构建通过 +
可执行文件存在」，两条都绿，问题一路溜到端到端才被发现。

launch 文件是 Python，语法错误和未定义名字只有**执行**才会暴露。
所以这里真的 import 它并调用 generate_launch_description()。
"""
import importlib.util
from pathlib import Path

import pytest

LAUNCH_DIR = Path(__file__).resolve().parent.parent / 'launch'


def _load(name):
    path = LAUNCH_DIR / f'{name}.launch.py'
    assert path.exists(), f'缺少启动文件 {path}'
    spec = importlib.util.spec_from_file_location(f'test_launch_{name}', str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize('name', ['sim_navigation', 'navigation', 'tf'])
def test_launch_file_loads_and_describes(name):
    mod = _load(name)
    ld = mod.generate_launch_description()
    assert len(ld.entities) > 0


def test_sim_navigation_declares_both_altitude_nodes():
    """中途升降的两个节点必须真的被挂进启动图。

    这条也是回归防线：它们一度写进了文件却因 NameError 根本没机会运行。
    """
    mod = _load('sim_navigation')
    ld = mod.generate_launch_description()
    names = []
    for e in ld.entities:
        for attr in ('node_executable', 'executable', '_Node__node_executable'):
            v = getattr(e, attr, None)
            if v is not None:
                names.append(str(v))
    blob = ' '.join(names)
    assert 'merge_cmd_vel' in blob, f'启动图里没有 merge_cmd_vel：{blob!r}'
    assert 'alt_profile' in blob, f'启动图里没有 alt_profile：{blob!r}'
