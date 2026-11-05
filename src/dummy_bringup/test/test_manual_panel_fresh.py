"""执行真实面板fresh方法；缺失/非法年龄拒绝，不能用默认零伪造新鲜状态。"""
import ast
from pathlib import Path
from types import SimpleNamespace
import pytest

NOW = 10_000_000_000

def fresh(state, arrival=NOW):
    source=Path(__file__).resolve().parents[1]/'scripts/manual_joint_panel.py'
    tree=ast.parse(source.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='JointPanel')
    node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='fresh')
    scope={'time':SimpleNamespace(monotonic_ns=lambda:NOW)}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),scope)
    return scope['fresh'](SimpleNamespace(last_state=state,last_state_ns=arrival))

@pytest.mark.parametrize('ages',[None,[],[0]*5,[0]*7,[-1]*6,[False]*6,[0.0]*6,[float('nan')]*6])
def test_missing_or_invalid_can_age_is_not_fresh(ages):
    value={} if ages is None else {'can_rx_age_us':ages}
    assert fresh(value) is False

def test_real_zero_and_combined_age_bound():
    assert fresh({'can_rx_age_us':(0,)*6})
    assert fresh({'can_rx_age_us':[1000]*6},NOW-249_000_000)
    assert not fresh({'can_rx_age_us':[1001]*6},NOW-249_000_000)
    assert not fresh({'can_rx_age_us':[0]*5+[251000]})
    assert not fresh({'can_rx_age_us':[0]*6},NOW+1)
    assert not fresh(None)
