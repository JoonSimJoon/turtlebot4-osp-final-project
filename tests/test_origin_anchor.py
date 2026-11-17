import math
import pytest
from tb4_mission.origin_anchor import inverse_planar_pose


@pytest.mark.parametrize('pose', [(0., 0., 0.), (3., -2., 0.), (2., 1., math.pi / 2), (-4., 5., -2.7)])
def test_initial_base_pose_becomes_zero(pose):
    x, y, yaw = pose
    tx, ty, angle = inverse_planar_pose(x, y, yaw)
    assert math.cos(angle) * x - math.sin(angle) * y + tx == pytest.approx(0., abs=1e-12)
    assert math.sin(angle) * x + math.cos(angle) * y + ty == pytest.approx(0., abs=1e-12)
    assert yaw + angle == pytest.approx(0.)
