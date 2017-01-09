from nose.tools import assert_equal
from openag_brain.software_modules.direct_controller import Direct

def test_direct_controller():
    direct = Direct()
    direct.set_point = 0
    assert_equal(-1, direct.update(1))
    assert_equal(1, direct.update(-1))

def test_multiplier():
    direct = Direct(-2)
    direct.set_point = 0
    assert_equal(2, direct.update(1))
    assert_equal(-2, direct.update(-1))
