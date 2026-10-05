"""Flight-controller integration checks with deterministic telemetry."""
import copy
import math
from unittest.mock import Mock

import pytest

from agrotechsimapi import HighLevelClient, HighLevelSimClient
from agrotechsimapi.control import CascadedController, Sample
from agrotechsimapi.calibration.store import runtime_pids


def client_at(x=7.0,y=-11.0,z=4.0,yaw=0.5):
    c = HighLevelClient()
    c._latest_sample = Sample(0,x,y,z,yaw,0,0,0,0,0)
    c._z_bias = z
    c._altitude = 0
    c._cascade.z_bias = z
    c._target_yaw = yaw
    c._target_position = (x,y)
    c._motors_locked = False
    c._armed_flag = True
    c._althold_flag = True
    return c


@pytest.mark.parametrize('mode',['velocity','position'])
def test_client_produces_calibrator_commands_for_same_samples(mode):
    c = client_at()
    assert HighLevelClient is HighLevelSimClient
    reference = CascadedController(copy.deepcopy(c._control_config),runtime_pids(c.calibration))
    reference.z_bias = 4.0
    c._control_mode = mode
    c._target_height = 1.0
    c._target_velocity = (0.15,-0.1)
    c._target_position = (7.25,-11.5)
    target = {'height':1,'height_relative':True,'yaw':.5,
              'velocity':c._target_velocity,'position':c._target_position}
    for tick in range(180):
        t=tick/55
        sample=Sample(t,7+.02*t,-11-.03*t,5+.01*math.sin(t),.5,
                      0,0,.02*math.sin(t),-.03*math.cos(t),.01*math.cos(t))
        expected=reference.step(sample,mode,target)
        c._latest_sample=sample
        c._control_step()
        assert c._rpy_vel_data == (expected['rc_roll'],expected['rc_pitch'],expected['rc_yaw'])
        assert c._throttle_data == expected['rc_throttle']
        assert c.get_control_telemetry()['target_ax_body'] == expected['target_ax_body']


def test_world_velocity_transform_and_no_raw_pwm_bypass():
    c=client_at(yaw=math.pi/2)
    c.set_velocity_xy(.15,0,frame='odom')
    assert c._rpy_vel_data == (1500,1500,1500)
    c._control_step()
    assert abs(c._cascade.target_body_velocity[0]) < 1e-9
    assert c._cascade.target_body_velocity[1] == pytest.approx(.15)
    with pytest.raises(ValueError):
        c.set_velocity_xy(1,2,'invalid')


def test_start_origin_yaw_relative_height_and_step_descent():
    c=HighLevelClient()
    yaw=.7
    kin={'location':[55,33,10], 'orientation':[0,0,-math.sin(yaw/2),math.cos(yaw/2)],
         'linear_velocity':[0,0,0]}
    c._kinematics_client=Mock()
    c._kinematics_client.get_kinametics_data.return_value=kin
    c.sim_kinematics_callback()
    assert c._target_position == (55,33)
    assert c._target_yaw == pytest.approx(yaw)
    assert c._z_bias == 10
    c._motors_locked=False
    c._altitude=2
    c._latest_sample=Sample(1,55,33,12,yaw,0,0,0,0,0)
    c.set_target_height(1)
    assert c._target_height == pytest.approx(1.95)
    c._update_descent(Sample(1.01,55,33,12,yaw,0,0,0,0,0))
    assert c._target_height == pytest.approx(1.95)
    c._update_descent(Sample(2,55,33,11.95,yaw,0,0,0,0,-.02))
    assert c._target_height == pytest.approx(1.90)
    c.set_target_height(3)
    assert c._target_height == 3 and c._descent_goal is None


def test_compatibility_callbacks_do_not_integrate_same_sample_again():
    c=client_at()
    c._target_height=1
    c.height_callback()
    integral=c._pid_height.integral
    output=c._rpy_vel_data
    c.velocity_callback(); c.yaw_callback(); c.position_callback()
    assert c._pid_height.integral == integral
    assert c._rpy_vel_data == output
    c.lock_motors()
    c._latest_sample=Sample(1,50,60,0,2,0,0,5,5,0)
    c._control_step()
    assert c._pid_height.integral == integral


def test_explicit_pid_override_and_profile_path(tmp_path):
    from agrotechsimapi.calibration.store import CalibrationStore
    store=CalibrationStore(tmp_path)
    path=store.export('default',tmp_path/'data.json')
    c=HighLevelClient(calibration_path=path,pid_accel_roll={'kp':.123})
    assert c._pid_accel_roll.kp == .123
    assert c._pid_accel_roll.ki == store.load()['pids']['pid_accel_roll']['ki']
    assert c._cascade.pids['pid_accel_roll'] is c._pid_accel_roll


def test_client_uses_edu_ext_preset_by_default():
    c = HighLevelClient()
    assert c.calibration['name'] == 'edu-ext'


def test_connect_reports_transport_failure_before_sending_rc(monkeypatch):
    transmitter = Mock(is_connect=False)
    factory = Mock(return_value=transmitter)
    control = Mock()
    monkeypatch.setattr('agrotechsimapi.high_level_client.TCPTransmitter', factory)
    monkeypatch.setattr('agrotechsimapi.high_level_client.MultirotorControl', control)
    client = HighLevelClient()
    with pytest.raises(ConnectionError, match='127.0.0.1:5762'):
        client.connect('127.0.0.1', '5762')
    factory.assert_called_once_with(('127.0.0.1', 5762))
    control.assert_not_called()


@pytest.mark.parametrize('port', [0, 65536, 'not-a-port'])
def test_connect_rejects_invalid_port_before_opening_socket(monkeypatch, port):
    factory = Mock()
    monkeypatch.setattr('agrotechsimapi.high_level_client.TCPTransmitter', factory)
    with pytest.raises(ValueError):
        HighLevelClient().connect('127.0.0.1', port)
    factory.assert_not_called()
