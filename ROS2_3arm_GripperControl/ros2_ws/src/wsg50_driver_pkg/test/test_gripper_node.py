import pytest
from unittest.mock import patch, MagicMock
from wsg50_driver.wsg_interface import WSGInterface

@pytest.fixture
def gripper():
    gripper = WSGInterface('127.0.0.1', 1000)
    gripper.sock = MagicMock()
    return gripper

@patch('wsg50_driver.wsg_interface.socket.socket')
def test_connect(mock_socket, gripper):
    mock_sock_instance = MagicMock()
    mock_socket.return_value = mock_sock_instance
    gripper.connect()
    mock_sock_instance.connect.assert_called_once_with(('127.0.0.1', 1000))

def test_send_and_recv(gripper):
    gripper.sock.recv.return_value = b'ACK CMD'
    gripper.send('CMD')
    assert gripper.recv() == 'ACK CMD'

def test_ack_and_fin_success(gripper):
    gripper.recv = MagicMock(side_effect=['ACK CMD', 'FIN CMD'])
    gripper.send = MagicMock()
    assert gripper._ack_and_fin('CMD')

def test_ack_only_success(gripper):
    gripper.recv = MagicMock(return_value='ACK CMD')
    gripper.send = MagicMock()
    assert gripper._ack_only('CMD')

def test_calibrate(gripper):
    gripper.home = MagicMock(side_effect=[True, True])
    gripper.get_position = MagicMock(side_effect=[0.1, 109.8])
    assert gripper.calibrate()
    assert pytest.approx(gripper.min_position, 0.01) == 0.1
    assert pytest.approx(gripper.max_position, 0.01) == 109.8

def test_home(gripper):
    gripper._ack_and_fin = MagicMock()
    assert gripper.home(direction=1)
    gripper._ack_and_fin.assert_called_with('HOME(1)')

def test_reconnect_success(gripper):
    gripper.close = MagicMock()
    gripper.connect = MagicMock()
    assert gripper.reconnect()

def test_reconnect_failure(gripper):
    gripper.close = MagicMock()
    gripper.connect = MagicMock(side_effect=Exception("fail"))
    assert not gripper.reconnect()

def test_move_to_width(gripper):
    gripper.recv = MagicMock(side_effect=['ACK MOVE', 'FIN MOVE'])
    gripper.send = MagicMock()
    assert gripper.move_to_width(50.0)

def test_grip(gripper):
    gripper.recv = MagicMock(side_effect=['ACK GRIP', 'FIN GRIP'])
    gripper.send = MagicMock()
    assert gripper.grip(10.0)

def test_release(gripper):
    gripper.recv = MagicMock(side_effect=['ACK RELEASE', 'FIN RELEASE'])
    gripper.send = MagicMock()
    assert gripper.release(5.0)

def test_stop(gripper):
    gripper.recv = MagicMock(return_value='ACK STOP')
    gripper.send = MagicMock()
    assert gripper.stop()

def test_fast_stop(gripper):
    gripper.recv = MagicMock(return_value='ACK FASTSTOP')
    gripper.send = MagicMock()
    assert gripper.fast_stop()

def test_acknowledge_fast_stop(gripper):
    gripper.recv = MagicMock(return_value='ACK FSACK')
    gripper.send = MagicMock()
    assert gripper.acknowledge_fast_stop()

def test_get_status_methods(gripper):
    gripper.send_cmd = MagicMock(side_effect=[
        'POS=10.0', 'SPEED=50.0', 'FORCE=20.0', 'GRIPSTATE=4'
    ])
    assert gripper.get_position() == 10.0
    assert gripper.get_speed() == 50.0
    assert gripper.get_force() == 20.0
    assert gripper.get_state() == 4

def test_autosend(gripper):
    gripper._ack_only = MagicMock(return_value=True)
    assert gripper.enable_autosend("POS", 100)
    assert gripper.enable_autosend("POS", 100, delta_or_change=0.1)
    assert gripper.enable_autosend("POS", 100, delta_or_change=True)

def test_error_handling_ack_and_fin(gripper):
    gripper.recv = MagicMock(side_effect=['ERR CMD 29'])
    gripper.send = MagicMock()
    with pytest.raises(RuntimeError, match="Axis blocked"):
        gripper._ack_and_fin("CMD")

def test_error_handling_ack_only(gripper):
    gripper.recv = MagicMock(side_effect=['ERR CMD 1'])
    gripper.send = MagicMock()
    with pytest.raises(RuntimeError, match="E_NOT_AVAILABLE"):
        gripper._ack_only("CMD")

def test_device_info_commands(gripper):
    gripper.send_cmd = MagicMock(side_effect=[
        'TEMP=36.6',
        'VERSION=1.2.3',
        'DEVTYPE=WSG50',
        'SN=WSG123456',
        'TAG=MainGripper',
        'SYSFLAGS=0x03',
        'SYSFLAGS[0]=1',
        'GRIPSTATS=[10,2,1]',
        'PWT=0.25',
        'CLT=5.5'
    ])
    assert gripper.get_temperature() == 36.6
    assert gripper.get_firmware_version() == '1.2.3'
    assert gripper.get_device_type() == 'WSG50'
    assert gripper.get_serial_number() == 'WSG123456'
    assert gripper.get_device_tag() == 'MainGripper'
    assert gripper.get_system_flags() == 'SYSFLAGS=0x03'
    assert gripper.get_system_flag(0) is True
    assert gripper.get_gripper_statistics() == [10, 2, 1]
    assert gripper.get_part_width_tolerance() == 0.25
    assert gripper.get_clamping_travel() == 5.5
