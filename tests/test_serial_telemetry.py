from unittest.mock import MagicMock
import struct
from vsss.comms.serial_sender import SerialSender, crc8_dallas
from vsss.comms.both_sender import BothSender
from vsss.comms.base import RobotCommand


def test_serial_telemetry_parsing():
    # Instantiate SerialSender with mock port
    sender = SerialSender(port="MOCK_PORT")

    # Mock the serial port object
    mock_ser = MagicMock()
    mock_ser.is_open = True
    sender.ser = mock_ser

    # Construct a valid telemetry packet:
    # header (0xBB), length (0x0A), robot_id (0), wheel_left (150 = 1.50 m/s), wheel_right (-150 = -1.50 m/s),
    # enc_delta_left (1000), enc_delta_right (-2000)
    wl = 150
    wr = -150
    ed_l = 1000
    ed_r = -2000

    payload = struct.pack("<BBBhhhh", 0xBB, 0x0A, 0, wl, wr, ed_l, ed_r)
    crc = crc8_dallas(payload)
    valid_packet = payload + bytes([crc])

    # Mock ser.in_waiting and ser.read
    mock_ser.in_waiting = len(valid_packet)
    mock_ser.read.return_value = valid_packet

    # Read telemetry
    packets = sender.read_telemetry()

    assert len(packets) == 1
    pkt = packets[0]
    assert pkt["robot_id"] == 0
    assert pkt["wheel_left_vel"] == 1.5
    assert pkt["wheel_right_vel"] == -1.5
    assert pkt["enc_delta_left"] == 1000
    assert pkt["enc_delta_right"] == -2000
    assert len(sender.rx_buffer) == 0


def test_serial_telemetry_garbage_handling():
    sender = SerialSender(port="MOCK_PORT")
    mock_ser = MagicMock()
    mock_ser.is_open = True
    sender.ser = mock_ser

    # Garbage bytes followed by a valid packet, followed by partial packet
    wl = 50
    wr = -50
    ed_l = 10
    ed_r = -20
    payload = struct.pack("<BBBhhhh", 0xBB, 0x0A, 1, wl, wr, ed_l, ed_r)
    crc = crc8_dallas(payload)
    valid_packet = payload + bytes([crc])

    garbage = b"\x00\x11\x22\xBB\x12"  # has a 0xBB but invalid length (0x12)
    partial = b"\xBB\x0A\x01"

    stream = garbage + valid_packet + partial

    mock_ser.in_waiting = len(stream)
    mock_ser.read.return_value = stream

    packets = sender.read_telemetry()

    assert len(packets) == 1
    pkt = packets[0]
    assert pkt["robot_id"] == 1
    assert pkt["wheel_left_vel"] == 0.5
    assert pkt["wheel_right_vel"] == -0.5
    assert pkt["enc_delta_left"] == 10
    assert pkt["enc_delta_right"] == -20

    # Remaining buffer should have the partial packet: 3 bytes
    assert len(sender.rx_buffer) == len(partial)
    assert sender.rx_buffer == bytearray(partial)


def test_both_sender_delegation():
    # Instantiate BothSender with mock port
    sender = BothSender(port="MOCK_PORT")

    # Mock both inner senders
    mock_sim = MagicMock()
    mock_serial = MagicMock()
    sender.sim_sender = mock_sim
    sender.serial_sender = mock_serial

    # Test open()
    sender.open()
    mock_sim.open.assert_called_once()
    mock_serial.open.assert_called_once()

    # Test close()
    sender.close()
    mock_sim.close.assert_called_once()
    mock_serial.close.assert_called_once()

    # Test send()
    cmds = [RobotCommand(robot_id=0, yellow_team=False, wheel_left=0.5, wheel_right=0.5)]
    sender.send(cmds)
    mock_sim.send.assert_called_once_with(cmds)
    mock_serial.send.assert_called_once_with(cmds)

    # Test read_telemetry()
    mock_serial.read_telemetry.return_value = [{"robot_id": 0}]
    telemetry = sender.read_telemetry()
    assert telemetry == [{"robot_id": 0}]
    mock_serial.read_telemetry.assert_called_once()
