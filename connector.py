import socket
import struct

import pid


def parse_raw_packet(packet):
    if packet[0:5] == b'DREF+':

        dataref_end = packet[9:].find(b'\x00')
        dataref = packet[9:dataref_end + 9].decode('ascii')
        value = struct.unpack('f', packet[5:9])

        return (dataref, value[0])
    else:
        return (None, None)

def create_raw_packet(dataref, value):
    packet = b'DREF0' + struct.pack('f', value) + dataref.encode('ascii')
    packet = packet.ljust(509, b'\x00')
    return packet

def ex():
    UDP_IP = "127.0.0.1"
    UDP_PORT = 48001

    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.bind((UDP_IP, UDP_PORT))

    p = pid.pid(
        kp=2000.0,
        ki=200.0,
        kd=-10.0,
        tau=0.01,
        out_limit_min=-0.03,
        out_limit_max=0.05,
        t=0.01
    )
    setpoint = 2000.0

    while True:
        # Receive packet and parse to basic values
        data, _addr = sock.recvfrom(1024)
        (dataref, value) = parse_raw_packet(data)

        if dataref == 'sim/cockpit2/gauges/indicators/altitude_ft_pilot':
            measurement = value
            yoke_pitch_ratio = p.update(setpoint, measurement)
            print('current altitude: ' + str(value) + ' computed yoke pitch ratio: ' + str(yoke_pitch_ratio))

            packet = create_raw_packet('sim/cockpit2/controls/yoke_pitch_ratio', yoke_pitch_ratio)
            sock.sendto(packet, ('192.168.0.94', 49000))

            # # throttle maximum
            # packet = create_raw_packet('sim/cockpit2/engine/actuators/throttle_ratio[0]', 1.0)

            # # +1, elevator up
            # packet = create_raw_packet('sim/cockpit2/controls/yoke_pitch_ratio', 1.0)
            # # 0, elevator neutral
            # packet = create_raw_packet('sim/cockpit2/controls/yoke_pitch_ratio', 0.0)
            # # -1, elevator down
            # packet = create_raw_packet('sim/cockpit2/controls/yoke_pitch_ratio', -0.1)

            # if value > 1000.0:
            #     sock.sendto(packet, ('192.168.0.94', 49000))

# sim/cockpit2/engine/actuators/throttle_ratio[0]
# sim/cockpit2/controls/yoke_pitch_ratio
# sim/cockpit/gps/course

if __name__ == "__main__":
    ex()
