import sys
import os
import argparse
import SlaveTA


if os.name == 'nt':  # sys.platform == 'win32':
    from serial.tools.list_ports_windows import comports
elif os.name == 'posix':
    from serial.tools.list_ports_posix import comports
else:
    raise ImportError("Sorry: no implementation for your platform ('{}') available".format(os.name))


def show_ports():
    iterator = sorted(comports())
    sys.stdout.write('Available ports: \n')
    for n, (port, desc, hwid) in enumerate(iterator, 1):
        sys.stdout.write('{:20}\n'.format(port))
        sys.stdout.write('    desc: {}\n'.format(desc))
        sys.stdout.write('    hwid: {}\n'.format(hwid))


def _relays_to_humane(data):
    if data is not None:
        return '{:05b}_{:05b}'.format((data & 0b1111100000) >> 5, data & 0b11111)
    else:
        return 'None'


def _port_to_humane(data):
    if data is not None:
        return '{:04b}'.format(data)
    else:
        return 'None'

def _humane_bytes(b):
    return ' '.join('{:02X}'.format(x) for x in b)


parser = argparse.ArgumentParser(description='Getting INFO')
parser.add_argument('--list', '-l', action='store_true', help='List available COM ports')

parser.add_argument('Port', type=str, nargs='?', help='COM port name, COMx')
parser.add_argument('Adr', type=str, nargs='?', help='Address: [1, 126]')


args = parser.parse_args()


if len(sys.argv) == 1:
    parser.print_help()
    parser.exit(0)

if args.list is True:
    show_ports()
    parser.exit(0)

if args.Port is None:
    sys.stderr.write('Com port name is invalid\n\n')
    parser.print_help()
    parser.exit(1)

if args.Adr is None:
    sys.stderr.write('Address must be defined\n\n')
    parser.print_help()
    parser.exit(2)

try:
    address = int(args.Adr)
    if address not in range(1, 127):
        raise ValueError

except ValueError as err:
    sys.stderr.write('Address [{}] is not valid\n'.format(args.Adr))
    parser.exit(2)

porta_idr, portb_idr, porta_odr, portb_odr, relays_idr, relays_odr, (wieg1_size, wieg1_data), (wieg2_size, wieg2_data)\
    = SlaveTA.get_all(args.Port, address, SlaveTA.SubCommands.SCMD_ALL.value)


print('Port Input : [A: {}] [B: {}]'.format(_port_to_humane(porta_idr), _port_to_humane(portb_idr)))
print('Port Output: [A: {}] [B: {}]'.format(_port_to_humane(porta_odr), _port_to_humane(portb_odr)))
print('Relays Input : [{}]'.format(_relays_to_humane(relays_idr)))
print('Relays Output: [{}]'.format(_relays_to_humane(relays_odr)))
print('Wiegand 1: {}, [{}]'.format(wieg1_size, _humane_bytes(wieg1_data)))
print('Wiegand 2: {}, [{}]'.format(wieg2_size, _humane_bytes(wieg2_data)))
