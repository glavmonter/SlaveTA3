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


def _resp_to_humane(data):
    return '{:05b}_{:05b}'.format((data & 0b1111100000) >> 5, data & 0b11111)


parser = argparse.ArgumentParser(description='SetPower Power IO')
parser.add_argument('--list', '-l', action='store_true', help='List available COM ports')

parser.add_argument('Port', type=str, nargs='?', help='COM port name, COMx')
parser.add_argument('Adr', type=str, nargs='?', help='Address: [1, 126]')

parser.add_argument('Pin', type=int, nargs='?', help='Pin number [0-17], (0-3 - PORTA, 4-7 PORTB, 8-17 - Relays)')
parser.add_argument('Width', type=int, nargs='?', help='Pulse width, ms')
parser.add_argument('Delay', type=int, nargs='?', help='Pulse start delay (optional), ms')

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

if args.Pin is None or args.Width is None:
    sys.stderr.write('Pin and Width must be defined\n\n')
    parser.print_help()
    parser.exit(3)


try:
    address = int(args.Adr)
    if address not in range(1, 127):
        raise ValueError

except ValueError as err:
    sys.stderr.write('Address [{}] is not valid\n'.format(args.Adr))
    parser.exit(2)

if args.Delay is None:
    delay = 0
else:
    delay = args.Delay

# port, address, pin, width, delay=0
ret = SlaveTA.send_pulse(port=args.Port, address=address, pin=args.Pin, width=args.Width, delay=delay)

parser.exit(0)
