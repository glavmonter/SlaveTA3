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


parser = argparse.ArgumentParser(description='Getting Wiegand')
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

ret, channel, len, data = SlaveTA.get_wiegand(args.Port, address)

if ret is False:
    print('Slave Not found')
    parser.exit(0)

print('Channel: {}'.format(channel))
print('Len: {}'.format(len))
print('Data: {}'.format(SlaveTA._humane_bytes(data)))

parser.exit(0)
