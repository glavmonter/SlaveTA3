import sys
import os
import argparse
import struct
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


parser = argparse.ArgumentParser(description='SetPorts Relays IO')
parser.add_argument('--list', '-l', action='store_true', help='List available COM ports')

parser.add_argument('Port', type=str, nargs='?', help='COM port name, COMx')
parser.add_argument('Adr', type=str, nargs='?', help='Address: [1, 126]')
parser.add_argument('Cmd', type=str, nargs='?', help='Command: RI (Read input), RO (Read output), '
                                                     'WZ (Write Zeros), WO (Write Ones), WD (Write Data), WT (Toggle)')

parser.add_argument('Data', type=str, nargs='?', help='Data for Relays (binary, 10 bits, xxxxx_xxxxx)')
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

if (args.Adr is None) or (args.Cmd is None):
    sys.stderr.write('Address and Command must be defined\n\n')
    parser.print_help()
    parser.exit(2)

args.Cmd = args.Cmd.upper()

if not (args.Cmd in ['RI', 'RO', 'WZ', 'WO', 'WD', 'WT']):
    sys.stderr.write('Command not valid')
    parser.print_help()
    parser.exit(2)

try:
    address = int(args.Adr)
    if address not in range(1, 127):
        raise ValueError

except ValueError as err:
    sys.stderr.write('Address [{}] is not valid\n'.format(args.Adr))
    parser.exit(2)


need_data = args.Cmd in ['WZ', 'WO', 'WD', 'WT']
if need_data and (args.Data is None):
    sys.stderr.write('Command {} need defined Data'.format(args.Cmd))
    parser.print_help()
    parser.exit(2)

if need_data:
    try:
        data = int(args.Data, 2)
        if data > 0b11111_11111:
            raise ValueError

    except ValueError:
        sys.stderr.write('Data not in binary format (xxxxx_xxxxx)!! [{}]'.format(args.Data))
        parser.exit(1)

relays = SlaveTA.Relays(port=args.Port, address=address)

commands = {'RI': relays.read_idr,
            'RO': relays.read_odr,
            'WZ': relays.write_zeros,
            'WO': relays.write_ones,
            'WD': relays.write_data,
            'WT': relays.write_toggle}

if args.Cmd in ['RI', 'RO']:
    ret, data = commands[args.Cmd]()
    if ret is True:
        print('{}: [{}]'.format(args.Cmd, _resp_to_humane(data)))
    else:
        print('{} Error'.format(args.Cmd))

else:
    ret = commands[args.Cmd](data)
    print('{} {}'.format(args.Cmd, 'Ok' if ret is True else 'Error'))

parser.exit(0)
