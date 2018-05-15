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


parser = argparse.ArgumentParser(description='SetPorts Discret IO')
parser.add_argument('--list', '-l', action='store_true', help='List available COM ports')

parser.add_argument('Port', type=str, nargs='?', help='COM port name, COMx')
parser.add_argument('Adr', type=str, nargs='?', help='Address (hex): [00, 7F]')
parser.add_argument('Cmd', type=str, nargs='?', help='Command: RI (Read input), RO (Read output), '
                                                     'WZ (Write Zeros), WO (Write Ones), WD (Write Data)')

parser.add_argument('DataA', type=str, nargs='?', help='Data for PORTA (binary, 4 bits)')
parser.add_argument('DataB', type=str, nargs='?', help='Data for PORTB (binary, 4 bits)')

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

if not (args.Cmd in ['RI', 'RO', 'WZ', 'WO', 'WD']):
    sys.stderr.write('Command not valid')
    parser.print_help()
    parser.exit(2)


tx_data = b''
need_data = args.Cmd in ['WZ', 'WO', 'WD']
if need_data and ((args.DataA is None) or (args.DataB is None)):
    sys.stderr.write('Command {} need defined DataA and DataB\n'.format(args.Cmd))
    parser.print_help()
    parser.exit(2)


if need_data:
    try:
        data_a = int(args.DataA, 2)
        data_b = int(args.DataB, 2)

        if (data_a > 0b1111) or (data_b > 0b1111):
            raise ValueError

    except ValueError:
        sys.stderr.write('DataA or DataB not in binary format!! [{}], [{}]'.format(args.DataA, args.DataB))
        parser.exit(1)

ports = SlaveTA.Ports(args.Port, int(args.Adr, 16))

commands = {'RI': ports.read_idr,
            'RO': ports.read_odr,
            'WZ': ports.write_zeros,
            'WO': ports.write_ones,
            'WD': ports.write_data}


if args.Cmd in ['RI', 'RO']:
    ret, data_a, data_b = commands[args.Cmd]()
    if ret is True:
        print('{}: [A: {:04b}] [B: {:04b}]'.format(args.Cmd, data_a, data_b))
    else:
        print('{} Error'.format(args.Cmd))

else:
    ret = commands[args.Cmd](data_a, data_b)
    print('{} {}'.format(args.Cmd, 'Ok' if ret is True else 'Error'))

parser.exit(0)
