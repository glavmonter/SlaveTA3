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


parser = argparse.ArgumentParser(description='SetAll outputs')
parser.add_argument('--list', '-l', action='store_true', help='List available COM ports')

parser.add_argument('Port', type=str, nargs='?', help='COM port name, COMx')
parser.add_argument('Adr', type=str, nargs='?', help='Address (hex): [00, 7F]')
parser.add_argument('Cmd', type=str, nargs='?', help='Command: WZ (Write Zeros), WO (Write Ones), WD (Write Data),'
                                                     'WT (Toggle outputs)')

parser.add_argument('DataA', type=str, nargs='?', help='Data for PORTA (binary, 4 bits)')
parser.add_argument('DataB', type=str, nargs='?', help='Data for PORTB (binary, 4 bits)')
parser.add_argument('Relays', type=str, nargs='?', help='Data for Relays (binary, 10 bits xxxxx_xxxxx)')

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

if not (args.Cmd in ['WZ', 'WO', 'WD', 'WT']):
    sys.stderr.write('Command not valid')
    parser.print_help()
    parser.exit(2)


try:
    porta = int(args.DataA, 2)
    portb = int(args.DataB, 2)
    relays = int(args.Relays, 2)
    adr = int(args.Adr, 10)
except ValueError as e:
    sys.stderr.write('DataA, DataB or Relays are not integer')
    sys.exit(2)

SlaveTA.set_all(args.Port, adr, porta, portb, relays, args.Cmd)
