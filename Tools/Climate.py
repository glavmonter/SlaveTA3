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


parser = argparse.ArgumentParser(description='Climate control')
parser.add_argument('--list', '-l', action='store_true', help='List available COM ports')

parser.add_argument('Port', type=str, nargs='?', help='COM port name, COMx')
parser.add_argument('Adr', type=str, nargs='?', help='Address: [1, 126]')
parser.add_argument('Cmd', type=str, nargs='?', help='Command: GET, SET')

parser.add_argument('Mode', type=str, nargs='?', help='Auto Mode: [auto, manual]')
parser.add_argument('Cooler', type=str, nargs='?', help='Cooler control: [0, 1]')
parser.add_argument('Heater', type=str, nargs='?', help='Heater control: [0, 1]')

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

if not (args.Cmd in ['GET', 'SET']):
    sys.stderr.write('Command not valid')
    parser.print_help()
    parser.exit(2)

need_data = args.Cmd in ['SET']
if need_data and ((args.Mode is None) or (args.Heater is None) or (args.Cooler is None)):
    sys.stderr.write('Command {} need Auto, Heater and Cooler defined'.format(args.Cmd))
    parser.print_help()
    parser.exit(2)

if need_data:
    args.Mode = args.Mode.upper()

    try:
        if args.Mode not in ['AUTO', 'MANUAL']:
            raise ValueError

        auto = 1 if args.Mode == 'AUTO' else 0

        heater = int(args.Heater)
        if heater not in [0, 1]:
            raise ValueError

        cooler = int(args.Cooler)
        if cooler not in [0, 1]:
            raise ValueError

    except ValueError:
        sys.stderr.write('Auto, Heater or Cooler invalid')
        parser.exit(2)

try:
    address = int(args.Adr)
    if address not in range(1, 127):
        raise ValueError

except ValueError as err:
    sys.stderr.write('Address [{}] is not valid\n'.format(args.Adr))
    parser.exit(2)


climate = SlaveTA.Climate(port=args.Port, address=address)

commands = {'GET': climate.get_data,
            'SET': climate.set_data}

if args.Cmd in ['GET']:
    ret, t_local, t_remote, t_local_alt, humidity, heater, cooler, automatic = commands[args.Cmd]()
    if ret is True:
        print('GET OK')
        print('Temperature Local:     {:>8.3f}K, {:>8.3f}C'.format(t_local, t_local - 273.15))
        print('Temperature External:  {:>8.3f}K, {:>8.3f}C'.format(t_remote, t_remote - 273.15))
        print('Temperature Local Alt: {:>8.3f}K, {:>8.3f}C'.format(t_local_alt, t_local_alt - 273.15))
        print('Humidity:              {:>6.1f}%'.format(humidity))
        print('Heater: {}'.format('ON' if heater else 'OFF'))
        print('Cooler: {}'.format('ON' if cooler else 'OFF'))
        print('Automatic: {}'.format('ON' if automatic else 'OFF'))

    else:
        print('{} Error'.format(args.Cmd))

elif args.Cmd in ['SET']:
    ret = commands[args.Cmd](auto, cooler, heater)
    print('{} {}'.format(args.Cmd, 'Ok' if ret is True else 'Error'))

else:
    print('Unknown command')

parser.exit(0)
