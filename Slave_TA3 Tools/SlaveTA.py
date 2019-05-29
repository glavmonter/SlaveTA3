import sys
import struct
import slave_pb2


try:
    import serial
except ModuleNotFoundError as err:
    print('Import module pyserial error: {}'.format(str(err)))
    print('Install module pyserial')
    exit(-1)

import Wake
from enum import Enum

BAUDRATE = 256000


class Commands(Enum):
    ERR = 0x01
    INFO = 0x03
    BOOT = 0x04

    PORTS_IDR = 0x06
    PORTS_ODRR = 0x07
    PORTS_ODRW = 0x08
    PORTS_SET = 0x09
    PORTS_RESET = 0x0A

    RELAYS_IDR = 0x0C
    RELAYS_ODDR = 0x0D
    RELAYS_ODRW = 0x0E
    RELAYS_SET = 0x0F
    RELAYS_RESET = 0x10

    POWERS_IDR = 0x11
    POWERS_ODRR = 0x12
    POWERS_ODRW = 0x13
    POWERS_SET = 0x14
    POWERS_RESET = 0x15

    WIEGAND = 0x0B

    CLIMATE_GET = 0x16
    CLIMATE_SET = 0x17

    READ_ALL = 0x50

    PORTS_TOGGLE = 0x52
    RELAYS_TOGGLE = 0x53


class SubCommands(Enum):
    SCMD_PORTS_IDR = (1 << 0)
    SCMD_PORTS_ODR = (1 << 1)
    SCMD_RELAYS_IDR = (1 << 2)
    SCMD_RELAYS_ODR = (1 << 3)
    SCMD_WIEGAND_1 = (1 << 4)
    SCMD_WIEGAND_2 = (1 << 5)
    SCMD_ALL = SCMD_PORTS_IDR | SCMD_PORTS_ODR | SCMD_RELAYS_IDR | SCMD_RELAYS_ODR | SCMD_WIEGAND_1 | SCMD_WIEGAND_2


def _humane_bytes(b):
    return ' '.join('{:02X}'.format(x) for x in b)


def get_info(port, address):
    try:
        request = Wake.wake_transmit(Commands.INFO, b'', address)
        ser = serial.Serial(port=port, baudrate=BAUDRATE, timeout=0.1)
        ser.write(request)
        ret = Wake.wake_decode(ser.read(64))
        print('Request:  ', _humane_bytes(request))
        print('Response: ', _humane_bytes(ret))
        ser.close()

        if not Wake.wake_check_crc(ret):
            return None, None, None, None

        flash_size = struct.unpack('<H', ret[4:6])[0]
        unique_id = ret[6:18]
        version = struct.unpack('<I', ret[18:22])[0]
        name = ret[22:-1].decode('utf-8')
        return name, flash_size, version, unique_id

    except serial.SerialException as err:
        sys.stderr.write(str(err) + '\n')
        return None, None, None, None

    except struct.error as err:
        sys.stderr.write('Can not parse returned data, {}\n'.format(str(err)))
        return None, None, None, None


def get_all(port, address, mask):
    ret_err = None, None, None, None, None, None, (0, b''), (0, b'')

    try:
        ser = serial.Serial(port=port, baudrate=BAUDRATE, timeout=0.1)
        request = Wake.wake_transmit(Commands.READ_ALL, struct.pack('<B', mask), address)
        ser.write(request)
        ret = Wake.wake_decode(ser.read(256))
        print('Request:  ', _humane_bytes(request))
        print('Response: {} ({} bytes)'.format(_humane_bytes(ret), len(ret)))
        ser.close()
        ret = ret[4:-1]
        print('Data: {} ({} bytes)'.format(_humane_bytes(ret), len(ret)))

        responce_all = slave_pb2.ResponceAll()
        responce_all.ParseFromString(ret)
        print(responce_all)
        porta_idr = responce_all.PORTA_IDR if responce_all.HasField('PORTA_IDR') else None
        portb_idr = responce_all.PORTB_IDR if responce_all.HasField('PORTB_IDR') else None
        porta_odr = responce_all.PORTA_ODR if responce_all.HasField('PORTA_ODR') else None
        portb_odr = responce_all.PORTB_ODR if responce_all.HasField('PORTB_ODR') else None
        relays_idr = responce_all.RELAYS_IDR if responce_all.HasField('RELAYS_IDR') else None
        relays_odr = responce_all.RELAYS_ODR if responce_all.HasField('RELAYS_ODR') else None

        wieg1_size = 0
        wieg1_data = b''
        wieg2_size = 0
        wieg2_data = b''

        wieg = responce_all.WiegandCh1 if responce_all.HasField('WiegandCh1') else None
        if wieg is not None:
            wieg1_size = wieg.size
            wieg1_data = wieg.data

        wieg = responce_all.WiegandCh2 if responce_all.HasField('WiegandCh2') else None
        if wieg is not None:
            wieg2_size = wieg.size
            wieg2_data = wieg.data

        return porta_idr, portb_idr, porta_odr, portb_odr, relays_idr, relays_odr, (wieg1_size, wieg1_data), (wieg2_size, wieg2_data)

    except serial.SerialException as err:
        sys.stderr.write(str(err) + '\n')
        return ret_err
    return ret_err


def get_wiegand(port, address):
    err_ret = False, 0, 0, b''
    try:
        ser = serial.Serial(port=port, baudrate=BAUDRATE, timeout=0.1)
        request = Wake.wake_transmit(Commands.WIEGAND, b'', address)
        ser.write(request)
        ret = Wake.wake_decode(ser.read(64))
        print('Request:  ', _humane_bytes(request))
        print('Response: ', _humane_bytes(ret))
        ser.close()

        if not Wake.wake_check_crc(ret):
            return err_ret

        channel = ret[4]
        wig_len = ret[5]
        return True, channel, wig_len, ret[6:-1]

    except serial.SerialException as err:
        sys.stderr.write(str(err) + '\n')
        return err_ret


def change_to_boot(port, address):
    key = bytes.fromhex('64 CA 56 BA 15 57 63 39 DA 57 40 21')

    _, _, version, unique_id = get_info(port, address)

    if unique_id is None:
        return False

    if len(key) != len(unique_id):
        return False

    cipher = b''
    for uid, k in zip(unique_id, key):
        cipher += bytes([uid ^ k])

    print('UID: {}'.format(_humane_bytes(unique_id)))
    print('Key: {}'.format(_humane_bytes(key)))
    print('Out: {}'.format(_humane_bytes(cipher)))

    try:
        ser = serial.Serial(port=port, baudrate=BAUDRATE, timeout=0.1)
        ser.write(Wake.wake_transmit(cmd=Commands.BOOT, data=cipher, adr=0x02))
        ret = Wake.wake_decode(ser.read(64))
        ser.close()

        if Wake.wake_check_crc(ret) is False:
            print('Check CRC Error\n')
            return False

        print('Ret: {}'.format(_humane_bytes(ret)))
        return ret[2] == Commands.BOOT.value  # Вернулась команда BOOT (0x04), Ok

    except serial.SerialException as err:
        sys.stderr.write(str(err) + '\n')
        return False
    return True


class Ports:
    def __init__(self, port, address=0x02):
        self._port = port
        self._address = address

    def read_idr(self):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.PORTS_IDR, data=b'', adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False, 0, 0
            if ret[2] != Commands.PORTS_IDR.value:
                return False, 0, 0

            return True, ret[4], ret[5]

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False, 0, 0

        except IndexError as err:
            sys.stderr.write('Response not valid: {}\n'.format(str(err)))
            return False, 0, 0

    def read_odr(self):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.PORTS_ODRR, data=b'', adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False, 0, 0
            if ret[2] != Commands.PORTS_ODRR.value:
                return False, 0, 0

            return True, ret[4], ret[5]

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False, 0, 0

        except IndexError as err:
            sys.stderr.write('Response not valid: {}\n'.format(str(err)))
            return False, 0, 0

    def write_data(self, data_a, data_b):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.PORTS_ODRW, data=bytes([data_a, data_b]), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False
            if ret[2] != Commands.PORTS_ODRW.value:
                return False

            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False

    def write_zeros(self, data_a, data_b):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.PORTS_RESET, data=bytes([data_a, data_b]), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False
            if ret[2] != Commands.PORTS_RESET.value:
                return False

            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False

    def write_ones(self, data_a, data_b):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.PORTS_SET, data=bytes([data_a, data_b]), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False
            if ret[2] != Commands.PORTS_SET.value:
                return False

            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False

    def write_toggle(self, data_a, data_b):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.PORTS_TOGGLE, data=bytes([data_a, data_b]), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False
            if ret[2] != Commands.PORTS_TOGGLE.value:
                return False
            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False


class Powers:
    def __init__(self, port, address=0x02):
        self._port = port
        self._address = address

    def read_idr(self):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.POWERS_IDR, data=b'', adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False, 0
            if ret[2] != Commands.POWERS_IDR.value:
                return False, 0

            return True, 0

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False, 0

        except IndexError as err:
            sys.stderr.write('Response not valid: {}\n'.format(str(err)))
            return False, 0

    def read_odr(self):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.POWERS_ODRR, data=b'', adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False, 0
            if ret[2] != Commands.POWERS_ODRR.value:
                return False, 0

            return True, (ret[4] << 8) | ret[5]

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False, 0

        except IndexError as err:
            sys.stderr.write('Response not valid: {}\n'.format(str(err)))
            return False, 0

    def write_data(self, data):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.POWERS_ODRW, data=struct.pack('>H', data), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False
            if ret[2] != Commands.POWERS_ODRW.value:
                return False

            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False

    def write_zeros(self, data):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.POWERS_RESET, data=struct.pack('>H', data), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False
            if ret[2] != Commands.POWERS_RESET.value:
                return False

            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False

    def write_ones(self, data):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.POWERS_SET, data=struct.pack('>H', data), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is None:
                return False
            if ret[2] != Commands.POWERS_SET.value:
                return False

            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False


class Relays:
    def __init__(self, port, address=0x02):
        self._port = port
        self._address = address

    def read_idr(self):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.RELAYS_IDR, data=b'', adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False, 0
            if ret[2] != Commands.RELAYS_IDR.value:
                return False, 0

            return True, (ret[4] << 8) | ret[5]

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False, 0

        except IndexError as err:
            sys.stderr.write('Response not valid: {}\n'.format(str(err)))
            return False, 0

    def read_odr(self):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.RELAYS_ODDR, data=b'', adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False, 0
            if ret[2] != Commands.RELAYS_ODDR.value:
                return False, 0

            return True, (ret[4] << 8) | ret[5]

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False, 0

        except IndexError as err:
            sys.stderr.write('Response not valid: {}\n'.format(str(err)))
            return False, 0

    def write_data(self, data):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.RELAYS_ODRW, data=struct.pack('>H', data), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False
            if ret[2] != Commands.RELAYS_ODRW.value:
                return False

            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False

    def write_zeros(self, data):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.RELAYS_RESET, data=struct.pack('>H', data), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False
            if ret[2] != Commands.RELAYS_RESET.value:
                return False

            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False

    def write_ones(self, data):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.RELAYS_SET, data=struct.pack('>H', data), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False
            if ret[2] != Commands.RELAYS_SET.value:
                return False

            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False

    def write_toggle(self, data):
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.RELAYS_TOGGLE, data=struct.pack('>H', data), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False
            if ret[2] != Commands.RELAYS_TOGGLE.value:
                return False
            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False


class Climate:
    def __init__(self, port, address=0x02):
        self._port = port
        self._address = address

    def get_data(self):
        err_ret = False, 0.0, 0.0, 0.0, 0.0, False, False, False

        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.CLIMATE_GET, data=b'', adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()
            if Wake.wake_check_crc(ret) is False:
                return err_ret
            if ret[2] != Commands.CLIMATE_GET.value:
                return err_ret

            t_local = struct.unpack('<f', ret[4:8])[0]
            t_remote = struct.unpack('<f', ret[8:12])[0]
            t_local_alt = struct.unpack('<f', ret[12:16])[0]
            humidity = struct.unpack('<f', ret[16:20])[0]

            heater = (ret[20] & 0b001) == 0b01
            cooler = (ret[20] & 0b010) == 0b10
            automatic = (ret[20] & 0b100) == 0b100

            return True, t_local, t_remote, t_local_alt, humidity, heater, cooler, automatic

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return err_ret

    def set_data(self, auto, cooler, heater):

        data = (auto << 2) | (cooler << 1) | (heater << 0)
        try:
            ser = serial.Serial(port=self._port, baudrate=BAUDRATE, timeout=0.1)
            request = Wake.wake_transmit(cmd=Commands.CLIMATE_SET, data=struct.pack('b', data), adr=self._address)
            ser.write(request)
            ret = Wake.wake_decode(ser.read(64))
            print('Request:  ', _humane_bytes(request))
            print('Response: ', _humane_bytes(ret))
            ser.close()

            if Wake.wake_check_crc(ret) is False:
                return False
            if ret[2] != Commands.CLIMATE_SET.value:
                return False

            return True

        except serial.SerialException as err:
            sys.stderr.write(str(err) + '\n')
            return False

        return True
