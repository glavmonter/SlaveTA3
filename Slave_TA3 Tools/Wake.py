try:
    import crcmod
except ModuleNotFoundError as err:
    print('Import module crcmod error: {}'.format(str(err)))
    print('Install module crcmod')
    exit(-1)

from struct import pack

FEND = b'\xC0'
FESC = b'\xDB'
TFEND = b'\xDC'
TFESC = b'\xDD'


def wake_transmit(cmd, data, adr=None):
    d_for_crc = FEND
    d_for_tx = b''

    if adr is not None:
        d_for_crc += pack('B', adr & 0x7F)
        d_for_tx += pack('B', adr + 0x80)

    d_for_crc += pack('B', cmd.value)
    d_for_tx += pack('B', cmd.value)

    d_for_crc += pack('B', len(data)) + data
    d_for_tx += pack('B', len(data)) + data

    crc8_func = crcmod.predefined.mkPredefinedCrcFun('crc-8-maxim')
    crc = crc8_func(d_for_crc)

    d_for_tx += pack('B', crc)
    d_for_tx = d_for_tx.replace(FESC, FESC + TFESC)
    d_for_tx = d_for_tx.replace(FEND, FESC + TFEND)

    return FEND + d_for_tx


def wake_decode(data):
    return data.replace(FESC + TFEND, FEND).replace(FESC + TFESC, FESC)


def wake_check_crc(data):
    if len(data) < 5:
        return False

    d_crc = b''
    d_crc += data[0:1]
    d_crc += bytes([data[1] & 0x7F])
    d_crc += data[2:-1]

    crc8_func = crcmod.predefined.mkPredefinedCrcFun('crc-8-maxim')
    crc_calc = crc8_func(d_crc)

    return crc_calc == data[-1:][0]
