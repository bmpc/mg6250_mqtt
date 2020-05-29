import logging
import serial
import signal
from time import sleep, time

LOGGING_LEVEL_CONSOLE = 'DEBUG'

PROTOCOL_BYTES = 46
EVENT_BYTES = 37
CTR_BYTES = 4
SERIAL_PORT = '/dev/tty.XXXXXXXX' # replace by your USB tty

# create logger
logger = logging.getLogger('paradox')
logger.setLevel(LOGGING_LEVEL_CONSOLE)

# create formatter and add it to the handlers
formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s')

# create console handler with a higher log level
ch = logging.StreamHandler()
ch.setLevel(LOGGING_LEVEL_CONSOLE)
ch.setFormatter(formatter)
logger.addHandler(ch)

class Message():
    def __init__(self, msg):

        self.msg = msg
        self.unknown0 = msg[0]
        self.unknown1 = msg[1]
        self.unknown2 = msg[2]
        self.msg_length = msg[3]
        self.unknown4 = msg[4]
        self.unknown5 = msg[5]
        self.unknown6 = msg[6]
        self.events = self.__createEvents(msg)
        self.checksum1 = msg[44]
        self.checksum2 = msg[45]

    def __createEvents(self, msg):
        events = []
        _events_length = msg[3] - 9
        num_events = int(_events_length / EVENT_BYTES)
        begin = 7
        for ev in range(1, num_events + 1):
            end = begin + EVENT_BYTES
            events.append(Event(msg[begin:end]))
            begin = end
        return events

    def __repr__(self):
        str = f'Message with {len(self.events)} events: \n'
        for ev in self.events:
            str += f'                                                                       {ev}\n'
        return str
    def __str__(self):
        str = f'Message with {len(self.events)} events: \n'
        for ev in self.events:
            str += f'                                                                       {ev}\n'
        return str

class Event():
    def __init__(self, msg):
        if (len(msg) != EVENT_BYTES):
            raise NameError('Invalid Message!')
        self.seq = msg[0]
        self.unknown8 = msg[1]
        self.unknown9 = msg[2]
        self.unknown10 = msg[3]
        self.unknown11 = msg[4]
        self.unknown12 = msg[5]
        self.event = msg[6]
        self.sub_event = msg[7]
        self.area = msg[8]
        self.century = msg[9]
        self.year = msg[10]
        self.month = msg[11]
        self.day = msg[12]
        self.hour = msg[13]
        self.minute = msg[14]
        self.unknown22 = msg[15]
        self.unknown23 = msg[16]
        self.unknown24 = msg[17]
        self.unknown25 = msg[18]
        self.unknown26 = msg[19]
        self.typed = msg[20]
        self.zone_label = ''.join([chr(x) for x in msg[21:37]])

    def __repr__(self):
        return f'[{self.year}-{self.month}-{self.day} {self.hour}:{self.minute}] Event: {self.event}; Sub-Event: {self.sub_event}; Area: {self.area}; Label: {self.zone_label}'
    def __str__(self):
        return f'[{self.year}-{self.month}-{self.day} {self.hour}:{self.minute}] Event: {self.event}; Sub-Event: {self.sub_event}; Area: {self.area}; Label: {self.zone_label}'

class Serial_Connection():
    def __init__(self,
                 port='/dev/tty.usbmodem1423301',
                 baudrate=57600,
                 timeout=5,
                 rtscts=False):
        '''Initialise Serial_Connection.'''
        logger.debug('Initialising Serial_Connection...')
        self.connection = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.rtscts = rtscts
        logger.debug('Initialised Serial_Connection.')

    def connect(self):
        '''Connect to serial port.'''
        logger.debug('Connecting to serial port...')
        self.connection = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            rtscts=self.rtscts)
        try:
            logger.debug(f'Is Open? %s', self.connection.isOpen())
            if (self.connection.isOpen() == False) :
                self.connection.open()

            logger.debug('Connected to serial port.')
            return True
        except e:
            logger.error('Could not connect to serial port: ' + e)
            return False

    def calc_checksum(self, message):
        crc = 0xFFFF
        for pos in message:
            crc ^= pos
            for i in range(8):
                if ((crc & 1) != 0):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    def write(self, message):
        '''Send a message.'''
        checksum = self.calc_checksum(message)
        if checksum:
            #byte1 = checksum & 0x000000FF
            #byte2 = (checksum & 0x0000FF00) >> 8
            message += (checksum).to_bytes(2, byteorder='little')

        logger.debug('Sending message {}...'.format(message))
        self.connection.write(message)
        logger.debug('Message sent.')

    def read(self, timeout=1):
        '''Read bytes from serial port waiting up to timeout.'''
        old_timeout = self.connection.timeout
        self.connection.timeout = timeout

        control = self.connection.read(CTR_BYTES)
        msg_length = control[CTR_BYTES - 1]
        while self.connection.in_waiting < (msg_length - CTR_BYTES):
            continue

        data = self.connection.read(msg_length - CTR_BYTES)

        self.connection.timeout = old_timeout

        # verify checksum
        content = control + data[0:(len(data) - 2)]

        actual_crc = self.calc_checksum(content)
        byte1 = data[len(data) - 1]
        byte2 = data[len(data) - 2]
        expected_crc = (byte1 << 8) | byte2

        if (actual_crc != expected_crc) :
            logger.error('Invalid or corrupted message. Actual checksum: {} != Expected checksum {} : {}'.format(actual_crc, expected_crc, content))
            return None

        return control + data

    def in_waiting(self):
        '''Check how many butes are waiting on connection.'''
        return self.connection.in_waiting

# end Message class

# WARNING. This failed miserably. We can't send messages to MG6250 yet.
def sendTestMessage(connection, signalNumber):
    print('Received: ', signalNumber)

    # Arm with any user - Tried to send a couple of messages but nothing works.
    # test=b'\xDE\x21\x00\x2E\x00\x00\x00\xFF\xFF\xFF\xFF\x01\x1F\x28\x63\x01\x14\x14\x04\x0C\x0E\x24\x00\x01\x00\xFB\xFF\x01\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20'
    # test=b'\xDE\x21\x00\x1E\x00\x00\x00\xFF\xFF\xFF\xFF\x01\x1F\x28\x63\x01\x14\x14\x04\x0C\x0E\x24\x00\x01\x00\xFB\xFF\x01'
    test=b'\xDE\x21\x00\x0C\x00\x00\x00\x28\x63\x01'

    connection.write(test)

    return

def main():
    logger.info('Establishing serial connection...')

    connection = Serial_Connection(port=SERIAL_PORT)

    connected = connection.connect()

     # register the SIGUSR1 signal to be caught and send a test message
    signal.signal(signal.SIGUSR1, lambda signalNumber, frame : sendTestMessage(connection, signalNumber))

    software_connected = False

    if (connected == True) :
         while True:
            if connection.in_waiting() >= CTR_BYTES:
                msg = connection.read()
                if msg != None:
                    m = Message(msg)

                    logger.debug(m)
            sleep(1)
    else:
        logger.info('Oh no!')

if __name__== '__main__':
  main()
