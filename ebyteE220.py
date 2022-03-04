# +---------------------------------------------+
# | 0 - M0  (set mode)        [*]               |
# | 0 - M1  (set mode)        [*]               |
# | 0 - RXD (TTL UART input)  [*]               |
# | 0 - TXD (TTL UART output) [*]               |
# | 0 - AUX (device status)   [*]               |
# | 0 - VCC (3.3-5.2V)                          +---+
# | 0 - GND (GND)                                SMA| Antenna
# +-------------------------------------------------+
#     [*] ALL COMMUNICATION PINS ARE 3.3V !!!
# Transmission modes :
# ==================
#   - Transparent : all modules have the same address and channel and
#        can send/receive messages to/from each other. No address and
#        channel is included in the message.
#
#   - Fixed : all modules can have different addresses and channels.
#        The transmission messages are prefixed with the destination address
#        and channel information. If these differ from the settings of the
#        transmitter, then the configuration of the module will be changed
#        before the transmission. After the transmission is complete,
#        the transmitter will revert to its prior configuration.
#
#        1. Fixed P2P : The transmitted message has the address and channel
#           information of the receiver. Only this module will receive the message.
#           This is a point to point transmission between 2 modules.
#
#        2. Fixed Broadcast : The transmitted message has address FFFF and a
#           channel. All modules with any address and the same channel of
#           the message will receive it.
#
#        3. Fixed Monitor : The receiver has adress FFFF and a channel.
#           It will receive messages from all modules with any address and
#           the same channel as the receiver.
#
# Operating modes :
# ===============
#   - Mode0 = Normal (M0=0,M1=0) : UART and LoRa radio are on.
#
#   - Mode1 = WOR sending mode (M0=1,M1=0) : Same as normal but When defined as a transmitting party,
#             preamble is automatically added before transmitting.
#
#   - Mode2 = WOR receiving mode (M0=0,M1=1) : Wireless transmission off,
#             Can only receive data in WOR transmission(mode 1)
#
#   - Mode3 = Deep Sleep (M0=1,M1=1) : UART is on, LoRa radio is off. Is used to
#             get/set module parameters or to reset the module.
#
######################################################################

# from machine import Pin, UART
# import utime, ujson


class UartSerialPortRate:
    BAUDRATE_1200 = '000'
    BAUDRATE_2400 = '001'
    BAUDRATE_4800 = '010'
    BAUDRATE_9600 = '011'
    BAUDRATE_19200 = '100'
    BAUDRATE_38400 = '101'
    BAUDRATE_57600 = '110'
    BAUDRATE_115200 = '111'


class SerialParityBit:
    SPB_8N1 = '00'
    SPB_8O1 = '01'
    SPB_8E1 = '10'


class AirDataRate:
    ADR_2_4k1 = '000'
    ADR_2_4k2 = '001'
    ADR_2_4k = '010'
    ADR_4_8k = '011'
    ADR_9_6k = '100'
    ADR_19_2k = '101'
    ADR_38_4k = '110'
    ADR_62_5k = '111'


class SubPacketSetting:
    SPS_200bytes = '00'
    SPS_128bytes = '01'
    SPS_64bytes = '10'
    SPS_32bytes = '11'


class RSSI_AmbientNoiseEnable:
    '''with REG1-4,3,2 Reserve'''
    '''disalbe 0 or enable 1 + reserve 000 '''
    RSSI_ANE_Disable = '0000'
    RSSI_ANE_Enable = '1000'


class TxPower30:
    dBm30 = '00'
    dBm27 = '01'
    dBm24 = '10'
    dBm21 = '11'


class TxPower22:
    dBm22 = '00'
    dBm17 = '01'
    dBm13 = '10'
    dBm10 = '11'


class EnableRSSIByte:
    EnableRSSIByte_Disable = '0'
    EnableRSSIByte_Enable = '1'


class TxMethod:
    '''with REG3-5 Reserve'''
    '''disalbe 0 or enable 1 + reserve 0 '''
    Transparent_transmission_mode = '00'
    Fixed_transmission_mode = '01'


class LBTEnable:
    '''with REG3-3 Reserve'''
    '''disalbe 0 or enable 1 + reserve 0 '''
    LBT_Disable = '00'
    LBT_Enable = '01'


class WORCycle:
    WOR_500ms = '000'
    WOR_1000ms = '001'
    WOR_1500ms = '010'
    WOR_2000ms = '011'
    WOR_2500ms = '100'
    WOR_3000ms = '101'
    WOR_3500ms = '110'
    WOR_4000ms = '111'


class EbyteE220:

    def __init__(self, pinM0, pinM1, pinAUX, model='E220-900T22D', port='U1',debug=False):
        self.models = ['E220-400T22S', 'E220-400T30S', 'E220-900T22S', 'E220-900T30S',
                       'E220-400T22D', 'E220-400T30D', 'E220-900T22D', 'E220-900T30D']

        ''' constructor for ebyte E32 LoRa module '''
        # configuration in dictionary
        self.config = {}
        self.config['model'] = model
        if model not in self.models:
            print('Unknown model name!')
            print('Set default model : E220-900T22D')
            self.config['model'] = self.models[6]
        self.config['port'] = port  # UART channel on the ESP32 (default U1)

        '''ADDH'''
        self.config['addh'] = '00'  # ADDH (default 00)

        '''ADDL'''
        self.config['addl'] = '00'  # ADDL (default 00)

        '''REG0'''
        self.config['baud_rate'] = UartSerialPortRate.BAUDRATE_9600  # UART baudrate (default 9600)
        self.config['parity'] = SerialParityBit.SPB_8N1  # UART Parity (default 8N1)
        self.config['air_data_rate'] = AirDataRate.ADR_2_4k  # wireless baudrate (default 2.4k)

        '''REG1'''
        self.config['sub_packet'] = SubPacketSetting.SPS_200bytes
        self.config['RSSI_ambient_noise'] = RSSI_AmbientNoiseEnable.RSSI_ANE_Disable  # RSSI Ambient noise (default disable)
        self.config['channel'] = 0x48  # target channel (default 0x48 for Korea)
        self.calcFrequency()  # calculate frequency (min frequency + channel*1 MHz)
        self.config['transmode'] = 0  # transmission mode (default 0 - tranparent)
        self.config['worcycle'] = '500ms'  # wakeup time from sleep mode (default 0 = 500ms)
        self.config['txpower'] = '00'  # transmission power (default 22dBm)
        self.PinM0 = pinm0  # M0 pin number
        self.PinM1 = pinm1  # M1 pin number
        self.PinAUX = pinaux  # AUX pin number
        self.M0 = None  # instance for M0 Pin (set operation mode)
        self.M1 = None  # instance for M1 Pin (set operation mode)
        self.AUX = None  # instance for AUX Pin (device status : 0=busy, 1=idle)
        self.serdev = None  # instance for UART
        self.debug = debug

    def start(self):
        ''' Start the ebyte E220 LoRa module '''
        try:
            # check parameters
            if self.config['port'] not in self.PORT:
                self.config['port'] = 'U1'
            if int(self.config['baudrate']) not in self.BAUDRATE:
                self.config['baudrate'] = 9600
            if self.config['parity'] not in self.PARSTR:
                self.config['parity'] = '8N1'
            if self.config['datarate'] not in self.AIRDATARATE:
                self.config['datarate'] = '2.4k'
            # make UART instance
            self.serdev = UART(self.PORT.get(self.config['port']))
            # init UART
            par = self.PARBIT.get(str(self.config['parity'])[1])
            self.serdev.init(baudrate=self.config['baudrate'], bits=8, parity=par, stop=1)
            if self.debug:
                print(self.serdev)
            # make operation mode & device status instances
            self.M0 = Pin(self.PinM0, Pin.OUT)
            self.M1 = Pin(self.PinM1, Pin.OUT)
            self.AUX = Pin(self.PinAUX, Pin.IN, Pin.PULL_UP)
            if self.debug:
                print(self.M0, self.M1, self.AUX)
            # set config to the ebyte E32 LoRa module
            self.setConfig('SetRegister')
            return "OK"

        except Exception as E:
            if self.debug:
                print("error on start UART", E)
            return "NOK"

    def sendMessage(self, to_address, to_channel, payload, useChecksum=False):
        ''' Send the payload to ebyte E32 LoRa modules in transparent or fixed mode. The payload is a data dictionary to
            accomodate key value pairs commonly used to store sensor data and is converted to a JSON string before sending.
            The payload can be appended with a 2's complement checksum to validate correct transmission.
            - transparent mode : all modules with the same address and channel of the transmitter will receive the payload
            - fixed mode : only the module with this address and channel will receive the payload;
                           if the address is 0xFFFF all modules with the same channel will receive the payload'''
        try:
            # type of transmission
            if (to_address == self.config['address']) and (to_channel == self.config['channel']):
                # transparent transmission mode
                # all modules with the same address and channel will receive the payload
                self.setTransmissionMode(0)
            else:
                # fixed transmission mode
                # only the module with the target address and channel will receive the payload
                self.setTransmissionMode(1)
            # put into wakeup mode (includes preamble signals to wake up device in WORrx or DeepSleep mode)
            self.setOperationMode('WORtx')
            # check payload
            if type(payload) != dict:
                print('payload is not a dictionary')
                return 'NOK'
            # encode message
            msg = []
            if self.config['transmode'] == 1:  # only for fixed transmission mode
                msg.append(to_address // 256)  # high address byte
                msg.append(to_address % 256)  # low address byte
                msg.append(to_channel)  # channel
            js_payload = ujson.dumps(payload)  # convert payload to JSON string
            for i in range(len(js_payload)):  # message
                msg.append(ord(js_payload[i]))  # ascii code of character
            if useChecksum:  # attach 2's complement checksum
                msg.append(int(self.calcChecksum(js_payload), 16))
            # debug
            if self.debug:
                print(msg)
            # wait for idle module
            self.waitForDeviceIdle()
            # send the message
            self.serdev.write(bytes(msg))
            return "OK"

        except Exception as E:
            if self.debug:
                print('Error on sendMessage: ', E)
            return "NOK"

    def recvMessage(self, from_address, from_channel, useChecksum=False):
        ''' Receive payload messages from ebyte E32 LoRa modules in transparent or fixed mode. The payload is a JSON string
            of a data dictionary to accomodate key value pairs commonly used to store sensor data. If checksumming is used, the
            checksum of the received payload including the checksum byte should result in 0 for a correct transmission.
            - transparent mode : payload will be received if the module has the same address and channel of the transmitter
            - fixed mode : only payloads from transmitters with this address and channel will be received;
                           if the address is 0xFFFF, payloads from all transmitters with this channel will be received'''
        try:
            # type of transmission
            if (from_address == self.config['address']) and (from_channel == self.config['channel']):
                # transparent transmission mode
                # all modules with the same address and channel will receive the message
                self.setTransmissionMode(0)
            else:
                # fixed transmission mode
                # only the module with the target address and channel will receive the message
                self.setTransmissionMode(1)
            # put into normal mode
            self.setOperationMode('normal')
            # receive message
            js_payload = self.serdev.read()
            # debug
            if self.debug:
                print(js_payload)
            # did we receive anything ?
            if js_payload == None:
                # nothing
                return {'msg': None}
            else:
                # decode message
                msg = ''
                for i in range(len(js_payload)):
                    msg += chr(js_payload[i])
                # checksum check
                if useChecksum:
                    cs = int(self.calcChecksum(msg), 16)
                    if cs != 0:
                        # corrupt
                        return {'msg': 'corrupt message, checksum ' + str(cs)}
                    else:
                        # message ok, remove checksum
                        msg = msg[:-1]
                # JSON to dictionary
                message = ujson.loads(msg)
                return message

        except Exception as E:
            if self.debug:
                print('Error on recvMessage: ', E)
            return "NOK"

    def calcChecksum(self, payload):
        ''' Calculates checksum for sending/receiving payloads. Sums the ASCII character values mod256 and returns
            the lower byte of the two's complement of that value in hex notation. '''
        return '%2X' % (-(sum(ord(c) for c in payload) % 256) & 0xFF)

    def reset(self):
        ''' Reset the ebyte E32 Lora module '''
        try:
            # send the command
            res = self.sendCommand('reset')
            # discard result
            return "OK"

        except Exception as E:
            if self.debug:
                print("error on reset", E)
            return "NOK"

    def stop(self):
        ''' Stop the ebyte E32 LoRa module '''
        try:
            if self.serdev != None:
                self.serdev.deinit()
                del self.serdev
            return "OK"

        except Exception as E:
            if self.debug:
                print("error on stop UART", E)
            return "NOK"

    def sendCommand(self, configcommand):
        ''' Send a command to the ebyte E32 LoRa module.
            The module has to be in sleep mode '''
        try:
            # put into sleep mode
            self.setOperationMode('DeepSleep')
            # send command
            HexCmd = self.CONFIGCMDS.get(configcommand)
            if HexCmd in [0xC0, 0xC2]:  # set config to device
                header = HexCmd
                HexCmd = self.encodeConfig(header)
                HexCmd[0] = header
            else:  # get config, get version, reset
                HexCmd = [HexCmd, 0x00, self.REGLEN]
            if self.debug:
                print(HexCmd)
            self.serdev.write(bytes(HexCmd))
            # wait for result
            utime.sleep_ms(50)
            # read result
            result = self.serdev.read()
            # wait for result
            utime.sleep_ms(50)
            # debug
            if self.debug:
                print(result)
            return result

        except Exception as E:
            if self.debug:
                print('Error on sendCommand: ', E)
            return "NOK"

    def getConfig(self):
        ''' Get config parameters from the ebyte E32 LoRa module '''
        try:
            # send the command
            result = self.sendCommand('GetRegister')
            # check result
            if len(result) != 6:
                return "NOK"
            # decode result
            self.decodeConfig(result)
            # show config
            self.showConfig()
            return "OK"

        except Exception as E:
            if self.debug:
                print('Error on GetRegister: ', E)
            return "NOK"

    def decodeConfig(self, message):
        ''' decode the config message from the ebyte E32 LoRa module to update the config dictionary '''
        # message byte 0 = header
        header = int(message[0])
        # message byte 1 & 2 = address
        self.config['address'] = int(message[1]) * 256 + int(message[2])
        # message byte 3 = speed (parity, baudrate, datarate)
        bits = '{0:08b}'.format(message[3])
        self.config['parity'] = self.PARINV.get(bits[0:2])
        self.config['baudrate'] = self.BAUDRINV.get(bits[2:5])
        self.config['datarate'] = self.DATARINV.get(bits[5:])
        # message byte 4 = channel
        self.config['channel'] = int(message[4])
        # message byte 5 = option (transmode, iomode, wutime, fec, txpower)
        bits = '{0:08b}'.format(message[5])
        self.config['transmode'] = int(bits[0:1])
        self.config['iomode'] = int(bits[1:2])
        self.config['wutime'] = int(bits[2:5])
        self.config['fec'] = int(bits[5:6])
        self.config['txpower'] = int(bits[6:])

    def encodeConfig(self, headercommand):
        ''' encode the config dictionary to create the config message of the ebyte E220 LoRa module '''
        # Initialize config message
        message = []
        # message byte 0 = header
        message.append(headercommand)
        # message byte 1 = high address
        message.append(self.config['address'] // 256)
        # message byte 2 = low address
        message.append(self.config['address'] % 256)
        # message byte 3 = speed (parity, baudrate, datarate)
        bits = '0b'
        bits += self.PARSTR.get(self.config['parity'])
        bits += self.BAUDRATE.get(self.config['baudrate'])
        bits += self.DATARATE.get(self.config['datarate'])
        message.append(int(bits))
        # message byte 4 = channel
        message.append(self.config['channel'])
        # message byte 5 = option (transmode, iomode, wutime, fec, txpower)
        bits = '0b'
        bits += str(self.config['transmode'])
        bits += str(self.config['iomode'])
        bits += '{0:03b}'.format(self.config['wutime'])
        bits += str(self.config['fec'])
        bits += '{0:02b}'.format(self.config['txpower'])
        message.append(int(bits))
        return message

    def showConfig(self):
        ''' Show the config parameters of the ebyte E32 LoRa module on the shell '''
        print('=================== CONFIG =====================')
        print('model       \tE32-%s' % (self.config['model']))
        print('frequency   \t%dMhz' % (self.config['frequency']))
        print('address     \t0x%04x' % (self.config['address']))
        print('channel     \t0x%02x' % (self.config['channel']))
        print('datarate    \t%sbps' % (self.config['datarate']))
        print('port        \t%s' % (self.config['port']))
        print('baudrate    \t%dbps' % (self.config['baudrate']))
        print('parity      \t%s' % (self.config['parity']))
        print('transmission\t%s' % (self.TRANSMODE.get(self.config['transmode'])))
        print('IO mode     \t%s' % (self.IOMODE.get(self.config['iomode'])))
        print('wakeup time \t%s' % (self.WUTIME.get(self.config['wutime'])))
        print('FEC         \t%s' % (self.FEC.get(self.config['fec'])))
        maxp = self.MAXPOW.get(self.config['model'][3:6], 0)
        print('TX power    \t%s' % (self.TXPOWER.get(self.config['txpower'])[maxp]))
        print('================================================')

    def waitForDeviceIdle(self):
        ''' Wait for the E32 LoRa module to become idle (AUX pin high) '''
        count = 0
        # loop for device busy
        while not self.AUX.value():
            # increment count
            count += 1
            # maximum wait time 100 ms
            if count == 10:
                break
            # sleep for 10 ms
            utime.sleep_ms(10)

    def saveConfigToJson(self):
        ''' Save config dictionary to JSON file '''
        with open('E220config.json', 'w') as outfile:
            ujson.dump(self.config, outfile)

    def loadConfigFromJson(self):
        ''' Load config dictionary from JSON file '''
        with open('E220config.json', 'r') as infile:
            result = ujson.load(infile)
        print(self.config)

    def validmodel(self, model):
        '''define model'''
        modelsplit = model.split('-')
        if len(modelsplit) > 1 and model[1][:3] in self.FREQ and model[1][-3:-1] in self.TXPOWER:
            pass
        else:
            self.config['model'] = 'E220-900T22D'

    def calcFrequency(self):
        ''' Calculate the frequency (= base frequency + channel * 1MHz)'''
        # get base and maximum frequency
        freqkey = self.config['model'].split('-')[1][:3]
        basefreq = self.FREQ.get(freqkey)[1]
        maxfreq = self.FREQ.get(freqkey)[2]

        # calculate frequency
        freq = basefreq + self.config['channel']
        if freq > maxfreq:
            self.config['frequency'] = maxfreq
            self.config['channel'] = hex(int(maxfreq - freq))
            print('Channel is over. set to maximum channel of frequency ')
        else:
            self.config['frequency'] = freq

    def setTransmissionMode(self, transmode):
        ''' Set the transmission mode of the E32 LoRa module '''
        if transmode != self.config['transmode']:
            self.config['transmode'] = transmode
            self.setConfig('SetRegister')

    def setConfig(self, save_cmd):
        ''' Set config parameters for the ebyte E220 LoRa module '''
        try:
            # send the command
            result = self.sendCommand(save_cmd)
            # check result
            if len(result) != 6:
                return "NOK"
            # debug
            if self.debug:
                # decode result
                self.decodeConfig(result)
                # show config
                self.showConfig()
            # save config to json file
            self.saveConfigToJson()
            return "OK"

        except Exception as E:
            if self.debug:
                print('Error on setConfig: ', E)
            return "NOK"

    def setOperationMode(self, mode):
        ''' Set operation mode of the E220 LoRa module '''
        # get operation mode settings (default TxRx)
        bits = self.OPERMODE.get(mode, '00')
        # set operation mode
        self.M0.value(int(bits[0]))
        self.M1.value(int(bits[1]))
        # wait a moment
        utime.sleep_ms(50)
