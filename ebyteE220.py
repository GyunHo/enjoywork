from machine import Pin, UART
import utime, ujson


def itr_comb(array):
    results = [[]]
    for i in range(len(array)):
        temp = []
        for res in results:
            for element in array[i]:
                temp.append(res + [element])
        results = temp
    return results


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
    SPB_8N1_ = '11'


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


class ebyteE220:
    TXPOWER = {'22': TxPower22, '30': TxPower30}
    MODELS = ['E220-400T22S', 'E220-400T30S', 'E220-900T22S', 'E220-900T30S',
              'E220-400T22D', 'E220-400T30D', 'E220-900T22D', 'E220-900T30D']
    OPERATION_MODE = {'TxRx': '00', 'WORTx': '01', 'WORRx': '10', 'DeepSleep': '11'}
    BAUD_RATE = {'000': 1200, '001': 2400, '010': 4800, '011': 9600, '100': 19200, '101': 38400, '110': 57600,
                 '111': 115200}
    PARITY = {'00': None, '01': 1, '10': 0, '11': None}

    def __init__(self, m0, m1, aux, address=0x0000, channel=71, model='E220-900T22D', uart_port=1,
                 tx=4, rx=5, debug=False):
        self.PinM0 = m0  # M0 pin number
        self.PinM1 = m1  # M1 pin number
        self.PinAUX = aux  # AUX pin number
        self.PinTX = tx
        self.PinRX = rx
        self.M0 = Pin(self.PinM0, Pin.OUT, value=0)  # M0 Pin for set operation mode
        self.M1 = Pin(self.PinM1, Pin.OUT, value=0)  # M1 Pin for set operation mode
        self.AUX = Pin(self.PinAUX, Pin.IN, Pin.PULL_UP)  # AUX Pin for device status (device status : 0=busy, 1=idle)
        self.device = None  # instance for UART
        self.debug = debug

        self.serial_baud_rate = 9600
        self.model = model
        if model not in self.MODELS:
            print('Unknown model name!')
            print('Set default model : E220-900T22D')
            self.model = 'E220-900T22D'
        self.uart_port = uart_port  # UART channel(default U1)
        self.address = address  # Lora module address(default 0x0000)

        '''ADDH'''
        self.addh = self.address // 256  # ADDH (default 00)

        '''ADDL'''
        self.addl = self.address % 256  # ADDL (default 00)

        '''REG0'''
        self.reg0_index = 5
        self.lora_baud_rate = UartSerialPortRate.BAUDRATE_9600  # UART baudrate (default 9600)
        self.serial_parity = SerialParityBit.SPB_8N1  # UART Parity (default 8N1)
        self.air_data_rate = AirDataRate.ADR_2_4k  # wireless baudrate (default 2.4k)
        self.reg0 = [UartSerialPortRate, SerialParityBit, AirDataRate]

        '''REG1'''
        self.reg1_index = 6
        self.sub_packet_setting = SubPacketSetting.SPS_200bytes
        self.rssi_ambient_noise = RSSI_AmbientNoiseEnable.RSSI_ANE_Disable  # RSSI Ambient noise (default disable)
        self.transmitting_power = TxPower22.dBm22

        '''REG2'''
        self.reg1_index = 7
        self.channel = channel  # target channel (default (base 850Mhz) + channel 71Mhz = 921Mhz for Korea)

        '''REG3'''
        self.reg3_index = 8
        self.enable_rssi_byte = EnableRSSIByte.EnableRSSIByte_Disable  # default disable
        self.transmission_method = TxMethod.Transparent_transmission_mode  # transmission mode (default 0 - tranparent)
        self.lbt_enable = LBTEnable.LBT_Disable  # default disable
        self.wor_cycle = WORCycle.WOR_500ms  # wakeup time from sleep mode (default 0 = 500ms)

        print(self.__start())

    def __cltodict(self, cl: list):
        result = {}
        temp_keys = []
        temp_values = []
        for c in cl:
            keys = []
            values = []
            for k, v in c.__dict__.items():
                if v.isdigit():
                    keys.append(v)
                    values.append(k)
            temp_keys.append(keys)
            temp_values.append(values)
        r_k = itr_comb(temp_keys)
        r_v = itr_comb(temp_values)
        for i in range(len(r_k)):
            result[''.join(r_k[i])] = r_v[i]
        return result

    def __start(self):
        try:
            self.device = UART(self.uart_port, baudrate=self.serial_baud_rate,
                               parity=self.PARITY[self.serial_parity], tx=Pin(self.PinTX), rx=Pin(self.PinRX),
                               bits=8)
            if self.debug:
                print(self.device)
            return "UART START OK"

        except Exception as E:
            if self.debug:
                print("error on start UART", E)
            return "UART START NOK"

    def __set_cmd_mode(self):
        try:
            self.device = UART(self.uart_port, baudrate=9600,
                               parity=None, tx=Pin(self.PinTX), rx=Pin(self.PinRX),
                               bits=8)
            if self.debug:
                print(self.device)
            return "Enter command setting mode"
        except Exception as E:
            print(E)
            return "Error entering command setting mode"

    def __wait_idle(self):
        utime.sleep_ms(50)
        count = 0
        while not self.AUX.value():
            if count > 10:
                break
            count += 1
            utime.sleep_ms(50)

    def __set_operation_mode(self, mode):
        bits = ebyteE220.OPERATION_MODE.get(mode, '00')
        self.M0.value(int(bits[0]))
        self.M1.value(int(bits[1]))
        self.__wait_idle()
        if self.debug:
            print('change mode -> ' + mode)

    def decode_response(self, response):
        if len(response) < 9 or 0xc1 not in response:
            return False
        else:
            resp = []
            for i in response:
                resp.append(i)
            c1_index = resp.index(0xc1)
            result = resp[c1_index:]
            return result

    def encode_cmd(self):
        command = [0xc0, 0x00, 0x06, self.addh, self.addl]
        reg0 = [self.lora_baud_rate, self.serial_parity, self.air_data_rate]
        reg1 = [self.sub_packet_setting, self.rssi_ambient_noise, self.transmitting_power]
        reg2 = self.channel
        reg3 = [self.enable_rssi_byte, self.transmission_method, self.lbt_enable, self.wor_cycle]
        command.append(int('0b' + ''.join(reg0)))
        command.append(int('0b' + ''.join(reg1)))
        command.append(reg2)
        command.append(int('0b' + ''.join(reg3)))
        return command

    def decode_cmd(self, res):

        pass

    def send_cmd(self, cmd: list):
        try:
            self.__set_operation_mode('DeepSleep')
            self.__set_cmd_mode()
            self.__wait_idle()
            self.device.write(bytes(cmd))
            self.__wait_idle()
            res = self.decode_response(self.device.read())
            self.__start()
            if self.debug:
                print(f'sending command -> {cmd}')
                print(f'result of sending command -> {res}')
            return res
        except Exception as E:
            if self.debug:
                print(f'sending command error -> {E}')
            return False

    def get_config(self):
        read_reg_cmd = [0xc1, 0x00, 0x06]
        res = self.send_cmd(read_reg_cmd)
        if not res:
            return 'get config error'
        else:
            return res

    def set_add(self, add: int):
        self.addh = add // 256
        self.addl = add % 256
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set address error'
        else:
            return self.get_config()

    def set_spr(self, rate):
        self.lora_baud_rate = rate
        if rate not in self.BAUD_RATE.keys():
            self.lora_baud_rate = UartSerialPortRate.BAUDRATE_9600
            print(f'input baud rate "{rate}"wrong ->set default 9600 ')
        self.serial_baud_rate = self.BAUD_RATE.get(rate, 9600)
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set spr error'
        else:
            print(f'set spr OK')
            print(self.device)
            return self.get_config()

    def set_spb(self, parity):
        self.serial_parity = parity
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set parity bit error'
        else:
            return self.get_config()

    def set_adr(self, airdata):
        self.air_data_rate = airdata
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set air data rate error'
        else:
            return self.get_config()

    def set_sub_packet(self, packet):
        self.sub_packet_setting = packet
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set sub-packet setting error'
        else:
            return self.get_config()

    def set_tx_power(self, power):
        self.transmitting_power = power
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set transmitting power error'
        else:
            return self.get_config()

    def set_ch(self, ch: int):
        self.channel = ch
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set channel error'
        else:
            conf = self.get_config()
            if ch > conf[7]:
                print(f'maximum channel for this module is {conf[7]}')
                print(f'channel set {conf[7]}')
            return self.get_config()

    def set_trans_mode(self):
        self.transmission_method = TxMethod.Transparent_transmission_mode
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set transparent transmission mode error'
        else:
            return self.get_config()

    def set_fixed_mode(self):
        self.transmission_method = TxMethod.Fixed_transmission_mode
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set fixed transmission mode error'
        else:
            return self.get_config()

    def set_wor(self, cycle):
        self.wor_cycle = cycle
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set WOR cycle error'
        else:
            return self.get_config()

    def tx(self, message, mode='TxRx'):
        try:
            self.__set_operation_mode(mode)
            self.device.write(message)
            if self.debug:
                print(f'send massage -> {message}')
            while not self.AUX.value():
                print('SENDING.......')
                utime.sleep_ms(100)
            return 'SEND OK'
        except Exception as E:
            print(E)
            return 'SEND FALSE'

    def rx(self, mode='TxRx'):
        try:
            self.__set_operation_mode(mode)
            msg = self.device.read().strip(b'\x00')
            if msg:
                return msg
        except Exception as E:
            print(E)

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
        try:
            self.setOperationMode('DeepSleep')
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
