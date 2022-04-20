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
    RSSIAmbientNoise_Disable = '0000'
    RSSIAmbientNoise_Enable = '1000'


class TxPower30:
    dBm_30 = '00'
    dBm_27 = '01'
    dBm_24 = '10'
    dBm_21 = '11'


class TxPower22:
    dBm_22 = '00'
    dBm_17 = '01'
    dBm_13 = '10'
    dBm_10 = '11'


class EnableRSSIByte:
    EnableRSSIByte_Disable = '0'
    EnableRSSIByte_Enable = '1'


class TxMethod:
    '''with REG3-5 Reserve'''
    '''disalbe 0 or enable 1 + reserve 0 '''
    Transparent_mode = '00'
    Fixed_mode = '01'


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
    BASE_FREQ = {'400': '', '900': 850.125}
    MODELS = ['E220-400T22S', 'E220-400T30S', 'E220-900T22S', 'E220-900T30S',
              'E220-400T22D', 'E220-400T30D', 'E220-900T22D', 'E220-900T30D']
    OPERATION_MODE = {'TxRx': '00', 'WORTx': '01', 'WORRx': '10', 'DeepSleep': '11'}
    BAUD_RATE = {'000': 1200, '001': 2400, '010': 4800, '011': 9600, '100': 19200, '101': 38400, '110': 57600,
                 '111': 115200}
    PARITY = {'00': None, '01': 1, '10': 0, '11': None}

    def __init__(self, m0, m1, aux, address=0x0000, channel=71, model='E220-900T22D', uart_port=1,
                 tx=4, rx=5, debug=False):
        self.mode = 'TxRx'
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

        self.model = model
        if model not in self.MODELS:
            print('Unknown model name!')
            print('Set default model : E220-900T22D')
            self.model = 'E220-900T22D'
        self.uart_port = uart_port  # UART channel(default U1)
        self.address = address  # Lora module address(default 0x0000)

        '''ADDH'''
        self.addh_index = 3
        self.addh = self.address // 256  # ADDH (default 00)

        '''ADDL'''
        self.addl_index = 4
        self.addl = self.address % 256  # ADDL (default 00)

        '''REG0'''
        self.reg0_index = 5
        self.lora_baud_rate = str('{0:08b}'.format(self.get_config()[self.reg0_index]))[
                              :3]  # UART baudrate set from lora module config
        self.serial_parity = SerialParityBit.SPB_8N1  # UART Parity (default 8N1)
        self.air_data_rate = AirDataRate.ADR_2_4k  # wireless baudrate (default 2.4k)

        '''REG1'''
        self.reg1_index = 6
        self.sub_packet_setting = SubPacketSetting.SPS_200bytes
        self.rssi_ambient_noise = RSSI_AmbientNoiseEnable.RSSIAmbientNoise_Disable  # RSSI Ambient noise (default disable)
        self.transmitting_power = TxPower22.dBm_22  # tx power default 22dBm

        '''REG2'''
        self.reg2_index = 7
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
            self.device = UART(self.uart_port, baudrate=self.BAUD_RATE.get(self.lora_baud_rate),
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

    def __decode_cmd(self, cmd: int):
        return '{0:08b}'.format(cmd)

    def show(self):
        device_config = self.get_config()
        reg0_list = [UartSerialPortRate, SerialParityBit, AirDataRate]
        reg1_list = [SubPacketSetting, RSSI_AmbientNoiseEnable, TxPower22]
        reg3_list = [EnableRSSIByte, TxMethod, LBTEnable, WORCycle]
        if self.model[-3:-1] == '30':
            reg1_list[3] = TxPower30
        reg0_dict = self.__cltodict(reg0_list)
        reg1_dict = self.__cltodict(reg1_list)
        reg3_dict = self.__cltodict(reg3_list)
        reg0_cmd = device_config[self.reg0_index]
        reg1_cmd = device_config[self.reg1_index]
        channel = device_config[self.reg2_index]
        reg3_cmd = device_config[self.reg3_index]
        reg0_key = '{0:08b}'.format(reg0_cmd)
        reg1_key = '{0:08b}'.format(reg1_cmd)
        reg3_key = '{0:08b}'.format(reg3_cmd)
        reg0 = reg0_dict[reg0_key]
        reg1 = reg1_dict[reg1_key]
        reg3 = reg3_dict[reg3_key]
        print('=' * 20 + 'Config' + '=' * 20)
        print('model                 ', self.model)
        print('frequency             ', self.BASE_FREQ.get(self.model.split('-')[1][:3], 850.125) + int(channel), 'Mhz')
        print('address               ', hex(device_config[self.addh_index] * 256 + device_config[self.addl_index]))
        print('uart rate             ', reg0[0])
        print('parity bit            ', reg0[1])
        print('air data rate         ', reg0[2])
        print('sub-packet setting    ', reg1[0].split('_')[1])
        print('RSSI ambient noise    ', reg1[1].split('_')[1])
        print('transmitting power    ', reg1[2].split('_')[1] + 'dBm')
        print('channel               ', channel)
        print('RSSI byte             ', reg3[0].split('_')[1])
        print('transmission method   ', reg3[1])
        print('LBT enable            ', reg3[2].split('_')[1])
        print('WOR cycle             ', reg3[3].split('_')[1])
        print('=' * 46)

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
            return 'OK'

    def set_spr(self, rate):
        self.lora_baud_rate = rate
        if rate not in self.BAUD_RATE.keys():
            self.lora_baud_rate = UartSerialPortRate.BAUDRATE_9600
            print(f'input baud rate "{rate}"wrong ->set default 9600 ')
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set spr error'
        else:
            print(f'set spr OK')
            print(self.device)
            return 'OK'

    def set_spb(self, parity):
        self.serial_parity = parity
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set parity bit error'
        else:
            return 'OK'

    def set_adr(self, airdata):
        self.air_data_rate = airdata
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set air data rate error'
        else:
            return 'OK'

    def set_sub_packet(self, packet):
        self.sub_packet_setting = packet
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set sub-packet setting error'
        else:
            return 'OK'

    def set_tx_power(self, power):
        self.transmitting_power = power
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set transmitting power error'
        else:
            return 'OK'

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
            return 'OK'

    def set_trans_mode(self):
        self.transmission_method = TxMethod.Transparent_transmission_mode
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set transparent transmission mode error'
        else:
            return 'OK'

    def set_fixed_mode(self):
        self.transmission_method = TxMethod.Fixed_transmission_mode
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set fixed transmission mode error'
        else:
            return 'OK'

    def set_wor(self, cycle):
        self.wor_cycle = cycle
        res = self.send_cmd(self.encode_cmd())
        if not res:
            return 'set WOR cycle error'
        else:
            return 'OK'

    def tx(self, message, mode='TxRx'):
        try:
            om = str(self.M0.value()) + str(self.M1.value())
            if om != self.OPERATION_MODE.get(mode):
                self.mode = mode
                self.__set_operation_mode(self.mode)
                self.__wait_idle()
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
            om = str(self.M0.value()) + str(self.M1.value())
            if om != self.OPERATION_MODE.get(mode):
                self.mode = mode
                self.__set_operation_mode(self.mode)
                self.__wait_idle()
            res = self.device.read()
            if res:
                msg = res.strip(b'\x00')
                if msg:
                    return msg
        except Exception as E:
            print(E)
            return 'RECEIVE FAIL'

    def calcChecksum(self, payload):
        ''' Calculates checksum for sending/receiving payloads. Sums the ASCII character values mod256 and returns
            the lower byte of the two's complement of that value in hex notation. '''
        return '%2X' % (-(sum(ord(c) for c in payload) % 256) & 0xFF)
