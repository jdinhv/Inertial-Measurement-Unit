
import smbus
from smbus import SMBus
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.SPI as SPI
from Adafruit_I2C import Adafruit_I2C
from tabulate import tabulate
import time
import re
import os
from math import *

WORDLEN = 32

bus = smbus.SMBus(2)

# I2C Address of the device
LSM6DS0_MAG_ADDRESS				= 0x1E
LSM6DS0_ACCL_ADDRESS			        = 0x6B
LSM6DS0_GYRO_ADDRESS			        = 0x6B

# LSM9DS0 gyrometer registers
LSM6DS0_WHO_AM_I_G				= 0x0F
LSM6DS0_CTRL_REG1_G				= 0x10
LSM6DS0_CTRL_REG4_G				= 0x11

LSM6DS0_OUT_X_L_G				= 0x22
LSM6DS0_OUT_X_H_G				= 0x23
LSM6DS0_OUT_Y_L_G				= 0x24
LSM6DS0_OUT_Y_H_G				= 0x25
LSM6DS0_OUT_Z_L_G				= 0x26
LSM6DS0_OUT_Z_H_G				= 0x27

# Gyro Datarate & Bandwidth configuration
LSM6DS0_GYRO_DR_95				= 0x00 # ODR = 95 Hz
LSM6DS0_GYRO_DR_190				= 0x40 # ODR = 190 Hz
LSM6DS0_GYRO_DR_380				= 0x80 # ODR = 380 Hz
LSM6DS0_GYRO_DR_760				= 0xC0 # ODR = 760 Hz
LSM6DS0_GYRO_BW_12_5			        = 0x00 # Cutoff = 12.5
LSM6DS0_GYRO_BW_25				= 0x10 # Cutoff = 25
LSM6DS0_GYRO_BW_50				= 0x20 # Cutoff = 50
LSM6DS0_GYRO_BW_70				= 0x30 # Cutoff = 70

# Gyro Power & Axis configuration
LSM6DS0_GYRO_PD					= 0x00 # Power down mode, Axis disabled
LSM6DS0_GYRO_ND					= 0x08 # Normal mode
LSM6DS0_GYRO_XAXIS				= 0x04 # X-Axis enabled
LSM6DS0_GYRO_YAXIS				= 0x02 # Y-Axis enabled
LSM6DS0_GYRO_ZAXIS				= 0x01 # Z-Axis enabled

# Gyro Full-scale selection & Mode configuration
LSM6DS0_GYRO_DEFAULT			        = 0x00 # Continuous update, LSB first, Normal mode
LSM6DS0_GYRO_BDU				= 0x80 # Output registers not updated until MSB and LSB read
LSM6DS0_GYRO_BLE_MSB			        = 0x40 # MSB first
LSM6DS0_GYRO_SCALE_245			        = 0x00 # 245 dps
LSM6DS0_GYRO_SCALE_500			        = 0x10 # 500 dps
LSM6DS0_GYRO_SCALE_2000			        = 0x20 # 2000 dps
LSM6DS0_GYRO_ST_0				= 0x02 # Self-Test 0
LSM6DS0_GYRO_ST_1				= 0x06 # Self-Test 1

# Shared addresses between Magnetometer & Accelerometer
LSM6DS0_WHO_AM_I_XM				= 0x0F
LSM6DS0_CTRL_REG1_XM			        = 0x10
LSM6DS0_CTRL_REG2_XM			        = 0x17
LSM9DS0_CTRL_REG5_XM			        = 0x24
LSM9DS0_CTRL_REG6_XM			        = 0x25
LSM9DS0_CTRL_REG7_XM			        = 0x26

# Accelerometer addresses
LSM6DS0_OUT_X_L_A				= 0x28
LSM6DS0_OUT_X_H_A				= 0x29
LSM6DS0_OUT_Y_L_A				= 0x2A
LSM6DS0_OUT_Y_H_A				= 0x2B
LSM6DS0_OUT_Z_L_A				= 0x2C
LSM6DS0_OUT_Z_H_A				= 0x2D

# Accl Datarate configuration
LSM6DS0_ACCL_DR_PD				= 0x00 # Power down mode
LSM6DS0_ACCL_DR_3_125			        = 0x10 # ODR = 3.125 Hz
LSM6DS0_ACCL_DR_6_25			        = 0x20 # ODR = 6.25 Hz
LSM6DS0_ACCL_DR_12_5			        = 0x30 # ODR = 12.5 Hz
LSM6DS0_ACCL_DR_25				= 0x40 # ODR = 25 Hz
LSM6DS0_ACCL_DR_50				= 0x50 # ODR = 50 Hz
LSM6DS0_ACCL_DR_100				= 0x60 # ODR = 100 Hz
LSM6DS0_ACCL_DR_200				= 0x70 # ODR = 200 Hz
LSM6DS0_ACCL_DR_400				= 0x80 # ODR = 400 Hz
LSM6DS0_ACCL_DR_800				= 0x90 # ODR = 800 Hz
LSM6DS0_ACCL_DR_1600			        = 0xA0 # ODR = 1600 Hz

# Accl Data update & Axis configuration
LSM6DS0_ACCL_BDU				= 0x00 # Continuous update, Axis disabled
LSM6DS0_ACCL_XAXIS				= 0x04 # X-Axis enabled
LSM6DS0_ACCL_YAXIS				= 0x02 # Y-Axis enabled
LSM6DS0_ACCL_ZAXIS				= 0x01 # Z-Axis enabled

# Acceleration Full-scale selection
LSM6DS0_ACCL_RANGE_2G			        = 0x00 # Full scale = +/-2g
LSM6DS0_ACCL_RANGE_4G			        = 0x08 # Full scale = +/-4g
LSM6DS0_ACCL_RANGE_6G			        = 0x10 # Full scale = +/-6g
LSM6DS0_ACCL_RANGE_8G			        = 0x18 # Full scale = +/-8g
LSM6DS0_ACCL_RANGE_16G			        = 0x20 # Full scale = +/-16g

# Register





class mavs():
 

  def __init__(self, cfg_file = '../cfg/pin.cfg', map_file = '../cfg/pin.map', \
          reg_file = '../cfg/reg.map', field_file='../cfg/field.map', \
          mem_test_cfg_file='../cfg/mem.cfg'):
    self.map_file = map_file
    self.cfg_file = cfg_file
    self.reg_file = reg_file
    self.mem_test_cfg_file = mem_test_cfg_file
    self.field_file = field_file
    self.cfg_list = self.load_cfg()
    self.reg_map = self.load_reg(self.reg_file)
    self.field_map = self.parse_fields(self.field_file)
    self.cfg_pins()
#    self.off()
    self.enable_spi()
    self.construct_rbscan()
    self.mem_test_list = self.config_mem(self.mem_test_cfg_file)
    ############################################################
  
    self.gyro_datarate()
    self.gyro_scale_selection()
    self.accl_datarate()
    self.accl_scale_selection()	
    self.turn_on_accelerometer()
    #self.turn_on_gyroscope()
    self.sig_motion()
    self.six_D_detection()

    

  def enable_spi(self):
    self.spi = SPI.SPI(1,0)
    self.spi.bpw = 16
    self.spi.mode = 1

  def map_pins(self):
    pinmap = dict()
    f = open(self.map_file, 'r')
    for line in f.readlines():
      words = line.replace('\n','').split(' ')
      words = filter(None, words)
      pinmap[words[0]] = words[1]
    f.close()
    return pinmap

  def load_cfg(self):
    pinmap = self.map_pins()
    cfg_list = dict()
    f = open(self.cfg_file, 'r')
    for line in f.readlines():
      cfg = dict()
      words = line.replace('\n','').split(' ')
      words = filter(None, words)
      pin = words[0]
      name = words[1]
      io = words[2]
      val = words[3]
      cfg['bb_pin'] = pin
      cfg['io'] = io
      cfg['val'] = val
      cfg['pin'] = pinmap[pin]
      cfg_list[name] = cfg
    f.close()
    return cfg_list

  def load_reg(self, reg_file):
    reg_map = dict()
    f = open(reg_file, 'r')
    for line in f.readlines():
      words = line.replace('\n','').split(' ')
      words = filter(None, words)
      reg_map[words[0]] = words[1]
    return reg_map

  def cfg_pins(self):
    print('configure GPIOs')
    for key in self.cfg_list.keys():
      cfg = self.cfg_list[key]
      if cfg['io'] == 'INPUT':
        GPIO.setup(cfg['bb_pin'], GPIO.IN)
      else:
        GPIO.setup(cfg['bb_pin'], GPIO.OUT)
        if cfg['val'] == '0':
          GPIO.output(cfg['bb_pin'], GPIO.LOW)
        else:
          GPIO.output(cfg['bb_pin'], GPIO.HIGH)
    print('DONE configure GPIOs')

  def parse_fields(self,field_file):
    f = open(field_file)
    fields = dict()
    for line in f.readlines():
        words = line.replace('\n','').split(' ')
        words = filter(None, words)
        field = dict()
        names = words[0].split('[')
        if len(names) > 1:
            length = int(names[1].replace(']','').split(':')[0]) + 1
        else:
            length = 1
        name = names[0]
        locations = words[1].replace('[','').replace(']','').split(':')
        if len(locations) > 1:
          location = int(locations[1])
        else:
          location = int(locations[0])
        maskb = '0b'
        for ii in range(WORDLEN - location - length):
            maskb = maskb + '0'
        for ii in range(length):
            maskb = maskb + '1'
        for ii in range(location):
            maskb = maskb + '0'

        mask = int(maskb,2)
        maskh = hex(mask)

        # parsing default value
        default = words[2]
        if default == '0':
            default = 0
        else:
            bh = default.split('\'')[1]
            if bh.startswith('b'):
                bh = bh.replace("b","")
                default = int(bh,2)
            elif bh.startswith('h'):
                bh = bh.replace("h","")
                default = int(bh,16)
            else:
                print("something is wrong")
        # combine default value in the register
        default_comb = default << (location)
        # sainty check
        if default_comb & mask != default_comb:
          print('something is wront')
          print(words)

        RW = False if words[3] == 'R' else True
        reg = words[4]
        field['length'] = length
        field['mask'] = mask
        field['default'] = default
        field['default_comb'] = default_comb
        field['RW'] = RW
        field['reg'] = reg 
        field['location'] = location

        fields[name] = field
    f.close()
    return fields

  def set_pin(self, pin, value):
    cfg = self.cfg_list[pin]
    if value == 0:
      GPIO.output(cfg['bb_pin'], GPIO.LOW)
    else:
      GPIO.output(cfg['bb_pin'], GPIO.HIGH)

  def get_pin(self, pin):
    cfg = self.cfg_list[pin]
    value = GPIO.input(cfg['bb_pin'])
    return value


  def mav_reset(self):
    print("reset Maverick")
    self.set_pin('MAV_RESETN', 0)

  def mav_enable(self):
    print("enable Maverick")
    self.set_pin('MAV_RESETN', 1)

  def slp_reset(self):
    print("reset Maverick sleep module")
    self.set_pin('SLEEP_RESETN', 0)

  def slp_enable(self):
    print("enable Maverick sleep module")
    self.set_pin('SLEEP_RESETN', 1)

  def pll_disable(self):
    #print("disable Maverick PLL")
    self.set_pin('MAV_PLL_PD', 1)

  def pll_enable(self):
    #print("enable Maverick PLL")
    self.set_pin('MAV_PLL_PD', 0)

  def rfic_reset(self):
    print("reset RFIC")
    self.set_pin('RFIC_RESETN', 0)

  def rfic_enable(self):
    print("enable RFIC")
    self.set_pin('RFIC_RESETN', 1)


  def powerup(self,pwr):
    cfg = self.cfg_list['PWR_EN_'+pwr]
    print("enable power domain " + pwr)
    GPIO.output(cfg['bb_pin'], GPIO.HIGH)

  def powerdn(self,pwr):
    cfg = self.cfg_list['PWR_EN_'+pwr]
    print("disable power domain " + pwr)
    GPIO.output(cfg['bb_pin'], GPIO.LOW)

  def powerup_dig_1v1(self):
    self.powerup('DIG_1V1')

  def powerdn_dig_1v1(self):
    self.powerdn('DIG_1V1')

  def powerup_io(self):
    self.powerup('IO')

  def powerdn_io(self):
    self.powerdn('IO')

  def powerup_ana_1v1(self):
    self.powerup('ANA_1V1')

  def powerdn_ana_1v1(self):
    self.powerdn('ANA_1V1')

  def powerup_pll(self):
    self.powerup('PLL')

  def powerdn_pll(self):
    self.powerdn('PLL')

  def powerup_ana_2v5(self):
    self.powerup('ANA_2V5')

  def powerdn_ana_2v5(self):
    self.powerdn('ANA_2V5')

  def temp_rstn_enable(self):
   print("enable temperature reset")
   self.set_pin('TEMP_RSTN',1)


  def on(self):
    self.mav_reset()
    self.rfic_reset()
    self.slp_reset()
    self.pll_disable()

    # power up sequence:
    # high-current --> low current
    self.powerup_pll()
    self.powerup_dig_1v1()
    self.powerup_io()
    self.powerup_ana_2v5()
    self.powerup_ana_1v1()
    #time.sleep(0.01)

    self.pll_enable()
    self.mav_enable()

  #  self.write_field('PLL_CORE_BYPASS', 1)
    # this line is only for debugging purpose to bring out the clock
    self.write_field('PIN_CLK_EN', 1)
    self.write_field('PLL_VIT_FOUTPOSTDIVPD', 1)
    self.write_field('PLL_VIT_BYPASS', 1)
    self.write_field('PLL_CORE_BYPASS', 1)

    self.write_field('PLL_CORE_FOUTPOSTDIVPD', 1)

    # try to check the "hidden" register status with a very indirect method...

    # first, let's do 20 times switching to see what happens... 
    for trials in range(20):
      self.pll_disable()
      self.pll_enable()
      time.sleep(0.1)

    success = False
    trials = 0
    max_trials = 20
    while (success == False and trials < max_trials):
      temp = self.read_field('PIN_CLK_EN')
      target = 1 - temp
      self.write_field('PIN_CLK_EN', target)
      if self.read_field('PIN_CLK_EN') != target:
        print('success')
        success = True
        self.mav_reset()
        self.mav_enable()
        # this is debug purpose
        self.write_field('PIN_CLK_EN', 1)
      else:
        success = False
        self.pll_disable()
        self.pll_enable()
        # this is NOT debug purpose
        self.write_field('PIN_CLK_EN', 1)
        trials = trials + 1
      time.sleep(0.1)
    print('tried ' + str(trials) + ' times.') 
    if not success:
      print('still not working, contact Locix')
      return

    self.mav_reset()
    self.mav_enable()


    #self.write_field('PLL_CORE_FOUTPOSTDIVPD', 1)
    #self.pll_disable()
    #self.pll_enable()
    #self.mav_reset()
    #self.mav_enable()

    #self.powerup_ana_2v5()
    #self.powerup_ana_1v1()

    self.write_field('DAC_OUTMODE', 3)
    self.write_field('DAC_INT_EXT_SEL', 3)
    self.rfic_enable()
    self.slp_enable()
    self.temp_rstn_enable()

  def off(self):
    self.mav_reset()
    self.rfic_reset()
    self.slp_reset()
    self.pll_disable()

    self.powerdn_pll()
    self.powerdn_ana_1v1()
    self.powerdn_ana_2v5()
    self.powerdn_io()
    self.powerdn_dig_1v1()

  def powerdn_all(self):
    self.powerdn_pll()
    self.powerdn_ana_1v1()
    self.powerdn_ana_2v5()
    self.powerdn_io()
    self.powerdn_dig_1v1()

  def list_fields(self):
    key_list = []
    x = []
    for ii, key in enumerate(self.field_map.keys()):
      x.append(key)
      if ii % 5 == 4:
        key_list.append(x)
        x = []
    print(tabulate(key_list))

  def list_regs(self):
    key_list = []
    x = []
    for ii, key in enumerate(self.reg_map.keys()):
      x.append(key)
      if ii % 5 == 4:
        key_list.append(x)
        x = []
    print(tabulate(key_list))

  def write_field(self, field, value):
    f = self.field_map[field]
    #print(f)
    reg = f['reg']
    loc = f['location']
    reg_value = self.read_reg(reg)
    new_value = (reg_value & (~f['mask'])) + ((value << loc) & f['mask'])
    self.write_reg(reg, new_value)

  def read_field(self, field):
    f = self.field_map[field]
    reg = f['reg']
    loc = f['location']
    reg_value = self.read_reg(reg)
    value = int((reg_value & f['mask']) >> loc)
    return value

  def update_default(self, field, default):
    f = self.field_map[field]
    location = f['location']
    mask = f['mask']
    # combine default value in the register
    default_comb = default << (location)
    # sainty check
    if default_comb & mask != default_comb:
      print('something is wront')
      print(words)
      return -1
    f['default'] = default
    f['default_comb'] = default_comb
    self.construct_rbscan()
    return 0



  def construct_rbscan(self):
    rbscan = []
    for ii in range(32*len(self.reg_map)):
      rbscan.append(-1)
    for field in self.field_map.keys():
      f = self.field_map[field]
      if not f['RW']:
          continue
      reg = f['reg']
      reg_addr = (int(self.reg_map[reg], 16) - int('0x9000',16))/4
      default_bin = []
      for digit in '{:0{width}b}'.format(f['default'],width=f['length'])[::-1]:
        default_bin.append(int(digit))
      for ii in range(f['length']):
        rbscan[(reg_addr)*32 + ii + f['location']] = default_bin[ii]
    #for jj in range(int(len(rbscan)/32)):
    #  print(rbscan[jj*32:jj*32+32])
    new_rbscan = []
    for jj in rbscan:
      if jj > -1:
        new_rbscan.append(jj)
    #print(new_rbscan[::-1])
    print('RBSCAN total length: ' + str(len(new_rbscan)))
    self.rbscan = new_rbscan[::-1]
    return 0

  def load_rbscan(self):
    self.set_pin('MAV_RBSCAN_CLK', 0)
    self.set_pin('MAV_RBSCAN_IN', 0)
    self.set_pin('MAV_RBSCAN_UP', 0)
    
    for data in self.rbscan:
      self.set_pin('MAV_RBSCAN_IN', data)
      self.set_pin('MAV_RBSCAN_CLK', 1)
      self.set_pin('MAV_RBSCAN_CLK', 0)

    self.set_pin('MAV_RBSCAN_UP', 1)
    self.set_pin('MAV_RESETN', 1)
    self.set_pin('MAV_RESETN', 0)
    return 0

  def check_rbscan(self):
    self.set_pin('MAV_RBSCAN_CLK', 0)
    self.set_pin('MAV_RBSCAN_IN', 0)
    self.set_pin('MAV_RBSCAN_UP', 0)
    
    for data in self.rbscan:
      self.set_pin('MAV_RBSCAN_IN', data)
      self.set_pin('MAV_RBSCAN_CLK', 1)
      self.set_pin('MAV_RBSCAN_CLK', 0)
    
    ii = 0
    prev_data = 0
    self.rbscan_out = []
#    for data in self.rbscan:
    for ii in range(len(self.rbscan)-1):
      
      self.set_pin('MAV_RBSCAN_IN', 0)
      self.set_pin('MAV_RBSCAN_CLK', 1)
      self.set_pin('MAV_RBSCAN_CLK', 0)
      ret = self.get_pin('MAV_RBSCAN_OUT')
      self.rbscan_out.append(ret)
      prev_data = self.rbscan[ii+1] 
      if prev_data != ret:
        print('RBSCAN failed at location ' + str(ii))
      #ii = ii + 1

    print("==============================")
    for ii in range(len(self.rbscan)):
        if ii % 50 == 49:
            print(self.rbscan[ii])
        else:
            print(self.rbscan[ii]),
    print("==============================")
    for ii in range(len(self.rbscan_out)):
        if ii % 50 == 49:
            print(self.rbscan_out[ii])
        else:
            print(self.rbscan_out[ii]),
    print("==============================")

    return 0

  def construct_spi(self, rw, addr, value = 0, burst = 1):
    # read  : rw = 0
    # write : rw = 1 
    addr_flip = int('{:0{width}b}'.format(addr, width=WORDLEN)[::-1],2)
    if burst == 1:
        b = 0
    elif burst == 4:
        b = 2 #0x01, flipped
    elif burst == 8:
        b = 1 #0x10, flipped
    elif burst == 16:
        b = 3 #0x11, flipped
    cmd = (rw << 31) + (addr_flip & 0x3FFFFF00) + b
    dat = int('{:0{width}b}'.format(value,width=WORDLEN)[::-1],2)
    #print('{:032b}'.format(cmd))
    #print('{:032b}'.format(dat))
    cmd0 = int((cmd >> 24) & 0xFF)
    cmd1 = int((cmd >> 16) & 0xFF)
    cmd2 = int((cmd >>  8) & 0xFF)
    cmd3 = int(cmd & 0xFF) 
    dat0 = int((dat >> 24) & 0xFF)
    dat1 = int((dat >> 16) & 0xFF)
    dat2 = int((dat >>  8) & 0xFF)
    dat3 = int(dat & 0xFF)
    tx = [cmd1, cmd0, cmd3, cmd2, dat1, dat0, dat3, dat2]
    return tx

  def write_reg(self, reg, value):
    if type(reg) == str:
      addr = int(self.reg_map[reg], 16) 
    elif type(reg) == int:
      addr = reg
    tx = self.construct_spi(1, addr, value)
    #print('w reg: ' + str(reg) + ', value:' + str(value))
    rx = self.spi.xfer2(tx)
    return 0

  def read_reg(self, reg):
    if type(reg) == str:
      addr = int(self.reg_map[reg], 16)
    elif type(reg) == int:
      addr = reg
    tx = self.construct_spi(0, addr)
    rx = self.spi.xfer2(tx)
    tmp = (rx[5] << 24) + (rx[4] << 16) + (rx[7] << 8) + rx[6]
    ret = int('{:0{width}b}'.format(tmp,width=WORDLEN)[::-1],2)
    #print('r reg: ' + str(reg) + ', value:' + str(ret))
    return ret


  def config_mem(self, config):
    mem_list = dict()
    f = open(config, 'r')
    for line in f.readlines():
      words = re.split(' ', line)
      words = filter(None, words)
      mem_list[words[1]] = int(words[3], 16)
    f.close()
    return mem_list

  def do_nothing(self):
    return

  def parse_test(self,line):
    words = re.split(" |:", line.replace('\n',''))
    cmd = dict()
    cmd['name'] = words[0]
    if cmd['name'] == 'MALLOC':
      cmd['mem'] = words[2]+':'+words[3]
      cmd['base'] = int(words[4])
    elif cmd['name'] == 'WRW' or cmd['name'] == 'POL':
      if words[2].startswith('REG'):
        cmd['mem'] = words[2]
        ii = 2
      else:
        cmd['mem'] = words[2]+':'+words[3]
        ii = 3
      cmd['offset'] = int(words[ii+1], 16)
      cmd['value'] = int(words[ii+2], 16)
      if cmd['name'] == 'POL':
        cmd['mask'] = int(words[ii+3], 16)
    return cmd
  def run_test(self, test):
    mem_list = dict()
    f = open(test)
    # constants
    mem_list = self.mem_test_list 

    cmd_list = []
    addr_list = []
    for line in f.readlines():
      if line[0:2] == '--':
        self.do_nothing()
      cmd = self.parse_test(line)
      if cmd['name'] == 'MALLOC':
        mem_list[cmd['mem']] = cmd['base']
        #print('MALLOC: {:08x}'.format(cmd['base']))
      if False and (cmd['base'] & 0xFF000000 != 0x02000000):
        f.close()
        print('out of range:'+str(hex(cmd['base'])))
        break
      elif cmd['name'] == 'COM':
        self.do_nothing()
      elif cmd['name'] == 'IDL':
        self.do_nothing()
      elif cmd['name'] == 'WRW':
        cmd['base'] = mem_list[cmd['mem']]
        cmd_list.append(cmd)
        if cmd['base'] not in addr_list:
          addr_list.append(cmd['base'])
      elif cmd['name'] == 'POL':
        cmd['base'] = mem_list[cmd['mem']]
        cmd_list.append(cmd)
        if cmd['base'] not in addr_list:
          addr_list.append(cmd['base'])
      else:
        self.do_nothing()
    f.close()
# checking to see if al the memory space is in the same region
# quit if not
    current_space = [] 
    print('all the memory spaces are:')
    for addr in addr_list:
      print('{:08x}'.format(addr))
      space = int(((addr & 0xFF000000) >> 24))
      if space not in current_space:
        current_space.append(space)
    print('current AXI space is : 0x{:02x}'.format(current_space[0]))
    if (len(current_space)) > 1:
      print('more than one memory space to access, quit!')
      return
    else:
      self.set_axi_space(current_space[0])
    for cmd in cmd_list:
      if cmd['name'] == 'WRW':
        addr = cmd['base']+cmd['offset']
        self.write_reg(addr, cmd['value'])
      elif cmd['name'] == 'POL':
        addr = cmd['base']+cmd['offset']
        value = int(self.read_reg(addr) & cmd['mask'])
        if value == cmd['value']:
          self.do_nothing()
          #print('matched, addr = '),
          #print(hex(addr)),
          #print(' value = '),
          #print(hex(value))
        else:
          self.do_nothing()
          print('not matched, addr = '),
          print(hex(addr)),
          print(' value = '),
          print(hex(value))
          print(' expect = '),
          print(hex(cmd['value']))

  def set_axi_space(self, space, method = 1):
    if method == 0:
      self.update_default('SPIRB_AWADDR_BITS', space)
      self.update_default('SPIRB_ARADDR_BITS', space)
      self.load_rbscan()
    elif method == 1:
      self.write_field('SPIRB_AWADDR_BITS', space)
      self.write_field('SPIRB_ARADDR_BITS', space)

  def run_all_tests(self):
    directory = "../vectors"
    tests = []
    for x in os.walk(directory):
      tests.append(x[0])
    tests.pop(0)

    for test in tests:
      self.on()
      if test == "../vectors/ecacheram" or test == "../vectors/core_mem_test":
        continue
      print('start test: '+test)
      test = test + '/out2_gate_level.txt'
      self.run_test(test)
      self.off()

  def write_reg_sp(self, reg, data):
    # write register through slave port
    # poll bits 24, 25 ,26 of MSLVCTRL1
    # write MSLVCTRL0 with target address, bit 1 and 0 should be 0
    # write the data to MSLVDATAT
    return 0

  def read_reg_sp(self, reg):
    # read register through slave port
    # poll bits 24, 25 ,26 of MSLVCTRL1
    # write MSLVCTRL0 with target address, bit 1 and 0 should be 0
    # read the data from MSLVDATATX
    return 0

  def dump_memory(self):
    # dump 32K memory, 4-byte aligned
    for ii in range(32*256):
      print(hex(self.read_reg(ii)))

  def test_adc_mode1(self):
    self.write_field('RB_DBG_ADDR', int('0b10000', 2) << 15)
  def test_adc_mode3(self):
    self.write_field('ADC_OM', 5)
    while (self.read_field('ADC_CALBUSY') > 0):
      print('.'),
    self.write_field('RB_DBG_ADDR', int('0b10001', 2) << 15)
  def read_ram(self, filename, length = 32*1024):
    f = open(filename, 'w')
    for ii in range(length):
      if length > 32*1024:
        print('too long!')
        return
      self.write_field('RB_DBG_ADDR', (int('0b10000', 2) << 15) + ii)
      self.write_field('RB_DBG_CTRL' , 0)
      self.write_field('RB_DBG_CTRL' , 3)
      output = self.read_field('RB_DBG_DATA_OUT')
      f.write(str(output >> 12)+','+str( output % 4096) + '\n')
    f.close()

  def test_dac_mode1(self):
    self.write_field('RB_DBG_ADDR', int('0b11101', 2) << 15)

  def DACWrRAM(self,Fsample,Fratio,FullScale): 
    #import numpy as np

    try:
      Fsig = Fsample/Fratio
    except ZeroDivisionError:
      print("Error: Dividing by Zero!")
  
    if( Fsample > 320000000):
      print("Error: Fsample exceed limit")
      return 0
         
    NumOfAddr =32768
    #Fsample = 320000000
    #FullScale =4095
    Amplitude_I = []
    Amplitude_Q = []
    for ii in range(NumOfAddr):
        Amplitude_I.append(2048 + int(FullScale * 0.5 * sin(2*pi*ii/Fratio)))
        Amplitude_Q.append(2048 + int(FullScale * 0.5 * cos(2*pi*ii/Fratio)))
    #NumSample= np.arange(NumOfAddr)
    #Amplitude_I = FullScale * np.sin(2*np.pi*Fsig* NumSample/Fsample)
    #Amplitude_Q = FullScale * np.cos(2*np.pi*Fsig* NumSample/Fsample)
    #Amplitude=(int(Amplitude_I), int(Amplitude_Q))
    #plot for Debug
    #plotAmplitude(NumSample,Amplitude,Amplitude_I,Amplitude_Q)
  
    #Return as a tupple: i,q = DACWrRAM(Fsample,Fratio,FullScale)
    return Amplitude_I, Amplitude_Q

  def write_ram(self, filename, length = 32*1024):
    #f = open(filename, 'r')
    #lines = f.readlines()
    #f.close()
    self.write_reg('DF_DAC_CTRL',1023)
    amp_i, amp_q = self.DACWrRAM(320000000, 40, 4095)
    for ii in range(length):
      #if ii >= len(lines):
      #  print('too long!')
      #  return
      #words = lines[ii].replace('\n','').split(',')
      #di = int(words[0])
      #dq = int(words[1])
      #di = ii % 4096
      #dq = ii % 4096
      di = amp_i[ii]
      dq = amp_q[ii]

      self.write_field('RB_DBG_ADDR', (int('0b11101', 2) << 15) + ii)
      #self.write_field('RB_DBG_ADDR', (int('0b11111', 2) << 15) + ii)
      self.write_field('RB_DBG_DATA_IN', ((di<<12)+dq))
      self.write_field('RB_DBG_CTRL' , 0)
      self.write_field('RB_DBG_CTRL' , 1)


  def read_data(self):
     """Read data back from device address, 6 bytes
     temp MSB, temp LSB, temp CRC, humidity MSB, 
                      humidity LSB, humidity CRC"""
     data = bus.read_i2c_block_data(0x44, 6)

     # Convert the data
     temp = data[0] * 256 + data[1]
     cTemp = -45 + (175 * temp / 65535.0)
     fTemp = -49 + (315 * temp / 65535.0)
     humidity = 100 * (data[3] * 256 + data[4]) / 65535.0     
     return {'c' : cTemp, 'f' : fTemp, 'h' : humidity}

  def write_command(self):
    """Select the temperature & humidity command
       from the given provided values"""
    COMMAND = [0x06]  
    bus.write_i2c_block_data(0x44,0x2C, COMMAND)                 


  def i2c_s(self):     
    while True:
      self.write_command()
      time.sleep(0.3)
      result = self.read_data()
      
      print " ******  SMBUS  ********* "
      print "Relative Humidity : %.2f %%"        %(result['h'])
      print "Temperature in Celsius : %.2f C"    %(result['c'])
      print "Temperature in Fahrenheit : %.2f F" %(result['f'])
      print " ************************ "
      time.sleep(1)


  def i2c_a(self):          
    while True:
      i2c = Adafruit_I2C(0x44,2,1)
      Adafruit_I2C.write16(i2c,0x2c,0x06)

      bytes = Adafruit_I2C.readList(i2c,0,6)
      rawTemp= bytes[0]<<8 | bytes[1]
      cTemp = -45.0 + (175.0 * (rawTemp) / 65535.0)
      fTemp = (cTemp * 1.8) + 32.0
      humidity = (100.0 * ((bytes[3]<<8) | bytes[4])) / 65535.0
     
      print(" *****  ADAFRUIT  ********* ")
      print("Relative Humidity:%.2f%%"%humidity)
      print("Temperature in Celsius:%.2f C"%cTemp)
      print("Temperature in Fahrenheit:%.2f F"%fTemp)
      print( " ************************ ")
      time.sleep(1)

#####################################################################################################
               
  
  def gyro_datarate(self):
		"""Select the data rate of the gyroscope from 
		                                    the given provided values"""
		GYRO_DATARATE = ( LSM6DS0_GYRO_DR_95    
		                  |LSM6DS0_GYRO_BW_12_5  
		                  |LSM6DS0_GYRO_ND       
		                  |LSM6DS0_GYRO_XAXIS    
		                  |LSM6DS0_GYRO_YAXIS    
  		                  |LSM6DS0_GYRO_ZAXIS  
                                 )
                						  
                bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x11, GYRO_DATARATE)       
                	
  def gyro_scale_selection(self):
		"""Select the full-scale values of the gyroscope from 
		                                     the given provided values"""
		GYRO_SCALE = ( #LSM6DS0_GYRO_DEFAULT 
                              LSM6DS0_GYRO_SCALE_2000
                             )
	        bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, LSM6DS0_CTRL_REG4_G, GYRO_SCALE)
	        time.sleep(0.5)
	
  def readgyro(self):
		"""Read data back from LSM9DS0_OUT_X_L_G(0x28), 2 bytes
		X-Axis Mag LSB, X-Axis Mag MSB"""
		data0 = bus.read_byte_data(LSM6DS0_GYRO_ADDRESS, LSM6DS0_OUT_X_L_G)
		data1 = bus.read_byte_data(LSM6DS0_GYRO_ADDRESS, LSM6DS0_OUT_X_H_G)
		
		xGyro = data1 * 256 + data0
		if xGyro > 32767 :
			xGyro -= 65536
		
		"""Read data back from LSM9DS0_OUT_Y_L_G(0x2A), 2 bytes
		Y-Axis Mag LSB, Y-Axis Mag MSB"""
		data0 = bus.read_byte_data(LSM6DS0_GYRO_ADDRESS, LSM6DS0_OUT_Y_L_G)
		data1 = bus.read_byte_data(LSM6DS0_GYRO_ADDRESS, LSM6DS0_OUT_Y_H_G)
		
		yGyro = data1 * 256 + data0
		if yGyro > 32767 :
			yGyro -= 65536
		
		"""Read data back from LSM9DS0_OUT_Z_L_G(0x2C), 2 bytes
		Z-Axis Mag LSB, Z-Axis Mag MSB"""
		data0 = bus.read_byte_data(LSM6DS0_GYRO_ADDRESS, LSM6DS0_OUT_Z_L_G)
		data1 = bus.read_byte_data(LSM6DS0_GYRO_ADDRESS, LSM6DS0_OUT_Z_H_G)
		
		zGyro = data1 * 256 + data0
		if zGyro > 32767 :
			zGyro -= 65536
		
		return {'x' : xGyro, 'y' : yGyro, 'z' : zGyro}
	
  def accl_datarate(self):
		"""Select the data rate of the accelerometer from 
		                                  the given provided values"""
		ACCL_DATARATE = ( LSM6DS0_ACCL_DR_100 
                                 |LSM6DS0_ACCL_XAXIS   
		                 |LSM6DS0_ACCL_YAXIS   
			         |LSM6DS0_ACCL_ZAXIS   
                                 )
						  
		
	        bus.write_byte_data(LSM6DS0_ACCL_ADDRESS, LSM6DS0_CTRL_REG1_XM, ACCL_DATARATE)

  def accl_scale_selection(self):
		"""Select the full-scale values of the accelerometer from 
		                                  the given provided values"""
		ACCL_SCALE = (LSM6DS0_ACCL_RANGE_2G)
		
                bus.write_byte_data(LSM6DS0_ACCL_ADDRESS, LSM6DS0_CTRL_REG2_XM, ACCL_SCALE)
	
	        time.sleep(0.5)
	
  def readaccl(self):
		"""Read data back from LSM9DS0_OUT_X_L_A(0x28), 2 bytes
		X-Axis Mag LSB, X-Axis Mag MSB"""
		data0 = bus.read_byte_data(LSM6DS0_ACCL_ADDRESS, LSM6DS0_OUT_X_L_A)
		data1 = bus.read_byte_data(LSM6DS0_ACCL_ADDRESS, LSM6DS0_OUT_X_H_A)
		
		xAccl = data1 * 256 + data0
		if xAccl > 32767 :
			xAccl -= 65536
		
		"""Read data back from LSM9DS0_OUT_Y_L_M(0x2A), 2 bytes
		Y-Axis Mag LSB, Y-Axis Mag MSB"""
		data0 = bus.read_byte_data(LSM6DS0_ACCL_ADDRESS, LSM6DS0_OUT_Y_L_A)
		data1 = bus.read_byte_data(LSM6DS0_ACCL_ADDRESS, LSM6DS0_OUT_Y_H_A)
		
		yAccl = data1 * 256 + data0
		if yAccl > 32767 :
			yAccl -= 65536
		
		"""Read data back from LSM9DS0_OUT_Z_L_M(0x2C), 2 bytes
		Z-Axis Mag LSB, Z-Axis Mag MSB"""
		data0 = bus.read_byte_data(LSM6DS0_ACCL_ADDRESS, LSM6DS0_OUT_Z_L_A)
		data1 = bus.read_byte_data(LSM6DS0_ACCL_ADDRESS, LSM6DS0_OUT_Z_H_A)
		
		zAccl = data1 * 256 + data0
		if zAccl > 32767 :
			zAccl -= 65536
		
		return {'x' : xAccl, 'y' : yAccl, 'z' : zAccl}

  def readD6D_SCR(self):
                data0 = bus.read_byte_data(LSM6DS0_ACCL_ADDRESS,0x59)
                #data1 = bus.read_byte_data(LSM6DS0_ACCL_ADDRESS,0x59)
                #xD6D = data1 * 256 + data0
                #if xD6D > 32767 :
                        #xD6D -= 65536
                xD6D=data0
                return  xD6D

  def turn_on_accelerometer(self):
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x10, 0x60)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x0D, 0x01)

  def turn_on_gyroscope(self):
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x11, 0x60)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x0D, 0x02)

  def six_D_detection(self):
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x10, 0x60)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x58, 0x80)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x59, 0x60)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x17, 0x01)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x5E, 0x04)

  def sig_motion(self):      
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x01, 0x80) #
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x13, 0x01)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x01, 0x00)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x10, 0x20)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x19, 0x05)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x0D, 0x40)
       
  def relative_tilt(self):
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x0D, 0x20) #
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x19, 0x0C)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x5E, 0x02)
           

  def disable_six_D_detection(self):
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x10, 0x00)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x58, 0x00)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x59, 0x00)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x17, 0x00)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x5E, 0x00)

  def disable_sig_motion(self):
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x01, 0x00) #
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x13, 0x00)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x01, 0x00)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x10, 0x00)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x19, 0x00)
       bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x0D, 0x00)

  def ag(self):
     #bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x12, 0x20) #set interrupt H -> L
     self.disable_sig_motion()
     self.disable_six_D_detection()
     self.turn_on_gyroscope()


     while True:
      gyro = self.readgyro()
      xcurrent=abs((gyro['x']))
      ycurrent=abs((gyro['y']))
      zcurrent=abs((gyro['z']))
      print "gyroxcurrent : %d" %(xcurrent) 
      print "gyroycurrent : %d" %(ycurrent) 
      print "gyrozcurrent : %d" %(zcurrent)
      time.sleep(0.5)
      gyro = self.readgyro()
      xnext=abs((gyro['x']))
      ynext=abs((gyro['y']))
      znext=abs((gyro['z']))
      print"gyroxnext : %d" %(xnext) 
      print"gyroynext : %d" %(ynext) 
      print"gyroznext : %d" %(znext)
      if (   xnext > xcurrent+100 or 
             ynext > ycurrent+100 or
             znext > zcurrent+100 
            ):
                bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x0D, 0x03)
              
      bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x0D, 0x00)
      time.sleep(0.2)
     
      accl = self.readaccl()
      xcurrent=(abs(accl['x']))
      ycurrent=(abs(accl['y']))
      zcurrent=(abs(accl['z']))
      print "acclxcurrent : %d" %(xcurrent) 
      print "acclycurrent : %d" %(ycurrent) 
      print "acclzcurrent : %d" %(zcurrent)
	   
      time.sleep(0.5)
	   
      accl = self.readaccl()
      xnext=(abs(accl['x']))
      ynext=(abs(accl['y']))
      znext=(abs(accl['z']))
      print "acclxnext : %d" %(xnext) 
      print "acclynext : %d" %(ynext) 
      print "acclznext : %d" %(znext)
      if (    xnext > xcurrent+100 or                     		
             ynext > ycurrent+100 or 			
             znext > zcurrent+100
            ):
            bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x0D, 0x03)
	    	
      bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x0D, 0x00)



  def sint(self):  
    bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x0D, 0x03)
    time.sleep(0.5)
    bus.write_byte_data(LSM6DS0_GYRO_ADDRESS, 0x0D, 0x00)
 
  def boot(self):
    bus.write_byte_data(LSM6DS0_GYRO_ADDRESS,0x0D, 0x04)
    bus.write_byte_data(LSM6DS0_GYRO_ADDRESS,0x12, 0x80)
