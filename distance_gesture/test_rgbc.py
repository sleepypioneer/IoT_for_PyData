import pyb
from time import sleep

from machine import Pin, I2C

from apds9960.const import *
from apds9960.device import uAPDS9960 as APDS9960

import micropython

bus = I2C(3, freq=400000)

apds = APDS9960(bus)

x11 = pyb.Pin('Y11')

ba= bytearray(512)
b1 = memoryview(ba)[:1]
b2 = memoryview(ba)[:2]

def hexb(a, off=0):
    if type(a) in [type(bytearray()),type(bytes()),type(memoryview(''))]:
        sbuf = ''
        for i in range(0,len(a),1):
            if i & 0xf == 0:
                if len(sbuf): print(sbuf)
                sbuf='%04x ' % (i+off)
            if i & 0x3 == 0:
                sbuf += ' '
            sbuf+=' %02x' % (a[i+0])
        print(sbuf)

def dump(a=0, n=256):
    if (a+n) > 256:
        n = 256-a
    for i in range(a, a+n):
        ba[i] = apds._read_byte_data(i)
    hexb(ba[a:a+n], a)

def iforce(p=None):
    bus.writeto(I2C_ADDR, b'\«e4')

def iclear(p=None):
    # print("Clearing proximity int")
    apds.clearProximityInt()

def igclear(p=None):
    print('cbg', p, x11())
    apds._write_byte_data(0xab, 0x06)       # clear gesture INT

icnt = 0

def cb(p):
    global icnt
    icnt += 1
    # print('cb', p, x11())
    micropython.schedule(iclear, 1)  # TODO figure out what is this

def cbg(p):
    global icnt
    icnt += 1
    micropython.schedule(igclear, 1)

def mrg():
    if not (apds.getMode() & 0b01000001) or not apds.isGestureAvailable():
        return -1
    if (apds.isGestureAvailable()):
        # read the current FIFO level
        fifo_status = apds._read_byte_data(REG_GSTATUS)
        if fifo_status & 2:
            print('OVL')
        else:
            print('GEST')
        fifo_level = apds._read_byte_data(REG_GFLVL)
        # if there's stuff in the FIFO, read it into our data block
        if fifo_level > 0:
            #fifo_data = []
            for i in range(0, fifo_level):
                apds.bus.readfrom_mem_into(apds.address, REG_GFIFO_U, memoryview(ba)[i*4:i*4+4])
                #fifo_data += apds._read_i2c_block_data(REG_GFIFO_U, 4)
            #print(end='.')
        return memoryview(ba)[:fifo_level*4]
    else:
        return '-'


def gest():
    print("Gesture Test")
    print("============")
    apds._write_byte_data(0xa0, 0x40)       # start gesture level
    apds._write_byte_data(0xa1, 0x20)       # end gesture level
    apds._write_byte_data(0xa2, 0x00)       # default
    apds._write_byte_data(0xa3, 0x07)       # GAIN:1x, 100 mA, 39.2 ms/sample

    #apds._write_byte_data(0xab, 0x02)       # gesture INT enable

    apds.setAmbientLightIntEnable(0)
    apds.setProximityIntEnable(0)
    exti = pyb.ExtInt(x11, mode=pyb.ExtInt.IRQ_FALLING, pull=pyb.Pin.PULL_NONE, callback=None)
    #exti = pyb.ExtInt(x11, mode=pyb.ExtInt.IRQ_FALLING, pull=pyb.Pin.PULL_NONE, callback=cbg)
    apds.enableGestureSensor()
    apds._write_byte_data(0xab, 0x00)       # gesture INT disable
    while True:
        sleep(0.5)
        res = mrg()
        hexb(res)
        #print(mrg())
        #if apds.isGestureAvailable():
        #    motion = apds.readGesture()
        #    print("%4d" % (motion))

import lcd160cr


lcd = lcd160cr.LCD160CR('X')


def draw_screen(r, g, b):
    lcd_x = int(lcd.w/2)
    lcd_y = int(lcd.h/2)
    rect_height = 50
    rect_width = 50
    lcd.set_pen(lcd.rgb(0, 0, 0), lcd.rgb(0, 0, 0))
    lcd.erase()
    lcd.set_orient(lcd160cr.PORTRAIT)
    lcd.set_pos(0, 0)
    lcd.set_text_color(lcd.rgb(r, g, b), lcd.rgb(0, 0, 0))
    lcd.set_font(1)
    lcd.write('Nice color')
    lcd.set_pen(lcd.rgb(r, g, b), lcd.rgb(r, g, b))
    lcd.rect(lcd_x-50, lcd_y, rect_height, rect_width)
    lcd.set_pen(lcd.rgb(apds.readRedLight(), apds.readGreenLight(), apds.readBlueLight()), lcd.rgb(apds.readRedLight(), apds.readGreenLight(), apds.readBlueLight()))
    lcd.rect((lcd_x), (lcd_y), rect_height, rect_width)


STABILITY_WINDOW = 3


class ColorSensor():
    def __init__(self):
        self.color = None

    def measure(self):
        if not self.color:
            self.color =  self.measure_current()
        i = STABILITY_WINDOW
        while i > 0:
            sleep(0.25)
            current_color = self.measure_current()

            if current_color != self.color:
                print("Resetting current color from (red:{}, green:{}, blue:{}) to (red:{}, green:{}, blue:{})".format(self.color[0], self.color[1], self.color[2],
                current_color[0], current_color[1], current_color[2]))
                self.color = current_color
                i = STABILITY_WINDOW
            else:
                i -= 1
        print("Stable color (red:{}, green:{}, blue:{})".format(self.color[0], self.color[1], self.color[2]))
        return self.color

    def measure_current(self):
        return self._quantize((apds.readRedLight(), apds.readGreenLight(), apds.readBlueLight()))

    def _quantize(self, color):
        return tuple(int(el/32)*32-16 for el in color)


def main():
    print("RGBC Test")
    print("=========")
    #exti = pyb.ExtInt(x11, mode=pyb.ExtInt.IRQ_FALLING, pull=pyb.Pin.PULL_NONE, callback=None)
    exti = pyb.ExtInt(x11, mode=pyb.ExtInt.IRQ_FALLING, pull=pyb.Pin.PULL_NONE, callback=cb)
    iclear()
    apds.enableLightSensor()
    apds.setAmbientLightIntEnable(0)

    apds.setProxIntHighThresh(200)
    apds.setProxIntLowThresh(0x0)
    apds.enableProximitySensor()
    while True:
        sleep(0.25)
        r = apds.readRedLight()
        g = apds.readGreenLight()
        b = apds.readBlueLight()
        c = apds.readAmbientLight()
        p = apds.readProximity()
        print('Red: %5d Green: %5d Blue: %5d Ambient: %5d proximity: %8d interrupt: %d' % (r, g, b, c, p, x11()))
        sensor = ColorSensor()
        # r, g, b = sensor.measure()
        draw_screen(r, g, b)
        # iclear()
