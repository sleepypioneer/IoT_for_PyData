import httpget
import network

def do_connect():
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('connecting to network...')
        #sta_if.connect('co_up_slow', 'clubmate')
        sta_if.connect('belkin.b27', 'therethereHorseyCome2ME')
        while not sta_if.isconnected():
            pass
    print('network config:', sta_if.ifconfig())

do_connect()
httpget.http_get('http://micropython.org/ks/test.html')
