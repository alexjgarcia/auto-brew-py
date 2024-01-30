
#import io
import utime
from machine import Pin, PWM
import onewire
import ds18x20
#import micropython
#import errno
import time


thermometer = Pin(16, Pin.IN)                       # DS18X20 Thermometer on GPIO 16 as input
one_wire_sensors = ds18x20.DS18X20(onewire.OneWire(thermometer))
utime.sleep(0.5)


def get_temp(one_wire_sensors):
    d = ""
    t = 0
    try:
        roms = one_wire_sensors.scan()
        sensor_count = len(roms)
        print(str(sensor_count) + " temp sensors found")

        if sensor_count > 0:
            one_wire_sensors.convert_temp()
            utime.sleep(0.5)
            for rom in roms:
                d = hex(int.from_bytes(rom, 'little'))
                t = one_wire_sensors.read_temp(rom)
                utime.sleep_ms(100)

    except:
        print("Error reading tempurature sensor")

    return d, t

while True:
    reading = get_temp(one_wire_sensors)
    print(f"temp = {reading}")
    time.sleep(10)
