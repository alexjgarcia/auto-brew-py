
 


def One():
    import utime
    from machine import Pin, PWM
    import onewire
    import ds18x20
    import time
    thermometer = Pin(16, Pin.IN)                       # DS18X20 Thermometer on GPIO 16 as input
    one_wire_sensors = ds18x20.DS18X20(onewire.OneWire(thermometer))
    utime.sleep(0.5)

    led = Pin("LED",Pin.OUT)
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
                    print(f"temp = {d} , {t}")
                    utime.sleep_ms(100)

        except:
            print("Error reading tempurature sensor")

        return d, t
    def myblink():
        led.on()
        blinkstart = 1
        for x in range(1,30):
            led.toggle()
            time.sleep(0.025)
        led.off()

    while True:
        myblink()    
        reading = get_temp(one_wire_sensors)
        #print(f"temp = {reading}")
        time.sleep(3)


def Two():
    import time
    import machine
    import onewire, ds18x20

    # the device is on GPIO12
    dat = machine.Pin(16)

    # create the onewire object
    ds = ds18x20.DS18X20(onewire.OneWire(dat))

    # scan for devices on the bus
    roms = ds.scan()
    print('found devices:', roms)

    # loop 10 times and print all temperatures
    while True:
        print('temperatures:', end=' ')
        ds.convert_temp()
        time.sleep_ms(750)
        for rom in roms:
            print(f"{ds.read_temp(rom)} / ", end=' ')
        print()    
     
One()   
#Two()