import io
import utime
from machine import Pin, PWM
import onewire
import ds18x20
from PicoOLED1point3spi import OLED_1inch3
import micropython
import errno

# Proportional Integral Derivative (PID) logic adapted from this repo: https://github.com/veebch/heat-o-matic/blob/main/main.py


# Global variables and constants
Pin_Interrupt = False
Display_Key0_Pressed = False
Display_Key1_Pressed = False
off = 0                             # Value for OFF
on = 1                              # Value for ON

current_temp = 0.0                  # Current temp reading
min_temp = 11.0                     # Minimum selectable temp
max_temp = 27.0                     # Maximum selectable temp
last_update = utime.time()          # 
check_in = 300                      # Temperature check interval (seconds)
no_device_check_in = 60             # Check interval for when no temperature sensor was detected previously (seconds)
no_device = False                   # Indicates if no temperature sensor was detected
last_display = utime.time()         # 
display_timeout = 30                # Turn off display to avoid burn-in
display_on = True                   # 
relay_on = False                    # True if a relay is on
switch_off_relays = utime.time()    # Turn off relays after this time


# Set up temperature regulation values
targetTemp = 19.0                   # Default target temperature to maintain
tolerance = 0.25                    # Allowable variance on either side of the target
integral = 0                        # 
last_variance = 0.0                 # 
Kp=10.                              # Proportional term - Basic steering (This is the first parameter you should tune for a particular setup)
Ki=.01                              # Integral term - Compensate for heat loss by vessel
Kd=150.                             # 


# Pin allocations
heating_relay = Pin(6, Pin.OUT)                     # Relay 1 on GPIO 6 as output
cooling_relay = Pin(7, Pin.OUT)                     # Relay 2 on GPIO 7 as output
display_key0 = Pin(15, Pin.IN, Pin.PULL_UP)         # Display button (key0) on GPIO 15 as input
display_key1 = Pin(17, Pin.IN, Pin.PULL_UP)         # Display button (key1) on GPIO 17 as input
thermometer = Pin(16, Pin.IN)                       # DS18X20 Thermometer on GPIO 16 as input



# Handle interrupts from GPIO pins
def handle_interrupt(Pin):
    global Pin_Interrupt
    Pin_Interrupt = True
    
    global display_on
    if display_on:
        if (Pin == display_key0):
            global Display_Key0_Pressed
            Display_Key0_Pressed = True

        if (Pin == display_key1):
            global Display_Key1_Pressed
            Display_Key1_Pressed = True
    else:
        display_on = True
    
    global last_display
    last_display = utime.time()
    utime.sleep(0.1)


display_key0.irq(trigger=Pin.IRQ_RISING, handler=handle_interrupt)
display_key1.irq(trigger=Pin.IRQ_RISING, handler=handle_interrupt)


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


# Display functions
def display_clear_all(display):
    display.fill(0x0000)


def display_clear_line1(display):
    display.fill_rect(0, 0, 128, 16, display.black)


def display_clear_line2(display):
    display.fill_rect(0, 16, 128, 16, display.black)


def display_clear_line3(display):
    display.fill_rect(0, 32, 128, 16, display.black)


def display_clear_line4(display):
    display.fill_rect(0, 48, 128, 16, display.black)


def display_current(display, current):
    display_clear_line1(display)
    txt = " Current: " + str(current)
    display.text(txt, 0, 4, display.white)


def display_target(display, target):
    display_clear_line2(display)
    txt = "  Target: " + str(target)
    display.text(txt, 0, 20, display.white)


def display_variance(display, variance):
    display_clear_line3(display)
    txt = "    Diff: " + str(variance)
    display.text(txt, 0, 36, display.white)


def display_message(display, message):
    display_clear_line4(display)
    txt = str(message)
    display.text(txt, 0, 52, display.white)


def display_splash(display):
    display_clear_all(display)
    display.rect(0, 0, 128, 64, display.white)
    display.text("    AutoBrew    ", 0, 22, display.white)
    display.text("     v1.1.2     ", 0, 40, display.white)
    utime.sleep(0.1)
    display.show()
    utime.sleep(5)
    display_clear_all(display)
    display.show()
    utime.sleep(0.1)
    

def read_stored_temp():
    global targetTemp
    temp = targetTemp
    with open('temp.txt','r') as f:
        temp = f.read()
    f.close()
    targetTemp = float(temp) 


def write_stored_temp():
    temp = targetTemp
    with open('temp.txt','w+') as f:
        f.write(str(temp))
    f.close()



# Main function
def main():

    global Pin_Interrupt
    global Display_Key0_Pressed
    global Display_Key1_Pressed
    global current_temp
    global last_update 
    global no_device
    global last_display
    global display_on
    global relay_on
    global switch_off_relays
    global targetTemp
    global integral
    global last_variance


    try:
        read_stored_temp()

        # Initialise display
        display = OLED_1inch3()

        # Set up thermometer
        one_wire_sensors = ds18x20.DS18X20(onewire.OneWire(thermometer))
        utime.sleep(0.5)

        display_splash(display)

        device, reading = get_temp(one_wire_sensors)
        if device != "":
            current_temp = reading
            print("Temp: " + str(current_temp))
            variance = targetTemp - current_temp

            display_clear_all(display)
            display_current(display, current_temp)
            display_target(display, targetTemp)
            display_variance(display, variance)
            display.show()

        else:
            no_device = True
            display_message(display, "Sensor not found")
            display.show()

      
        while True:
            if Pin_Interrupt:

                if Display_Key0_Pressed and no_device == False:
                    if targetTemp < max_temp:
                        targetTemp = targetTemp + 0.5
                        print(targetTemp)
                        write_stored_temp()
                    Display_Key0_Pressed = False

                if Display_Key1_Pressed and no_device == False:
                    if targetTemp > min_temp:
                        targetTemp = targetTemp - 0.5
                        print(targetTemp)
                        write_stored_temp()
                    Display_Key1_Pressed = False

                if no_device:
                    display_clear_all(display)
                    display_message(display, "Sensor not found")
                    display.show()
                else:
                    var = targetTemp - current_temp
                    display_current(display, current_temp)
                    display_target(display, targetTemp)
                    display_variance(display, var)
                    display.show()

                Pin_Interrupt = False

            else:
                now = utime.time()
                
                # Check how long the display has been on for
                if (now >= last_display + display_timeout):
                    display_on = False
                    display_clear_all(display)
                    display.show()

                # Change the check_seconds if the temp sensor is missing
                check_seconds = check_in
                if no_device:
                    check_seconds = no_device_check_in

                # Check if it is time to switch off the relays
                if relay_on:
                    if (now >= switch_off_relays):
                        heating_relay(off)
                        cooling_relay(off)
                        print("Relays off")
                        relay_on = False
                        if display_on:
                            display_clear_line4(display)
                            display.show()

                time_diff = now - last_update

                # Check if it is time for a new reading
                if time_diff > check_seconds:

                    device, reading = get_temp(one_wire_sensors)
                    print(str(device) + " : " + str(reading))

                    if device != "":
                        no_device = False
                        current_temp = reading
                        print("Temp: " + str(current_temp))
                        variance = targetTemp - current_temp
                        print("Variance: " + str(variance))

                        if display_on:
                            display_clear_all(display)
                            display_current(display, current_temp)
                            display_target(display, targetTemp)
                            display_variance(display, variance)
                            display.show()

                        if abs(variance) > tolerance:
                            integral = integral + time_diff * variance
                            derivative = (variance - last_variance) / time_diff
                            output = Kp * variance + Ki * integral + Kd * derivative
                            out = round(output)
                            print("Output: " + str(output))
                            print("Output rounded: " + str(out))

                            if out > 0:
                                print("Heating on")
                                if display_on:
                                    display_message(display, "   HEATING ON   ")
                                    display.show()
                                relay_on = True
                                switch_off_relays = utime.time() + abs(out)
                                heating_relay(on)

                            if out < 0:
                                print("Cooling on")
                                if display_on:
                                    display_message(display, "   COOLING ON   ")
                                    display.show()
                                relay_on = True
                                switch_off_relays = utime.time() + abs(out)
                                cooling_relay(on)
                            
                        last_variance = variance

                    else:
                        no_device = True
                        if display_on:
                            display_clear_all(display)
                            display_message(display, "Sensor not found")
                            display.show()

                    last_update = now
                    
                    
    except OSError as err:
        print("Error: " + str(err))
    finally:
        heating_relay(off)
        cooling_relay(off)
        main()


if __name__ == '__main__':
    main()
