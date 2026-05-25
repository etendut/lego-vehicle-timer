# If you are having trouble flashing, running this first can help make a bigger flash work.
from pybricks.parameters import Color
from pybricks.tools import wait
LED_FLASHING_SEQUENCE = [75] * 5 + [1000]
try:
    # this import will fail if the city hub is not connected.
    from pybricks.hubs import CityHub
    hub = CityHub()
    print('Lego City Hub found')
    hub.light.blink(Color.BLUE, LED_FLASHING_SEQUENCE)
    
except ImportError as ex1:
    print(ex1)
    try:
        from pybricks.hubs import TechnicHub
        hub = TechnicHub()
        print('Lego Technic Hub found')
        hub.light.blink(Color.MAGENTA, LED_FLASHING_SEQUENCE)       

    except ImportError as ex2:
        print(ex2)
        raise Exception('This program only support Lego City hub and Lego Technic hub')

print(hub.battery.voltage())

while True:
     wait(500)