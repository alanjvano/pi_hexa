import evdev
import threading

print("initializing controller...")
devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
for device in devices:
    print(device.name, device.path)
    if device.name == 'Sony PLAYSTATION(R)3 Controller':
        dev = device.path

ps3 = evdev.InputDevice(dev)

for event in ps3.read_loop():
    #if event.type == evdev.ecodes.EV_KEY:
    #    print(evdev.categorize(event))
    print(event)
