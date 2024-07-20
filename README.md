# ESP32 remote controlled toy car
This is a simple RC car library which uses Expressif's ESP-NOW for wireless control. Written in C++ using PlatformIO with the Arduino framework in VS Code.

## Bill of materials
 - 2 x ESP32 dev boards (Tested on HW-394 boards)
 - 1 x 5-pin joystick module
 - 2 x L298N H-bridge dual motor driver module
 - 4 x generic DC motor + wheels (Tested with d=65mm ones)
 - Chassis compatible with the motors
 - 2 x power source (USB power bank, 9v battery, etc.)
 - Jumper wires

## Physical build
### Car body
Fasten the motors to the chassis according to the manufacturer's instructions. Afterwards wire in the components as follows:

![car body wiring diagram](docs/carBody.svg)

Make sure to bridge the 5VEN pins on the motor drivers (indicated by yellow wires).

### Remote controller
Connect the joystick to the ESP32.

![joystick controller wiring diagram](docs/joyControl.svg)

You may want to build a chassis for the remote for ease of use. I recommend getting creative with it. 🙂

![remote controller tennis ball](docs/remoteComplete.jpg)


## Programming the ESPs
Using The PlatformIO extension in VS Code, build and upload `joyControl` to the remote controller's ESP, and `carBody` to the car ESP. On Linux, you must enable IO on the connecting USB terminal:
```bash
sudo chmod a+rw /dev/ttyUSB0
```
Your terminal may be different. Check if the PlatformIO extension can detect the port your ESP board is connected. If none are found, try restarting your computer, and if it still does not show up, try another USB cable.

## Operation
The remote controller reads the values of the joystick. This is comprised of the readings of two perpendicularly aligned potentiometers and that of a button (used for braking). The values of the pots are aligned in a square, with additional dead space in the circumircle.

The remote controller converts these readings into polar notation, with the distance `r` from the joystick's center being calculated as the maximum of the two coordinates (Chebyshev distance), and the direction `theta` calculated using 2-argument arctangent, resulting in the following radian representation:

![atan2 directions in radians](docs/atan2.png)

The car's microcontroller receives these values, and performs skid steering. The speed of the motors on each side is calculated by multiplying `r` by the following coefficient function based on `theta`:

![skid steering functions](docs/car-motors.svg)