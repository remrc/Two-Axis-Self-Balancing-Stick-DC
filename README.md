# Two-Axis-Self-Balancing-Stick-DC
 
Arduino nano, DC motors, TB6612 motor driver, AS5600 position sensors, MPU6050, 3S 500 mAh LiPo battery.

Balancing controllers can be tuned remotely over bluetooth.

Example (change K1):

Send p+ (or p+p+p+p+p+p+p+) for increase K1.

Send p- (or p-p-p-p-p-p-p-) for decrease K1.

Send i, a, s if you need to change K2, K3, K4.

Send c+ from serial monitor for calibrating procedure. Place the stick at the balancing point (as accurately as possible). Send c- from serial monitor. The stick will start balancing and write the offsets values ​​into the EEPROM.

<img src="/pictures/stick.jpg" alt="Balancing stick pic"/>
<img src="/pictures/schematic.png" alt="Schematic"/>

More about this:

https://youtu.be/wacuNeEO2zE

https://youtu.be/Nkm9PoihZOI