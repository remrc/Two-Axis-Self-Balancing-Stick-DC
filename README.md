# Two-Axis-Self-Balancing-Stick-DC
 
Arduino nano, DC motors, TB6612 motor driver, AS5600 position sensors, MPU6050, 3S 500 mAh LiPo battery.

Balancing controllers can be tuned remotely over bluetooth (it's not necessary, only do this if you know what you're doing).

Example (change K1):

Send p+ (or p+p+p+p+p+p+p+) for increase K1.

Send p- (or p-p-p-p-p-p-p-) for decrease K1.

Send i, a, s if you need to change K2, K3, K4.

Send c+ from serial monitor for calibrating procedure. Place the stick at the balancing point (as accurately as possible). Send c- from serial monitor. The stick will start balancing and write the offsets values ​​into the EEPROM.

Important! Do this only over Bluetooth. You need to connect bluetooth module (for example HC-05).

<img src="/pictures/stick.jpg" alt="Balancing stick pic"/>
<img src="/pictures/schematic.png" alt="Schematic"/>

If the balancing works but sometimes stops, this is a problem with the DC motors. Sparkling brushes. You will find what to do in this case after watching the video.

More about this:

https://youtu.be/wacuNeEO2zE

This video may also help:

https://youtu.be/Nkm9PoihZOI