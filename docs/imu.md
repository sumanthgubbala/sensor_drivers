
<a name="xsens-imu"></a>
#  Xsens IMU

## Image
<img width="20%" src="https://www.mouser.in/images/marketingid/2023/img/109066689.png">

## Changes 

### 1. Find device info

```bash
dmesg --follow
```
>Run this, then plug in your Xsens IMU. You'll see vendor ID, product ID, serial number, and device path (`like ttyACM0`). Note these values for reference.

### 2. Set up udev rules
```bash
sudo nano /etc/udev/rules.d/99-xsens-imu.rules
```
Add this line:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ACTION=="add", GROUP="dialout", MODE="0660"
```

### 3. Apply rulesS
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -a -G dialout $USER
```


---