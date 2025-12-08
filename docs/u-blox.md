
# GPS (u-blox F9P)

## Image
<img width="20%" src="https://www.gateworks.com/wp-content/uploads/gw16143-angle-800px.png">

## File

```
/sensors_drivers/ublox/ublox_gps/config/ardusimple.yaml
```

## Changes

- `Open a terminal`: Run this command below and look for your `USB ACM device`, then note down the ID (like `ttyACM0`) - This shows live system messages so you can see when your GPS device gets plugged in

```bash
dmesg --follow
```

 - `Open yaml file`:   Change the device parameter to match your USB ACM device (ex: `/dev/ttyACM0`)

```yaml
device: /dev/ttyACM0  # Enter the device path from dmesg output
```


## Key Parameters
- `device`: Serial device path (set via Udev rule)
- `frame_id`: TF frame for GPS data (`default: gps`)
- `config_on_startup`: Whether to configure GPS on startup (`default: false`)
- `uart1/baudrate`: Serial baud rate (`default: 460800`)
- `rate`: Measurement rate in Hz
- `nav_rate`: Navigation rate in Hz
- `publish/all`: Publish all available GPS data (`default: true`)

## Available Launch FilesS
- `ardusimple.launch`: Basic GPS launch with configuration
- `ekf.launch`: GPS with EKF fusion
- `mapping.launch`: GPS for mapping applications
- `ublox_device.launch`: Generic ublox device launch

