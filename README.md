# GardenMon

- [GardenMon](#gardenmon)
  - [Hardware](#hardware)
    - [Raspberry Pi](#raspberry-pi)
    - [Sensors](#sensors)
    - [GardenMon Interface Board](#gardenmon-interface-board)
    - [Connections](#connections)
  - [Software](#software)
    - [Raspberry Pi Setup](#raspberry-pi-setup)
      - [Further Setup](#further-setup)
    - [Useful Commands](#useful-commands)

## Hardware

### Raspberry Pi
- Raspberry Pi Zero 2 W: [Amazon](https://a.co/d/aA3E14W)
- Raspberry Pi Zero 2 W 2x20 Header: [Amazon](https://a.co/d/92REUrK)
- microSD Card, something like 128GB: [Amazon](https://a.co/d/crgGpk7)
- USB to TTL Serial Adapter (for debug): [Amazon](https://a.co/d/1D9rg9l)

### Sensors
- SHT30 Temperature/Humidity Sensor (I2C Interface): [Amazon](https://a.co/d/8ex6dXB)
- DS18B20 Temperature Sensor (1Wire Interface): [Amazon](https://a.co/d/eyS4yjb)
- Ambient Light Sensor (I2C Interface): [DFRobot](https://www.dfrobot.com/product-2664.html)
- Soil Moisture Sensor (Analog Interface): [Amazon](https://a.co/d/6MesPOF)
- ADS1115 ADC (for Soil Moisture Meter, I2C Interface): [Amazon](https://a.co/d/3aM6eM3)

### GardenMon Interface Board

The GardenMon uses the [GardenMon Interface Board](https://github.com/anthonyneedles/gardenmon-interfaceboard) for connecting the sensors to the RPi.

### Connections

![rpi_zero2w_pinout.png](rpi_zero2w_pinout.png)

## Software

### Raspberry Pi Setup

1. Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to image microSD card with "Raspberry Pi OS (Legacy, 64-bit) Lite". When prompted, edit OS customization settings.
   1.  Enter hostname as desired, but it must be unique among devices.
   2.  Enter username/password as desired.
   3.  Configure wireless LAN information.
   4.  Set locale.
   5.  Enable SSH, with public-key authentication with your public key added.
2. When booted with imaged microSD ssh in using the credentials provided above. Install git, clone this repo, and run the init script:
```
sudo apt install git -y
git clone https://github.com/anthonyneedles/gardenmon.git
cd gardenmon
./init_rpi.sh
```
3. After successful running the init script, reboot with:
```
sudo reboot
```

#### Further Setup

To share the log drives (for example, with the user `aneedles`):

```
sudo apt install samba samba-common-bin

sudo mkdir -m 1777 /var/log/gardenmon

sudo echo "[gardenmon_logs]
path = /var/log/gardenmon
writeable = yes
browseable = yes
create mask = 0777
directory mask = 0777
public = no" >> /etc/samba/smb.conf

# Probably enter the same password as login.
sudo smbpasswd -a aneedles

sudo systemctl restart smbd
```

Should be found on Windows at `\\gardenmon\gardenmon_logs`.

### Useful Commands

To start/stop/restart the gardenmon service:
```
sudo systemctl start gardenmon
sudo systemctl stop gardenmon
sudo systemctl restart gardenmon
```

To observe the status of the gardenmon service:
```
sudo systemctl status gardenmon
```

To read the output of the service:
```
journalctl -eu gardenmon
```
