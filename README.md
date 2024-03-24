# GardenMon

- [GardenMon](#gardenmon)
  - [Block Diagram](#block-diagram)
  - [Hardware](#hardware)
    - [Raspberry Pi](#raspberry-pi)
    - [Sensors](#sensors)
    - [GardenMon Interface Board](#gardenmon-interface-board)
    - [Connections](#connections)
  - [Software](#software)
    - [Raspberry Pi Setup](#raspberry-pi-setup)
      - [Further Setup](#further-setup)
    - [Database](#database)
    - [Useful Commands](#useful-commands)

## Block Diagram

The GardenMon is mean to interface with a database host device to store environmental data.
Any number of GardeMon devices can be setup to interface with this centralized database.

![gardenmon_block_diagram.jpg](./docs/gardenmon_block_diagram.jpg)

## Hardware

### Raspberry Pi
- Raspberry Pi Zero 2 W: [Amazon](https://a.co/d/aA3E14W)
- Raspberry Pi Zero 2 W 2x20 Header: [Amazon](https://a.co/d/92REUrK)
- microSD Card, something like 128GB: [Amazon](https://a.co/d/crgGpk7)
- Power Adapter 5V 2.5A microUSB: [Amazon](https://a.co/d/dAorZ26)
- Waterproof Enclosure: [Adafruit](https://www.adafruit.com/product/3931)
- 2x Extra Cable Glands for case: [Adafruit](https://www.adafruit.com/product/762)
- USB to TTL Serial Adapter (optional, for debug): [Amazon](https://a.co/d/1D9rg9l)

### Sensors
- SHT30 Temperature/Humidity Sensor (I2C Interface): [Amazon](https://a.co/d/8ex6dXB)
- DS18B20 Temperature Sensor (1Wire Interface): [Amazon](https://a.co/d/eyS4yjb)
- Ambient Light Sensor (I2C Interface): [DFRobot](https://www.dfrobot.com/product-2664.html)
- Soil Moisture Sensor (Analog Interface\*): [Amazon](https://a.co/d/6MesPOF)

_\* SMS analog input is converted via MCP3221 ADC on the GardenMon Interface Board (I2C Interface)_

### GardenMon Interface Board

The GardenMon uses the [GardenMon Interface Board](https://github.com/anthonyneedles/gardenmon-interfaceboard) for connecting the sensors to the RPi.

### Connections

![rpi_zero2w_pinout.png](./docs/rpi_zero2w_pinout.png)

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

### Database

In addition to being written to CSVs, data is published to a database.

See [docs/mariadb.md](docs/mariadb.md) for info on setting up the database with MariaDB.

### Useful Commands

To start/stop/restart the gardenmon service:
```
sudo systemctl start gardenmon
sudo systemctl stop gardenmon
sudo systemctl restart gardenmon
```

To observe the status of the gardenmon service:
```
systemctl status gardenmon
```

To read the stdout of the service:
```
journalctl -eu gardenmon
```

To open a live stream of the csv data:
```
tail -F /var/log/gardenmon/main.csv
```
