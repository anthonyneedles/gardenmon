# MariaDB

## Setup

### Installing MariaDB on Database Server

This guide walks through the installation and configuration of MariaDB on a database server.
This server should probably be a centralized machine rather than a GardenMon device, due to memory/processor limitations (e.g. a Raspberry Pi 4).

First, install MariaDB.

```
sudo apt-get install mariadb-server
```

To access the MariaDB instance, use the following command:

```
sudo mysql -u root -p
```

You will need to setup a root password the first time.

### Creating the Database, Table, User, and Permissions

The following commands will be ran in the MariaDB CLI.

First, create the database.
`<DATABASE_NAME>` should probably just be `gardenmon`.

```
CREATE DATABASE IF NOT EXISTS <DATABASE_NAME>;
```

Create the table with the following DDL statement.
`<DATABASE_TABLE>` should probably just be `environmental_data`.

```
CREATE TABLE <DATABASE_TABLE> (
    cpu_temp_f FLOAT,
    ambient_light_lx FLOAT,
    soil_moisture_val INT,
    soil_moisture_level INT,
    soil_temp_f FLOAT,
    ambient_temp_f FLOAT,
    ambient_humidity FLOAT,
    insert_time TIMESTAMP,
    device VARCHAR(50)
);
```

Create the user for the database add necessary permissions.

- `<DATABASE_USER>` should probably be the hostname of the GardenMon (i.e. `gardenmon` or `gardenmon2`).
- `<GARDENMON_IP>` must the IP of the GardenMon.
- `<DATABASE_PASSWORD>` can be any password for the database user.
- `<DATABASE_NAME>` is as defined above.
- `<DATABASE_TABLE>` is as defined above.

**NOTE: THIS WILL NEED TO BE DONE FOR EACH GARDENMON IN NETWORK.**

```
CREATE USER '<DATABASE_USER>'@'<GARDENMON_IP>' IDENTIFIED BY '<DATABASE_PASSWORD>';
GRANT USAGE ON <DATABASE_NAME>.<DATABASE_TABLE> to '<DATABASE_USER>'@'<GARDENMON_IP>' IDENTIFIED BY '<DATABASE_PASSWORD>'
GRANT ALL PRIVILEGES ON <DATABASE_NAME>.<DATABASE_TABLE> to '<DATABASE_USER'@'<GARDENMON_IP>' IDENTIFIED BY '<DATABASE_PASSWORD>';
FLUSH PRIVILEGES;
```

### Update GardenMon

Update `local_options.py` in the GardenMon.
This should have been made from [local_options.py.template](../local_options.py.template) when running [init_rpi.sh](../init_rpi.sh).
`<DATABASE_HOST>` is the hostname or IP of the database host.

```
database_host=<DATABASE_HOST>
database_name=<DATABASE_NAME>
database_table=<DATABASE_TABLE>
database_user=<DATABASE_USER>
database_password=<DATABASE_PASSWORD>
```
