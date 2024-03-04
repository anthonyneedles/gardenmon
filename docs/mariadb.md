Ripped from this website https://devopscube.com/install-mariadb-on-ubuntu/

Configuring MariaDB on a Plex Server
This guide walks through the installation and configuration of MariaDB on a Plex server, tailored specifically for an Ubuntu system. We will follow the steps outlined on the website devopscube.com and then proceed to create a database table with the structure provided.

sudo apt-get install mariadb-server

To access the MariaDB instance, use the following command:

sudo mysql -u root -p

Creating the Database, Table, and User

First, run this to create a database
CREATE DATABASE IF NOT EXISTS gardenmon;

Next, create the table.
Here's a DDL statement for the specified table:

CREATE TABLE environmental_data (
    cpu_temp_f FLOAT,
    ambient_light_lx FLOAT,
    soil_moisture_val INT,
    soil_moisture_level INT,
    soil_temp_f FLOAT,
    ambient_temp_f FLOAT,
    ambient_humidity FLOAT,
    insert_time TIMESTAMP
);

After that, run this to create the user and add necessary permissions.
CREATE USER 'gardenmon'@'192.168.10.67' IDENTIFIED BY 'top_secret_password';
GRANT USAGE ON *.* to 'gardenmon'@'192.168.10.67' IDENTIFIED BY 'top_secret_password'
GRANT ALL PRIVILEGES ON *.* to 'gardenmon'@'192.168.10.67' IDENTIFIED BY 'top_secret_password';

And when running the python script, you need to add the password as a flag like python gardenmon.py top_secret_password
