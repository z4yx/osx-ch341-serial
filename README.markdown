# osx-ch341-serial
CH340/CH341 based USB to Serial Port Adapter driver for Mac OSX. 

Currently tested on OSX 10.10.5 Yosemite.

This is an unofficial open source driver, use at your own risk.

# Features
- Unix device node /dev/cu.CH341-*
- Standard baudrate (300~230400)
- Databits (5, 6, 7, 8)
- Stopbits (1, 2)
- Parity check (none, odd, even)
- Manual RTS/DTR signal control

# Acknowledgements
Driver is inspired by the following projects:

- [osx-pl2303](https://github.com/mpepping/osx-pl2303)
- Linux kernel ch341.c

