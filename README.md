# onyx-serial

onyx-serial: the D simple serial port library.


## Key features:
 - Open/close serial port.
 - Write/read to/from serial port.
 - Setup speed - standard value from 50 to 4,000,000.
 - parity - none, odd, even.
 - Setup read timeout in mS.
 - Check is port open.
 - Working on posix OS (Linux, OSX, FreeBSD, Solaris).


## Examples:

```D
	import onyx.serial;

	/* Create ports */
	auto port1 = OxSerialPort("dev/ttyS1", Speed.B9600, Parity.none, 1000);
	auto port2 = OxSerialPort("dev/ttyS2", Speed.B9600, Parity.none, 1000);


	port1.open;
	port2.open;

	ubyte[] data = [0x22, 0x33, 0xCC];

	port1.write(data);

	ubyte[] buf = port2.read(3);

	assert (buf == data);

	port1.close();
	port2.close();

```


```D
	string[] s1 =
		["[port]",
		 "name = /dev/ttyr06",
		 "speed = 57600",
		 "parity = none",
		 "time_out = 1500"];

	auto bundle = new immutable Bundle(s1);

	auto port3 = OxSerialPort(bundle);

```


```D

	auto bundle = new immutable Bundle("./config/port4.conf");

	auto port4 = OxSerialPort(bundle);

```
