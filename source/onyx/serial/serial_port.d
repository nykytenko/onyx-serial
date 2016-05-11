/**
 * Serial port library
 *
 * Copyright: © 2014-2015
 *
 * License: MIT license. License terms written in licence.txt file
 *
 * Authors: Oleg Nykytenko, onyx.itdevelopment@gmail.com
 *
 * Version: 0.xx
 *
 * Date: 25.03.2014
 *
 */
 
module onyx.serial.serial_port;


import onyx.bundle;
import onyx.serial.read_mode;

import std.string;
import std.conv;
import core.thread;

import std.stdio:StdioException;



/**
 * Base Serial port exception
 */
abstract class SerialPortException: StdioException
{
	private string _port;

	string port() {return _port;}

	this(string port, string msg)
	{
		super(msg);
	}
}


/**
 * Construct exception with parrent: SerialPortException
 */
template childSerialPortException(string exceptionName)
{
	const char[] childSerialPortException =

	"class " ~exceptionName~":SerialPortException 
	{
		this(string port, string msg)
		{
			super(port, msg);
		}
	}";
}


/**
 * Realize child Exceptions
 */
mixin(childSerialPortException!"SerialPortSetupException");
mixin(childSerialPortException!"SerialPortOpenException");
mixin(childSerialPortException!"SerialPortCloseException");
mixin(childSerialPortException!"SerialPortIOException");
mixin(childSerialPortException!"SerialPortTimeOutException");


template getterImplMember(string type, string name)
{
	const char[] getterImplMember = 
	
	type ~ " " ~ name ~ "()
	{
		return impl." ~ name ~ ";
	}";
}


/**
 * Serial port.
 *
 * Hardware independent level
 */
struct OxSerialPort
{
	/**
	 * Low level Serial port implementation
	 */
	private Impl impl;
	
	
	mixin(getterImplMember!("string", "name"));


	/**
	 * Create new serial port
	 */
	pure
	nothrow
	this(string portName, Speed speed = Speed.B9600, Parity parity = Parity.none, uint timeOut = 0)
	{
		impl =  Impl(portName, speed, parity, timeOut);
	}


	/**
	 * Create new serial port
	 *
	 * Throws: SerialPortSetupException
	 */
	deprecated
	this(string portName, uint speed = 9600, string parity = "none", uint timeOut = 0)
	{
		impl =  Impl(portName, speed, parity, timeOut);
	}


	/**
	 * Create new serial port
	 *
	 * Throws: SerialPortSetupException
	 * Throws: ValueNotFoundException, GlKeyNotFoundException, BundleException
	 * Throws: ConvException, ConvOverflowException
	 */
	this(immutable Bundle bundle)
	{
		string extractName()
		{
			string name;
			try
			{
				name = bundle.value("port", "name");
			}
			catch(KeyNotFoundException ke)
			{
				throw new BundleException("Not found port name: " ~ke.msg);
			}
			return name;
		}

		uint extractSpeed()
		{
			uint portSpeed;
			try
			{
				portSpeed = bundle.value!uint("port", "speed");
			}
			catch(KeyNotFoundException ke)
			{
				portSpeed = 9600;
			}
			return portSpeed;
		}

		string extractParity()
		{
			string parity;
			try
			{
				parity = bundle.value("port", "parity");
			}
			catch(KeyNotFoundException ke)
			{
				parity = "none";
			}
			return parity;
		}

		uint extractTimeOut()
		{
			uint portTimeOut;
			try
			{
				portTimeOut = bundle.value!uint("port", "time_out");
			}
			catch(KeyNotFoundException ke)
			{
				portTimeOut = 0;
			}
			return portTimeOut;
		}

		impl =  Impl(extractName, extractSpeed, extractParity, extractTimeOut);
	}


	~this()
	{
		close();
	}


	/**
	 * Open serial port (and setup)
	 *
	 * Return true if port opened
	 *
	 * Throws: SerialPortOpenException, SerialPortSetupException
	 */
	bool open()
	{
		scope(failure) close();
		if (!impl.isOpen())
		{
			impl.open();
			impl.setup();
		}
		else
		{
			throw new SerialPortOpenException(impl.name, " Port already opened");
		}
		return impl.isOpen();
	}


	/**
	 * Close serial port
	 *
	 * Return true if port closed
	 *
	 * Throws: SerialPortCloseException
	 */
	bool close()
	{
		if (impl.isOpen())
		{
			impl.close();
		}
		return !impl.isOpen();
	}


	/**
	 * write data to port
	 *
	 * Throws: SerialPortIOException
	 */
	void write(ubyte[] buf)
	{
		impl.write(buf);
	}
	
	
	/**
	 * read data from port
	 *
	 * Throws: SerialPortIOException, SerialPortTimeOutException
	 */
	ubyte[] read(int byteCount, ReadMode readMode = ReadMode.waitForTimeout)
	{
		return impl.read(byteCount, readMode);
	}

	/**
	 * read data from port
	 *
	 * Throws: SerialPortIOException, SerialPortTimeOutException
	 */
	deprecated("Please use read(int, ReadMode) instead")
	ubyte[] read(int byteCount, bool wait)
	{
		return impl.read(byteCount, wait ? ReadMode.waitForTimeout : ReadMode.noWait);
	}


	/**
	 * Check if port open
	 *
	 * Return true if port open
	 *
	 */
	bool isOpen() nothrow
	{
		return impl.isOpen();
	}
}



/**
 * Parity variants
 *
 */
enum Parity:string
{
	none = "none",
	odd = "odd",
	even = "even",
	//mark = "mark",
	//space = "space",
	error = "error"
}


/**
 * Speed variants
 *
 */
enum Speed:uint
{

	B0 = 0,
	B50 = 50,
	B75 = 75,
	B110 = 110,
	B134 = 134,
	B150 = 150,
	B200 = 200,
	B300 = 300,
	B600 = 600,
	B1200 = 1200,
	B1800 = 1800,
	B2400 = 2400,
	B4800 = 4800,
	B9600 = 9600,
	B19200 = 19200,
	B38400 = 38400,
	B57600 = 57600,
	B115200 = 115200,
	B230400 = 230400,
	B460800 = 460800,
	B500000 = 500000,
	B576000 = 576000,
	B921600 = 921600,
	B1000000 = 1000000,
	B1152000 = 1152000,
	B1500000 = 1500000,
	B2000000 = 2000000,
	B2500000 = 2500000,
	B3000000 = 3000000,
	B3500000 = 3500000,
	B4000000 = 4000000,
}



version(Posix)
{
	alias Impl = PosixImpl;
}


private struct PosixImpl
{

	import core.sys.posix.termios;
	import core.sys.posix.fcntl;

	import core.sys.posix.poll;

	alias int Handle;

	private Handle handle = -1;


	/**
	 * Port name.
	 * For posix systems as rule: /dev/tty*
	 */
	private string _name;

	string name() pure nothrow
	{
		return _name;
	}


	/**
	 * Port speed.
	 * Must be from standard row: 0, 50, 75 .. 4_000_000
	 */
	private Speed _speed;

	Speed speed() pure nothrow
	{
		return _speed;
	}


	/**
	 * Port parity.
	 * Must be from: "none, odd, even, mark, space"
	 */
	private Parity _parity;

	Parity parity() pure nothrow
	{
		return _parity;
	}


	/**
	 * Port data read timeout.
	 * in msecs
	 */
	private uint _readTimeOut;

	@property
	uint readTimeOut() pure nothrow
	{
		return _readTimeOut;
	}


	/**
	 * Create serial port implementation
	 */
	this(string name, Speed speed, Parity parity, uint timeOut) pure nothrow
	{
		_name = name;

		_speed = speed;

		_parity = parity;

		_readTimeOut = timeOut;
	}



	/**
	 * Create serial port implementation
	 *
	 * Throws: SerialPortSetupException
	 */
	this(string name, uint speed, string parity, uint timeOut)
	{
		_name = name;

		if (checkSpeed(speed))
		{
			_speed = getSpeedByNum(speed);
		}
		else
		{
			throw new SerialPortSetupException(name, "Invalid speed value: " ~ to!string(speed));
		}

		if (checkParity(parity))
		{
			_parity = getParityByName(parity);
		}
		else
		{
			throw new SerialPortSetupException(name, "Invalid parity value: " ~ parity);
		}

		_readTimeOut = timeOut;
	}





	/**
	 * Open serial port 
	 *
	 * Throws: SerialPortOpenException
	 */
	void open()
 	{
		Handle h = core.sys.posix.fcntl.open(name.toStringz, O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (h != -1)
		{
			handle = h;
		}
		else
		{
			throw new SerialPortOpenException(name, "Can't open serial port");
		}
 	}


 	/**
	 * Close serial port 
	 *
	 * Throws: SerialPortCloseException
	 */
 	void close()
 	{
		if (core.sys.posix.unistd.close(handle) == -1)
		{
			throw new SerialPortCloseException(name, "Can't close serial port");
		}
		handle = -1;
 	}


 	/**
	 * Setup serial port parameters
	 *
	 * Throws: SerialPortSetupException
	 */
 	void setup()
 	{
		/* set flags */
		setFlags();

		/* set speed */
		setSpeed();
		setParity();
 	}


 	/**
 	 * Set port speed
 	 *
 	 * Throws: SerialPortSetupException
 	 */
 	private void setSpeed()
 	{
		auto set = getTermios();
		speed_t baud = getBaudRateByNum(speed);

 		if((cfsetispeed(&set, baud) < 0) || (cfsetospeed(&set, baud) < 0) || (tcsetattr(handle, TCSANOW, &set) == -1))
 		{
 			throw new SerialPortSetupException(name, "Can't set speed " ~ to!string(speed) ~ " for serial port");
 		}

 	}


 	/**
 	 * Set port speed
 	 *
 	 * Throws: SerialPortSetupException
 	 */
 	private void setParity()
 	{
 		auto set = getTermios();
 		/* clear parity bits */
 		set.c_cflag &= ~(PARENB | PARODD);
 		set.c_iflag &= ~INPCK;

 		auto par = getParityByName(parity);
 		final switch(par)
 		{
 			case Parity.odd:
 				set.c_cflag |= (PARENB | PARODD);
 				set.c_iflag |= INPCK;
 				break;
 			case Parity.even:
 				set.c_cflag |= PARENB;
 				set.c_iflag |= INPCK;
 				break;
 			case Parity.none:
 				break;
 			case Parity.error:
 		}
 		if (tcsetattr(handle, TCSANOW, &set) == -1)
		{
			throw new SerialPortSetupException(name, "Can't set parity " ~ par ~ " for serial port");	
		}
 	}


 	/**
 	 * Set port flags
 	 *
 	 * Throws: SerialPortSetupException
 	 */
 	private void setFlags()
 	{
 		auto set = getTermios();
		/* set flags */
		set.c_cflag |= (CREAD | CLOCAL);
		set.c_cflag &= ~CRTSCTS;
		set.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ECHOCTL | ECHOPRT | ECHOKE | ISIG | IEXTEN);
		set.c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR | PARMRK | ISTRIP | IGNBRK | BRKINT | INLCR | IGNCR| ICRNL);
		set.c_oflag &= ~OPOST;

		/* Minimum number of characters as 0 and we don't want to use any timer */
		set.c_cc[VMIN] = 0;
		set.c_cc[VTIME] = 0;

		if (tcsetattr(handle, TCSANOW, &set) == -1)
		{
			throw new SerialPortSetupException(name, "Can't save setup for serial port");	
		}

 	}


	/**
	 * get termios structure
	 *
	 * Throws: SerialPortSetupException
	 */
 	private termios getTermios()
 	{
 		termios set;
		if (tcgetattr(handle, &set) < 0)
		{
			throw new SerialPortSetupException(name, "Can't setup serial port");
		}
		return set;
 	}



 	/**
	 * open port check
	 */
 	bool isOpen() nothrow//@safe pure nothrow
 	{
 		return (handle > 0)?true:false;
 	}


 	/**
	 * write data to port
	 *
	 * Throws: SerialPortIOException
	 */
 	void write(ubyte[] buf)
 	{
 		if (core.sys.posix.unistd.write(handle, cast(void *)buf.ptr, buf.length) == -1)
 		{
 			throw new SerialPortIOException(name, "Error Writing to serial port");
 		}
 	}


 	/**
	 * read data from port
	 *
	 * Throws: SerialPortIOException, SerialPortTimeOutException
	 */
 	ubyte[] read(uint byteCount, ReadMode readMode)
 	{
 		ubyte[] data = new ubyte[byteCount];

 		size_t byteRemains = byteCount;

 		enum timeOutTickMax = 10; // msecs
 		auto timeOutTick = cast(int)((readTimeOut >= timeOutTickMax)?timeOutTickMax:readTimeOut);

		/* start time in hnsecs */
		import std.datetime;
 		auto startTime = Clock.currStdTime();
 		do
 		{
	 		pollfd pfd = pollfd(handle, POLLIN, 0);

	 		int rc = poll(&pfd, 1, timeOutTick);
	 		if ((rc > 0) && (pfd.revents & POLLIN))
	 		{
	 			ssize_t chanck = core.sys.posix.unistd.read(handle, cast(void*)(data.ptr + byteCount - byteRemains), byteRemains);
	 			if (chanck == -1)
	 			{
	 				throw new SerialPortIOException(name, "Error reading from serial port");
	 			}
	 			byteRemains -= chanck;
	 		}
 			if (byteRemains > 0)
 			{
 				Thread.sleep( dur!("msecs")(timeOutTick));
 			}
 			else
 			{
 				break;
 			}
	 	}
	 	while(	readMode == ReadMode.waitForTimeout && (readTimeOut > (Clock.currStdTime() - startTime)/(1000*10)) ||
	 			readMode == ReadMode.waitForAllData ||
	 			readMode == ReadMode.waitForData && byteRemains == byteCount);
	 	
	 	if (byteRemains == byteCount)
	 		throw new SerialPortTimeOutException(name, "Port data read timeout. ");
	 	data = data[0..(byteCount-byteRemains)];
 		return data;
 	}



 	bool checkParity(string parity) nothrow pure
 	{
 		return (getParityByName(parity) != Parity.error)?true:false;
 	}



 	Parity getParityByName(string strParity) nothrow pure
 	{
 		switch(strParity)
 		{
 			case Parity.none:
 				return Parity.none;
 			case Parity.odd:
 				return Parity.odd;
 			case Parity.even:
 				return Parity.even;
 			//case Parity.mark:
 			//	return Parity.mark;
 			//case Parity.space:
 			//	return Parity.space;
 			default:
 				return Parity.error;
 		}
 	}



 	bool checkSpeed(uint speed) nothrow pure
 	{
 		return (getBaudRateByNum(speed) != -1)?true:false;
 	}



	/**
	 * Convert speed number to Speed
	 */
	Speed getSpeedByNum(uint speedNum) nothrow pure
	{
		switch(speedNum)
		{
			case 0:	 		return Speed.B0;
			case 50: 		return Speed.B50;
			case 75: 		return Speed.B75;
			case 110:		return Speed.B110;
			case 134:		return Speed.B134;
			case 150:		return Speed.B150;
			case 200: 		return Speed.B200;
			case 300: 		return Speed.B300;
			case 600: 		return Speed.B600;
			case 1200:		return Speed.B1200;
			case 1800:		return Speed.B1800;
			case 2400:		return Speed.B2400;
			case 4800:		return Speed.B4800;
			case 9600: 		return Speed.B9600;
			case 19200: 	return Speed.B19200;
			case 38400:		return Speed.B38400;
			case 57600:		return Speed.B57600;
			case 115200:	return Speed.B115200;
			case 230400: 	return Speed.B230400;
			case 460800: 	return Speed.B460800;
			case 500000: 	return Speed.B500000;
			case 576000:	return Speed.B576000;
			case 921600:	return Speed.B921600;
			case 1000000:	return Speed.B1000000;
			case 1152000:	return Speed.B1152000;
			case 1500000:	return Speed.B1500000;
			case 2000000: 	return Speed.B2000000;
			case 2500000: 	return Speed.B2500000;
			case 3000000: 	return Speed.B3000000;
			case 3500000:	return Speed.B3500000;
			case 4000000:	return Speed.B4000000;
			default:		return Speed.B0;
		}
	}



	speed_t getBaudRateByNum(uint speedNum) nothrow pure
	{
		switch(speedNum)
		{
			case 0:	 		return B0;
			case 50: 		return B50;
			case 75: 		return B75;
			case 110:		return B110;
			case 134:		return B134;
			case 150:		return B150;
			case 200: 		return B200;
			case 300: 		return B300;
			case 600: 		return B600;
			case 1200:		return B1200;
			case 1800:		return B1800;
			case 2400:		return B2400;
			case 4800:		return B4800;
			case 9600: 		return B9600;
			case 19200: 	return B19200;
			case 38400:		return B38400;
			case 57600:		return B57600;
			case 115200:	return B115200;
			case 230400: 	return B230400;
			case 460800: 	return B460800;
			case 500000: 	return B500000;
			case 576000:	return B576000;
			case 921600:	return B921600;
			case 1000000:	return B1000000;
			case 1152000:	return B1152000;
			case 1500000:	return B1500000;
			case 2000000: 	return B2000000;
			case 2500000: 	return B2500000;
			case 3000000: 	return B3000000;
			case 3500000:	return B3500000;
			case 4000000:	return B4000000;
			default:		return -1;
		}
	}


 	version(linux)
 	{
		enum B57600  	= 0x1001;
		enum B115200 	= 0x1002;
		enum B230400 	= 0x1003;
		enum B460800 	= 0x1004;
		enum B500000 	= 0x1005;
		enum B576000 	= 0x1006;
		enum B921600 	= 0x1007;
		enum B1000000	= 0x1008;
		enum B1152000 	= 0x1009;
		enum B1500000 	= 0x100A;
		enum B2000000 	= 0x100B;
		enum B2500000 	= 0x100C;
		enum B3000000 	= 0x100D;
		enum B3500000 	= 0x100E;
		enum B4000000 	= 0x100F;

		enum CRTSCTS 	= 0x80000000;

		enum ECHOCTL 	= 0x200;
		enum ECHOPRT 	= 0x400;
		enum ECHOKE 	= 0x800;
	}
	else version (OSX)
	{
		enum B57600  	= 57600;
		enum B115200 	= 115200;
		enum B230400 	= 230400;
		enum B460800 	= 460800;
		enum B500000 	= 500000;
		enum B576000 	= 576000;
		enum B921600 	= 921600;
		enum B1000000	= 1000000;
		enum B1152000 	= 1152000;
		enum B1500000 	= 1500000;
		enum B2000000 	= 2000000;
		enum B2500000 	= 2500000;
		enum B3000000 	= 3000000;
		enum B3500000 	= 3500000;
		enum B4000000 	= 4000000;

		enum CCTS_OFLOW	= 0x00010000; /* CTS flow control of output */
		enum CRTS_IFLOW	= 0x00020000; /* RTS flow control of input */
		enum CRTSCTS	= (CCTS_OFLOW | CRTS_IFLOW);

		enum ECHOKE 	= 0x00000001; /* visual erase for line kill */
		enum ECHOPRT	= 0x00000020; /* visual erase mode for hardcopy */
		enum ECHOCTL 	= 0x00000040; /* echo control chars as ^(Char) */
	}
	else version (FreeBSD)
	{
		enum B57600  	= 57600;
		enum B115200 	= 115200;
		enum B230400 	= 230400;
		enum B460800 	= 460800;
		enum B500000 	= 500000;
		enum B576000 	= 576000;
		enum B921600 	= 921600;
		enum B1000000	= 1000000;
		enum B1152000 	= 1152000;
		enum B1500000 	= 1500000;
		enum B2000000 	= 2000000;
		enum B2500000 	= 2500000;
		enum B3000000 	= 3000000;
		enum B3500000 	= 3500000;
		enum B4000000 	= 4000000;

		enum CCTS_OFLOW	= 0x00010000; /* CTS flow control of output */
		enum CRTS_IFLOW	= 0x00020000; /* RTS flow control of input */
		enum CRTSCTS	= (CCTS_OFLOW | CRTS_IFLOW);

		enum ECHOKE 	= 0x00000001; /* visual erase for line kill */
		enum ECHOPRT	= 0x00000020; /* visual erase mode for hardcopy */
		enum ECHOCTL 	= 0x00000040; /* echo control chars as ^(Char) */
	}

}




version (vOnyxSerialTest)
{
	import std.stdio;
	import std.conv;

	unittest
	{

		string[] s1 = 
			["[port]",
			 "name = /dev/ttyS0",
			 "speed = 57600",
			 "data_bits = 8",
			 "stop_bits = 1",
			 "parity = none",
			 "set_RTS = no",
			 "set_DTR = no",
			 "time_out = 1500"];

		string[] s2 = 
			["[port]",
			 "name = /dev/ttyr07",
			 "speed = 57600",
			 "data_bits = 8",
			 "stop_bits = 1",
			 "parity = none",
			 "set_RTS = no",
			 "set_DTR = no",
			 "time_out = 1500"];


		auto port1 = new OxSerialPort(new immutable Bundle(s1));
		//auto port2 = new OxSerialPort(new immutable Bundle(s2));

		port1.open();
		//port2.open();

		ubyte[] data = [0x22, 0x33, 0xCC];

		port1.write(data);

		//ubyte[] buf = port2.read(3);

		//assert (buf == data);

		port1.close();
		//port2.close();


		port1 = new OxSerialPort("dev/ttyS0", Speed.B9600, Parity.none, 1000);

		port1.open;
		port1.write(data);
		port1.close;
	}
}
