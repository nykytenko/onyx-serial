/**
 * Serial port library
 *
 * Copyright: © 2014-2021
 * License: MIT license. License terms written in licence.txt file
 *
 * Authors: Oleg Nykytenko, onyx.itdevelopment@gmail.com
 *
 * Version: 0.xx Date: 2014
 */

module onyx.serial;


import onyx.bundle;

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







mixin (genSpeed);


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
    this(string portName, Speed speed = Speed.S9600, Parity parity = Parity.none, uint timeOut = 0)
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
 * Generation Speed type
 *
 * Use example: Speed.s57600
 */
const (char[]) genSpeed()
{
    const (char)[] res = "enum Speed:uint {";
    foreach (speed; speedsRange)
    {
        res = res ~ 'S' ~ speed ~ " = " ~ speed ~ ", ";
    }
    res ~= '}';
    return res;
}

/**
 * Speed variants
 *
 */
version (linux)
{
    const char[][] speedsRange = ["0", "50", "75", "110", "134", "150", "200", "300", "600", "1200", "1800", "2400",
        "4800", "9600", "19200", "38400", "57600", "115200", "230400", "460800", "500000", "576000", "921600",
        "1000000", "1152000", "1500000", "2000000", "2500000", "3000000", "3500000", "4000000"];
}
else version (OSX)
{
    const char[][] speedsRange = ["0", "50", "75", "110", "134", "150", "200", "300", "600", "1200", "1800", "2400",
        "4800", "9600", "19200", "38400", "7200", "14400", "28800", "57600", "76800", "115200", "230400"];
}
else version (FreeBSD)
{

    const char[][] speedsRange = ["0", "50", "75", "110", "134", "150", "200", "300", "600", "1200", "1800", "2400",
        "4800", "9600", "19200", "38400", "7200", "14400", "28800", "57600", "76800", "115200", "230400", "460800", "921600"];
}
else version (Solaris)
{

    const char[][] speedsRange = ["0", "50", "75", "110", "134", "150", "200", "300", "600", "1200", "1800", "2400",
        "4800", "9600", "19200", "38400", "57600", "76800", "115200", "153600", "230400", "307200", "460800", "921600"];
}



/**
 *	Variants for the read method
 */
enum ReadMode
{
    noWait,
    waitForTimeout,
    waitForData,
    waitForAllData
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
        static import core.sys.posix.unistd;
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
        static import core.sys.posix.unistd;
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
                static import core.sys.posix.unistd;
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



    static const (char[]) genGetSpeedByNumBody()
    {
        const (char)[] res = "switch(speedNum) {";
        foreach (speed; speedsRange)
        {
            res = res ~ "case " ~ speed ~ ": return Speed.S" ~ speed ~ "; \n";
        }
        res ~= "default: return Speed.S0;";
        res ~= '}';
        return res;
    }


    /**
     * Convert speed number to Speed
     */
    Speed getSpeedByNum(uint speedNum) nothrow pure
    {
        mixin (genGetSpeedByNumBody);
    }


    static const (char[]) genGetBaudRateByNumBody()
    {
        const (char)[] res = "switch(speedNum) {";
        foreach (speed; speedsRange)
        {
            res = res ~ "case " ~ speed ~ ": return B" ~ speed ~ "; \n";
        }
        res ~= "default: return -1;";
        res ~= '}';
        return res;
    }


    speed_t getBaudRateByNum(uint speedNum) nothrow pure
    {
        mixin (genGetBaudRateByNumBody);
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
        enum B7200  	= 7200;
        enum B14400  	= 14400;
        enum B28800  	= 28800;
        enum B57600  	= 57600;
        enum B76800  	= 76800;
        enum B115200 	= 115200;
        enum B230400 	= 230400;

        enum CCTS_OFLOW	= 0x00010000; /* CTS flow control of output */
        enum CRTS_IFLOW	= 0x00020000; /* RTS flow control of input */
        enum CRTSCTS	= (CCTS_OFLOW | CRTS_IFLOW);

        enum ECHOKE 	= 0x00000001; /* visual erase for line kill */
        enum ECHOPRT	= 0x00000020; /* visual erase mode for hardcopy */
        enum ECHOCTL 	= 0x00000040; /* echo control chars as ^(Char) */
    }
    else version (FreeBSD)
    {
        enum B7200  	= 7200;
        enum B14400  	= 14400;
        enum B28800  	= 28800;
        enum B57600  	= 57600;
        enum B76800  	= 76800;
        enum B115200 	= 115200;
        enum B230400 	= 230400;
        enum B460800 	= 460800;
        enum B921600 	= 921600;


        enum CCTS_OFLOW	= 0x00010000; /* CTS flow control of output */
        enum CRTS_IFLOW	= 0x00020000; /* RTS flow control of input */
        enum CRTSCTS	= (CCTS_OFLOW | CRTS_IFLOW);

        enum ECHOKE 	= 0x00000001; /* visual erase for line kill */
        enum ECHOPRT	= 0x00000020; /* visual erase mode for hardcopy */
        enum ECHOCTL 	= 0x00000040; /* echo control chars as ^(Char) */
    }
    else version (Solaris)
    {
        //enum CRTSCTS 	= 0x10000000;

        //enum ECHOCTL 	= 0x200;
        //enum ECHOPRT 	= 0x400;
        //enum ECHOKE 	= 0x800;
    }
}




version (vOnyxSerialTest)
{
    import std.stdio;
    import std.conv;

    unittest
    {
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
        }

        {

            //import std.stdio;
            //writeln(sp);

            auto port1 = new OxSerialPort("/dev/ttyS0", Speed.S9600, Parity.none, 1000);

            port1.open;
            ubyte[] data = [0x22, 0x33, 0xCC];
            port1.write(data);
            port1.close;
        }
    }
}
