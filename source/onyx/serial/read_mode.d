module onyx.serial.read_mode;

enum ReadMode
{
	noWait,
	waitForTimeout,
	waitForData,
	waitForAllData
}
