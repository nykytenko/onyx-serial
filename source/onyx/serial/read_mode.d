module onyx.serial.read_mode;

enum ReadMode
{
	no_wait,
	wait_for_timeout,
	wait_for_data,
	wait_for_full_buffer
}
