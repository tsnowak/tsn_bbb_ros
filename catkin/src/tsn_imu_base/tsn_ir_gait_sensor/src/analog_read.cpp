int adc_read(unsigned int pin)
{
	int fd, len, j;
	char buf[MAX_BUF];
	char val[3];

	len = snprintf(buf, sizeof(buf), "sys/bus/iio/devices/iio:device0/in_voltage%d_raw", pin);

	fd = open(buf, O_RDONLY);

	if (fd < 0)	{
		perror("adc/get-value");
	}

	read(fd, &val, 3);
	close(fd);
}

printf ("value of pin ADC%d  = %.4s \n", pin, val);
return atoi(&val);

}