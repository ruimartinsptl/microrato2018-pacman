
pio-init:
	 pio init -b megaatmega2560 --ide clion

pio-upload:
	 pio run -t upload -e mega

pio-plot:
	python tcp_serial_redirect.py /dev/cu.usbmodem1421 115200 -P 8000

pio-run:
	pio run

#megaflash:
#    pio run
#
#    ./megaflash 192.168.1.177 .pioenvs/mega/firmware.hex
