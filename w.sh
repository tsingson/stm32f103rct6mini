
west build -p always -b stm32f103_mini .
west flash --runner openocd

picocom -b 115200 /dev/cu.usbserial-210
