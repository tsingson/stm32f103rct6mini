# LIS3DSH 三轴加速度

更新上述代码后重新编译。如果串口一直卡在 Searching for sensor... Read WHO_AM_I = 0x00，请逐一确认以下物理连接：

MISO 与 MOSI 接反：

板子 PA7 (MOSI) -> 传感器 SDA/SDI

板子 PA6 (MISO) -> 传感器 SDO

片选引脚 (CS) 未接或接错：

必须将板子的 PA4 引脚连接到传感器的 CS 引脚上。如果 CS 悬空，传感器将默认处于 I2C 模式，绝对不会响应 SPI 信号。

传感器供电与地：

确保传感器的 VCC 接了 3.3V，且传感器的 GND 与开发板的 GND 共地。

SPI 模式不兼容：

如果硬件是某些特定的国产兼容芯片，可能不支持 Mode 3。你可以尝试修改代码中 spi_dev 的声明，删掉 | SPI_MODE_CPOL | SPI_MODE_CPHA，降回 Mode 0 试试。
