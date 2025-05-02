#ifndef SPI_TASK_H
#define SPI_TASK_H

// SPI configuration macros
#ifndef CONFIG_SPI_HOST
#define CONFIG_SPI_HOST SPI2_HOST
#endif

#ifndef CONFIG_SPI_MISO_GPIO
#define CONFIG_SPI_MISO_GPIO 2
#endif

#ifndef CONFIG_SPI_BGT60L_MISO_GPIO
#define CONFIG_SPI_BGT60L_MISO_GPIO 0  // New separate MISO for BGT60L
#endif

#ifndef CONFIG_SPI_MOSI_GPIO
#define CONFIG_SPI_MOSI_GPIO 12
#endif

#ifndef CONFIG_SPI_SCLK_GPIO
#define CONFIG_SPI_SCLK_GPIO 4
#endif

#ifndef CONFIG_SPI_CS_GPIO
#define CONFIG_SPI_CS_GPIO 5
#endif

#ifndef CONFIG_BGT60L_CS_GPIO
#define CONFIG_BGT60L_CS_GPIO 25
#endif

#ifndef CONFIG_BGT60L_RST_GPIO      // This pin is rerouted for I2C's SCL
#define CONFIG_BGT60L_RST_GPIO 22   // Select an available GPIO for RST
#endif

extern TaskHandle_t spiTaskHandle;
void init_cc1101(void);
void init_bgt60l(void);

#endif // SPI_TASK_H