#include <MCP251XFD.h>
#include <cstdint>

# define SPI3_BUS_SPEED

namespace SPIMotorCAN {};

MCP251XFD create_driver_config(uint32_t spi_bus_speed, uint8_t spi_bus_number, uint8_t spi_cs);