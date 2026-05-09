#include "ArmMotorSPI.h"
//#include <Interface/MCP251XFD_V71InterfaceSync.h>
#include <soc/spi_struct.h>

// Docs for the MCP251XFD Driver can be found at:
// https://github.com/Emandhal/MCP251XFD/blob/master/Docs/MCP251XFD%20driver%20library%20guide%20(v1.0.5%20synchronous%20driver).pdf


MCP251XFD create_driver_config(uint32_t spi_bus_speed, uint8_t spi_bus_register) {
    MCP251XFD device_config = MCP251XFD{
        .UserDriverData = NULL,
        //--- Driver configuration ---
        .DriverConfig = MCP251XFD_DRIVER_NORMAL_USE,
        //--- IO configuration ---
        .GPIOsOutLevel = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_HIGH,
        //--- Interface driver call functions ---
        .SPI_ChipSelect = SPI_CS_EXT1, // Here the chip select of the EXT1 interface is 1
        .InterfaceDevice = SPI0, // Here this point to the address memory of the peripheral SPI0
        //--- Interface clocks ---
        .SPIClockSpeed = spi_bus_speed, // 17MHz
        .fnSPI_Init = MCP251XFD_InterfaceInit_V71,
        .fnSPI_Transfer = MCP251XFD_InterfaceTransfer_V71,
        //--- Time call function ---
        .fnGetCurrentms = GetCurrentms_V71,
        //--- CRC16-CMS call function ---
        .fnComputeCRC16 = ComputeCRC16_V71,
    };
    Init_MCP251XFD();
    return device_config;
};

MCP251XFD_Config create_can_config() {
    MCP251XFD_Config can_config = MCP251XFD_Config{
        .XtalFreq =,
        .OscFreq = ,
        .SysclkConfig =,
        .ClkoPinConfig =,
        .SYSCLK_Result =,

        .NominalBitrate =,
        .DataBitrate =,
        .BitTimeStats=,
        .Bandwidth=,
        .ControlFlags=,

        .GPIO0PinMode=,
        .GPIO1PinMode=,
        .INTsOutMode=,
        .TXCANOutMode=,
        .SysInterruptFlags=,
    };
}

