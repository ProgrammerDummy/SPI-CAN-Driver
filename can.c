#include "can.h"

void SPI_to_CAN_master_init(void) {
    stdio_init_all();

    spi_set_format(spi_default, MSG_SIZE, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    uint real_baudrate = spi_init(spi_default, CLK_SPD);

    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);

    gpio_pull_up(PICO_DEFAULT_SPI_RX_PIN);
    gpio_pull_up(PICO_DEFAULT_SPI_TX_PIN);

    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, HIGH);

    /*
    i can build in support for multiple slaves later by using CSn pins connected to the pico2
    would involve taking in a CSn as an argument for the write/read functions
    i can also use bit manipulation to minimize the total memory of program
    */

}

//need bit padding for 4 bytes
//need polling after reset 
//learn DMA for large data transfer
//REMEMBER MSB FIRST


int SPI_read_word_from_MCP(uint16_t addr, uint32_t *data) {
    uint8_t txbuffer[6] = {0};
    uint8_t rxbuffer[6] = {0};

    txbuffer[0] = (MCP2518FD_INSTR_READ << 4) | (addr >> 8) & 0x0F;
    txbuffer[1] = (addr << 8) & 0xFF; 

    uint8_t error = spi_write_to_MCP(txbuffer, rxbuffer, 6);

    *data = (rxbuffer[2] << 0) | (rxbuffer[3] << 8) | (rxbuffer[4] << 16) | (rxbuffer[5] << 24);

    return error;

}

int SPI_write_word_to_MCP(uint16_t addr, uint32_t data) {

    uint8_t txbuffer[6] = {0};
    uint8_t rxbuffer[6] = {0};

    txbuffer[0] = (MCP2518FD_INSTR_WRITE << 4) | ((addr >> 8) & 0x0F);
    txbuffer[1] = (addr << 8) & 0xFF;
    txbuffer[2] = (data >> 0) & 0xFF;
    txbuffer[3] = (data >> 8) & 0xFF;
    txbuffer[4] = (data >> 16) & 0xFF;
    txbuffer[5] = (data >> 24) & 0xFF;

    uint8_t error = spi_write_to_MCP(txbuffer, rxbuffer, 6);

    return error;
}


int spi_write_to_MCP(uint8_t *txbuffer, uint8_t *rxbuffer, uint16_t len) {
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, LOW);

    spi_write_read_blocking(PICO_DEFAULT_SPI, txbuffer, rxbuffer, len);

    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, HIGH);

    return 0;
}

int spi_reset_MCP_chip() {

    uint8_t txbuffer[6] = {0};
    uint8_t rxbuffer[6] = {0};
    txbuffer[0] = (MCP2518FD_INSTR_RESET << 4) | (0x00 >> 4) & 0x0F;
    txbuffer[1] = 0x00;

    uint8_t error = spi_write_to_MCP(txbuffer, rxbuffer, 2);

    return error;
}

int MCP2518fd_set_mode(CAN_OPERATION_MODE mode) {
    //see header file line 202 for CAN_OPERATION_MODE enum 
    REG_CiCON CAN_ctrl_reg;

    uint8_t error = SPI_read_word_from_MCP(MCP2518FD_REG_CiCON, &CAN_ctrl_reg.word);
    
    if(!error) {
        return -1;
        //SPI communication failed
    }

    if(CAN_ctrl_reg.bF.OpMode == mode) {
        return mode;
    }

    CAN_ctrl_reg.bF.RequestOpMode = mode;

    error = SPI_write_word_to_MCP(MCP2518FD_REG_CiCON, CAN_ctrl_reg.word);

    uint16_t timeout = 10000;

    while(timeout--) {
        error = SPI_read_word_from_MCP(MCP2518FD_REG_CiCON, &CAN_ctrl_reg.word);

        if(!error) {
            return -1;
            //SPI failed
        }

        if(CAN_ctrl_reg.bF.OpMode == mode) {
            break;
        }

        sleep_us(100);

    }

    if(!timeout) {
        return -2;
        //failed to set control mode in time
    }

}

int MCP2518fd_oscillator_config() {
    REG_OSC osc_ctrl_reg; 
    //check line 544 of can.h for more info on REG_OSC bitfield union

    sleep_ms(2);
    //give time for oscillator to start

    //oscillator is started

    uint16_t timeout = 10000;

    uint8_t error;

    while(timeout--) {
        error = SPI_read_word_from_MCP(MCP2518FD_REG_OSC, &osc_ctrl_reg.word);

        if(!error) {
            return -1; 
            //SPI communication has failed here since 0 bytes has been read
        }

        if(osc_ctrl_reg.bF.OscReady) {
            break;
            //means that the oscillator is stable and ready to be modified
        }

        sleep_us(100);
    }

    if(!timeout) {
        return -2;
        //oscillator failed to stabilize
    }

    return 0;

    /*
    I will be using 40 Mhz internal crystal oscillator (no PLL)
    which determines SYSCLK (for CAN FD controller module and RAM message memory access)
    */


    //NEED TO ENTER CONFIG MODE HERE

    osc_ctrl_reg.bF.PllEnable = 0;
    osc_ctrl_reg.bF.OscDisable = 0;
    osc_ctrl_reg.bF.LowPowerModeEnable = 0;
    osc_ctrl_reg.bF.SCLKDIV = 0;
    osc_ctrl_reg.bF.CLKODIV = 0;

    //now write to system clock with new configuration and poll again

    if(SPI_write_word_to_MCP(MCP2518FD_REG_OSC, osc_ctrl_reg.word) == 0) {
        return -1;
        //again, checks if spi communication failed
    }

    sleep_ms(5);

    timeout = 10000;

    while(timeout--) {
        error = SPI_read_word_from_MCP(MCP2518FD_REG_OSC, &osc_ctrl_reg.word);

        if(!error) {
            return -1;
            //spi failed
        }

        if(osc_ctrl_reg.bF.SclkReady) {
            break;
            //system clock stabilized
        }
    }

    if(!timeout) {
        return -2;
        //system clock failed to stabilize
    }

    return 0;

    //from here we can continue with rest of MCP initialization (move onto FIFO configs)

}

int MCP2518fd_FIFO_config() {
    

    /*
    - must set mcp2518fd chip into configuration mode (see CiCON register, more specifically REQOP bit)
    */


    return 0;
}

int devid_verification() {

}


int MCP2518fd_init() {
    spi_reset_MCP_chip();

    if(!MCP2518fd_oscillator_config()) { //i need to double check if this logic is right
        return -1;
    } 

    if(!MCP2518fd_FIFO_config()) {
        return -2; 
    }
}