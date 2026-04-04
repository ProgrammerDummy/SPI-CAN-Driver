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
//REMEMBER MSB FIRST for individual bytes but for multi-bytes LSB first (for data transmission and reconstruction)

void SPI_read_word_from_MCP(uint16_t addr, uint32_t *data) {
    uint8_t txbuffer[6] = {0};
    uint8_t rxbuffer[6] = {0};

    txbuffer[0] = (MCP2518FD_INSTR_READ << 4) | (addr >> 8) & 0x0F;
    txbuffer[1] = addr & 0xFF; 

    spi_write_to_MCP(txbuffer, rxbuffer, 6);

    *data = (rxbuffer[2] << 0) | (rxbuffer[3] << 8) | (rxbuffer[4] << 16) | (rxbuffer[5] << 24);

}

void SPI_write_word_to_MCP(uint16_t addr, uint32_t data) {

    uint8_t txbuffer[6] = {0};
    uint8_t rxbuffer[6] = {0};

    txbuffer[0] = (MCP2518FD_INSTR_WRITE << 4) | ((addr >> 8) & 0x0F);
    txbuffer[1] = addr & 0xFF;
    txbuffer[2] = (data >> 0) & 0xFF;
    txbuffer[3] = (data >> 8) & 0xFF;
    txbuffer[4] = (data >> 16) & 0xFF;
    txbuffer[5] = (data >> 24) & 0xFF;

    spi_write_to_MCP(txbuffer, rxbuffer, 6);
}

void spi_write_to_MCP(uint8_t *txbuffer, uint8_t *rxbuffer, uint16_t len) {
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, LOW);

    spi_write_read_blocking(spi_default, txbuffer, rxbuffer, len);

    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, HIGH);

}

void spi_reset_MCP_chip() {

    uint8_t txbuffer[6] = {0};
    uint8_t rxbuffer[6] = {0};
    txbuffer[0] = (MCP2518FD_INSTR_RESET << 4) | (0x00 >> 4) & 0x0F;
    txbuffer[1] = 0x00;

    spi_write_to_MCP(txbuffer, rxbuffer, 2);

}

int8_t MCP2518fd_set_mode(CAN_OPERATION_MODE mode) {
    //see header file line 202 for CAN_OPERATION_MODE enum 
    REG_CiCON CAN_ctrl_reg;

    SPI_read_word_from_MCP(MCP2518FD_REG_CiCON, &CAN_ctrl_reg.word);

    if(CAN_ctrl_reg.bF.OpMode == mode) {
        return 0;
    }

    CAN_ctrl_reg.bF.RequestOpMode = mode;

    SPI_write_word_to_MCP(MCP2518FD_REG_CiCON, CAN_ctrl_reg.word);

    uint16_t timeout = 10000;

    while(timeout--) {
        SPI_read_word_from_MCP(MCP2518FD_REG_CiCON, &CAN_ctrl_reg.word);

        if(CAN_ctrl_reg.bF.OpMode == mode) {
            return 0;
        }

        sleep_us(100);

    }

    return -1;

}

int8_t MCP2518fd_oscillator_check() {
    REG_OSC osc_ctrl_reg; 
    //check line 544 of can.h for more info on REG_OSC bitfield union

    sleep_ms(2);
    //give time for oscillator to start if it hasn't already

    uint16_t timeout = 10000;

    uint8_t error;

    while(timeout--) {
        SPI_read_word_from_MCP(MCP2518FD_REG_OSC, &osc_ctrl_reg.word);

        if(osc_ctrl_reg.bF.OscReady) {
            return 0;
            //means that the oscillator is stable and ready to be modified
        }

        sleep_us(100);
    }

    return -1;

    /*
    I will be using 40 Mhz internal crystal oscillator (no PLL)
    which determines SYSCLK (for CAN FD controller module and RAM message memory access)
    */


}

int8_t MCP2518fd_devid_verify() {
    REG_DEVID devid_reg;
    
    SPI_read_word_from_MCP(MCP2518FD_REG_DEVID, &devid_reg.word);

    if(devid_reg.bF.DEV!= 0x01) {
        return -1;
    }

    return 0;


}

int8_t MCP2518fd_CAN_controller_config() {

    REG_CiCON CiCON_reg;

    SPI_read_word_from_MCP(MCP2518FD_REG_CiCON, &CiCON_reg.word);

    uint8_t timeout = 10000;

    while(CiCON_reg.bF.isBusy && timeout--) {
        sleep_us(100);
        SPI_read_word_from_MCP(MCP2518FD_REG_CiCON, &CiCON_reg.word);
    }

    if(!timeout) {
        return -1;
    }
    
    /*
    I am choosing the simplest configuration to make a working prototype that can be tweaked later on
    */

    CiCON_reg.bF.DNetFilterCount = 0x0;
    CiCON_reg.bF.IsoCrcEnable = 1;
    CiCON_reg.bF.ProtocolExceptionEventDisable = 0;
    CiCON_reg.bF.WakeUpFilterEnable = 0;
    CiCON_reg.bF.WakeUpFilterTime = 0;
    CiCON_reg.bF.BitRateSwitchDisable = 0;
    CiCON_reg.bF.RestrictReTxAttempts = 0;
    CiCON_reg.bF.EsiInGatewayMode = 0;
    CiCON_reg.bF.SystemErrorToListenOnly = 0;
    CiCON_reg.bF.StoreInTEF = 0;
    CiCON_reg.bF.TXQEnable = 1;
    CiCON_reg.bF.TxBandWidthSharing = 0x0;

    SPI_write_word_to_MCP(MCP2518FD_REG_CiCON, CiCON_reg.word);
    //do i need to poll again?

    return 0;

    
}

void MCP2518fd_nominal_bit_timing_config() {
    /*
    TQ (time quantum) = (BRP+1)/(SYSCLK)

    BRP is baud rate prescalar

    TQ = 25 nanoseconds if we choose BRP = 0


    every bit in CAN is transmitted/received like this:

    SYNC (1 TQ) + PROP (N TQ) + PHASE 1 (N TQ) + PHASE 2 (N TQ)

    PROP + PHASE 1 >= 2*(propagation delay + transceiver delay)/TQ (for shorter wires)

    PHASE 2 >= 2 TQ

    sampling point = (1 + PROP + PHASE 1)/(1 + PROP + PHASE 1 + PHASE 2) --> gives percentage of bit time where sampling occurs, aim for 75 - 80%? 


    NBT (nominal bit time) = 1 + PROP + PHASE 1 + PHASE 2

    Bit rate = SYSCLK / ((BRP+1)*NBT)

    Synchronization jump width (SJW) <= MIN(PHASE 1, PHASE 2) but typically 4 TQ wide


    set BRP = 0
    and set total time for each bit to be 80 TQ
    then our bit rate with a 40Mhz SYSCLK will be 500 kbps

    with 80 TQ, 1 will automatically be allocated to the SYNC section (unchangeable)

    I will go for a sampling point percentage of 80%, which means our PROP + PHASE 1 (TSEG1) will be allocated 63 TQ and our PHASE 2 (TSEG2) will be allocated 16 TQ
    for maximum flexibility i will do SJW = 16 TQ 
    */ 


    REG_CiNBTCFG NBT_reg; 

    NBT_reg.bF.SJW = 15;
    NBT_reg.bF.TSEG2 = 15;
    NBT_reg.bF.TSEG1 = 62;
    NBT_reg.bF.BRP = 0;

    //0-based register so subtract one to desired number of TQ to assign

    SPI_write_word_to_MCP(MCP2518FD_REG_CiNBTCFG, NBT_reg.word);



}

void MCP2518fd_data_bit_timing_config() {
    REG_CiDBTCFG DBT_reg;

    //nearly the same principles as NBT, but faster for data transmission, and signal sampling point will be at around 75%
    //so fewer TQ will be used (20 TQ), NBT = 20 TQ

    /*
    TQ = (BRP+1)/SYSCLK = 25 ns

    bit rate = SYSCL/((1+BRP)*NBT)
    sample point = (1+TSEG1)/NBT

    TSEG1 = 14 TQ
    TSEG2 = 5 TQ
    */

    DBT_reg.bF.BRP = 0;
    DBT_reg.bF.SJW = 3;
    DBT_reg.bF.TSEG1 = 13;
    DBT_reg.bF.TSEG2 = 4;

    SPI_write_word_to_MCP(MCP2518FD_REG_CiDBTCFG, DBT_reg.word);

}

void MCP2518fd_TDC_config() {

    REG_CiTDC TDC_reg;

    TDC_reg.bF.TDCMode = 0b10; //activate auto TDC mode
    TDC_reg.bF.TDCOffset = 0;
    TDC_reg.bF.EdgeFilterEnable = 0; //enable this if there are noise and synchronization issues in startup
    TDC_reg.bF.SID11Enable = 0;

    SPI_write_word_to_MCP(MCP2518FD_REG_CiTDC, TDC_reg.word);

}

void MCP2518fd_TXQ_FIFO_config() { //CiTXQCON
    REG_CiTXQCON TXQ_reg;


    TXQ_reg.txBF.TxNotFullIE = 0;
    TXQ_reg.txBF.TxEmptyIE = 0;
    TXQ_reg.txBF.TxAttemptIE = 0;

    //for now not using any interrupts for TX queue

    TXQ_reg.txBF.FRESET = 1; //reset all FIFOs
    TXQ_reg.txBF.TxPriority = 31; //set as highest priority FIFO
    TXQ_reg.txBF.TxAttempts = 0; //unlimited attempts for message transmission because we are treating the TX queue as highest priority, so messages are critical

    TXQ_reg.txBF.FifoSize = 3; //4 messages per slot 
    TXQ_reg.txBF.PayLoadSize = 0b000; //8 bytes 

    //FifoSize defines the number of slots within the FIFO. each slot holds a singular CAN message.
    
    SPI_write_word_to_MCP(MCP2518FD_REG_CiTXQCON, TXQ_reg.word);
    
   

}

void MCP2518fd_set_TXQ_UINC() { //UINC belongs in the CiTEFCON and CiTXQCONregister
    REG_CiTXQCON TXQ_reg;

    SPI_read_word_from_MCP(MCP2518FD_REG_CiTXQCON, &TXQ_reg.word);

    TXQ_reg.txBF.UINC = 1;

    SPI_write_word_to_MCP(MCP2518FD_REG_CiTXQCON, TXQ_reg.word);


    //should i poll here?

    //in terms of the design, UINC is meant to be incremented whenever i write a new message within the transmit buffer
    //should i do this in tandem with the TXREQ bit?


}

void MCP2518fd_FIFO_config(uint8_t n, uint8_t m) { //CiFIFOCON

    REG_CiFIFOCON TXFIFOCON_reg;

    TXFIFOCON_reg.txBF.PayLoadSize = 0b000;
    TXFIFOCON_reg.txBF.FifoSize = 9;

    TXFIFOCON_reg.txBF.TxAttempts = 2;
    TXFIFOCON_reg.txBF.TxPriority = 0; //lowest priority 
    TXFIFOCON_reg.txBF.FRESET = 1; //reset this FIFO

    TXFIFOCON_reg.txBF.TxEnable = 1; //set as Tx FIFO

    TXFIFOCON_reg.txBF.RTREnable = 1;
    TXFIFOCON_reg.txBF.TxAttemptIE = 1;
    TXFIFOCON_reg.txBF.TxEmptyIE = 1;
    TXFIFOCON_reg.txBF.TxHalfFullIE = 1;
    TXFIFOCON_reg.txBF.TxNotFullIE = 1;

    for(uint8_t i = 1; i <= n; i++) {
        SPI_write_word_to_MCP(MCP2518FD_REG_CiFIFOCON+i*12, TXFIFOCON_reg.word);
    }


    //check header files at lines 123-126 to ensure base address of FIFO and REG_STRIDE is correct

    REG_CiFIFOCON RXFIFOCON_reg;

    RXFIFOCON_reg.rxBF.PayLoadSize = 0b000;
    RXFIFOCON_reg.rxBF.FifoSize = 9;
    
    RXFIFOCON_reg.rxBF.FRESET = 1;
    RXFIFOCON_reg.rxBF.TxEnable = 0;
    RXFIFOCON_reg.rxBF.RxTimeStampEnable = 0;
    RXFIFOCON_reg.rxBF.RxOverFlowIE = 1;
    RXFIFOCON_reg.rxBF.RxFullIE = 1;
    RXFIFOCON_reg.rxBF.RxHalfFullIE = 1;
    RXFIFOCON_reg.rxBF.RxNotEmptyIE = 1;
    

    for(uint8_t j = 1; j <= m; j++) {
        SPI_write_word_to_MCP(MCP2518FD_REG_CiFIFOCON+(n+j)*12, RXFIFOCON_reg.word);
    }

    MCP2518fd_filter_and_mask_enable_config(n, m);    

}

void MCP2518fd_filter_and_mask_enable_config(uint8_t n, uint8_t m) { //CiFLTCON

    //setting catch-all filters only for RX FIFOs

    REG_CiFLTCON filter_control_reg;
    REG_CiFLTOBJ filter_reg;
    REG_CiMASK mask_reg;

    filter_reg.word = 0;
    mask_reg.word = 0;
    
    for(int i = 0; i < m; i++) {

        uint8_t fifo_num = n+1+i;

        SPI_write_word_to_MCP(MCP2518FD_REG_CiFLTOBJ + i*MCP2518FD_FILTER_REG_STRIDE, filter_reg.word);
        SPI_write_word_to_MCP(MCP2518FD_REG_CiMASK + i*MCP2518FD_FILTER_REG_STRIDE, mask_reg.word);

        uint16_t filter_control_reg_addr = MCP2518FD_REG_CiFLTCON + (i/4)*4;

        SPI_read_word_from_MCP(filter_control_reg_addr, &filter_control_reg.word);

        switch(i%4) {
            case 0:
                filter_control_reg.bF.FLTEN0 = 1;
                filter_control_reg.bF.F0BP = fifo_num;
                break;

            case 1:
                filter_control_reg.bF.FLTEN1 = 1;
                filter_control_reg.bF.F1BP = fifo_num;
                break;

            case 2:
                filter_control_reg.bF.FLTEN2 = 1;
                filter_control_reg.bF.F2BP = fifo_num;
                break;

            case 3:
                filter_control_reg.bF.FLTEN3 = 1;
                filter_control_reg.bF.F3BP = fifo_num;
                break;
        }

        SPI_write_word_to_MCP(filter_control_reg_addr, filter_control_reg.word);
    }


  
}


//CiINT
//IOCON
//set to normal mode (poll)

int8_t MCP2518fd_available_RAM_calc() {
    //RAM used = (8 + PayLoadSize)*(FifoSize+1), 
    //8 additional bytes from the header (ID word + Control word), 
    //but is 12 with timestamps (disabled)
    return 0;

}

int8_t MCP2518fd_init() {
    SPI_to_CAN_master_init();

    spi_reset_MCP_chip(); //should set CAN controller mode to configuration mode already but further checks are made later

    sleep_ms(2);

    REG_OSC osc;

    uint32_t timeout = 1000;

    while(--timeout) {
        SPI_read_word_from_MCP(MCP2518FD_REG_OSC, &osc.word);
        if(osc.bF.OscReady) {
            break;
        }
        sleep_ms(1);
    }

   

    //set to CAN FD normal at the end using the MCP2518fd_set_mode function
    //is CiCON address right??
}