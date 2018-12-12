#include <msp430.h>
#include <stdint.h>

#include <libio/console.h>
#include <libmspware/driverlib.h>
#include "fxl6408.h"

#define FXL_ADDR 0x43 // 100 0011

#define FXL_REG_ID         0x01
#define FXL_REG_IO_DIR     0x03
#define FXL_REG_OUT_STATE  0x05
#define FXL_REG_HIGH_Z     0x07
#define FXL_REG_IN_DEF     0x09
#define FXL_REG_PULL_EN    0x0B
#define FXL_REG_PULL_DIR   0x0D
#define FXL_REG_INT_STATUS 0x11
#define FXL_REG_INT_MASK   0x13

#define FXL_ID_SW_RST   0x1
#define FXL_ID_MFG_MASK 0xE0
#define FXL_ID_MFG      0xA0

static uint8_t out_state =     0x0;
static uint8_t dir_state =     0x0;
static uint8_t pull_state =    0x0;
static uint8_t int_state =     0x0;
static uint8_t def_state =     0x0;
static uint8_t pull_en_state = 0x0;

static void set_reg(unsigned reg, unsigned val)
{
  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start

  // Have to wait for addr transmission to finish, otherwise the TXIFG does not
  // behave as expected (both reg and val writes below fall through, despite
  // waits on TXIFG).
  while((UCB0CTLW0 & UCTXSTT));

  while(!(UCB0IFG & UCTXIFG));
  UCB0TXBUF = reg;

  while(!(UCB0IFG & UCTXIFG));
  UCB0TXBUF = val;

  while(!(UCB0IFG & UCTXIFG));
  UCB0CTLW0 |= UCTXSTP; // stop

  while (UCB0STATW & UCBBUSY);
}

static void set_reg_safe(unsigned reg, unsigned val)
{
  // Set slave address //
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = FXL_ADDR; // Set slave address
  UCB0CTLW0 &= ~UCSWRST; // enable
  while (UCB0STATW & UCBBUSY); // is bus busy? then wait!

  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start

  // Have to wait for addr transmission to finish, otherwise the TXIFG does not
  // behave as expected (both reg and val writes below fall through, despite
  // waits on TXIFG).
  while((UCB0CTLW0 & UCTXSTT));

  while(!(UCB0IFG & UCTXIFG));
  UCB0TXBUF = reg;

  while(!(UCB0IFG & UCTXIFG));
  UCB0TXBUF = val;

  while(!(UCB0IFG & UCTXIFG));
  UCB0CTLW0 |= UCTXSTP; // stop

  while (UCB0STATW & UCBBUSY);
}

/*static void set_reg_safe(unsigned reg, unsigned val) {
	EUSCI_B_I2C_disable(EUSCI_B0_BASE);
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, FXL_ADDR);
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
	while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
  
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, reg);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, val);
  EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
}*/

fxl_status_t fxl_init()
{
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = FXL_ADDR;
  UCB0CTLW0 &= ~UCSWRST; // enable

  while (UCB0STATW & UCBBUSY);

  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start
  while(!(UCB0IFG & UCTXIFG));
  UCB0TXBUF = FXL_REG_ID;
  while(!(UCB0IFG & UCTXIFG));

  UCB0CTLW0 &= ~UCTR; // receive mode
  UCB0CTLW0 |= UCTXSTT; // repeated start

  // wait for addr transmission to finish, data transfer to start
  while(UCB0CTLW0 & UCTXSTT);
  UCB0CTLW0 |= UCTXSTP; // stop

  while(!(UCB0IFG & UCRXIFG));
  uint8_t id = UCB0RXBUF;

  while (UCB0STATW & UCBBUSY);

  if ((id & FXL_ID_MFG_MASK) != FXL_ID_MFG) {
    PRINTF("FXL: invalid MFG id: 0x%02x (expected 0x%02x)\r\n", id & FXL_ID_MFG_MASK, FXL_ID_MFG);
    return FXL_ERR_BAD_ID;
  }

  //PRINTF("FXL: id 0x%02x\r\n", id);

  set_reg_safe(FXL_REG_ID, id | FXL_ID_SW_RST); // reset to synch with local state
  out_state = 0x00;

  set_reg_safe(FXL_REG_HIGH_Z, 0x00); // output from Output State reg, not High-Z

  return FXL_SUCCESS;
}

fxl_status_t fxl_out(uint8_t bit)
{
    dir_state |= bit;
    set_reg_safe(FXL_REG_IO_DIR, dir_state);
    return FXL_SUCCESS;
}

fxl_status_t fxl_in(uint8_t bit)
{
    dir_state &= ~bit;
    set_reg_safe(FXL_REG_IO_DIR, dir_state);
    return FXL_SUCCESS;
}

fxl_status_t fxl_set_io_reg(uint8_t val)
{
    dir_state = val;
    set_reg_safe(FXL_REG_IO_DIR,dir_state);
    return FXL_SUCCESS;
}

fxl_status_t fxl_pull_up(uint8_t bit)
{
    pull_state |= bit;
    pull_en_state |= bit;
    set_reg(FXL_REG_PULL_DIR, pull_state);
    set_reg(FXL_REG_PULL_EN, pull_en_state);
    return FXL_SUCCESS;
}

fxl_status_t fxl_pull_down(uint8_t bit)
{
    pull_state &= ~bit;
    pull_en_state |= bit;
    set_reg(FXL_REG_PULL_DIR, pull_state);
    set_reg(FXL_REG_PULL_EN, pull_en_state);
    return FXL_SUCCESS;
}

fxl_status_t fxl_set(uint8_t bit)
{
    out_state |= bit;
    set_reg_safe(FXL_REG_OUT_STATE, out_state);
    return FXL_SUCCESS;
}

fxl_status_t fxl_clear(uint8_t bit)
{
    out_state &= ~bit;
    set_reg_safe(FXL_REG_OUT_STATE, out_state);
    return FXL_SUCCESS;
}

fxl_status_t fxl_mask_int(uint8_t bit)
{
    int_state |= bit;
    set_reg_safe(FXL_REG_INT_MASK, int_state);
    return FXL_SUCCESS;
}

fxl_status_t fxl_unmask_int(uint8_t bit)
{
    int_state &= ~bit;
    set_reg_safe(FXL_REG_INT_MASK, int_state);
    return FXL_SUCCESS;
}

fxl_status_t fxl_set_default_low(uint8_t bit)
{
    def_state |= bit;
    set_reg_safe(FXL_REG_IN_DEF, def_state);
    return FXL_SUCCESS;
}

fxl_status_t fxl_set_default_high(uint8_t bit)
{
    def_state &= ~bit;
    set_reg_safe(FXL_REG_IN_DEF, def_state);
    return FXL_SUCCESS;
}

fxl_int_status_t fxl_read_int_status()
{
    fxl_int_status_t int_status;
    // Set slave address //
    UCB0CTLW0 |= UCSWRST; // disable
    UCB0I2CSA = FXL_ADDR; // Set slave address
    UCB0CTLW0 &= ~UCSWRST; // enable

    while (UCB0STATW & UCBBUSY); // is bus busy? then wait!

    // Query reg
    UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start
    while(!(UCB0IFG & UCTXIFG)); // wait until txbuf is empty

    UCB0TXBUF = FXL_REG_INT_STATUS; // fill txbuf with reg address

    while(!(UCB0IFG & UCTXIFG)); // wait until txbuf is empty

    UCB0CTLW0 &= ~UCTR; // receive mode
    UCB0CTLW0 |= UCTXSTT; // repeated start

    // wait for addr transmission to finish, data transfer to start
    while(UCB0CTLW0 & UCTXSTT);

    UCB0CTLW0 |= UCTXSTP; // stop

    while(!(UCB0IFG & UCRXIFG)); // wait until txbuf is empty
    int_status = UCB0RXBUF; // read out of rx buf

    while (UCB0STATW & UCBBUSY); // hang out until bus is quiet

    return int_status;
}


