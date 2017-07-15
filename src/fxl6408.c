#include <msp430.h>
#include <stdint.h>

#include <libio/console.h>

#include "fxl6408.h"

#define FXL_ADDR 0x43 // 100 0011

#define FXL_REG_ID 0x01

#define FXL_ID_MFG_MASK 0xE0
#define FXL_ID_MFG      0xA0


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
    LOG("FXL: invalid MFG id: 0x%02x (expected 0x%02x)\r\n", id & FXL_ID_MFG_MASK, FXL_ID_MFG);
    return FXL_ERR_BAD_ID;
  }

  LOG("FXL: id 0x%02x\r\n", id);

  return FXL_SUCCESS;
}
