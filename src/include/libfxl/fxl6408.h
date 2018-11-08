#ifndef FXL6408_H
#define FXL6408_H

#include <stdint.h>

typedef enum {
    FXL_SUCCESS = 0,
    FXL_ERR_BAD_ID,
} fxl_status_t;

typedef uint8_t fxl_int_status_t;

fxl_status_t fxl_init();
fxl_status_t fxl_out(uint8_t bit);
fxl_status_t fxl_in(uint8_t bit);
fxl_status_t fxl_set(uint8_t bit);
fxl_status_t fxl_clear(uint8_t bit);
fxl_status_t fxl_pull_up(uint8_t bit);
fxl_status_t fxl_pull_down(uint8_t bit);
fxl_status_t fxl_unmask_int(uint8_t bit);
fxl_status_t fxl_mask_int(uint8_t bit);
fxl_int_status_t fxl_read_int_status();
fxl_status_t fxl_set_default_low(uint8_t bit);
fxl_status_t fxl_set_default_high(uint8_t bit);

#endif // FXL6408_H
