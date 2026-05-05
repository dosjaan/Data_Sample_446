#ifndef AMT203_H
#define AMT203_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* AMT203 SPI commands (8-bit, MSB first) */
#define AMT203_NOP               0x00  /* no operation sent by master    */
#define AMT203_WAIT              0xA5  /* encoder busy / no data ready   */
#define AMT203_RD_POS            0x10  /* read absolute position         */
#define AMT203_SET_ZERO          0x70  /* set zero point (saves to EEPROM) */
#define AMT203_SET_ZERO_SUCCESS  0x80  /* zero-set confirmed by encoder  */
#define AMT203_RD_FAILED         0xFFFF

#define AMT203_TIMEOUT_LIMIT     200   /* raised to cover 0xA5 wait sequences */
#define AMT203_READ_DELAY_US     20    /* recommended inter-read delay (datasheet p.9) */

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
} AMT203_t;

void     AMT203_Init(AMT203_t *enc, SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *cs_port, uint16_t cs_pin);
uint8_t  AMT203_GetByte(AMT203_t *enc, uint8_t send);
uint16_t AMT203_GetPosition(AMT203_t *enc);
bool     AMT203_SetZero(AMT203_t *enc);

#endif /* AMT203_H */
