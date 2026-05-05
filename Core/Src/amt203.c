#include "amt203.h"

void AMT203_Init(AMT203_t *enc, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    enc->hspi    = hspi;
    enc->cs_port = cs_port;
    enc->cs_pin  = cs_pin;
    /* CS idles HIGH (encoder deselected) */
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    /* Wait for encoder startup: datasheet specifies <100 ms, use 200 ms margin */
    HAL_Delay(200);
}

/*
 * Transfer one byte to/from the encoder.
 * Per datasheet p.9: each byte MUST be followed by a CS release.
 * CS goes LOW -> 8 clock cycles -> CS goes HIGH.
 */
uint8_t AMT203_GetByte(AMT203_t *enc, uint8_t send)
{
    uint8_t recv = 0;
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(enc->hspi, &send, &recv, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(enc->cs_port, enc->cs_pin, GPIO_PIN_SET);
    return recv;
}

/*
 * Read the 12-bit absolute position.
 *
 * Protocol (datasheet p.9):
 *  1. Send 0x10 (rd_pos).  Encoder returns an idle byte.
 *  2. Keep sending 0x00 (nop) while encoder returns 0xA5 (busy/wait).
 *  3. When encoder echoes 0x10, the next two bytes are the position:
 *     - MSB: lower 4 bits = bits [11:8] of position
 *     - LSB: bits [7:0] of position
 *  4. Wait 20 µs before the next read (datasheet recommendation).
 */
uint16_t AMT203_GetPosition(AMT203_t *enc)
{
    uint8_t  response;
    uint8_t  msb, lsb;
    uint16_t position;
    uint16_t timeout = 0;

    uint16_t timeout_temp = 0;

    /* Step 1: send rd_pos command */

    response = AMT203_GetByte(enc, AMT203_RD_POS);
    HAL_Delay(1);

    /* Step 2: poll with NOP until encoder echoes rd_pos back
     * Encoder may return 0xA5 (wait) any number of times before echoing */
    while ((response != AMT203_RD_POS) && (timeout < AMT203_TIMEOUT_LIMIT))
    {
        response = AMT203_GetByte(enc, AMT203_NOP);
//        HAL_Delay(1);
        timeout_temp = 0;
		while(timeout_temp < 100){
			timeout_temp++;
		}
        timeout++;
    }

    if (timeout >= AMT203_TIMEOUT_LIMIT)
    {
        return AMT203_RD_FAILED;
    }

    /* Step 3: read MSB and LSB position bytes */
    msb = AMT203_GetByte(enc, AMT203_NOP);

    timeout = 0;
    while (timeout < AMT203_TIMEOUT_LIMIT)
    {
        timeout++;
    }
    lsb = AMT203_GetByte(enc, AMT203_NOP);

    timeout = 0;
//    while (timeout < AMT203_TIMEOUT_LIMIT)
//    {
//        timeout++;
//    }

    /* Combine into 12-bit value: lower nibble of MSB = bits [11:8] */
    position = (uint16_t)((msb & 0x0F) << 8) | (uint16_t)lsb;

    /* Step 4: recommended 20 µs inter-read delay */
    /* HAL_Delay() is 1 ms resolution — use DWT or a short busy loop instead */
    /* For simplicity here a 1 ms delay is used; replace with DWT_Delay if needed */
    HAL_Delay(1);

    return position;
}

/*
 * Set the current position as the new zero point.
 *
 * Protocol (datasheet p.9):
 *  1. Send 0x70 (set_zero_point).
 *  2. Poll with NOP until encoder responds with 0x80 (success).
 *  3. IMPORTANT: encoder must be power-cycled after this call
 *     for the new zero offset to take effect.
 *
 * Returns true if encoder confirmed success (0x80), false on timeout.
 */
bool AMT203_SetZero(AMT203_t *enc)
{
    uint8_t  response;
    uint16_t timeout = 0;

    AMT203_GetByte(enc, AMT203_SET_ZERO);

    do {
        response = AMT203_GetByte(enc, AMT203_NOP);
        timeout++;
    } while ((response != AMT203_SET_ZERO_SUCCESS) &&
             (timeout < AMT203_TIMEOUT_LIMIT));

    /*
     * Even on success the encoder MUST be power-cycled.
     * The new zero is stored in EEPROM but not applied until next power-on.
     */
    return (response == AMT203_SET_ZERO_SUCCESS);
}
