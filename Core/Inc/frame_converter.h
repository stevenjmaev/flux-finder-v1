
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FRAME_CONVERTER_H
#define __FRAME_CONVERTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>

#define NUM_PX 64
#define NUM_BIT_PER_PX 24
#define NUM_PARTS_PER_BIT_CODE 2

#define ZERO_CODE_A 17
#define ZERO_CODE_B 45
#define ONE_CODE_A 45
#define ONE_CODE_B 17

#define COUNT_PER_BIT (ZERO_CODE_A + ZERO_CODE_B)

#define OFFSET 14400
// note it's '+1' because the first one should be 14400 (for the reset/initialization)...
// Had to remove the +1 because we were off by one toggle (otherwise we were leaving the line high during idle)
#define DMA_LEN (NUM_PX / 2 * NUM_BIT_PER_PX * NUM_PARTS_PER_BIT_CODE)

uint32_t rgb_to_grb(uint32_t rgb);
void init_test_frame(void);
void set_px_color(uint16_t idx, uint32_t color);
void update_frame_brightness(uint8_t increase);
void load_half_frame(void);

extern uint16_t dma_buf [DMA_LEN];
extern uint32_t frame_pxs [NUM_PX];

#ifdef __cplusplus
}
#endif

#endif // __FRAME_CONVERTER_H
