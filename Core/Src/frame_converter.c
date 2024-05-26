#include "main.h"

#define MAX_CCR 65535

#define NUM_PX 256
#define NUM_BIT_PER_PX 24
#define NUM_PARTS_PER_BIT_CODE 2

#define ZERO_CODE_A 15
#define ZERO_CODE_B 35
#define ONE_CODE_A 35
#define ONE_CODE_B 15

#define COUNT_PER_BIT 50 // 15 + 35

// note it's '+1' because the first one should be zero (for the reset/initialization)...
#define FRAME_BUF_SIZE (NUM_PX * NUM_BIT_PER_PX * NUM_PARTS_PER_BIT_CODE) + 1

uint16_t frame_data_buf [FRAME_BUF_SIZE];

// 'color' should be 24-bit
static void set_px_color(uint16_t idx, uint32_t color){
    uint16_t* px;// = NULL;
    px = &frame_data_buf[(NUM_PARTS_PER_BIT_CODE * NUM_BIT_PER_PX) * idx + 1]; // offset by one (the first arr number should be 0)

    uint16_t starting_count = (idx * COUNT_PER_BIT * NUM_BIT_PER_PX) % MAX_CCR;

    int bit_idx;
    for (bit_idx = (NUM_BIT_PER_PX - 1); bit_idx > 0; bit_idx--){
        if (color & (1 << bit_idx)){ // it's a '1'
            *(px++) = (starting_count + ONE_CODE_A) % MAX_CCR;
            *(px++) = (starting_count + ONE_CODE_A + ONE_CODE_B) % MAX_CCR;
        }
        else { // it's a '0'
            *(px++) = (starting_count + ZERO_CODE_A) % MAX_CCR;
            *(px++) = (starting_count + ZERO_CODE_A + ZERO_CODE_B) % MAX_CCR;
        }
        starting_count = (starting_count + COUNT_PER_BIT) % MAX_CCR;
    }
    
}