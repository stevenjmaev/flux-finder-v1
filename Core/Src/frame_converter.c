#include "main.h"
#ifdef __FRAME_CONVERTER_H
#define MAX_CCR 65535

uint16_t dma_buf [DMA_LEN] = {0};
uint32_t frame_pxs [NUM_PX] = {0};

void print_frame(UART_HandleTypeDef huart1){
    int i;
    int len = 0;
    char buf [64] = {'\0'};

    for (i = 0; i < DMA_LEN - 1; i++){
        len = snprintf(buf, sizeof(buf), "%04d: %d\n", i, dma_buf[i]);
        HAL_UART_Transmit(&huart1, buf, len, HAL_MAX_DELAY);
    }
}

// static const uint32_t pxs [] = {
//     0x110000,
//     0x240200,
//     0x350200,
//     0x480200,
//     0x7d0200,
//     0xa20200,
//     0x02e600,
//     0x031200
// };
// TODO: need to figure out how to make them all the same luminous intensity
static const uint32_t pxs [] = {
    0x110000,
    0x1100,
    0x11
};
void init_test_frame(void){
    int i;
    int px_sel = 0;
    dma_buf[0] = OFFSET;

    uint32_t num_test_colors;
    num_test_colors = sizeof(pxs) / sizeof(pxs[0]);
  
    for (i = 0; i < NUM_PX; i++){
        set_px_color(i, pxs[px_sel]);
        px_sel = (px_sel + 1) % num_test_colors;
    }
}


void update_frame_brightness(uint8_t increase){

    // TODO: FIX THIS!! USE HSV COLOR MAPPING!!
    // uint16_t idx = 0;
    // uint32_t prev = 0;
    // float blue,green,red;

    // for (idx = 0; idx < NUM_PX; idx ++){
    //     prev = frame_pxs[idx];
    //     red = (float)((prev & 0xFF0000) >> 16);
    //     green = (float)((prev & 0x00FF00) >> 8);
    //     blue = (float)(prev & 0x0000FF);

    //     if (increase){
    //         red = MIN(255, float(red * 1.05));
    //         green = MIN(255, float(green * 1.05));
    //         blue = MIN(255, float(blue * 1.05));
    //     }
    //     else{
    //         red = MAX(0, float(red * 0.95));
    //         green = MAX(0, float(green * 0.95));
    //         blue = MAX(0, float(blue * 0.95));
    //     }
        
    //     if (red < 0 || green < 0 || blue < 0) return;
    //     else if (red > 255 || green > 255 || blue > 255) return;


    //     set_px_color(idx, (red << 16) | (green << 8) | blue);
    // }
}

uint32_t rgb_to_grb(uint32_t rgb){
    uint32_t grb = 0;
    grb |= (rgb & 0xFF0000) >> 8;
    grb |= (rgb & 0x00FF00) << 8;
    grb |= (rgb & 0x0000FF);
    return grb;
}

// 'color' should be 24-bit, RGB
void set_px_color(uint16_t idx, uint32_t rgb){
    uint16_t* px;// = NULL;
    px = &dma_buf[(NUM_PARTS_PER_BIT_CODE * NUM_BIT_PER_PX) * idx + 1]; // offset by one (the first arr number should be 14400)
    frame_pxs[idx] = rgb;
    uint16_t starting_count = (OFFSET + (idx * COUNT_PER_BIT * NUM_BIT_PER_PX)) % MAX_CCR;

    uint16_t bit_idx;
    uint32_t color;
    color = rgb_to_grb(rgb);
    for (bit_idx = (NUM_BIT_PER_PX); bit_idx > 0; bit_idx--){
        if (color & (1 << (bit_idx - 1))){ // it's a '1'
            *(px++) = (starting_count + ONE_CODE_A) % MAX_CCR;
            if (idx == NUM_PX - 1 && bit_idx == 1) return; 
            *(px++) = (starting_count + ONE_CODE_A + ONE_CODE_B) % MAX_CCR;
        }
        else { // it's a '0'
            *(px++) = (starting_count + ZERO_CODE_A) % MAX_CCR;
            if (idx == NUM_PX - 1 && bit_idx == 1) return; 
            *(px++) = (starting_count + ZERO_CODE_A + ZERO_CODE_B) % MAX_CCR;
        }
        starting_count = (starting_count + COUNT_PER_BIT) % MAX_CCR;
    }
    
}
#endif