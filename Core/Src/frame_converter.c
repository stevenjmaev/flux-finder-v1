#include "main.h"
#ifdef __FRAME_CONVERTER_H
#define MAX_CCR 65535

uint16_t dma_buf [DMA_LEN] = {0};
uint32_t frame_pxs [NUM_PX] = {0};

static uint16_t * dma_buf_ptr = &dma_buf[0] + 1;
static uint16_t px_idx = 0;

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

//     // These are at 82% value 
//     // (easy for viewing on google color picker)
//     // 0xd13400,
//     // 0xd15e00,
//     // 0xd17600,
//     // 0xd19600,
//     // 0xd1b500,
//     // 0xd1ca00,
//     // 0xa4d100,
//     // 0x5ed100
    
//     // These are at 6.5% value (won't stress the 5V line too much..)
//     0x110400,
//     0x110700,
//     0x110900,
//     0x110c00,
//     0x110e00,
//     0x111000,
//     0x0d1100,
//     0x071100
// };

static const uint32_t pxs [] = {
    0x110000,
    0x1100,
    0x11
};
void init_test_frame(void){
    uint8_t i = 0;
    uint8_t color_sel = 0;
    dma_buf[0] = OFFSET;

    uint32_t num_test_colors;
    num_test_colors = sizeof(pxs) / sizeof(pxs[0]);
  
    for (i = 0; i < NUM_PX; i++){
        frame_pxs[i] = pxs[color_sel];
        color_sel = (color_sel + 1) % num_test_colors;
    }
    load_half_frame();
    load_half_frame();
}


void update_frame_brightness(uint8_t increase){

    uint16_t idx = 0;
    uint32_t prev = 0;
    RgbColor rgb;
    HsvColor hsv;

    for (idx = 0; idx < NUM_PX; idx ++){
        prev = frame_pxs[idx];
        rgb.r = (float)((prev & 0xFF0000) >> 16);
        rgb.g = (float)((prev & 0x00FF00) >> 8);
        rgb.b = (float)(prev & 0x0000FF);

        hsv = RgbToHsv(rgb);

        if (increase){
            hsv.v++;
        }
        else{
            hsv.v--;
        }
        rgb = HsvToRgb(hsv);

        // if (red < 0 || green < 0 || blue < 0) return;
        // else if (red > 255 || green > 255 || blue > 255) return;

        set_px_color(idx, (rgb.r << 16) | (rgb.g << 8) | rgb.b);
    }
}

uint32_t rgb_to_grb(uint32_t rgb){
    uint32_t grb = 0;
    grb |= (rgb & 0xFF0000) >> 8;
    grb |= (rgb & 0x00FF00) << 8;
    grb |= (rgb & 0x0000FF);
    return grb;
}

void load_half_frame(void){
    int i;
    if (dma_buf_ptr == &dma_buf[0] + DMA_LEN) dma_buf_ptr = &dma_buf[0];
    if (px_idx == NUM_PX) px_idx = 0;

    for (i = 0; i < NUM_PX / 4; i++){
        set_px_color(px_idx, frame_pxs[px_idx]);
        px_idx++;
    }
}

// 'color' should be 24-bit, RGB
void set_px_color(uint16_t idx, uint32_t rgb){
    uint16_t* px;// = NULL;
    px = dma_buf_ptr; // &dma_buf[(NUM_PARTS_PER_BIT_CODE * NUM_BIT_PER_PX) * idx + 1]; // offset by one (the first arr number should be 14400)
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
    // if (idx == NUM_PX) *(px++) = starting_count + OFFSET;
    dma_buf_ptr = px;
}
#endif