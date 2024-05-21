#include "main.h"

uint32_t g_int_cnt = 0;
void matrix_select(int row, int col){
    // TODO: rework this so it uses less function calls.
    uint8_t row0, row1, row2, row3;
    uint8_t col0, col1, col2, col3;

    row0 = (row & (1 << 0)) ? 1 : 0;
    row1 = (row & (1 << 1)) ? 1 : 0;
    row2 = (row & (1 << 2)) ? 1 : 0;
    row3 = (row & (1 << 3)) ? 1 : 0;

    col0 = (col & (1 << 0)) ? 1 : 0;
    col1 = (col & (1 << 1)) ? 1 : 0;
    col2 = (col & (1 << 2)) ? 1 : 0;
    col3 = (col & (1 << 3)) ? 1 : 0;

    HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin, row0);
    HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, row1);
    HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, row2);
    HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, row3);

    HAL_GPIO_WritePin(COL0_GPIO_Port, COL0_Pin, col0);
    HAL_GPIO_WritePin(COL1_GPIO_Port, COL1_Pin, col1);
    HAL_GPIO_WritePin(COL2_GPIO_Port, COL2_Pin, col2);
    HAL_GPIO_WritePin(COL3_GPIO_Port, COL3_Pin, col3);

}
