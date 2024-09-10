# %%
MAX_VALUE = 65535

WIDTH_A = 45
WIDTH_B = 17
# WIDTH_A = 35
# WIDTH_B = 15

INCLUDE_START_0 = False
APPLY_OFFSET = True


# Ideas:
# use 0x110000 for all three, use 0xFF0000 for all three, use 0x010000 for all three
# increase time a bit (by 1)

# NOTE: The data bytes are: <GREEN><RED><BLUE>
single_px = 0x110000
single_px2 = 0x001100
single_px3 = 0x000011
pxs = [single_px, single_px2, single_px3]

NUM_PXS = 256
# TODO: Something weird's going on: with a pattern of 3 repeating pixels,
#       I can only do this number of pixels: (n*3) + 1 (e.g. 4, 10, 13, etc)
# TODO: Also, when it's a different number of pixels (like 11) and we use 0xEE instead of 0xFF
#       It messes with the colors of px's [4:] (0-indexed).. I think there is some bit mismatch
#       somewhere in the transmission. NEED LOGIC ANALYZER!
max_data_len = 24 * NUM_PXS # 24 bits per px, 256 px's
# Note: bitrate is ~1MHz, so the max "framerate" is:
#  1MHz / (24*255) = 163 Hz.... That should be plenty fast

data = 0
px_sel = 0
for i in range(NUM_PXS - 1, -1, -1):
    data |= (pxs[px_sel]) << (24 * i)
    print(f"byte: {i:>4}, px = {hex(pxs[px_sel])}")
    px_sel = (px_sel + 1) % 3

arr = []
if INCLUDE_START_0:
    arr.append(0)
i = True
current = 0


if APPLY_OFFSET:
    offset = 1400
    offset = 14400
    arr.append(offset)
    current += offset

current_bit = max_data_len
while current_bit > 0:
    if data & (1 << (current_bit - 1)): # it's a '1'
        width = WIDTH_A if i else WIDTH_B

    else: # it's a '0'
        width = WIDTH_B if i else WIDTH_A


    current = (current + width) % MAX_VALUE
    # current = (current + width)
    if not i:
        current_bit -= 1

        # TODO: When I add this line, it causes too much power draw on the 5V net (I think it's because it's working..)
        # current += WIDTH_B//2 # add gap between each 
    i = not i
    arr.append(current)

s = f"#define DMA_LEN {len(arr)} \n"
s += "static const uint16_t dma_buf [DMA_LEN] = {\n"

for i, a in enumerate(arr):
    if i == len(arr)-1:
        s += f"{a}"
    else:
        s += f"{a}, "
    if i % 15 == 0 and i != 0:
        s += "\n"
s += "};"
print(s)

with open("Core/Inc/test_frame.h", "+w") as f:
    f.write(r"""
            
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TEST_FRAME_H
#define __TEST_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>

    """)

    f.write(s)
    
    f.write(r"""

#ifdef __cplusplus
}
#endif

#endif // __TEST_FRAME_H
            
    """)

# %%
