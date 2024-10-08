# %%
import colorsys

rgbs = [
    0xd13400,
    0xd15e00,
    0xd17600,
    0xd19600,
    0xd1b500,
    0xd1ca00,
    0xa4d100,
    0x5ed100
]

# %%
for rgb in rgbs:
    r = (rgb & (0xff << 16)) >> 16
    g = (rgb & (0xff << 8)) >> 8
    b = (rgb & (0xff << 0)) >> 0
    hsv = list(colorsys.rgb_to_hsv(r/255, g/255, b/255))
    hsv[2] = 0.065

    new_rgb = colorsys.hsv_to_rgb(*hsv)
    print(f"0x{round(new_rgb[0]*255):02x}{round(new_rgb[1]*255):02x}{round(new_rgb[2]*255):02x}")
    
# %%
offset=20000
cnt_bit=17+45
num_bit=24
max_ccr=65535
def get_last(idx):
    start=(offset+(idx*cnt_bit*num_bit))%max_ccr
    return start

for i in range(0,256):
    last = get_last(i)
    if last >=0 and last <= (cnt_bit*num_bit):
        if i <= 253:
            next_last = get_last(i+1)
            nextnext_last = get_last(i+2)
        print(f"bit {i:>03} : last={last:>5} diff={offset-last:>5} --> {((offset-last)/48)} us")
        print(f"bit {i+1:>03} : nextlast={next_last:>5} diff={offset-next_last:>5} --> {((offset-next_last)/48)} us")
        print(f"bit {i+2:>03} : nextnextlast={nextnext_last:>5} diff={offset-nextnext_last:>5} --> {((offset-nextnext_last)/48)} us")
        print("")
# %%
