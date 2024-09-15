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
