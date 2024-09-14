#ifndef __COLOR_MAPPING_H
#define __COLOR_MAPPING_H 
typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;

RgbColor HsvToRgb(HsvColor hsv);

HsvColor RgbToHsv(RgbColor rgb);
#endif // #ifndef __COLOR_MAPPING_H