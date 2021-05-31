#ifndef TEST_PATTERN_H_
#define TEST_PATTERN_H_

// Characters 0..63 matches the original 6847 with high confidence.
//
// Chararacters 64..95 are additional:
// 64: £
// 65: a
// ..
// 66: z
// 91: {
// 92: |
// 93: }
// 94: ↓
// 95: →
//
// the lower case have true decenders and match the style of the upper case

uint8_t fontdata_6847[] =
{
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x02, 0x1A, 0x2A, 0x2A, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x14, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3C, 0x12, 0x12, 0x1C, 0x12, 0x12, 0x3C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x20, 0x20, 0x20, 0x22, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3C, 0x12, 0x12, 0x12, 0x12, 0x12, 0x3C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3E, 0x20, 0x20, 0x3C, 0x20, 0x20, 0x3E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3E, 0x20, 0x20, 0x3C, 0x20, 0x20, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1E, 0x20, 0x20, 0x26, 0x22, 0x22, 0x1E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x02, 0x22, 0x22, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x24, 0x28, 0x30, 0x28, 0x24, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x36, 0x2A, 0x2A, 0x22, 0x22, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x32, 0x2A, 0x26, 0x22, 0x22, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3E, 0x22, 0x22, 0x22, 0x22, 0x22, 0x3E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3C, 0x22, 0x22, 0x3C, 0x20, 0x20, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x22, 0x22, 0x2A, 0x24, 0x1A, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3C, 0x22, 0x22, 0x3C, 0x28, 0x24, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x10, 0x08, 0x04, 0x22, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x14, 0x14, 0x08, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x2A, 0x2A, 0x36, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x14, 0x08, 0x14, 0x22, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x14, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3E, 0x02, 0x04, 0x08, 0x10, 0x20, 0x3E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x38, 0x20, 0x20, 0x20, 0x20, 0x20, 0x38, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x20, 0x20, 0x10, 0x08, 0x04, 0x02, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x1C, 0x2A, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x08, 0x10, 0x3E, 0x10, 0x08, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x14, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x14, 0x14, 0x36, 0x00, 0x36, 0x14, 0x14, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x1E, 0x20, 0x1C, 0x02, 0x3C, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x32, 0x32, 0x04, 0x08, 0x10, 0x26, 0x26, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x28, 0x28, 0x10, 0x2A, 0x24, 0x1A, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x10, 0x20, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x08, 0x1C, 0x3E, 0x1C, 0x08, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x10, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x02, 0x02, 0x04, 0x08, 0x10, 0x20, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x18, 0x24, 0x24, 0x24, 0x24, 0x24, 0x18, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x18, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x02, 0x1C, 0x20, 0x20, 0x3E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x02, 0x0C, 0x02, 0x22, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x04, 0x0C, 0x14, 0x3E, 0x04, 0x04, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3E, 0x20, 0x3C, 0x02, 0x02, 0x22, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x20, 0x20, 0x3C, 0x22, 0x22, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3E, 0x02, 0x04, 0x08, 0x10, 0x20, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x22, 0x1C, 0x22, 0x22, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x22, 0x1E, 0x02, 0x02, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x08, 0x10, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x04, 0x08, 0x10, 0x20, 0x10, 0x08, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x18, 0x24, 0x04, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00,

    /* Lower case */
    0x00, 0x00, 0x00, 0x0C, 0x12, 0x10, 0x38, 0x10, 0x10, 0x3C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x02, 0x1E, 0x22, 0x1E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x20, 0x20, 0x3C, 0x22, 0x22, 0x22, 0x3C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x20, 0x20, 0x20, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x02, 0x02, 0x1E, 0x22, 0x22, 0x22, 0x1E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x22, 0x3E, 0x20, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x0C, 0x12, 0x10, 0x38, 0x10, 0x10, 0x10, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x22, 0x22, 0x22, 0x1E, 0x02, 0x1C,
    0x00, 0x00, 0x00, 0x20, 0x20, 0x3C, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x00, 0x18, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x04, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x24, 0x18,
    0x00, 0x00, 0x00, 0x20, 0x20, 0x24, 0x28, 0x38, 0x24, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x18, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x2A, 0x2A, 0x2A, 0x2A, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x32, 0x22, 0x22, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x22, 0x22, 0x22, 0x3C, 0x20, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x22, 0x22, 0x22, 0x1E, 0x02, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x32, 0x20, 0x20, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x20, 0x1C, 0x02, 0x3C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x10, 0x3C, 0x10, 0x10, 0x10, 0x12, 0x0C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x26, 0x1A, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x14, 0x14, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x2A, 0x2A, 0x1C, 0x14, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x14, 0x08, 0x14, 0x22, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x1E, 0x02, 0x1C,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x04, 0x08, 0x10, 0x3E, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x10, 0x10, 0x20, 0x10, 0x10, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x00, 0x08, 0x08, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x04, 0x04, 0x02, 0x04, 0x04, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x2A, 0x1C, 0x08, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x04, 0x3E, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00
};

// This font matches the CoCo3 GIME VDC
//
// It used ntsc_round_fontdata8x12 from MAME, shifter up one pixel and
// right one pixel so it matches the alignent of the 6847T1.

uint8_t fontdata_gime[] =
{
    0x00, 0x1C, 0x22, 0x02, 0x1A, 0x26, 0x26, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x14, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3C, 0x12, 0x12, 0x1C, 0x12, 0x12, 0x3C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x20, 0x20, 0x20, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3C, 0x12, 0x12, 0x12, 0x12, 0x12, 0x3C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x20, 0x20, 0x38, 0x20, 0x20, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x20, 0x20, 0x38, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x20, 0x20, 0x26, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x24, 0x28, 0x30, 0x28, 0x24, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x36, 0x2A, 0x2A, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x32, 0x2A, 0x26, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3C, 0x22, 0x22, 0x3C, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x22, 0x22, 0x2A, 0x24, 0x1A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3C, 0x22, 0x22, 0x3C, 0x28, 0x24, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x20, 0x1C, 0x02, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x22, 0x14, 0x14, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x22, 0x22, 0x2A, 0x36, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x14, 0x08, 0x14, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x14, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x02, 0x04, 0x08, 0x10, 0x20, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x1C, 0x2A, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x10, 0x3E, 0x10, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x14, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x14, 0x14, 0x3E, 0x14, 0x3E, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x1E, 0x28, 0x1C, 0x0A, 0x3C, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x30, 0x32, 0x04, 0x08, 0x10, 0x26, 0x06, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x28, 0x28, 0x10, 0x2A, 0x24, 0x1A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x08, 0x10, 0x10, 0x10, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x08, 0x04, 0x04, 0x04, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x2A, 0x1C, 0x1C, 0x2A, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x20, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x26, 0x2A, 0x32, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x18, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x02, 0x1C, 0x20, 0x20, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x02, 0x04, 0x02, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x0C, 0x14, 0x24, 0x3E, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x20, 0x3C, 0x02, 0x02, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x20, 0x20, 0x3C, 0x22, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x02, 0x04, 0x08, 0x10, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x22, 0x1C, 0x22, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x22, 0x1E, 0x02, 0x02, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x08, 0x08, 0x10, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x08, 0x10, 0x20, 0x10, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3E, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x02, 0x04, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,

    /* Lower case */
    0x00, 0x08, 0x14, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x02, 0x1E, 0x22, 0x1E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x2C, 0x32, 0x22, 0x32, 0x2C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x20, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x02, 0x1A, 0x26, 0x22, 0x26, 0x1A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x3E, 0x20, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x0A, 0x08, 0x1C, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1A, 0x26, 0x22, 0x26, 0x1A, 0x02, 0x1C, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x2C, 0x32, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x00, 0x18, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x00, 0x02, 0x02, 0x02, 0x02, 0x22, 0x1C, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x24, 0x28, 0x30, 0x28, 0x24, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x18, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3C, 0x2A, 0x2A, 0x2A, 0x2A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x2C, 0x32, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3C, 0x22, 0x22, 0x22, 0x3C, 0x20, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1E, 0x22, 0x22, 0x22, 0x1E, 0x02, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x2C, 0x32, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1E, 0x20, 0x1C, 0x02, 0x3C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x10, 0x38, 0x10, 0x10, 0x12, 0x0C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x26, 0x1A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x14, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x2A, 0x2A, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x14, 0x08, 0x14, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x1E, 0x02, 0x1C, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3E, 0x04, 0x08, 0x10, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x08, 0x08, 0x10, 0x08, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x08, 0x08, 0x00, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x08, 0x08, 0x04, 0x08, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x2A, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00
};

// This font matches the font in the data sheet for the the MC6847T1, and also
// matches photos on the internet of real CoCo2 machines with the XC80652P,
// Motorola's pre-release version of the MC6847T1.

uint8_t fontdata_6847t1[] = {
    0x00, 0x1C, 0x22, 0x02, 0x1A, 0x26, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x14, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3C, 0x12, 0x12, 0x1C, 0x12, 0x12, 0x3C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x20, 0x20, 0x20, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3C, 0x12, 0x12, 0x12, 0x12, 0x12, 0x3C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x20, 0x20, 0x38, 0x20, 0x20, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x20, 0x20, 0x38, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1E, 0x20, 0x20, 0x26, 0x22, 0x22, 0x1E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x22, 0x3E, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x02, 0x02, 0x02, 0x22, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x24, 0x28, 0x30, 0x28, 0x24, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x36, 0x2A, 0x2A, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x32, 0x2A, 0x26, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3C, 0x22, 0x22, 0x3C, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x22, 0x22, 0x2A, 0x24, 0x1A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3C, 0x22, 0x22, 0x3C, 0x28, 0x24, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x20, 0x1C, 0x02, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x22, 0x14, 0x14, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x22, 0x22, 0x2A, 0x36, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x14, 0x08, 0x14, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x22, 0x22, 0x14, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x02, 0x04, 0x08, 0x10, 0x20, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x1C, 0x2A, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x10, 0x3E, 0x10, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x14, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x14, 0x14, 0x3E, 0x00, 0x3E, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x1E, 0x20, 0x1C, 0x02, 0x3C, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x30, 0x32, 0x04, 0x08, 0x10, 0x26, 0x06, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x28, 0x28, 0x10, 0x2A, 0x24, 0x1A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x18, 0x18, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x08, 0x10, 0x10, 0x10, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x08, 0x04, 0x04, 0x04, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x2A, 0x1C, 0x1C, 0x2A, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x10, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x26, 0x2A, 0x32, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x18, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x02, 0x1C, 0x20, 0x20, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x02, 0x0C, 0x02, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x0C, 0x14, 0x24, 0x3E, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x20, 0x3C, 0x02, 0x02, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x0C, 0x10, 0x20, 0x3C, 0x22, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x02, 0x04, 0x08, 0x10, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x22, 0x1C, 0x22, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x22, 0x1E, 0x02, 0x04, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x10, 0x20, 0x00, 0x00,
    0x00, 0x04, 0x08, 0x10, 0x20, 0x10, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3E, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x08, 0x04, 0x02, 0x04, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1C, 0x22, 0x02, 0x04, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x14, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x02, 0x1E, 0x22, 0x1E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x2C, 0x32, 0x22, 0x32, 0x2C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x20, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x02, 0x1A, 0x26, 0x22, 0x26, 0x1A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x3E, 0x20, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x0A, 0x08, 0x1C, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1A, 0x26, 0x26, 0x1A, 0x02, 0x22, 0x1C, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x2C, 0x32, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x00, 0x18, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x02, 0x00, 0x02, 0x02, 0x02, 0x22, 0x1C, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x20, 0x24, 0x28, 0x30, 0x28, 0x24, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x18, 0x08, 0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x34, 0x2A, 0x2A, 0x2A, 0x2A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x2C, 0x32, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1C, 0x22, 0x22, 0x22, 0x1C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x2C, 0x32, 0x22, 0x32, 0x2C, 0x20, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1A, 0x26, 0x22, 0x26, 0x1A, 0x02, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x2C, 0x32, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1E, 0x20, 0x1C, 0x02, 0x3C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x0A, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x26, 0x1A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x14, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x2A, 0x2A, 0x14, 0x14, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x14, 0x08, 0x14, 0x22, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x1E, 0x02, 0x22, 0x1C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x3E, 0x04, 0x08, 0x10, 0x3E, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x04, 0x08, 0x08, 0x10, 0x08, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x08, 0x08, 0x00, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x08, 0x08, 0x04, 0x08, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x10, 0x2A, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00
};


#define RED_1 0x0400
#define RED_2 0x0800
#define RED 0x0C00

#define GREEN_1 0x1000
#define GREEN_2 0x2000
#define GREEN 0x3000

#define BLUE_1 0x4000
#define BLUE_2 0x8000
#define BLUE 0xC000

#define BLACK 0
#define YELLOW (RED | GREEN)
#define YELLOW_1 (RED_1 | GREEN_1)
#define WHITE (RED | GREEN | BLUE)
#define WHITE_1 (RED_1 | GREEN_1 | BLUE_1)
#define WHITE_2 (WHITE_1 << 1)
#define COLOUR YELLOW
#define CYAN (GREEN | BLUE)
#define ORANGE (RED | GREEN_2)
#define MAGENTA (RED | BLUE)

uint16_t text_palette[8] = {
    GREEN,
    YELLOW,
    GREEN,
    RED,
    ORANGE,
    CYAN,
    ORANGE,
    ORANGE

};

uint16_t colour_palette_atom[8] = {
    GREEN,
    YELLOW,
    BLUE,
    RED,
    WHITE,
    CYAN,
    MAGENTA,
    ORANGE,
};

uint16_t colour_palette_vga80[8] = {
    BLACK,
    BLUE,
    GREEN,
    CYAN,
    RED,
    MAGENTA,
    YELLOW,
    WHITE
};

uint16_t colour_palette_improved[4] = {
    BLACK,
    YELLOW,
    GREEN,
    MAGENTA,
};

// Masks to extract pixel colours from SG4 and SG6 bytes
#define SG4_COL_MASK    0x70
#define SG6_COL_MASK    0xC0

#define SG4_COL_SHIFT   4
#define SG6_COL_SHIFT   6

// Masks to extact bit paterns from SG4 and SG6
#define SG4_PAT_MASK    0x0F
#define SG6_PAT_MASK    0x3F

// Bytes / char array for text / semigraphics modes
// SG mode                     4  8  12 24  6
const uint sg_bytes_row[5]  = {1, 4, 6, 12, 1};

#define TEXT_INDEX  0
#define SG4_INDEX   0
#define SG8_INDEX   1
#define SG12_INDEX  2
#define SG24_INDEX  3
#define SG6_INDEX   4

uint16_t *colour_palette = colour_palette_atom;

inline bool is_colour(uint mode)
{
    return !(mode & 0b10);
};

//                             0  1a   1   2a    2   3a    3   4a    4
//                             0  1    2    3    4    5    6    7    8
const uint width_lookup[9] =  {32, 64, 128, 128, 128, 128, 128, 128, 256};
const uint height_lookup[9] = {16, 64,  64,  64,  96,  96, 192, 192, 192};

uint get_width(uint mode)
{
    uint m = (mode + 1) / 2;
    return width_lookup[m];
};

uint get_height(uint mode)
{
    uint m = (mode + 1) / 2;
    return height_lookup[m];
};

uint bytes_per_row(uint mode)
{
    return is_colour(mode) ? get_width(mode) * 2 / 8 : get_width(mode) / 8;
};

#endif /* TEST_PATTERN_H_ */
