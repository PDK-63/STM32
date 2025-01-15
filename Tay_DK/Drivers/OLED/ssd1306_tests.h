#ifndef __SSD1306_TEST_H__
#define __SSD1306_TEST_H__

#include <_ansi.h>
#include "main.h"
_BEGIN_STD_C

extern void ssd1306_TestBorder(void);
extern void ssd1306_TestFonts1(void);
extern void ssd1306_TestFonts2(void);
extern void ssd1306_TestFPS(void);
extern void ssd1306_TestAll(void);
extern void ssd1306_TestLine(void);
extern void ssd1306_TestRectangle(void);
extern void ssd1306_TestRectangleFill();
extern void ssd1306_TestCircle(void);
extern void ssd1306_TestArc(void);
extern void ssd1306_TestPolyline(void);
extern void ssd1306_TestDrawBitmap(void);
extern const unsigned char image_battery_128x64[];
extern const unsigned char Image_64_64[];
extern const unsigned char epd_bitmap__a_frm0_40[];
extern const unsigned char test_image[];
extern const unsigned char background3[];
extern const unsigned char Image4[];
extern const unsigned char garfield_128x64[];
_END_STD_C

#endif // __SSD1306_TEST_H__
