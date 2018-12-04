//Derek DeCramer

#ifndef __ECE353_HW2_FONTS_H__
#define __ECE353_HW2_FONTS_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "lcd.h"



#include "TM4C123.h"
#include "gpio_port.h"
//#include "lcd.h"
#include "lcd_images.h"
#include "validate.h"
#include "ece353_hw2_fonts.h"




#define       FONT_HEIGHT     16	// height of characters in pixels
#define       FONT_WIDTH      11	// width of characters in pixels
#define				CHAR_COLUMNS		(COLS/FONT_WIDTH)		// number of characters that fit across screen
#define				CHAR_ROWS				(ROWS/FONT_HEIGHT)	// number of lines of text that fit on screen
#define				X_PADDING       ((COLS % FONT_WIDTH)/2)

/* Font data for Sitka Small 12pt */
extern const uint8_t courierNewBitmap[];

/**********************************************************
* Function Name: lcd_print_character
**********************************************************
* Will render the pixels of a character bitmap with 
* the bottom left of the character starting at 
* pixel (X_pixel,Y_pixel).
**********************************************************/
void lcd_print_character(
	uint16_t X_pixel, 
	uint16_t Y_pixel,
  uint16_t fg_color,
	uint16_t bg_color,
	char character);
		
/**********************************************************
* Function Name: lcd_print_stringXY
**********************************************************
* Summary: prints a string to the LCD screen at a specified
* XY location in specified foreground and background colors
* X will specify the number of characters across with X=0
* being the left most character position, and X=20 being
* the right most.  Y will specify the number of characters
* down the screen with Y=0 being the top of the screen and
* Y=19 being the bottom row of characters.  NOTE: There should
* be a 4 pixel offset in the X location.  The font is 11 pixels
* wide and we have 21 characters across 21*11 = 231 which leaves
* 9 pixels remaining in screen width.  We will have a padding of
* 4 pixels on the left and 5 pixels on the right.  NOTE: none
* of the hard numbers mentioned here should be used.  There
* are macros defined: CHAR_COLUMNS, CHAR_ROWS, X_PADDING in
* the associated .h file that should be used.
* Returns: Nothing
**********************************************************/
void lcd_print_stringXY(
    char *msg, 
    int8_t X,
		int8_t Y,
    uint16_t fg_color, 
    uint16_t bg_color
);
    
#endif
