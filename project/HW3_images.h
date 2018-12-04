#ifndef __HW3_IMAGES_H__
#define __HW3_IMAGES_H__

#include <stdint.h>
#include <stdio.h>

//#define       player_HEIGHT 	58	// height of player image in pixels
#define       player_HEIGHT 	37	// height of player image in pixels
//#define       player_WIDTH  	47	// width of player image in pixels
#define       player_WIDTH  	32	// width of player image in pixels
#define				MISSLE_HEIGHT	15	// height of missle image in pixels
#define				MISSLE_WIDTH	7		// width of missle image in pixels
#define				PLATFORM_HEIGHT	4
#define				PLATFORM_WIDTH	20
#define				ARROW_WIDTH 8
#define				ARROW_HEIGHT	7
#define				MONSTER_WIDTH 32
#define				MONSTER_HEIGHT 29

extern const uint8_t playerBitmap[];
extern const uint8_t missleBitmap[];
extern const uint8_t missleErase[];
extern const uint8_t platformBitmap[];
extern const uint8_t arrowBitmap[];
extern const uint8_t monsterBitmap[];

#endif
