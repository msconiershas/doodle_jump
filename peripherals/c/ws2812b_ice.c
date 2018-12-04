#include <ws2812b.h>
void ws2812b_rotate(WS2812B_t *base, uint8_t num_leds){
	WS2812B_t firstcopy = base[0];
	int i;
	for(i = 0; i < num_leds - 1; i++){
		base[i] = base[i + 1];
	}
	base[num_leds - 1] = firstcopy;
}
void ws2812b_pulse(WS2812B_t *base, uint8_t num_leds){
	static bool direction = true;
	int i;
	//initialize all reds to 0.
	/*for(i = 0; i < num_leds; i++){
		base[i].red = 0x00;
	}*/
	if(direction == true && base[0].red < 0xFF){
		for(i = 0; i < num_leds; i++){
			base[i].red++;
		}
	}
	else if(direction == true && base[0].red == 0xFF){
		direction = false;
		for(i = 0; i < num_leds; i++){
			base[i].red--;
		}
	}
	if(direction == false && base[0].red > 0x00){
		for(i = 0; i < num_leds; i++){
			base[i].red--;
		}
	}
	if(direction == false && base[0].red == 0x00){
		direction = true;
		for(i = 0; i < num_leds; i++){
			base[i].red++;
		}
	}
}

