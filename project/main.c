// Copyright (c) 2015-16, Joe Krachey
// All rights reserved.
//
// Redistribution and use in source or binary form, with or without modification, 
// are permitted provided that the following conditions are met:
//
// 1. Redistributions in source form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in 
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "main.h"
#include "lcd.h"
#include "timers.h"
#include "ps2.h"
#include "launchpad_io.h"
#include "HW3_images.h"
#include "math.h"
#include "ft6x06.h"
#include <stdlib.h>
#include "eeprom.h"
#include "wireless.h"
#include "accel.h"
#include "spi.h"
#include "spi_select.h"


char group[] = "05";
char individual_1[] = "McKinley Sconiers-Hasan";
char individual_2[] = "Derek DeCramer";

///////////////////////////
// Global declared next //
///////////////////////////
volatile bool debounce_int = false;
volatile bool joystick_int = false;
volatile bool timerA_alert = false;		//used for updating missile position and debouncing SW1 for 40ms
volatile bool timerB_alert = false;		//used for updating player position and getting ADC values
uint16_t ps2_x;		//used to get 12bit value from SSFIFO
uint16_t ps2_y;		//used to get 12bit value from SSFIFO
volatile uint32_t numtenmsinterrupts;	//used as time variable in player movement equation
float initial_player_y_loc = (320 - 20) - player_HEIGHT/2;//set initial location in the center of the screen
float current_player_y_loc;
float current_player_x_loc;
struct platform* new_platform;
bool canfiremissile = false;
bool addMonster = false;



volatile bool accel_alert = true;//accelerometer interrupt check


//accelerometer measurements
volatile int8_t ax_low;
volatile int8_t ax_high;

uint16_t addr = 256;	//used as address for eeprom value

player_t player;			//player variable

struct missle * m_head = NULL;
struct missle * m_tail = NULL;
struct platform* p_head = NULL;
struct platform* monster = NULL;

typedef enum 
{
  DEBOUNCE_ONE,
  DEBOUNCE_1ST_ZERO,
  DEBOUNCE_2ND_ZERO,
  DEBOUNCE_PRESSED
} DEBOUNCE_STATES;

//generate platforms and 1 monster at random locations on the screen.
void Generate_Platforms(){
	int randnum;
	int randnum2;
	int randnum3;
	int counter = 0;
	int y_spot = 240;
	randnum2 = rand() % 4;	//get random y location for monster
	randnum3 = (rand() % 180) + 30;//get random x location for monster
	while(y_spot > 0){
		//get random x location
		randnum = (rand() % 220) + 8;
		//generate a monster
		if(addMonster == true){
			if(counter == randnum2){
				new_platform = malloc(sizeof(struct platform));
				new_platform->x_loc = randnum3;
				new_platform->y_loc = y_spot;
				new_platform->nxt = p_head;
				new_platform->isMonster = true;
				p_head = new_platform;
				lcd_draw_image(randnum3, MONSTER_WIDTH, y_spot, MONSTER_HEIGHT, monsterBitmap, LCD_COLOR_RED, LCD_COLOR_BLACK);
				monster = p_head;
				monster->death = false;
			}
		}
		//generate platform
		new_platform = malloc(sizeof(struct platform));
		new_platform->x_loc = randnum;
		new_platform->y_loc = y_spot;
		new_platform->nxt = p_head;
		new_platform->isMonster = false;
		p_head = new_platform;
		lcd_draw_image(new_platform->x_loc, PLATFORM_WIDTH, new_platform->y_loc, PLATFORM_HEIGHT, platformBitmap, LCD_COLOR_WHITE, LCD_COLOR_BLACK);
		y_spot = y_spot - 30;
		counter++;
	}
	addMonster = !addMonster;	//only generate a monster every other screen shift
}

void DisableInterrupts(void)
{
  __asm {
         CPSID  I
  }
}

//*****************************************************************************
//*****************************************************************************
void EnableInterrupts(void)
{
  __asm {
    CPSIE  I
  }
}


//*****************************************************************************
// GPIOD and accelerometer handler. Clears both interrupts and set variable
// values
//*****************************************************************************
void GPIOD_Handler(void) {
	//clear GPIOD
	//put_string("GPIOD_HANDLER\n\r");
	DisableInterrupts();
	ax_low = accel_read_reg(0xA8);
	accel_read_reg(0xAA);
	accel_read_reg(0xAB);
	ax_high = accel_read_reg(0xA9);
	EnableInterrupts();
	
	GPIOD->ICR |= (1 << 3);
	accel_alert = true;
	return;
}


//*****************************************************************************
// Clears TimerA interrupt 
//*****************************************************************************
void TIMER0A_Handler(void) {
	static int count = 0;
	DisableInterrupts();
	numtenmsinterrupts++;	//update time variable
	EnableInterrupts();
	timerA_alert = true;
	if(count == 0) {
		canfiremissile = true;	//allow player to fire a missile
	}
	else if(count == 20) {
	}
	count = (count + 1) % 40;
	//clear interrupt
	TIMER0->ICR |= TIMER_ICR_TATOCINT;
	return;
}



//*****************************************************************************
// Clears TimerB interrupt 
//*****************************************************************************
void TIMER0B_Handler(void) {
	static int count = 0;
	timerB_alert = true;
	if(count == 0) {
	}
	else if(count == 20) {
	}
	count = (count + 1) % 40;
	//clear interrupt
	TIMER0->ICR |= TIMER_ICR_TBTOCINT;
	return;
}
//*****************************************************************************
// Clears Sample Sequencer 2 interrupt 
//*****************************************************************************
void ADC0SS2_Handler(void){
	//get y value from SSFIFO2
	ps2_y = (ADC0->SSFIFO2);
	//get x value from SSFIFO2
	ps2_x = (ADC0->SSFIFO2);	
	//clear interrupt 
	ADC0->ISC = ADC_ISC_IN2;
	return;
}

//*****************************************************************************
//wait for sw1 pin to be low for 40ms before returning true
//*****************************************************************************
bool sw1_debounce_fsm(void)//6D
{
  static DEBOUNCE_STATES state = DEBOUNCE_ONE;
  bool pin_logic_level;
  
  pin_logic_level = lp_io_read_pin(SW1_BIT);
  
  switch (state)
  {
    case DEBOUNCE_ONE:
    {
      if(pin_logic_level)
      {
        state = DEBOUNCE_ONE;
      }
      else
      {
        state = DEBOUNCE_1ST_ZERO;
				while(timerA_alert == false){} //wait 10ms
				while(timerA_alert == false){} //wait 10ms
      }
      break;
    }
    case DEBOUNCE_1ST_ZERO:
    {
      if(pin_logic_level)
      {
        state = DEBOUNCE_ONE;
      }
      else
      {
        state = DEBOUNCE_2ND_ZERO;
				while(timerA_alert == false){} //wait 10ms
				while(timerA_alert == false){} //wait 10ms
      }
      break;
    }
    case DEBOUNCE_2ND_ZERO:
    {
      if(pin_logic_level)
      {
        state = DEBOUNCE_ONE;
      }
      else
      {
        state = DEBOUNCE_PRESSED;
      }
      break;
    }
    case DEBOUNCE_PRESSED:
    {
      if(pin_logic_level)
      {
        state = DEBOUNCE_ONE;
      }
      else
      {
        state = DEBOUNCE_PRESSED;
      }
      break;
    }
    default:
    {
      while(1){};
    }
  }
  
  if(state == DEBOUNCE_2ND_ZERO )
  {
    return true;
  }
  else
  {
    return false;
  }
}

//*******************************************************************************
// Initalize all the GPIO pins, timers, ADCs, and other perifierals for the game
//*******************************************************************************
void initialize_hardware(void)
{
	
	initialize_serial_debug();
	
	//// setup lcd GPIO, config the screen, and clear it ////
	gpio_enable_port(PS2_GPIO_BASE);
	gpio_enable_port(GPIOF_BASE);
	gpio_config_enable_input(GPIOF_BASE,4);
	gpio_config_enable_output(GPIOF_BASE, 1);
	gpio_config_enable_output(GPIOF_BASE, 3);
	
	lcd_config_screen();
	lcd_clear_screen(LCD_COLOR_BLACK);

	//// setup the timers ////
	gp_timer_config_16(TIMER0_BASE, TIMER_TAMR_TAMR_PERIOD, false, true);
	
	//// setup GPIO for LED drive ////
  lp_io_init();
	
	//// Setup ADC to convert on PS2 joystick using SS2 and interrupts ////
	initialize_adc_new(ADC0_BASE);
	ps2_initialize();
	
	
	//init eeprom and touchscreen and acceleromter
	ft6x06_init();
	eeprom_init();
	accel_initialize();
}


//*****************************************************************************
//*****************************************************************************
int main(void)
{
	//////////////////////INITALIZE VALUES/////////////////////////////////////
	
	struct missle* m_curr;				//used to iterate through missile linked list
	struct platform* p_curr;			//used to iterate through platform linked list
	int i;
	float accel;		//acceleration of player
	float veloc;		//velocity of player
	float accelderiv;		//used to find when y movement of player is negative
	float velocderiv;		//used to find when y movement of player is negative
	struct platform* p_prev = NULL;		//used to iterate through platform linked list
	struct platform* p_temp = NULL;		//used to iterate through platform linked list
	struct missle* m_prev = NULL;		//used to iterate through missile linked list
	struct missle* m_temp = NULL;		//used to iterate through missile linked list
	uint16_t player_y_loc;
	uint16_t x,y;			//used to read touch screen values
  uint8_t touch_event;		//used to recognize a touch of the screen
	bool breakagain = false; 
	bool breaktwice = false;
	bool playerdeath = false;
	
	numtenmsinterrupts = 0;
	current_player_y_loc = initial_player_y_loc;
  initialize_hardware();
	eeprom_byte_write(I2C1_BASE, addr, 0);	//store current score in eeprom
	
	//////////////////////END INITALIZE VALUES/////////////////////////////////////
	
	while(1){
		
		//////////////////////////////////INITALIZE GAME AND SCREEN///////////////////////////////////////////////////////////////
		
		breakagain = false;		//used to check if player selects play again
		breaktwice = false;		//used to check if player selects exit
		playerdeath = false;		//used to check if player has died from a monster
		numtenmsinterrupts = 0;
		eeprom_byte_write(I2C1_BASE, addr, 0);	//store current score in eeprom
			
		//// Initialize player location and image ////
		player.x_loc = COLS/2;
		player.y_loc = initial_player_y_loc;
		current_player_y_loc = initial_player_y_loc;
		lcd_draw_image(player.x_loc,player_WIDTH, initial_player_y_loc,player_HEIGHT, playerBitmap, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
		current_player_x_loc = player.x_loc;
		
		//initialize initial ground platforms
		for(i = 0; i < 12; i++){
			new_platform = malloc(sizeof(struct platform));
			new_platform->x_loc = i*20;
			new_platform->y_loc = 300;//20 pixels above bottom of screen
			new_platform->nxt = p_head;
			new_platform->isMonster = false;
			p_head = new_platform;
			lcd_draw_image(new_platform->x_loc, PLATFORM_WIDTH, new_platform->y_loc, PLATFORM_HEIGHT, platformBitmap, LCD_COLOR_WHITE, LCD_COLOR_BLACK);
		}
		
		//generate the initial screen
		Generate_Platforms();
		
		//first screen will not have a monster
		addMonster = false;
		
		//////////////////////////////////END INITALIZE GAME AND SCREEN///////////////////////////////////////////////////////////////
			
		//IN GAME WHILE LOOP
		while(1){
			breakagain = false;
			
			/////////////////////////////////FINAL EXIT SCREEN///////////////////////////////////////
			
			//if player has lost go to end-game display screen.
			//Display final score on screen. Ask if they'd like to play again.
			//If not, exit infinite loop.
			if(player.y_loc > 340 || playerdeath == true){//player has fallen off bottom of the screen

				uint8_t current_score;
				int arrow_x = 20;//location of arrow to exit
				int arrow_y = 83;//location of play again
				char str[10];
				
				//read players score from eeprom and put into a string variable
				eeprom_byte_read(I2C1_BASE, addr, &current_score);
				sprintf(str, "%d", current_score);

				//print to LCD screen
				lcd_clear_screen(LCD_COLOR_BLACK);
				lcd_print_stringXY("YOUR SCORE:",2,3,LCD_COLOR_WHITE,LCD_COLOR_BLACK);
				lcd_print_stringXY(str,14,3,LCD_COLOR_WHITE,LCD_COLOR_BLACK);
				lcd_print_stringXY("PLAY AGAIN?",2,5,LCD_COLOR_WHITE,LCD_COLOR_BLACK);
				lcd_print_stringXY("EXIT",2,7,LCD_COLOR_WHITE,LCD_COLOR_BLACK);
				lcd_draw_image(arrow_x, ARROW_WIDTH, arrow_y, ARROW_HEIGHT, arrowBitmap, LCD_COLOR_BLUE2, LCD_COLOR_BLACK);
				
				//EXIT SCREEN WHILE LOOP
				while(1){
					
					
					//use interrupts to get ps2 values
					if(timerB_alert == true){
						
						//start conversion for ADC converter
						kick_off(ADC0_BASE);
						
						//play again
						if(ps2_y > 0xFFF*.75){
							lcd_draw_image(arrow_x, ARROW_WIDTH, arrow_y, ARROW_HEIGHT, arrowBitmap, LCD_COLOR_BLACK, LCD_COLOR_BLACK);
							arrow_y = 83; //play again option select
							lcd_draw_image(arrow_x, ARROW_WIDTH, arrow_y, ARROW_HEIGHT, arrowBitmap, LCD_COLOR_BLUE2, LCD_COLOR_BLACK);
						}
						//exit
						else if(ps2_y < .25*0xFFF){
							lcd_draw_image(arrow_x, ARROW_WIDTH, arrow_y, ARROW_HEIGHT, arrowBitmap, LCD_COLOR_BLACK, LCD_COLOR_BLACK);
							arrow_y = 120; //exit option select
							lcd_draw_image(arrow_x, ARROW_WIDTH, arrow_y, ARROW_HEIGHT, arrowBitmap, LCD_COLOR_BLUE2, LCD_COLOR_BLACK);
						}
						timerB_alert = false;
					}
					
					//if button is pressed
					if(sw1_debounce_fsm()){
						
						//if user want to play again
						if(arrow_y == 83){
							breakagain = true;
							break;
						}
						
					//if player want to exit
					 else{
						 lcd_clear_screen(LCD_COLOR_BLACK);
						 breakagain = true;
						 breaktwice = true;
						 break;
					 }
					}
				}
				
				//option on end-game screen was selected
				if(breakagain){
					//remove all platforms and missiles and clear screen
					struct platform* p_tmp;
					struct missle* m_tmp;
					while(p_head != NULL){
						p_tmp = p_head->nxt;
						free(p_head);
						p_head = p_tmp;
					}
					while(m_head != NULL){
						m_tmp = m_head->nxt;
						free(m_head);
						m_head = m_tmp;
					}
					monster = NULL;
					lcd_clear_screen(LCD_COLOR_BLACK);
					break;
				}
				
			}
			///////////////////////////////////END FINAL SCREEN SECTION///////////////////////////////////
			
		 ///////////////////////////MOVE PLAYER BASED ON ACCELEROMETER VALUES////////////////////////
				if(accel_alert) {
						//if the board is shifted more than 10 pixels to the left
					//move the player location one pixel left
						if(ax_high > 10){
							player.x_loc = player.x_loc - 1;
						}
						//if the board is shifted more than 10 pixels to the right
						//move the player location one pixel right
						else if(ax_high < -10){
							player.x_loc = player.x_loc + 1;
						}		
						accel_alert = false;
					}
			///////////////////////////END MOVE PLAYER BASED ON ACCELEROMETER VALUES//////////////////////
			
			///////////////////////////////////////BUTTON FIRE MISSLES//////////////////////////////////////////
			if(sw1_debounce_fsm()) {
				
				//create missile, init position, update linked list
				struct missle* new_missile = malloc(sizeof(struct missle));
				new_missile->x_loc = player.x_loc;
				new_missile->y_loc = player.y_loc - player_HEIGHT/2;
				new_missile->isfromtap = false;
				new_missile->nxt = m_head;
				m_head = new_missile;
				canfiremissile = false;		//wait a short amount of time before missile can be refired.
			}
			///////////////////////////////////////END BUTTON FIRE MISSLES//////////////////////////////////////////
			
			///////////////////////////////////////TOUCHSCREEN FIRE MISSLES//////////////////////////////////////////
			
			//check to see if screen was touched
			touch_event = ft6x06_read_td_status();
			
			if(touch_event > 0 && canfiremissile){
				struct missle* new_missile = malloc(sizeof(struct missle));
				
				//read where screen was touched
				x = ft6x06_read_x();
				y = ft6x06_read_y();				
				
				//create missile, init position, update linked list
				new_missile->x_loc = player.x_loc;
				new_missile->y_loc = player.y_loc - player_HEIGHT/2;
				new_missile->isfromtap = true;
				new_missile->target_x_vector = x - player.x_loc;
				new_missile->target_y_vector = y - player.y_loc;
				new_missile->nxt = m_head;
				m_head = new_missile;
				canfiremissile = false;		//wait a short amount of time before missile can be refired.
			}
			
			///////////////////////////////////////END TOUCHSCREEN FIRE MISSLES//////////////////////////////////////////
			
      ///////////////////////////////////////UPDATE PLAYER LOCATION WITH TIMERA////////////////////////////////////			
			
			//if 10ms has passed
			if(timerA_alert){
				timerA_alert = false;
				//set up variables for iteration of linked list
				player_y_loc = player.y_loc;
				p_prev = NULL;
				p_temp = NULL;
				m_prev = NULL;
				m_temp = NULL;
				m_curr = m_head;
				p_curr = p_head;
				
			///////////////////////////////////////END UPDATE PLAYER LOCATION WITH TIMERA////////////////////////////////////
				
				
			/////////////////////////////////////UPDATE MISSILES////////////////////////////////////////////////////////////////////
				
				//iterate through linked list and update missile positions.
				//delete missiles that fly off the screen.
				while(m_curr != NULL){
					
					//clear missile
					lcd_draw_image(m_curr->x_loc, MISSLE_WIDTH, m_curr->y_loc, MISSLE_HEIGHT, missleBitmap, LCD_COLOR_BLACK, LCD_COLOR_BLACK);
					
					//if button was used to fire missile have it go straight up
					if(m_curr->isfromtap == false){
						m_curr->y_loc = m_curr->y_loc - 5;
					}
					
					//if touch screen was used to fire missile have it go in direction of touch
					else{
						float mag = sqrt(pow(m_curr->target_x_vector, 2) + pow(m_curr->target_y_vector, 2));
						m_curr->x_loc = m_curr->x_loc + 5*(m_curr->target_x_vector / mag);
						m_curr->y_loc = m_curr->y_loc + 5*(m_curr->target_y_vector / mag);
					}
					
					//redraw missile at updated location
					lcd_draw_image(m_curr->x_loc, MISSLE_WIDTH, m_curr->y_loc, MISSLE_HEIGHT, missleBitmap, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
					
					//if a monster is on the screen check if any missiles hit it.
					//set variable to remove monster to true
					if(monster != NULL){
						if(m_curr->y_loc < monster->y_loc + 10 && m_curr->y_loc > monster->y_loc - 10 && m_curr->x_loc < monster->x_loc + 10 && m_curr->x_loc > monster->x_loc - 10){
							monster->death = true;
						}
					}
					
					//if missile is off screen
					if(m_curr->y_loc < -20 || m_curr->y_loc > 340 || m_curr->x_loc < -10 || m_curr->x_loc > 250){
						
						//remove missile from linked list and reset head
						if(m_prev == NULL){
							m_temp = m_head;
							m_head = m_head->nxt;
							m_curr = m_head;
							free(m_temp);	//free missile memory
							continue;
						}
						
						//remove missile from linked list
						else{
							m_temp = m_prev->nxt;
							m_prev->nxt = m_curr->nxt;
							m_curr = m_prev->nxt;
							free(m_temp); //free missile memory
							continue;
						}
					}
					
					//keep track of previous missile to help with removing from linked list
					m_prev = m_curr;
					m_curr = m_curr->nxt;
				}
				
				/////////////////////////////////////END UPDATE MISSILES/////////////////////////////////////////////////////////
				
				
				////////////////////////////////////CALCULATE PLAYER POSITION////////////////////////////////////////////////////
				
				//calculate y position
				accel = 300.0*(pow((((double)numtenmsinterrupts)*.01),2));
				accelderiv = 300.0*2*.0001*numtenmsinterrupts;						//used later in code to see if player is falling rather than rising
				veloc = 300.0*((double)numtenmsinterrupts)*.01;
				velocderiv = 300.0*.01;																		//used later in code to see if player is falling rather than rising
				player.y_loc = accel - veloc + (double)current_player_y_loc;
				
				
				//wrap screen for player to move off the screen and back on
				if(player.x_loc < 0){
					player.x_loc = 239;
				}
				else if(player.x_loc > 240){
					player.x_loc = 0;
				}
				
				////////////////////////////////////END CALCULATE PLAYER POSITION////////////////////////////////////////////////////
				
				
				
				/////////////////////////REDRAW PLATFORMS AND SEE IF PLAYER SHOULD JUMP/////////////////////////////////////////////
				while(p_curr != NULL){
					//if platform is the monster
					if(p_curr->isMonster){
						lcd_draw_image(p_curr->x_loc, MONSTER_WIDTH, p_curr->y_loc, MONSTER_HEIGHT, monsterBitmap, LCD_COLOR_RED, LCD_COLOR_BLACK);
						
						
					//check if player has hit the monster and should die
				 if(player.x_loc + player_WIDTH > p_curr->x_loc && player.x_loc < p_curr->x_loc + PLATFORM_WIDTH && player.y_loc > p_curr->y_loc + PLATFORM_HEIGHT*.8 && player.y_loc < p_curr->y_loc + PLATFORM_HEIGHT){
							playerdeath = true;
						}						
					}
					//if platform is not a monster
					else{
						lcd_draw_image(p_curr->x_loc, PLATFORM_WIDTH, p_curr->y_loc, PLATFORM_HEIGHT, platformBitmap, LCD_COLOR_WHITE, LCD_COLOR_BLACK);
					}
					
					//if player is moving downward, check to see if player should jump
					if(accelderiv > velocderiv){
						
						//if player has landed on platform
						if(player.y_loc + player_HEIGHT/2 >= p_curr->y_loc - 6 && player.y_loc + player_HEIGHT/2 <= p_curr->y_loc + 6 && player.x_loc >= p_curr->x_loc - 20 && player.x_loc <= p_curr->x_loc + 20){
							DisableInterrupts();
							numtenmsinterrupts = 0;		//reset time variable to 0
							EnableInterrupts();
							current_player_y_loc = p_curr->y_loc - player_HEIGHT/2;
							break;
						}
					}
					
					//if platform is the monster
					if(p_curr->isMonster){
						//if monster should die
						if(p_curr->death == true){
							
							//update score value in eeprom
							uint8_t current_score;
							eeprom_byte_read(I2C1_BASE, addr, &current_score);
							current_score++;
							eeprom_byte_write(I2C1_BASE, addr, current_score);
							
							//remove platform(monster) from linked list and reset head
							if(p_prev == NULL){
								p_temp = p_head;
								p_head = p_head->nxt;
								p_curr = p_head;
								free(p_temp);	//free platform memory
								lcd_draw_image(monster->x_loc, MONSTER_WIDTH, monster->y_loc, MONSTER_HEIGHT, monsterBitmap, LCD_COLOR_BLACK, LCD_COLOR_BLACK);
								monster = NULL;
								continue;
							}
							
							//remove platform(monster) from linked list
							else{
								lcd_draw_image(monster->x_loc, MONSTER_WIDTH, monster->y_loc, MONSTER_HEIGHT, monsterBitmap, LCD_COLOR_BLACK, LCD_COLOR_BLACK);
								monster = NULL;
								p_temp = p_prev->nxt;
								p_prev->nxt = p_curr->nxt;
								p_curr = p_prev->nxt;
								free(p_temp); //free platform memory
								continue;
							}
						}
					}
					
					//update linked list
					p_prev = p_curr;
					p_curr = p_curr->nxt;
				}
				/////////////////////////END REDRAW PLATFORMS AND SEE IF PLAYER SHOULD JUMP/////////////////////////////////////////////
				
				
				
				/////////////////////////////SHIFT SCREEN WHEN PLAYER GOES UPWARDS//////////////////////////////////////////////////////
				
				//if player has moved up near the top of the screen
				if(player.y_loc < 40){
					uint8_t current_score;
					player.y_loc = player.y_loc + 200;		//shift player downward
					current_player_y_loc = player.y_loc;
					p_curr = p_head;
					p_prev = NULL;
					
					//iterate through all the platforms in the linked list
					while(p_curr != NULL){
						
						//if platform will be off screen on screen shift
						if(p_curr->y_loc > 120){
							
							//if previous is null, remove platform from linked list and reset head
							if(p_prev == NULL){
								p_temp = p_head;
								p_head = p_head->nxt;
								p_curr = p_head;
								free(p_temp);	//free platform memory
								continue;
							}
							//remove platform from linked list
							else{
								if(p_curr->isMonster) monster = NULL;
								p_temp = p_prev->nxt;
								p_prev->nxt = p_curr->nxt;
								p_curr = p_prev->nxt;
								free(p_temp); //free platform memory
								continue;
							}
						}
						
						//shift platforms that should still be on screen
						else{
							p_curr->y_loc = p_curr->y_loc + 200;
						}
						
						//keep track of previous platform to help with removing from linked list
						p_prev = p_curr;
						p_curr = p_curr->nxt;
					}
					
					//clear screen
					lcd_clear_screen(LCD_COLOR_BLACK);
					
					//generate new platforms
					Generate_Platforms();
					
					//update score value in eeprom
					eeprom_byte_read(I2C1_BASE, addr, &current_score);
					current_score++;
					eeprom_byte_write(I2C1_BASE, addr, current_score);
				}
				///////////////////////////////END SHIFT SCREEN WHEN PLAYER GOES UPWARDS///////////////////////////////////////////////////
				
				
				
				////////////////////////////////REDRAW PLAYER WHEN WE SHIFT SCREEN //////////////////////////////////////////////////////////
				
				//delete initial player drawing
				lcd_draw_image(current_player_x_loc,player_WIDTH, player_y_loc,player_HEIGHT, playerBitmap, LCD_COLOR_BLACK, LCD_COLOR_BLACK);
				//redraw player at updated position
				lcd_draw_image(player.x_loc,player_WIDTH, player.y_loc,player_HEIGHT, playerBitmap, LCD_COLOR_YELLOW, LCD_COLOR_BLACK);
				current_player_x_loc = player.x_loc;
				
				////////////////////////////////END REDRAW PLAYER WHEN WE SHIFT SCREEN ////////////////////////////////////////////////////////
				
				
				//clear timer alert variable
				timerA_alert = false;
			}
			
		}		// end of while(1) loop
		//if exit was selcected, break to end of main
		if(breaktwice == true) break;
	} // end of while(1) loop
} //end of main


