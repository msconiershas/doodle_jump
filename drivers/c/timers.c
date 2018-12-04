#include "timers.h"

volatile int8_t DUTY_CYCLE;
volatile bool ALERT_ADC_UPDATE;
//*****************************************************************************
// Verifies that the base address is a valid GPIO base address
//*****************************************************************************
static bool verify_base_addr(uint32_t base_addr)
{
   switch( base_addr )
   {
     case TIMER0_BASE:
     case TIMER1_BASE:
     case TIMER2_BASE:
     case TIMER3_BASE:
     case TIMER4_BASE:
     case TIMER5_BASE:
     {
       return true;
     }
     default:
     {
       return false;
     }
   }
}

//*****************************************************************************
// Returns the RCGC and PR masks for a given TIMER base address
//*****************************************************************************
static bool get_clock_masks(uint32_t base_addr, uint32_t *timer_rcgc_mask, uint32_t *timer_pr_mask)
{
  // Set the timer_rcgc_mask and timer_pr_mask using the appropriate
  // #defines in ../include/sysctrl.h
  switch(base_addr)
  {
    case TIMER0_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R0;
      *timer_pr_mask = SYSCTL_PRTIMER_R0;
      break;
    }
    case TIMER1_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R1;
      *timer_pr_mask = SYSCTL_PRTIMER_R1;
      break;
    }
    case TIMER2_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R2;
      *timer_pr_mask = SYSCTL_PRTIMER_R2;
      break;
    }
    case TIMER3_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R3;
      *timer_pr_mask = SYSCTL_PRTIMER_R3;
      break;
    }
    case TIMER4_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R4;
      *timer_pr_mask = SYSCTL_PRTIMER_R4;
      break;
    }
    case TIMER5_BASE:
    {
      *timer_rcgc_mask = SYSCTL_RCGCTIMER_R5;
      *timer_pr_mask = SYSCTL_PRTIMER_R5;
      break;
    }
    default:
    {
      return false;
    }
  }
  return true;
}


//*****************************************************************************
// Waits for 'ticks' number of clock cycles and then returns.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_wait(uint32_t base_addr, uint32_t ticks)
{
  TIMER0_Type *gp_timer;
  
  // Verify the base address.
  if ( ! verify_base_addr(base_addr) )
  {
    return false;
  }

  // Type cast the base address to a TIMER0_Type struct
  gp_timer = (TIMER0_Type *)base_addr;

  //*********************    
  // ADD CODE
  //*********************
  gp_timer->CTL &=  ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN);
	gp_timer->TAILR = ticks;
	gp_timer->ICR |= TIMER_ICR_TATOCINT;
	gp_timer->CTL |=  TIMER_CTL_TAEN;
	while((gp_timer->RIS |= TIMER_RIS_TATORIS)){};
	
  return true;
}


//*****************************************************************************
// Configure a general purpose timer to be a 32-bit timer.  
//
// Paramters
//  base_address          The base address of a general purpose timer
//
//  mode                  bit mask for Periodic, One-Shot, or Capture
//
//  count_up              When true, the timer counts up.  When false, it counts
//                        down
//
//  enable_interrupts     When set to true, the timer generates and interrupt
//                        when the timer expires.  When set to false, the timer
//                        does not generate interrupts.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_config_32(uint32_t base_addr, uint32_t mode, bool count_up, bool enable_interrupts)
{
  uint32_t timer_rcgc_mask;
  uint32_t timer_pr_mask;
  TIMER0_Type *gp_timer;
  
  // Verify the base address.
  if ( ! verify_base_addr(base_addr) )
  {
    return false;
  }
  
  // get the correct RCGC and PR masks for the base address
  get_clock_masks(base_addr, &timer_rcgc_mask, &timer_pr_mask);
  
  // Turn on the clock for the timer
  SYSCTL->RCGCTIMER |= timer_rcgc_mask;

  // Wait for the timer to turn on
  while( (SYSCTL->PRTIMER & timer_pr_mask) == 0) {};
  
  // Type cast the base address to a TIMER0_Type struct
  gp_timer = (TIMER0_Type *)base_addr;
    
  //*********************    
  // ADD CODE
  //*********************
   gp_timer->CTL &=  ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN);
	 gp_timer->CFG = TIMER_CFG_32_BIT_TIMER;
		gp_timer->TAMR &= ~TIMER_TAMR_TAMR_M;
		gp_timer->TAMR  |= mode;
		
		if(count_up)
			gp_timer->TAMR |= (1 << 4);
		
		
		if(enable_interrupts) {
		gp_timer->IMR |= TIMER_IMR_TATOIM;
			// <ADD CODE> Set the priority to 0.  Be sure to call uart_get_irq_num(uart_base) to   // get the correct IRQn_Type
		//IRQn_Type m = ;
	 NVIC_SetPriority(TIMER0A_IRQn, 1);
   // <ADD CODE> Enable the NVIC.  Be sure to call uart_get_irq_num(uart_base) to get
   // the correct IRQn_Type
	 NVIC_EnableIRQ(TIMER0A_IRQn);
		}
		else
			
    
  return true;  
}


//*****************************************************************************
// Set up prescale and interval load registers for given timer.  
//
// Paramters
//  gp_timer          The pointer to the timer
//
//*****************************************************************************
void gp_timer_start16(TIMER0_Type * gp_timer) {
		gp_timer->TAPR = 0x8;//8
	  gp_timer->TBPR = 0x10;//16
	  gp_timer->TAILR = 0xF424;//62500 -> (8*62500) = 500,000 = (50MHz)/(1/10-3ms)
	  gp_timer->TBILR = 0xF424;
}

//*****************************************************************************
// Configure a general purpose timer to be a 16-bit timer.  
//
// Paramters
//  base_address          The base address of a general purpose timer
//
//  mode                  bit mask for Periodic, One-Shot, or Capture
//
//  count_up              When true, the timer counts up.  When false, it counts
//                        down
//
//  enable_interrupts     When set to true, the timer generates and interrupt
//                        when the timer expires.  When set to false, the timer
//                        does not generate interrupts.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_config_16(uint32_t base_addr, uint32_t mode, bool count_up, bool enable_interrupts)
{
  uint32_t timer_rcgc_mask;
  uint32_t timer_pr_mask;
  TIMER0_Type *gp_timer;
  
  // Verify the base address.
  if ( ! verify_base_addr(base_addr) )
  {
    return false;
  }
  
  // get the correct RCGC and PR masks for the base address
  get_clock_masks(base_addr, &timer_rcgc_mask, &timer_pr_mask);
  
  // Turn on the clock for the timer
  SYSCTL->RCGCTIMER |= timer_rcgc_mask;

  // Wait for the timer to turn on
  while( (SYSCTL->PRTIMER & timer_pr_mask) == 0) {};
  
  // Type cast the base address to a TIMER0_Type struct
  gp_timer = (TIMER0_Type *) base_addr;
    
  //*********************    
  // ADD CODE
  //*********************
    gp_timer->CTL &=  ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN);
	  gp_timer->CFG = TIMER_CFG_16_BIT;//configure as 16 bit timer
		gp_timer->TAMR &= ~TIMER_TAMR_TAMR_M; //disable timerA
		gp_timer->TBMR &= ~TIMER_TBMR_TBMR_M; //disable timerB
		gp_timer->TAMR  |= mode;	//set mode bits
		gp_timer->TBMR |= mode;		//set mode bits
		
		//set up prescalar and ILR
		gp_timer_start16(gp_timer);
		
		//set direction of timer based on count_up parameter
		if(count_up) {
			gp_timer->TAMR |= TIMER_TAMR_TACDIR;
			gp_timer->TBMR |= TIMER_TBMR_TBCDIR;
		}
		else {
			gp_timer->TAMR &= ~TIMER_TAMR_TACDIR;
			gp_timer->TBMR &= ~TIMER_TBMR_TBCDIR;//Pin 4 in direction bit
		}
		//if interrupts should be enabled
		if(enable_interrupts) {
		  gp_timer->IMR |= TIMER_IMR_TBTOIM;//Interrupt mask register
			gp_timer->IMR |= TIMER_IMR_TATOIM;//Interrupt mask register
			
			NVIC_SetPriority(TIMER0A_IRQn, 1);	//set priority of interrupt
			NVIC_SetPriority(TIMER0B_IRQn , 2);	//set priority of interrupt
			
	    NVIC_EnableIRQ(TIMER0A_IRQn);			//enable IRQ
			NVIC_EnableIRQ(TIMER0B_IRQn);			//enable IRQ
		}
		
	//clear interrupts	
	TIMER0->ICR |= TIMER_ICR_TATOCINT;
	TIMER0->ICR |= TIMER_ICR_TBTOCINT;
	
	//reenable timers
	gp_timer->CTL |=  (TIMER_CTL_TAEN | TIMER_CTL_TBEN);
    
  return true;  
}



