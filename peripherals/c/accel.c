#include "wireless.h"
#include "spi_select.h"
#include "spi.h"

extern void spiTx(uint32_t base, uint8_t *tx_data, int size, uint8_t *rx_data);
extern bool spiVerifyBaseAddr(uint32_t base);

//*****************************************************************************
// Manually sets the SPI chip select line low
//*****************************************************************************
static __INLINE void  wireless_CSN_low(void)
{
	//#define   RF_CS_PIN          	PA3
  GPIOA->DATA &= ~RF_CS_PIN;
}

//*****************************************************************************
// Manually sets the SPI chip select line high
//*****************************************************************************
static __INLINE void  wireless_CSN_high(void)
{
  GPIOA->DATA |= RF_CS_PIN;
}

//*****************************************************************************
// Transmits 4 bytes of data to the remote device.
//*****************************************************************************
int8_t accel_read_reg(uint8_t reg) {
	//#define   RF_SPI_BASE        	SSI0_BASE
  uint8_t rx_data[2];
	uint8_t tx_data[2];
	tx_data[0] = reg;
	
	wireless_CSN_low();
	spiTx(RF_SPI_BASE, tx_data, 2, rx_data);
	wireless_CSN_high();
	return rx_data[1];
}


//*****************************************************************************
// Transmits 4 bytes of data to the remote device.
//*****************************************************************************
void accel_write_reg(uint8_t reg, uint8_t data) {
	//#define   RF_SPI_BASE        	SSI0_BASE
	uint8_t rx_data[2];
	uint8_t tx_data[2];
	tx_data[0] = reg;
	tx_data[1] = data;

	wireless_CSN_low();
	spiTx(RF_SPI_BASE, tx_data, 2, rx_data);
	wireless_CSN_high();
}


void accel_initialize(void)
{  
  //RF_GPIO_BASE == GPIOA_BASE
	//#define   RF_CLK_PIN         	PA2
	
  gpio_enable_port(RF_GPIO_BASE);
  
	//#define   RF_SPI_CLK_PCTL_M  	GPIO_PCTL_PA2_M
	//#define   RF_CLK_PIN_PCTL    	GPIO_PCTL_PA2_SSI0CLK
	
  // Configure SPI CLK
  gpio_config_digital_enable(RF_GPIO_BASE, RF_CLK_PIN);
  gpio_config_alternate_function(RF_GPIO_BASE, RF_CLK_PIN);
  gpio_config_port_control(RF_GPIO_BASE, RF_SPI_CLK_PCTL_M, RF_CLK_PIN_PCTL);
  
  // Configure SPI MISO
  gpio_config_digital_enable(RF_GPIO_BASE, RF_MISO_PIN);
  gpio_config_alternate_function(RF_GPIO_BASE, RF_MISO_PIN);
  gpio_config_port_control(RF_GPIO_BASE, RF_SPI_MISO_PCTL_M, RF_MISO_PIN_PCTL);
  
  // Configure SPI MOSI
  gpio_config_digital_enable(RF_GPIO_BASE, RF_MOSI_PIN);
  gpio_config_alternate_function(RF_GPIO_BASE, RF_MOSI_PIN);
  gpio_config_port_control(RF_GPIO_BASE, RF_SPI_MOSI_PCTL_M, RF_MOSI_PIN_PCTL);
  
	//#define   RF_CS_BASE        	GPIOA_BASE
	//#define   RF_CS_PIN          	PA3
	
  // Configure CS to be a normal GPIO pin that is controlled 
  // explicitly by software
  gpio_enable_port(RF_CS_BASE);
  gpio_config_digital_enable(RF_CS_BASE,RF_CS_PIN);
  gpio_config_enable_output(RF_CS_BASE,RF_CS_PIN);
  
	//#define   RF_CE_GPIO_BASE     GPIOD_BASE
	//#define   RF_CE_PIN           PD6
	
  // Configure CE Pin as an output  
  gpio_enable_port(RF_CE_GPIO_BASE);
  gpio_config_digital_enable(RF_CE_GPIO_BASE,RF_CE_PIN);
  gpio_config_enable_output(RF_CE_GPIO_BASE,RF_CE_PIN);
	
	spi_select_init();
  spi_select(MODULE_2);//The Module 2 SPI site must be selected for the proper SS to be asserted. 

//#define   RF_SPI_BASE        	SSI0_BASE

  initialize_spi( RF_SPI_BASE, 3, 10);//spi mode 3
  GPIOD->DATA |= RF_CE_PIN;//PD6
	
	//for interrupt
  gpio_enable_port(GPIOD_BASE);//GPIOD_BASE
  
	//////////////////////////////////////////////////////////
	accel_write_reg(0x0D, 0x01);//enable interrupt upon data ready
	accel_write_reg(0x10, 0x53);//setup accel for 208Hz data rate
	accel_write_reg(0x11, 0x52);//setup gyro for 416Hz data rate
	accel_write_reg(0x14,0x60);//turn rounding on 
	/////////////////////////////////////////////////////////
	
  // Configure interrupt
  gpio_config_digital_enable(GPIOD_BASE, PD3);
	gpio_config_enable_input(GPIOD_BASE,PD3);
	
	GPIOD->IM &= ~GPIO_IM_GPIO_M;//clear mask field
	
	GPIOD->ICR |= (1 << 3);//For a GPIO edge-detect interrupt, the RIS bit in the GPIORIS register
                            //is cleared by writing a ‘1’ to the corresponding bit in the GPIO Interrupt Clear (GPIOICR) register.
	GPIOD->IEV |= (1 << 3);//set to rising edge 
  GPIOD->IM |= (1 << 3);//interrupt mask enable reg
	
	NVIC_SetPriority(GPIOD_IRQn , 1);//set interrupt priority for GPIOD
	NVIC_EnableIRQ(GPIOD_IRQn);
	
/*	. Mask the corresponding port by clearing the IME field in the GPIOIM register.
2. Configure the IS field in the GPIOIS register and the IBE field in the GPIOIBE register.
3. Clear the GPIORIS register.
4. Unmask the port by setting the IME field in the GPIOIM register*/
	//#define   RF_IRQ_GPIO_BASE    GPIOD_BASE
  //#define   RF_IRQ_PIN          PD3
	
	// clear interrupts just to be sure
	accel_read_reg(0xA9);
	
}


	
