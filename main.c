/**
  ******************************************************************************
  * @file    ADC3_DMA/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#define UARTBUFFER 100
uint32_t DMA_result;
//char result1[100];
//char result2[100];
char result[UARTBUFFER];
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void TOGGLE_ORANGE_LED(void)  // i am assuming it's only one LED to be toggled and it's red one. 
{
	
GPIO_InitTypeDef  GPIO_InitStructure;
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	

 /* PD12 to be toggled */
    GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
    
    /* Insert delay */
    Delay(0x3FF);
	
}	

void TOGGLE_RED_LED(void)  // i am assuming it's only one LED to be toggled and it's red one. 
{
	
GPIO_InitTypeDef  GPIO_InitStructure;
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);


 /* PD12 to be toggled */
    GPIO_ToggleBits(GPIOD, GPIO_Pin_14);
    
    /* Insert delay */
    Delay(0x3FF);
	
}	

void TOGGLE_BLUE_LED(void)  // i am assuming it's only one LED to be toggled and it's red one. 
{
	
GPIO_InitTypeDef  GPIO_InitStructure;
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);


 /* PD12 to be toggled */
    GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
    
    /* Insert delay */
    Delay(0x3FF);
	
}	


void TOGGLE_LED(void)
{
	
GPIO_InitTypeDef  GPIO_InitStructure;
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);


 /* PD12 to be toggled */
    GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    
    /* Insert delay */
    Delay(0x3FF);
	
	 GPIO_ToggleBits(GPIOD, GPIO_Pin_14);


	Delay(0x3FF);
	
	 GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
	 
	 Delay(0x3FF);
	 
	  GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
	 
    
	
}	


void TOGGLE_GREEN_LED(void)  // i am assuming it's only one LED to be toggled and it's red one. 
{
	
GPIO_InitTypeDef  GPIO_InitStructure;
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);


 /* PD12 to be toggled */
    GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    
    /* Insert delay */
    Delay(0x3FF);
	
}	



void DMA_TX(char * tx_string)
{
		
	// IF STREAM 6 TRANSFER IS COMPLETE

  
  
  

	while(!((USART2->SR) & (USART_SR_TC)));
  strcpy(result, tx_string);
  USART2->CR3|=(USART_CR3_DMAT|USART_CR3_DMAR);
  DMA1_Stream6->CR|=(1u<<0); 
  while (!((DMA1->HISR) & (1U<<21)));
  if((DMA1->HISR & (1U<<21)))

	{
  //TOGGLE_BLUE_LED();
  //Delay(0xffff);
  USART2->CR3&=~USART_CR3_DMAT;
  USART2->CR3&=~USART_CR3_DMAR;  
  DMA1_Stream6->CR&=~(1u<<0);  
  DMA1->HIFCR|=((1U<<21)|(1U<<20)|(1U<<16));  
  DMA1_Stream6->NDTR=UARTBUFFER;    
  memset(result,'\0',UARTBUFFER);
  //TOGGLE_BLUE_LED();
  //Delay(0xffff); 
  
  }




}


void DMA1_Stream6_IRQHandler(void) // DMA _UART
{
if(DMA1->HISR & (1U<<21)) // IF STREAM 6 TRANSFER IS COMPLETE
{
//TOGGLE_LED();
DMA1_Stream6->CR&=~(1u<<0);	
}	
}	
void DMA2_Stream0_IRQHandler(void) // DMA_ADC
{
  char value;
 char printed_result[3];
if(DMA2->LISR & 0x20 )
{
  DMA2->LIFCR |=DMA_LIFCR_CTCIF0|DMA_LIFCR_CHTIF0|DMA_LISR_FEIF1;
  value= (char) DMA_result;

 sprintf(printed_result,"%d",value);

/* turn the ADC and the DMA off to read data and print it on UART*/  
  ADC1->SR|=ADC_SR_STRT;
  DMA2_Stream0->CR&=~(1u<<0);
  TOGGLE_GREEN_LED();
  Delay(0xffff);
  DMA_TX("accessing ADC VALUE : ");
  DMA_TX(printed_result);
  DMA_TX("\n");
 
  
 
  
  /* restart the whole DMA AGAIN */ 
  
  DMA2_Stream0->NDTR=1;
  
  DMA2_Stream0->CR|=(1u<<0);
  TOGGLE_GREEN_LED();
  
}
}	



void TIM3_IRQHandler(void)
{
  if (TIM3->SR & TIM_SR_CC1IF)
  {	
  TIM3->SR &=~(TIM_SR_CC1IF);	

    
 DMA_TX("Acessing Timer 3 ISR ");
 DMA_TX("at T= "); 
 DMA_TX(__TIME__);
  DMA_TX("\n");  
  
   
  //TOGGLE_LED();	


  }


}

void Timer_Init()
{
	
  //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;                // Enable GPIOA clock
  //GPIOA->MODER |= (2u<<12);
	//GPIOA->MODER |= (1u<<12);
	
	// Enable AF mode for A6
 // GPIOA->AFR[0] |=0x02000000;                           
/* Timer 3 is initialized @ channel 1 */
RCC->APB1ENR|=RCC_APB1ENR_TIM3EN; //enabling timer 3 clock 
/* according to the snapshot taken, timer 3 is on apb1 which is 42MHz*/
	
TIM3->DIER|=TIM_DIER_CC1IE; // enabling interrupt on timer 3  channel 1
/* watch TIM3->SR->CC1F */
TIM3->CCMR1|=(3u<<4); //counter compare register sets the OC1 pin active mode (toggle, freeze, off)	{chosen toggle mode}
TIM3->CCMR1&=~(1U<<0); // set as output
TIM3->CCER|=(1u<<0); // oc1 active on compare as an output pin	
TIM3->CCER &=~TIM_CCER_CC1P;	
TIM3->ARR=10000;	
TIM3->PSC=41999;
/* 42MHz/42Khz = 1KHz
		therefore, 1 pulse = 0.5 ms.
		thus, after 1 sec=1000 @ Counter Compare 1 (i.e ccr1);
*/

TIM3->CCR1=10000; 	

TIM3->CR1|=(1u<<0);	//enable timer
}


void SPI_MASTER_INIT()
{
 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //enable gpio b clock
 GPIOB->MODER|=(1u<<18);
 GPIOB->MODER|=(2U<<20)|(2U<<28)|(2U<<30); // @ pins 10,14,15 pin 9 will be manually triggered
 GPIOB->AFR[1] |=0x55000500; // AF 5 for spi
 RCC->APB1ENR|=RCC_APB1ENR_SPI2EN; // enbaling SPI 2 clock
 SPI2->CR1|= /* CPOL, CPHA @ BITS 0,1*/ (1U<<2) /*MASTER, FOR SLAVE TURN IT ZERO*/ | (1U<<3)   /* CLK/4 */   /* MSB FIRST @ BIT 7 */  /*do not try SSM for now */ ;
 SPI2->CR2|= (1U<<2) | (1U<<7)| (1U<<6); /* ss output enable, RX NOT EMPTY, TX EMPTY */ 
  
 SPI2->CR1 |= (1U<<6); // enable SPI  
 GPIOB->ODR|=(1u<<9); 
} 

char SPI_TRX_DATA ( uint8_t data)
{
int counter=0; 
 uint8_t data_rx;
// data_rx=SPI2->DR; // flush old data away  
while(!(SPI2->SR & SPI_SR_TXE)); 
GPIOB->ODR&=~(1u<<9);
SPI2->DR=data;
//while((SPI2->SR & SPI_SR_BSY)); // trying to avoid busy flag as it is confusing somehow 
//while(!(SPI2->SR & SPI_SR_RXNE)); 
 
while(((SPI2->SR & SPI_SR_RXNE)==0) && (counter<50))
 {
  counter++; 
 }
GPIOB->ODR|=(1u<<9); 
data_rx=SPI2->DR; 
return data_rx;  
}

//char SPI_READ_DATA()
//{
// int counter=0; 
// uint8_t data_rx; 
// while(!(SPI2->SR & SPI_SR_RXNE) && counter<50)
// {
//  counter++; 
// }  
//data_rx= SPI2->DR;
//return data_rx;  
//} 

void ADC_INIT(void)
{
  
RCC->APB2ENR|=RCC_APB2ENR_ADC1EN;
RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;
GPIOA->MODER|=(3U<<12);	
ADC1->CR1|= (6U<<0) /*| (1U<<8) */ | (2U<<24) ; // 	channel 6, EOCIE, 8 bit resoln
ADC1->CR2|=(1U<<0)/* TURN ADC ON*/| (1U<<9)  | (1U<<8) /* DMA MODE*/ | (3U<<28)/* edge trigger */ | (7U<<24) /* regular channel*/| (1u<<10) /*EOCS*/ ; //turn on adc, continous mode @ bit 1, dma @ bits: 8,9 p. 419 , right alignment @ bit 11, bits 27 for timers 	
ADC1->SMPR2|=(2U<<18); // 28 cycles in order to give it enough time for conversion
//ADC1->HTR|=0X100;
ADC1->LTR|=0;
ADC1->SQR3|=(6U<<0);
//ADC1->CR2|=ADC_CR2_SWSTART;// START CONVERSION	

}	

uint32_t READ_ADC1(void)
{
uint32_t x,y;	
x= DMA_result;
y=  x*3.3/255;

return y;	
}	

void mco_init(void)
{
RCC->CFGR|=(2U<<21); // HSE selected @ MCO1, sysclk selected @ MCO2 
RCC->CFGR|=(6U<<24); // prescale mco1 4
RCC->CFGR|=(7U<<27); // prescale mco2 5
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;                // Enable GPIOA clock
  GPIOA->MODER |= (2u<<16);
	
 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;                // Enable GPIOc clock
 GPIOC->MODER |= (2u<<18);	
	//GPIOA->MODER |= (1u<<12);
	
	// Enable AF mode for A6
  GPIOA->AFR[1] =0x00000000;   
	GPIOC->AFR[1] =0x00000000;
	
}	

void UART2INIT(void)
{

RCC->APB1ENR|=RCC_APB1ENR_USART2EN;
RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN;
GPIOA->MODER|=((2U<<4)|(2U<<6));	
GPIOA->AFR[0]=0x7777;
USART2->CR1|=(1U << 13) | (1U<<5);
// USART2->CR1|=USART_CR1_M; //IF YOU NEED TO TURN IT ON TO SELECT ANOTHER WORD LENGTH
USART2->CR2 &=~ (1U << 13);
// CR2 IS MAINLY USED IN CASE IF YOU WANTED TO USE UART AS UNIVERSAL SYNCHRONOUS, THUS IT IS NOT WEIRD YOU CAN FIND POLARITY AND PHASE SETTINGS BY WHICH YOU CAN SET IT UP
 /*
   The baud rate is computed using the following formula:
    - IntegerDivider = ((PCLKx) / (8 * (OVR8+1) * (USART_InitStruct->USART_BaudRate)))
    - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 8 * (OVR8+1)) + 0.5 
   Where OVR8 is the "oversampling by 8 mode" configuration bit in the CR1 register. */



USART2->BRR=((42*1000000)/19200)+0.5;
USART2->CR1|=USART_CR1_TCIE|USART_CR1_RXNEIE;	//enabling exceptions mask
USART2->CR3|=USART_CR3_DMAT|USART_CR3_DMAR;

USART2->CR1|=(1U<<2)|(1U<<3);	

while(!(USART2->SR) & (USART_SR_TC));
}



uint8_t  USART_TX(uint8_t DATA_TX)
{
uint8_t ACK=0;	

// turning on both send and receive
while(!((USART2->SR) & (USART_SR_TC)));
USART2->DR=DATA_TX;
while(!((USART2->SR) & (USART_SR_TC)));
ACK=1;	
return ACK;
}
void DMA_UART_INIT(void)
{

  RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;
  DMA1_Stream6->CR=0;
  while(DMA1_Stream6->CR);
  DMA1->LISR=0;
  DMA1->HISR=0;
  DMA1_Stream6->M0AR= (uint32_t) &result;
  DMA1_Stream6->PAR=0x40004404;
  DMA1_Stream6->NDTR=UARTBUFFER; //x items
  DMA1_Stream6->CR&=~((1U<<5)|(1U<<9)|(1U<<11)|(1U<<13)|(1U<<15)|(1u<<21)  );  
  DMA1_Stream6->CR|=(1u<<4)| (1u<<6)|(1u<<10)|(2u<<16)|(4u<<25)/*|(3u<<23)*/|(1U<<1);//| (1u<<8);

  //USART2->SR&=~(1U<<6);	
  //DMA1_Stream6->CR|=(1u<<0);	

	
}	



void DMA_ADC_INIT(void)
{
//uint32_t * PTR_RESULT= & DMA_result; 	
RCC->AHB1ENR|=RCC_AHB1ENR_DMA2EN;
DMA2_Stream0->CR=0;
while(DMA1_Stream0->CR);
DMA2->LISR=0;
DMA2->HISR=0;
DMA2_Stream0->M0AR= (uint32_t) &(DMA_result); //destination
DMA2_Stream0->PAR=0x4001204C; // source
DMA2_Stream0->NDTR=1; //1 item
DMA2_Stream0->CR|=(1u<<4)|(2u<<16)|(2U<<13)|(2u<<11); // check bit 15
//DMA2_Stream0->CR&=~((1U<<5)|(1U<<9)|(1U<<15)|(1u<<23)|(1u<<21));
//USART2->SR&=~(1U<<6);	
DMA2_Stream0->CR|=(1u<<0);	

	
}	

void I2C_INIT(void)
{
  /* initializing the pins*/
  
  RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
  
  GPIOB->AFR[1]=0x00000044;
  GPIOB->MODER |= ((2u<<16)|(2u<<18));
  GPIOB->PUPDR &= ~(3U << 16);	//clear bits 16 & 17 (PB6)
  GPIOB->PUPDR &= ~(3U<< 18);	//clear bits 18 & 19 (PB9)

  GPIOB->OTYPER |=(1U<<8);	//PB8 open drain
  GPIOB->OTYPER |=(1U<<9);//PB9 open drain
  /* clocks and registers control */ 
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  I2C1->CR2=42; //42MHZ
  I2C1->CCR=210; // TPCLK*FACTOR=Ton WHICH IS EQUAL TO 5000NS (DONT KNOW WHY)
  I2C1->TRISE=43; // TRISE AT MAXIMUM 1000 NS. THUS 1000/500 TPCLK=2 AND ADD 1 TO IT 
  I2C1->CR1|=(1U<<0); //ENABLE
 //DMA_TX("i2c_init COMPLETE ");
//DMA_TX("\n");  
  
}

void I2C_START(void)
{
I2C1->CR1|=I2C_CR1_START;
  

I2C1->CR1&=~I2C_CR1_START;
  
while(!(I2C1->SR1 & I2C_SR1_SB)); 
//DMA_TX("START BIT SENT");
  //DMA_TX("\n");
  
  
 if(I2C1->SR2 & I2C_SR2_MSL)
  {
 // DMA_TX("MASTER MODE INITIATED");
  //DMA_TX("\n");
  } 
}

void I2C_STOP(void)
{
I2C1->CR1|=I2C_CR1_STOP;
while(I2C1->SR2 & I2C_SR2_BUSY);
//DMA_TX("STOP BIT SENT");
//DMA_TX("\n");  
}  

void I2C_ADDR_W(uint8_t ADDRESS)
{
char DUMMY_READ;
I2C1->DR=ADDRESS|0;
while(!(I2C1->SR1 &  I2C_SR1_ADDR));
DUMMY_READ=I2C1->SR2;  
//DMA_TX("ADDRESS_WRITE SENT");
DMA_TX("\n");   
}

void I2C_ADDR_R(uint8_t ADDRESS)
{
char DUMMY_READ;
I2C1->DR=ADDRESS|1;
while(!(I2C1->SR1 &  I2C_SR1_ADDR));
DUMMY_READ=I2C1->SR2;  
//DMA_TX("ADDRESS_READ SENT");
//DMA_TX("\n");
  
}

void I2C_WRITE (uint8_t DATA_TX)
{
char printed_result[1];  
I2C1->DR=DATA_TX;
while(!(I2C1->SR1 &  I2C_SR1_BTF));
//DMA_TX("BYTE TRANSFER COMPLETE: ");
//sprintf(printed_result,"%x",DATA_TX);
//DMA_TX(printed_result);  
//DMA_TX("\n");   
}  

uint8_t I2C_READ(uint8_t ACK)
{

  
char printed_result[1];
uint8_t DATA_RX;  

if (ACK)
{  
I2C1->CR1|=I2C_CR1_ACK;
//DMA_TX("ACK..");
//DMA_TX("\n");
while (!(I2C1->SR1 & I2C_SR1_RXNE));
DATA_RX=I2C1->DR;  
}  
  

if (!(ACK))
{
I2C1->CR1 &=~I2C_CR1_ACK;
//DMA_TX("NACK..");
//DMA_TX("\n");
  
while (!(I2C1->SR1 & I2C_SR1_RXNE));  
DATA_RX=I2C1->DR;   
}    


//DMA_TX("BYTE RECEIVE COMPLETE: ");
//sprintf(printed_result,"%d",DATA_RX);
//DMA_TX(printed_result); 
//DMA_TX("\n");




  


return DATA_RX;
}  





uint32_t TEMP_CALCULATOR(uint8_t TEMP_MSB,uint8_t TEMP_LSB)
{
char printed_result[10];  
uint32_t TEMPRATURE;
float float_calc;  
uint16_t Stemp=0;
 
Stemp|=(TEMP_MSB<<8);
sprintf(printed_result,"%d",Stemp);
   
  
Stemp|=TEMP_LSB;
sprintf(printed_result,"%d",Stemp);

  
Stemp&=~((1u<<0)|(1u<<1));
sprintf(printed_result,"%d",Stemp);

  
float_calc=(float)Stemp/(powf(2.0,16.0));

float_calc=(float)float_calc*175.2f;

float_calc=(float)float_calc-46.85f;
sprintf(printed_result,"%.2f",float_calc);
DMA_TX("value in decimals: ");
DMA_TX(printed_result);
DMA_TX("\n");
TEMPRATURE=(uint32_t) float_calc ;  
return TEMPRATURE;  
}  

uint8_t TEMP_MSB;
uint8_t TEMP_LSB;
void READ_2B(uint8_t ADDRESS)
{
  
char printed_result[1];
//uint8_t DATA_RX;  
char DUMMY_READ;
I2C1->DR=ADDRESS|1;
while(!(I2C1->SR1 &  I2C_SR1_ADDR));  

I2C1->CR1 &=~I2C_CR1_ACK;
I2C1->CR1 |=I2C_CR1_POS;
//DMA_TX("NACK..POS=1");
//DMA_TX("\n");  
DUMMY_READ=I2C1->SR2;  

while(!(I2C1->SR1 &  I2C_SR1_RXNE));

I2C_STOP();
TEMP_MSB=I2C1->DR;
//DMA_TX("BYTE RECEIVE COMPLETE: ");
sprintf(printed_result,"%d",TEMP_MSB);
DMA_TX(printed_result); 
DMA_TX("\n");  

TEMP_LSB=I2C1->DR;
//DMA_TX("BYTE RECEIVE COMPLETE: ");
sprintf(printed_result,"%d",TEMP_LSB);
DMA_TX(printed_result); 
DMA_TX("\n");  
}  

void TEMP_SENSOR(void)
{
char printed_result[1];  
char temp_string[10];  
//uint8_t TEMP_MSB;
//uint8_t TEMP_LSB;
uint8_t Cyclic;    
uint32_t TEMPRATURE;

TOGGLE_RED_LED();   
I2C_START();
I2C_ADDR_W(0x80); // address write
I2C_WRITE(0xFE); //soft reset command
I2C_STOP(); // stop
Delay(0xffffff);  //a delay of 15 ms is supposed to be done
//DMA_TX("SOFT RESET AND DELAY TEMP SENSOR...");
DMA_TX("\n");  
I2C_START();
I2C_ADDR_W(0x80); // address write
I2C_WRITE(0xE3); //temprature read command hold master  
I2C_START();
//READ_2B(0X80);
//READ_2B(0X80);  
I2C_ADDR_R(0x80);
TEMP_MSB=I2C_READ(1);  
TEMP_LSB=I2C_READ(1);
Cyclic=I2C_READ(0);
//TEMP_LSB=I2C1->DR;
I2C_STOP();
TEMPRATURE=TEMP_CALCULATOR(TEMP_MSB,TEMP_LSB);
DMA_TX("FINAL TEMPRATURE VALUE: ");  
sprintf(temp_string,"%d",TEMPRATURE);
DMA_TX(temp_string);
DMA_TX("\n");

sprintf(printed_result,"%x",Cyclic);
DMA_TX("final checksum: ");
DMA_TX(printed_result);
DMA_TX("\n");
TOGGLE_RED_LED();
}  

<<<<<<< HEAD
//TEST
=======

//test

>>>>>>> Headersbranch
int main(void)
{
  
  
  UART2INIT();
	DMA_UART_INIT();
  
  //ADC_INIT();
  //DMA_ADC_INIT();
	
  //Timer_Init();
	//NVIC_EnableIRQ(TIM3_IRQn);
	//NVIC_EnableIRQ(DMA2_Stream0_IRQn);
 // NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  DMA_TX("i2c_init: ");
  DMA_TX("\n");
  I2C_INIT();
  Delay(0xffffff);
  
  //TEMP_SENSOR();
   while (1)
  {
 
     
	 TEMP_SENSOR();
    
  }
}




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {  
	
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
