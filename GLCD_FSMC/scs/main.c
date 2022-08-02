/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
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
#include "GLCDIN.h"

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern unsigned short og_color,tg_color;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
  char buf[200];
void delay_ms(unsigned ms)
{
  unsigned int i,j;
  for(i=0;i<ms;i++)
    for(j=0;j<9000;j++);
}

#define DEL 800
int main(void)
{
 unsigned int i,j,index,k;

  LCD_Initializtion();

	  LCD_Clear(Black);
	  	   
  while (0)
  {	
     LCD_Clear(Blue);
	 delay_ms(1000);
	 LCD_Clear(White);
     delay_ms(1000);
	 LCD_Clear(Black);
     delay_ms(1000);
	 LCD_Clear(White);
     delay_ms(1000);
	 LCD_Clear(Red);
     delay_ms(1000);
	 LCD_Clear(Blue2);
     delay_ms(1000);
	 LCD_Clear(Red);
     delay_ms(1000);
	 LCD_Clear(Magenta);
     delay_ms(1000);
	 LCD_Clear(Green);
     delay_ms(1000);
	 LCD_Clear(Green);
     delay_ms(1000); 


  }
}


////////////////////////////////////////////////////////////////////////


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
