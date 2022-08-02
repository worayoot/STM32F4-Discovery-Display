
#include "stm32f4_discovery.h"
#include "GLCDIN.h"
#include <math.h>

#define LCD_REG              (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define LCD_RAM              (*((volatile unsigned short *) 0x60020000))

extern const char ascii_7x11[95][14];
unsigned short og_color,tg_color;
////////////////////////////////////////////////////////////////////
uint16_t DeviceCode;

void LCD_CtrlLinesConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE |
                         RCC_AHB1Periph_GPIOB, ENABLE);

/*-- GPIO Configuration ------------------------------------------------------*/
  /* SRAM Data lines,  NOE and NWE configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 |
                                GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15 |
                                GPIO_Pin_4 |GPIO_Pin_5 |GPIO_Pin_11 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


}

///////////////////////////////////////////////////////////////////////////////

void LCD_FSMCConfig(void)
{																						 
  FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  p;
   
  /* Enable FSMC clock */
  RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
  
/*-- FSMC Configuration ------------------------------------------------------*/
/*----------------------- SRAM Bank 3 ----------------------------------------*/
  /* FSMC_Bank1_NORSRAM4 configuration */
  p.FSMC_AddressSetupTime = 7;
  p.FSMC_AddressHoldTime = 0;
  p.FSMC_DataSetupTime = 7;
  p.FSMC_BusTurnAroundDuration = 0;
  p.FSMC_CLKDivision = 0;
  p.FSMC_DataLatency = 0;
  p.FSMC_AccessMode = FSMC_AccessMode_B;


  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;	  

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

  /* Enable FSMC Bank1_SRAM Bank */
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
 // FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);
}

 void LCD_init(void)
{
 /* Configure the LCD Control pins --------------------------------------------*/
  LCD_CtrlLinesConfig();

/* Configure the FSMC Parallel interface -------------------------------------*/
  LCD_FSMCConfig();
}

/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : Writes to the selected LCD register.
* Input          : - LCD_Reg: address of the selected register.
*                  - LCD_RegValue: value to write to the selected register.
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
 void LCD_WriteReg(uint8_t LCD_Reg,uint16_t LCD_RegValue)
{
  /* Write 16-bit Index, then Write Reg */
  LCD_REG = LCD_Reg;
  /* Write 16-bit Reg */
  LCD_RAM = LCD_RegValue;
}
void GLCD_Write_Command(unsigned char LCD_Reg)
{
   LCD_REG = LCD_Reg;
}

/*******************************************************************************
* Function Name  : LCD_WriteReg
* Description    : Reads the selected LCD Register.
* Input          : None
* Output         : None
* Return         : LCD Register Value.
* Attention		 : None
*******************************************************************************/
uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{
  /* Write 16-bit Index (then Read Reg) */
  LCD_REG = LCD_Reg;
  /* Read 16-bit Reg */
  return (LCD_RAM);
}

/*******************************************************************************
* Function Name  : LCD_WriteRAM_Prepare
* Description    : Prepare to write to the LCD RAM.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_WriteRAM_Prepare(void)
{
  LCD_REG = R34;
}

/*******************************************************************************
* Function Name  : LCD_WriteRAM
* Description    : Writes to the LCD RAM.
* Input          : - RGB_Code: the pixel color in RGB mode (5-6-5).
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_WriteRAM(uint16_t RGB_Code)					 
{
  /* Write 16-bit GRAM Reg */
  LCD_RAM = RGB_Code;
}

/*******************************************************************************
* Function Name  : LCD_ReadRAM
* Description    : Reads the LCD RAM.
* Input          : None
* Output         : None
* Return         : LCD RAM Value.
* Attention		 : None
*******************************************************************************/
uint32_t LCD_ReadRAM(void)
{
  volatile uint32_t dummy; 
  /* Write 16-bit Index (then Read Reg) */
  LCD_REG = R34; /* Select GRAM Reg */
  /* Read 16-bit Reg */
  dummy = LCD_RAM; 
  
  return LCD_RAM;
}

////////////////////////////////////////////////////////////////////////////////

/*******************************************************************************
* Function Name  : LCD_Initializtion
* Description    : Initialize TFT Controller.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/

void LCD_Delay(uint16_t nCount)
{
 uint16_t TimingDelay; 
 while(nCount--)
   {
    for(TimingDelay=0;TimingDelay<2;TimingDelay++);
   }
}
void LCD_Initializtion(void)
{
  LCD_init();
  LCD_Delay(5);  /* delay 50 ms */		
  DeviceCode = LCD_ReadReg(0x0000);		/* 读取屏ID	*/
  if(DeviceCode==0x9320 || DeviceCode==0x9300)
  {
    LCD_WriteReg(0x00,0x0000);
	LCD_WriteReg(0x01,0x0100);	/* Driver Output Contral */
	LCD_WriteReg(0x02,0x0700);	/* LCD Driver Waveform Contral */
	LCD_WriteReg(0x03,0x1018);	/* Entry Mode Set */
	
	LCD_WriteReg(0x04,0x0000);	/* Scalling Contral */
    LCD_WriteReg(0x08,0x0202);	/* Display Contral */
	LCD_WriteReg(0x09,0x0000);	/* Display Contral 3.(0x0000) */
	LCD_WriteReg(0x0a,0x0000);	/* Frame Cycle Contal.(0x0000) */
    LCD_WriteReg(0x0c,(1<<0));	/* Extern Display Interface Contral */
	LCD_WriteReg(0x0d,0x0000);	/* Frame Maker Position */
	LCD_WriteReg(0x0f,0x0000);	/* Extern Display Interface Contral 2. */
	
    LCD_Delay(10);  /* delay 100 ms */		
	LCD_WriteReg(0x07,0x0101);	/* Display Contral */
    LCD_Delay(10);  /* delay 100 ms */		

	LCD_WriteReg(0x10,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));	/* Power Control 1.(0x16b0)	*/
	LCD_WriteReg(0x11,0x0007);								/* Power Control 2 */
	LCD_WriteReg(0x12,(1<<8)|(1<<4)|(0<<0));				/* Power Control 3.(0x0138)	*/
	LCD_WriteReg(0x13,0x0b00);								/* Power Control 4 */
	LCD_WriteReg(0x29,0x0000);								/* Power Control 7 */
	
	LCD_WriteReg(0x2b,(1<<14)|(1<<4));
		
	LCD_WriteReg(0x50,0);       /* Set X Start */
	LCD_WriteReg(0x51,239);	    /* Set X End */
	LCD_WriteReg(0x52,0);	    /* Set Y Start */
	LCD_WriteReg(0x53,319);	    /* Set Y End */
	
	LCD_WriteReg(0x60,0x2700);	/* Driver Output Control */
	LCD_WriteReg(0x61,0x0001);	/* Driver Output Control */
	LCD_WriteReg(0x6a,0x0000);	/* Vertical Srcoll Control */
	
	LCD_WriteReg(0x80,0x0000);	/* Display Position? Partial Display 1 */
	LCD_WriteReg(0x81,0x0000);	/* RAM Address Start? Partial Display 1 */
	LCD_WriteReg(0x82,0x0000);	/* RAM Address End-Partial Display 1 */
	LCD_WriteReg(0x83,0x0000);	/* Displsy Position? Partial Display 2 */
	LCD_WriteReg(0x84,0x0000);	/* RAM Address Start? Partial Display 2 */
	LCD_WriteReg(0x85,0x0000);	/* RAM Address End? Partial Display 2 */
	
    LCD_WriteReg(0x90,(0<<7)|(16<<0));	/* Frame Cycle Contral.(0x0013)	*/
	LCD_WriteReg(0x92,0x0000);	/* Panel Interface Contral 2.(0x0000) */
	LCD_WriteReg(0x93,0x0001);	/* Panel Interface Contral 3. */
    LCD_WriteReg(0x95,0x0110);	/* Frame Cycle Contral.(0x0110)	*/
	LCD_WriteReg(0x97,(0<<8));	
	LCD_WriteReg(0x98,0x0000);	/* Frame Cycle Contral */

    LCD_WriteReg(0x07,0x0173);
  }
	 
  LCD_Delay(5);  /* delay 50 ms */
  		
}

void LCD_SetCursor(uint16_t Xpos,uint16_t Ypos)
{
  if(DeviceCode==0x8989)
  {
    LCD_WriteReg(0x004e,Xpos); /* Row */
    LCD_WriteReg(0x004f,Ypos); /* Line */ 
  }
  else if(DeviceCode==0x9919)
  {
    LCD_WriteReg(0x004e,Xpos); /* Row */
    LCD_WriteReg(0x004f,Ypos); /* Line */	
  }
  else
  {
    LCD_WriteReg(0x0020,Xpos); /* Row */
    LCD_WriteReg(0x0021,Ypos); /* Line */
  }
}

void LCD_Clear(uint16_t Color)
{
  uint32_t index=0;
  //LCD_SetCursor(0,0);
  LCD_SetWindows1(0,0,239,319); 
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  for(index=0;index<76800;index++)
   {
     LCD_RAM=Color;
   }
}
void LCD_ClearXY(uint16_t x1 ,uint16_t y1 ,uint16_t x2 ,uint16_t y2 ,uint16_t Color1 ,uint16_t Color2)
{
  uint32_t index=0,f=0;
  //LCD_SetCursor(0,0);
  LCD_SetWindows1(x1,y1,x2,y2); 
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  for(index=0;index<((x2-x1-1)*(y2-y1-1));index++)
   {
     if(f==0){
	    LCD_RAM=Color1;
		f=1;
	 }
	 else if(f==1)
	  {
	    LCD_RAM=Color2;
		//c=LCD_ReadRAM();
		//LCD_RAM=c;
		f=0;
	  }
   }

   
}



/******************************************************************************
* Function Name  : LCD_SetWindows
* Description    : Sets Windows Area.
* Input          : - StartX: Row Start Coordinate 
*                  - StartY: Line Start Coordinate  
*				   - xLong:  
*				   - yLong: 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_SetWindows(uint16_t xStart,uint16_t yStart,uint16_t xLong,uint16_t yLong)
{
  LCD_SetCursor(xStart,yStart); 
  LCD_WriteReg(0x0050,xStart);         /* 水平GRAM起始位置 */
  LCD_WriteReg(0x0051,xStart+xLong-1); /* 水平GRAM终止位置 */
  LCD_WriteReg(0x0052,yStart);         /* 垂直GRAM起始位置 */
  LCD_WriteReg(0x0053,yStart+yLong-1); /* 垂直GRAM终止位置 */ 
}
void LCD_SetWindows1(uint16_t xStart,uint16_t yStart,uint16_t xLong,uint16_t yLong)
{
  uint16_t x1,x2;
   x1=(xLong)<<8;
   x2=x1|xStart;
  LCD_WriteReg(0x11, 0x6070);
  LCD_SetCursor(xStart,yStart); 
  LCD_WriteReg(0x0044,x2);         /* 水平GRAM起始位置 */
//  LCD_WriteReg(0x0051,xLong-1); /* 水平GRAM终止位置 */
  LCD_WriteReg(0x0045,yStart);         /* 垂直GRAM起始位置 */
  LCD_WriteReg(0x0046,yLong); /* 垂直GRAM终止位置 */ 
}


/******************************************************************************
* Function Name  : LCD_GetPoint
* Description    : 获取指定座标的颜色值
* Input          : - Xpos: Row Coordinate
*                  - Xpos: Line Coordinate 
* Output         : None
* Return         : Screen Color
* Attention		 : None
*******************************************************************************/
uint16_t LCD_GetPoint(uint16_t Xpos,uint16_t Ypos)
{
  LCD_SetCursor(Xpos,Ypos);
  if( DeviceCode==0x7783 || DeviceCode==0x4531 || DeviceCode==0x8989 )
    return ( LCD_ReadRAM() );
  else
    return ( LCD_BGR2RGB(LCD_ReadRAM()) );
}
/******************************************************************************
* Function Name  : LCD_BGR2RGB
* Description    : RRRRRGGGGGGBBBBB 改为 BBBBBGGGGGGRRRRR 格式
* Input          : - color: BRG 颜色值  
* Output         : None
* Return         : RGB 颜色值
* Attention		 : 内部函数调用
*******************************************************************************/
uint16_t LCD_BGR2RGB(uint16_t color)
{
  uint16_t  r, g, b, rgb;

  b = ( color>>0 )  & 0x1f;
  g = ( color>>5 )  & 0x3f;
  r = ( color>>11 ) & 0x1f;
 
  rgb =  (b<<11) + (g<<5) + (r<<0);

  return( rgb );
}
/******************************************************************************
* Function Name  : LCD_SetPoint
* Description    : 在指定座标画点
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_SetPoint(uint16_t Xpos,uint16_t Ypos,uint16_t point)
{
  if ( ( Xpos > 239 ) ||( Ypos > 319 ) ) return;
  LCD_SetCursor(Xpos,Ypos);
  LCD_WriteRAM_Prepare();
  LCD_WriteRAM(point);
}

/******************************************************************************
* Function Name  : LCD_DrawPicture
* Description    : 在指定坐标范围显示一副图片
* Input          : - StartX: Row Start Coordinate 
*                  - StartY: Line Start Coordinate  
*				   - EndX: Row End Coordinate 
*				   - EndY: Line End Coordinate   
* Output         : None
* Return         : None
* Attention		 : 图片取模格式为水平扫描，16位颜色模式
*******************************************************************************/
void LCD_DrawPicture(uint16_t StartX,uint16_t StartY,uint16_t EndX,uint16_t EndY,uint16_t *pic)
{
  uint16_t  i;
  LCD_SetCursor(StartX,StartY);  
  LCD_WriteRAM_Prepare();
  for (i=0;i<(EndX*EndY);i++)
  {
      LCD_WriteRAM(*pic++);
  }
}


/******************************************************************************
* Function Name  : LCD_DrawLine
* Description    : 画一条直线
* Input          : - x1: 行座标开始
*                  - y1: 列座标开始 
*				   - x2: 行座标结束
*				   - y2: 列座标结束  
*				   - bkColor: 背景颜色 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LCD_DrawLine(int x1, int y1, int x2, int y2,uint16_t bkColor)  
{ 
  int x,y,dx,dy,Dx,Dy,e,i; 
  Dx=x2-x1; 
  Dy=y2-y1; 

  dx=fabs(x2-x1); 
  dy=fabs(y2-y1); 
  x=x1; 
  y=y1; 
  if(dy>dx) 
  { 
    e=-dy; 
    for(i=0;i<dy;i++) 
    { 
      LCD_SetPoint(x,y,bkColor); 
      if(Dy>=0) y++;   
      else y--;    
      e+=2*dx; 
      if(e>=0) 
      { 
        if(Dx>=0) x++; 
        else x--;  
        e-=2*dy; 
      } 
    } 
  } 
  else 
  { 
    e=-dx; 
    for(i=0;i<dx;i++) 
    { 
      LCD_SetPoint(x,y,bkColor); 
      if(Dx>=0) x++; 
      else x--; 
      e+=2*dy; 
      if(e>=0) 
      { 
        if(Dy>=0) y++; 
        else y--;
        e-=2*dx;
      } 
    } 
  } 
} 

/****************************************************************************/
/**                Function Print Text 1 Charecter size 7x11                */
/****************************************************************************/
/* Parameter : row      = Ascii Code (Position buffer keep text)		    */
/*             adx,ady  = Position X,Y for begin plot text by will 			*/
/*                        begin plot from bottom left to top left   		*/
/*             fg_clr   = Color of text										*/
/*             bg_clr   = Color background of text(if bg_clr = no_bg or 1=	*/
/*                        non color background)								*/
/****************************************************************************/
void text_7x11_hor(char row,unsigned short adx,unsigned short ady,unsigned short fg_clr,unsigned short bg_clr)
{
  long ax,ay;
  char m,n,tx;
     
  ax = adx;
  ay = ady; 

  row = row-0x20;

  // Print Text 1 Charecter(data 14 Byte) 	  
  for(m=0;m<14;m++)
  {
    tx = ascii_7x11[row][m];  																//Read data Ascii
	
	for(n=0;n<8;n++)		       															//Loop Sent data  1 byte(8bit)
	{
	  if(tx & 0x80)				   															//if data bit7 = 1 ,Plot Color area Charecter
	  {              
	    GLCD_Write_Command(0x20);  															//Command Set Adddress Hor(X)
        LCD_WriteRAM(ax);  																//Sent X_Address CGRAM	 		
        GLCD_Write_Command(0x21);  															//Command Set Address Ver(Y)
        LCD_WriteRAM(ay);  																//Sent Y_Address CGRAM
		GLCD_Write_Command(0x22);  															//Command Write data 
        LCD_WriteRAM(fg_clr); 
	  }
	  else						   															//if data bit7 = 0 ,Plot Color area back ground Charecter
	  {
	    if(bg_clr != 1)     
        {
		  GLCD_Write_Command(0x20);  														//Command Set Adddress Hor(X)
          LCD_WriteRAM(ax);  															//Sent X_Address CGRAM	
          GLCD_Write_Command(0x21);  														//Command Set Adddress Ver(Y)
          LCD_WriteRAM(ay);  															//Sent Y_Address CGRAM
		  GLCD_Write_Command(0x22);  														//Command Write data
          LCD_WriteRAM(bg_clr);  														//Sent Data
		}
	  }

	  tx <<= 1;  																			//Shift Right data 1 bit
	  ay   = ay-1;  																		//Decrement Y-address
	} 

	m++;  																					//Increment Next pointter byte Data 


	// Sent data byte2=3 bit 

	tx = ascii_7x11[row][m];  																//Read data byte2
	
	for(n=0;n<3;n++)			   															//Loop sent data byte2 = 3 bit
	{				
	  if(tx & 0x80)				   															//if data bit7 = 1 ,Plot Color area Charecter
	  {              
	    GLCD_Write_Command(0x20);  															//Command Set Adddress Hor(X)
        LCD_WriteRAM(ax);  																//Sent X_Address CGRAM		
        GLCD_Write_Command(0x21);  															//Command Set Adddress Ver(Y)
        LCD_WriteRAM(ay);  																//Sent Y_Address CGRAM
		GLCD_Write_Command(0x22);  															//Command Write data
        LCD_WriteRAM(fg_clr);
	  }
	  else						   															//if data bit7 = 0 ,Plot Color area back ground Charecter
	  {
	    if(bg_clr != 1)     
        {
		  GLCD_Write_Command(0x20);  														//Command Set Adddress Hor(X)
          LCD_WriteRAM(ax);  															//Sent X_Address CGRAM	 		
          GLCD_Write_Command(0x21);  														//Command Set Adddress Ver(Y)
          LCD_WriteRAM(ay);  															//Sent Y_Address CGRAM
		  GLCD_Write_Command(0x22);  														//Command Write data
          LCD_WriteRAM(bg_clr);
		}
	  }

	  tx <<= 1;  																			//Shift Right data 1 bit
	  ay = ay-1;  																			//Decrement Y-address
	} 

	ax = ax+1; 																				//Complet sent data 2 byte(11bit) Increment X-Address
	ay = ady; 																				//Set Position Y-address old
  }	
  // Fill Back ground Color Position space between Charecter 1 Colum 
 
  if(bg_clr != 1)     
  {
    for(n=0;n<11;n++)
	{
	  GLCD_Write_Command(0x20);  															//Command Set Adddress Hor(X)
      LCD_WriteRAM(ax);  																//Sent X_Address CGRAM	 		
      GLCD_Write_Command(0x21);  															//Command Set Adddress Ver(Y)
      LCD_WriteRAM(ay);  																//Sent Y_Address CGRAM
	  GLCD_Write_Command(0x22);  															//Command Write data
      LCD_WriteRAM(bg_clr);
	  ay = ay-1;  																			//Incre ment Y-Address
	}
  }
}


void LCD_OUT(char *str,unsigned short cur_x,unsigned short cur_y)
{
   char i=0;

		for(i=0;str[i]!='\0';i++){
		    text_7x11_hor(str[i],cur_x,cur_y,og_color,tg_color);
			cur_x += 8;
		   }

}




