/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "GUI.h"
#include "GUIDEMO.h"
#include "math.h"
#include <string.h>
/* USER CODE BEGIN Includes */
#define PI 3.14
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

#define ID_WINDOW_0 (GUI_ID_USER + 0x00)
#define ID_GRAPH_0 (GUI_ID_USER + 0x01)
#define ID_PROGBAR_0 (GUI_ID_USER + 0x02)
#define ID_TEXT_0 (GUI_ID_USER + 0x03)

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_FMC_Init(void);
void MX_LTDC_Init(void);
void MX_SPI5_Init(void);
extern void LCD_Init();
extern void SDRAM_Init();
extern WM_HWIN CreateWindow(void);

unsigned int *Pointer;
//I16 ShowData[10] = {1,2,3,4,5,6,7,8,9,10};
I16 *ShowData = (I16 *)0XD07F0000;
I16 *ShowData2 = (I16 *)0XD07FF000;
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void UserDraw(WM_HWIN hWin, int Stage)
{
    GUI_RECT RectH, RectV;
    int FontSizeY;
    
    if(Stage == GRAPH_DRAW_LAST)
    {
        GUI_SetFont(&GUI_Font8x10_ASCII);
        FontSizeY = GUI_GetFontSizeY();
        
        WM_GetInsideRect(&RectH);
        WM_GetInsideRect(&RectV);
        
        RectV.x1 = RectV.x0 + FontSizeY;
        RectH.y0 = RectV.y1 - FontSizeY;
        GUI_DispStringInRectEx("Amplitude", &RectV, GUI_TA_HCENTER, strlen("Amplitude"), GUI_ROTATE_CCW);
        GUI_DispStringInRectEx("Phase", &RectH, GUI_TA_HCENTER, strlen("Phase"), GUI_ROTATE_0);
    }
}

int main(void)
{
    unsigned char x;
    unsigned short angle;
    WM_HWIN hWin;
    PROGBAR_Handle hProg;
    GRAPH_SCALE_Handle v_scale_handle;
    GRAPH_SCALE_Handle h_scale_handle;
    GRAPH_Handle hGraph;
    GRAPH_DATA_Handle hGraphData;
    GRAPH_DATA_Handle hGraphData2;
    
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    SDRAM_Init();


    __CRC_CLK_ENABLE();
    GUI_Init();
    
    PROGBAR_SetDefaultSkin(PROGBAR_SKIN_FLEX);
    
    hWin = CreateWindow();
    WM_ShowWindow(hWin);
    
    hProg = WM_GetDialogItem(hWin,ID_PROGBAR_0);
    PROGBAR_SetValue(hProg, 0);
    
    hGraph = WM_GetDialogItem(hWin,ID_GRAPH_0);
    GRAPH_SetBorder(hGraph,30,10,10,30);
    
    v_scale_handle = GRAPH_SCALE_Create(15,GUI_TA_LEFT|GUI_TA_TOP,GRAPH_SCALE_CF_VERTICAL,30);
    h_scale_handle = GRAPH_SCALE_Create(175,GUI_TA_LEFT|GUI_TA_TOP,GRAPH_SCALE_CF_HORIZONTAL,30);
    
    GRAPH_SCALE_SetTextColor(v_scale_handle, GUI_GREEN);
    GRAPH_SCALE_SetTextColor(h_scale_handle, GUI_GREEN);
    
    GRAPH_SCALE_SetFactor(v_scale_handle, 0.1);
    GRAPH_SCALE_SetFactor(h_scale_handle, 0.06666);
    
    GRAPH_SetColor(hGraph, GUI_RED, GRAPH_CI_BORDER);
    
    GRAPH_AttachScale(hGraph, v_scale_handle);
    GRAPH_AttachScale(hGraph, h_scale_handle);
    
    GRAPH_SetUserDraw(hGraph, UserDraw);
    
    GRAPH_SetGridDistX(hGraph, 30);
    GRAPH_SetGridDistY(hGraph, 30);
    GRAPH_SetLineStyleH(hGraph,GUI_LS_DOT);
    GRAPH_SetLineStyleV(hGraph, GUI_LS_DOT);
    GRAPH_SetGridVis(hGraph, 1);
    
    hGraphData = GRAPH_DATA_YT_Create(GUI_BLUE, 180, ShowData, 0);
    GRAPH_AttachData(hGraph, hGraphData);
    
    hGraphData2 = GRAPH_DATA_YT_Create(GUI_GREEN, 180, ShowData2, 0);
    GRAPH_AttachData(hGraph, hGraphData2);
    
    GUI_Exec();
    angle = 0;
    x = 0;
    while (1)
    {      
        HAL_Delay(100);
        x++;
        if(x>100)
            x = 0;
        PROGBAR_SetValue(hProg, x);
        
        GRAPH_DATA_YT_AddValue(hGraphData, x);
        
        GRAPH_DATA_YT_AddValue(hGraphData2, 70*sin(angle*PI/180)+70);
        angle++;
        GUI_Exec();
//        GUIDEMO_Main();
    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_ActivateOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_16;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* LTDC init function */
void MX_LTDC_Init(void)
{

  LTDC_LayerCfgTypeDef pLayerCfg;

  __LTDC_CLK_ENABLE();
  
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 9;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 29;
  hltdc.Init.AccumulatedVBP = 3;
  hltdc.Init.AccumulatedActiveW = 269;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 279;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  HAL_LTDC_Init(&hltdc);

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 239;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 319;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xD0000000;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0);

}

/* SPI5 init function */
void MX_SPI5_Init(void)
{
  __SPI5_CLK_ENABLE();
  
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi5.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi5);

}
/* FMC initialization function */
void MX_FMC_Init(void)
{
  FMC_SDRAM_TimingTypeDef SdramTiming;

  /** Perform the SDRAM1 memory initialization sequence
  */
  __FMC_CLK_ENABLE();
  
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 6;
  SdramTiming.WriteRecoveryTime = 2;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  HAL_SDRAM_Init(&hsdram1, &SdramTiming);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __GPIOF_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
