/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "readout_driver.h"
#include "MPU9250.h"
// needed to retarget stdio
#include <stdio.h>
#include <errno.h>
#include <sys/unistd.h>

#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Used to clamp the sensor readings between 1 and 0
#define AI_MIN(x_, y_) \
  ( ((x_)<(y_)) ? (x_) : (y_) )

#define AI_MAX(x_, y_) \
  ( ((x_)>(y_)) ? (x_) : (y_) )

#define AI_CLAMP(x_, min_, max_, type_) \
  (type_) (AI_MIN(AI_MAX(x_, min_), max_))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef status;
FMC_SDRAM_CommandTypeDef command;

volatile bool collect_data = false;
volatile bool collect_done = false;

#ifdef FLOAT_Pressure
static float tactileData_scaled[DATA_SIZE];
#else
static uint16_t tactileData[DATA_SIZE];
#endif
#ifdef TEST_THRESH
static uint16_t tactileThreshold[DATA_SIZE] = {1714, 1632, 1725, 1616, 1635, 1655, 1599, 1556, 1599, 1623, 1635, 1589, 1631, 1602, 1801, 1583, 1608, 1641, 1830, 1646, 1595, 1605, 1594, 1563, 1579, 1600, 1686, 1632, 1581, 1566, 1570, 1559, 1703, 1622, 1642, 1608, 1623, 1583, 1614, 1557, 1587, 1603, 1595, 1601, 1633, 1608, 1743, 1571, 1624, 1612, 1863, 1670, 1625, 1622, 1608, 1562, 1585, 1610, 1634, 1622, 1591, 1581, 1603, 1567, 1691, 1629, 1622, 1586, 1606, 1634, 1591, 1563, 1639, 1609, 1604, 1555, 1617, 1591, 1684, 1571, 1611, 1595, 1790, 1628, 1604, 1608, 1664, 1565, 1580, 1594, 1607, 1597, 1601, 1589, 1589, 1573, 1527, 1527, 1527, 1528, 1527, 1528, 1529, 1528, 1531, 1529, 1538, 1545, 1551, 1544, 1717, 1573, 1597, 1587, 1764, 1651, 1636, 1649, 1695, 1581, 1603, 1610, 1628, 1583, 1578, 1591, 1591, 1563, 1529, 1526, 1527, 1527, 1527, 1527, 1527, 1529, 1533, 1529, 1537, 1541, 1542, 1541, 1661, 1615, 1608, 1596, 1759, 1644, 1657, 1659, 1676, 1601, 1661, 1637, 1701, 1601, 1583, 1570, 1579, 1571, 1542, 1529, 1529, 1527, 1527, 1531, 1529, 1533, 1535, 1539, 1535, 1545, 1548, 1553, 1634, 1665, 1617, 1595, 1679, 1621, 1621, 1683, 1594, 1602, 1608, 1581, 1642, 1578, 1565, 1557, 1572, 1566, 1593, 1565, 1608, 1583, 1578, 1578, 1609, 1643, 1689, 1723, 1559, 1643, 1666, 1815, 1618, 1672, 1616, 1621, 1698, 1634, 1709, 1730, 1609, 1617, 1597, 1610, 1658, 1567, 1571, 1553, 1552, 1548, 1649, 1590, 1608, 1620, 1581, 1621, 1599, 1639, 1733, 1734, 1567, 1623, 1647, 1708, 1614, 1655, 1618, 1595, 1661, 1599, 1673, 1628, 1583, 1593, 1575, 1581, 1596, 1564, 1598, 1566, 1635, 1564, 1657, 1603, 1597, 1598, 1630, 1600, 1594, 1621, 1776, 1697, 1568, 1586, 1637, 1700, 1617, 1646, 1633, 1560, 1598, 1567, 1592, 1582, 1565, 1586, 1563, 1605, 1657, 1636, 1658, 1687, 1618, 1597, 1527, 1527, 1526, 1527, 1527, 1529, 1527, 1531, 1529, 1527, 1535, 1543, 1539, 1545, 1674, 1711, 1614, 1572, 1591, 1580, 1583, 1586, 1566, 1585, 1599, 1625, 1635, 1568, 1579, 1606, 1604, 1595, 1696, 1640, 1623, 1585, 1594, 1715, 1566, 1625, 1673, 1667, 1658, 1679, 1649, 1561, 1733, 1751, 1639, 1594, 1606, 1625, 1569, 1580, 1557, 1588, 1597, 1612, 1638, 1571, 1584, 1584, 1621, 1644, 1704, 1665, 1680, 1600, 1609, 1696, 1563, 1633, 1619, 1624, 1691, 1685, 1684, 1557, 1689, 1683, 1629, 1568, 1578, 1585, 1574, 1586, 1561, 1609, 1577, 1617, 1677, 1597, 1615, 1621, 1626, 1617, 1657, 1620, 1597, 1596, 1571, 1707, 1549, 1629, 1623, 1667, 1642, 1674, 1627, 1555, 1698, 1670, 1685, 1606, 1621, 1654, 1580, 1629, 1587, 1686, 1630, 1620, 1666, 1593, 1608, 1617, 1611, 1593, 1529, 1529, 1527, 1528, 1526, 1532, 1528, 1531, 1535, 1529, 1544, 1553, 1545, 1544, 1739, 1645, 1677, 1591, 1606, 1662, 1589, 1603, 1577, 1650, 1591, 1606, 1633, 1580, 1589, 1592, 1626, 1584, 1543, 1529, 1529, 1527, 1528, 1533, 1529, 1534, 1533, 1529, 1551, 1569, 1566, 1551, 1742, 1631, 1653, 1588, 1585, 1668, 1586, 1594, 1568, 1676, 1569, 1594, 1617, 1575, 1590, 1595, 1688, 1630, 1584, 1628, 1607, 1605, 1574, 1607, 1556, 1598, 1603, 1583, 1672, 1689, 1572, 1597, 1751, 1639, 1635, 1575, 1575, 1612, 1575, 1560, 1550, 1634, 1561, 1597, 1632, 1577, 1592, 1572, 1629, 1607, 1605, 1647, 1599, 1597, 1584, 1602, 1565, 1608, 1636, 1632, 1733, 1721, 1599, 1616, 1910, 1648, 1690, 1589, 1585, 1616, 1592, 1574, 1553, 1640, 1568, 1597, 1644, 1584, 1601, 1585, 1619, 1617, 1576, 1621, 1592, 1599, 1579, 1603, 1577, 1639, 1646, 1605, 1761, 1696, 1623, 1613, 1833, 1595, 1679, 1588, 1544, 1575, 1558, 1564, 1546, 1666, 1644, 1694, 1711, 1680, 1624, 1678, 1607, 1612, 1527, 1525, 1525, 1525, 1526, 1527, 1525, 1527, 1525, 1526, 1527, 1526, 1525, 1526, 1525, 1525, 1532, 1528, 1527, 1529, 1529, 1529, 1528, 1538, 1532, 1580, 1566, 1567, 1557, 1533, 1529, 1535, 1532, 1531, 1527, 1525, 1524, 1525, 1523, 1527, 1526, 1526, 1525, 1527, 1525, 1525, 1525, 1525, 1537, 1529, 1527, 1527, 1527, 1525, 1525, 1537, 1529, 1553, 1598, 1555, 1591, 1530, 1533, 1531, 1526, 1527, 1526, 1526, 1525, 1526, 1524, 1526, 1525, 1525, 1525, 1525, 1526, 1525, 1526, 1527, 1529, 1528, 1529, 1529, 1527, 1529, 1527, 1542, 1534, 1657, 1701, 1669, 1708, 1533, 1531, 1535, 1532, 1524, 1529, 1524, 1526, 1527, 1525, 1527, 1524, 1527, 1527, 1529, 1527, 1523, 1527, 1526, 1537, 1529, 1531, 1527, 1527, 1530, 1527, 1538, 1533, 1678, 1855, 1700, 1725, 1533, 1530, 1532, 1527, 1525, 1526, 1525, 1525, 1525, 1526, 1525, 1527, 1525, 1525, 1526, 1525, 1526, 1527, 1527, 1530, 1529, 1527, 1527, 1527, 1527, 1527, 1533, 1528, 1602, 1651, 1580, 1632, 1531, 1529, 1533, 1535, 1529, 1529, 1525, 1525, 1527, 1525, 1525, 1526, 1525, 1525, 1525, 1525, 1525, 1526, 1525, 1539, 1532, 1527, 1528, 1526, 1528, 1527, 1529, 1527, 1557, 1559, 1563, 1573, 1531, 1527, 1530, 1542, 1525, 1525, 1525, 1524, 1525, 1525, 1526, 1525, 1525, 1525, 1525, 1527, 1523, 1525, 1525, 1536, 1529, 1527, 1529, 1527, 1527, 1525, 1527, 1527, 1653, 1665, 1689, 1615, 1527, 1527, 1529, 1533, 1527, 1527, 1525, 1527, 1526, 1524, 1525, 1527, 1526, 1525, 1526, 1525, 1529, 1528, 1526, 1537, 1530, 1529, 1528, 1527, 1527, 1527, 1531, 1528, 1579, 1593, 1582, 1582, 1532, 1529, 1531, 1541, 1528, 1530, 1525, 1528, 1525, 1528, 1527, 1527, 1525, 1526, 1527, 1525, 1525, 1527, 1526, 1541, 1527, 1530, 1527, 1529, 1528, 1525, 1532, 1527, 1598, 1583, 1585, 1582, 1530, 1530, 1529, 1533, 1527, 1527, 1525, 1525, 1526, 1525, 1525, 1525, 1525, 1526, 1527, 1525, 1523, 1525, 1527, 1535, 1531, 1527, 1527, 1526, 1527, 1526, 1530, 1527, 1577, 1593, 1582, 1579, 1527, 1527, 1530, 1529, 1525, 1527, 1525, 1526, 1527, 1526, 1527, 1524, 1525, 1527, 1525, 1525, 1527, 1525, 1525, 1529, 1528, 1528, 1529, 1526, 1529, 1525, 1531, 1527, 1609, 1607, 1589, 1583, 1527, 1527, 1529, 1535, 1527, 1530, 1527, 1528, 1527, 1526, 1527, 1527, 1526, 1525, 1526, 1525, 1525, 1527, 1526, 1535, 1527, 1530, 1527, 1529, 1528, 1527, 1529, 1528, 1568, 1602, 1578, 1558, 1527, 1528, 1528, 1527, 1525, 1527, 1527, 1525, 1526, 1526, 1526, 1525, 1526, 1527, 1527, 1525, 1525, 1526, 1525, 1529, 1527, 1527, 1528, 1526, 1528, 1526, 1528, 1526, 1547, 1560, 1578, 1542, 1527, 1527, 1528, 1526, 1527, 1527, 1525, 1525, 1527, 1527, 1527, 1525, 1527, 1527, 1527, 1525, 1526, 1526, 1525, 1527, 1527, 1527, 1527, 1526, 1527, 1527, 1527, 1527, 1541, 1552, 1544, 1530, 1527, 1527, 1527};
static bool check_thresh = false;
#endif

#ifdef FB_RES_1K
// 1kOhm feedback resistor readout board
const uint16_t tactile_shift = 1820;
const float tactile_scale = 1/(2700.0 - 1820.0);
#elseif FB_RES_3K
// 3kOhm feedback resistor readout board
const uint16_t tactile_shift = 1510;
const float tactile_scale = 1/(2700.0 - 1510.0);
#else
// dummy variables
const uint16_t tactile_shift = 1;
const float tactile_scale = 1.0;
#endif

const uint16_t n_frames = 4000;   // number of frames to collect
                                  // divide by 100 to get recording time in sec.
static uint16_t frame_count = 0;

/*float accData[3];
float magData[3];
float gyroData[3];*/

int16_t accData[3];
int16_t magData[3];
int16_t gyroData[3];

//uint16_t pre_tick;
//uint16_t post_tick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
static void BSP_SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command);
int _write(int fd, char *buff, int count);
static void Send_Frame(uint16_t *pBuffer, uint32_t size);
static void Send_Frame_float(uint16_t *pBuffer, uint32_t size);
static void Send_MPU_Data(int16_t *pBuffer);
static void Send_MPU_Data_float(float *pBuffer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  BSP_SDRAM_Initialization_Sequence(&hsdram1, &command);
  // Start communication with IMU
  MPU9250_Init();
  // Enable the reference voltage for the readout circuit
  En_ReadoutCircuit();
  HAL_TIM_Base_Start(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (collect_data)
    {
      #ifdef TEST_THRESH
      HAL_GPIO_WritePin(GPIOJ, LED_RED_Pin, GPIO_PIN_RESET);
      #endif
      //__HAL_TIM_SET_COUNTER(&htim6, 0);
      //pre_tick = __HAL_TIM_GET_COUNTER(&htim6);
      //status = MPU9250_GetData_Phys(accData, magData, gyroData);
      status = MPU9250_GetData_Dig(accData, magData, gyroData);
      if (status != HAL_OK)
      {
        Error_Handler();
      }
      printf("%u\t", (frame_count+1)*10);
      Send_MPU_Data(accData);
      Send_MPU_Data(gyroData);
      printf("\n");
      //post_tick = __HAL_TIM_GET_COUNTER(&htim6);
      //printf("Acquiring and sending IMU data took %u microseconds.\n", post_tick - pre_tick);

      __HAL_TIM_SET_COUNTER(&htim6, 0);
      //pre_tick = __HAL_TIM_GET_COUNTER(&htim6);
      status = Read_Frame(tactileData);
      //post_tick = __HAL_TIM_GET_COUNTER(&htim6);
      //printf("Acquiring frame took %u microseconds.\n", post_tick - pre_tick);
      #ifdef TEST_THRESH
      for (int i=0; i<DATA_SIZE; i++)
      {
        if (tactileData[i] > tactileThreshold[i])
        {
          check_thresh = true;
          break;
        }
      }
      if (check_thresh)
      {
        HAL_GPIO_WritePin(GPIOJ, LED_RED_Pin, GPIO_PIN_SET);
        check_thresh = false;
      }
      #endif
      if (status != HAL_OK)
      {
        Error_Handler();
      }

      //pre_tick = __HAL_TIM_GET_COUNTER(&htim6);
      status = HAL_SDRAM_Write_16b(&hsdram1, (uint32_t *)(SDRAM_BANK_ADDR + frame_count*SDRAM_FRAME_OFFSET), tactileData, DATA_SIZE);
      //post_tick = __HAL_TIM_GET_COUNTER(&htim6);
      //printf("Moving frame to SDRAM took %u microseconds.\n", post_tick - pre_tick);
      if (status != HAL_OK)
      {
        Error_Handler();
      }
      frame_count++;
      collect_data = false;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (collect_done)
    {
      HAL_TIM_Base_Stop_IT(&htim7);
      collect_done = false;
      //printf("Collected %u frames.\n", frame_count);
      for (int i=0; i<frame_count; i++)
      {
        status = HAL_SDRAM_Read_16b(&hsdram1, (uint32_t *)(SDRAM_BANK_ADDR + i*SDRAM_FRAME_OFFSET), tactileData, DATA_SIZE);   
        #ifdef FLOAT_Pressure
        Send_Frame_float(tactileData, DATA_SIZE);
        #else
        Send_Frame(tactileData, DATA_SIZE);
        #endif
        if ((i % 500) == 0) // to make sure the receiving buffer doesn't overflow
        {
          HAL_Delay(1);
        }
      }
      frame_count = 0;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00301739;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 719;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 6;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 6;
  SdramTiming.WriteRecoveryTime = 2;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, LED_RED_Pin|Dmux_EN_Pin|LED_GREEN_Pin|Dmux_sel3_Pin 
                          |Dmux_sel0_Pin|Mux_sel2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Dmux_sel1_Pin|Mux_sel0_Pin|Mux_sel1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Dmux_sel2_Pin|Mux_sel3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(P_EN_GPIO_Port, P_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin Dmux_EN_Pin LED_GREEN_Pin Dmux_sel3_Pin 
                           Dmux_sel0_Pin Mux_sel2_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|Dmux_EN_Pin|LED_GREEN_Pin|Dmux_sel3_Pin 
                          |Dmux_sel0_Pin|Mux_sel2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : Dmux_sel1_Pin Mux_sel0_Pin Mux_sel1_Pin */
  GPIO_InitStruct.Pin = Dmux_sel1_Pin|Mux_sel0_Pin|Mux_sel1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Dmux_sel2_Pin Mux_sel3_Pin */
  GPIO_InitStruct.Pin = Dmux_sel2_Pin|Mux_sel3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : P_EN_Pin */
  GPIO_InitStruct.Pin = P_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(P_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Perform the SDRAM external memory initialization sequence
  * @param  hsdram: SDRAM handle
  * @param  Command: Pointer to SDRAM command structure
  * @retval None
  */
static void BSP_SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
  __IO uint32_t tmpmrd =0;
  /* Step 3:  Configure a clock configuration enable command */
  Command->CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

  /* Step 4: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);

  /* Step 5: Configure a PALL (precharge all) command */
  Command->CommandMode = FMC_SDRAM_CMD_PALL;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

  /* Step 6 : Configure a Auto-Refresh command */
  Command->CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 8;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

  /* Step 7: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_2           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command->CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 1;
  Command->ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, SDRAM_TIMEOUT);

  /* Step 8: Set the refresh rate counter */
  /* (15.62 us x Freq) - 20 */
  /* Set the device refresh counter */
  hsdram->Instance->SDRTR |= ((uint32_t)((1292)<< 1));
}

// Retarget stdio
int _write(int fd, char *buff, int count)
{
  if ((count < 0) && (fd != STDOUT_FILENO) && (fd != STDERR_FILENO)) {
      errno = EBADF;
      return -1;
  }

  status = HAL_UART_Transmit(&huart1, (uint8_t *)buff, count, HAL_MAX_DELAY);

  return (status == HAL_OK ? count : 0);
}

// Timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim7.Instance)
  {
    if (n_frames == frame_count)
    {
      collect_done = true;
      HAL_GPIO_WritePin(GPIOJ, LED_GREEN_Pin, GPIO_PIN_RESET);
    }
    else
    {
      collect_data = true;
    }
  }
}

// User button interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Toggle data collection
  //collect_data = !collect_data;
  printf("Timestamp [ms]\tAcc_x [g]\tAcc_y [g]\tAcc_z [g]\tGyro_x [deg/s]\tGyro_y [deg/s]\tGyro_z [deg/s]\t\n");
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_GPIO_WritePin(GPIOJ, LED_GREEN_Pin, GPIO_PIN_SET);
}

void Send_Frame(uint16_t *pBuffer, uint32_t size)
{
  for (int i=0; i<size; i++)
  {
    printf("%u\t", pBuffer[i]);
  }
  printf("\n");
}

void Send_Frame_float(uint16_t *pBuffer, uint32_t size)
{
  float tmp_;
  for (int i=0; i<size; i++)
  {
    tmp_ = ((float)(pBuffer[i] - tactile_shift))*tactile_scale;
    tmp_ = AI_CLAMP(tmp_, 0.0, 1.0, float); 
    printf("%f\t", tmp_);
  }
  printf("\n");
}

void Send_MPU_Data(int16_t *pBuffer)
{
  for (int i=0; i<3; i++)
  {
    printf("%i\t", pBuffer[i]);
  }
}

void Send_MPU_Data_float(float *pBuffer)
{
  for (int i=0; i<3; i++)
  {
    printf("%f\t", pBuffer[i]);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  printf("HAL status is %d.\n", (int) status);
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOJ, LED_RED_Pin);
    HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
