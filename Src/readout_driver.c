#include "readout_driver.h"

// declare global variables
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim6;
uint16_t adc_ch4_val;
uint16_t adc_ch6_val;
uint8_t dmux_sel = 0;
uint8_t mux_sel = 0;
uint16_t tick;
HAL_StatusTypeDef status;


/**
  * @brief  sets the power enable pin for the reference voltage in the readout circuit.
  */
void En_ReadoutCircuit()
{
  HAL_GPIO_WritePin(P_EN_GPIO_PORT, P_EN_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
}

/**
  * @brief  resets the power enable pin for the reference voltage in the readout circuit.
  */
void Dis_ReadoutCircuit()
{
  HAL_GPIO_WritePin(P_EN_GPIO_PORT, P_EN_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  reads out one entire frame into the pDstBuffer.
  * @param  pDstBuffer Pointer to destination buffer that must be of size 1024.
  * @retval HAL status
  */
HAL_StatusTypeDef Read_Frame(uint16_t *pDstBuffer)
{
	// Reset all GPIO pins
  HAL_GPIO_WritePin(GPIOJ, Dmux_EN_Pin|Dmux_sel3_Pin|Dmux_sel0_Pin
                          |Mux_sel2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, Dmux_sel1_Pin|Mux_sel0_Pin|Mux_sel1_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOF, Dmux_sel2_Pin|Mux_sel3_Pin, GPIO_PIN_RESET);

  dmux_sel = 0;

  // Acquire one entire frame
  for (int i=0; i<32; i++)
  {
    if (i == 16)
    {
      // Enable the second demultiplexer and disable the first
      HAL_GPIO_WritePin(GPIOJ, Dmux_EN_Pin, GPIO_PIN_SET);
      // Reset the selection variable
      dmux_sel = 0;
    }
    // Select row 
    // Use bit masking to find out which pins need to be set to high
    HAL_GPIO_WritePin(GPIOJ, Dmux_sel0_Pin, ((dmux_sel >> 0) & 1));
    HAL_GPIO_WritePin(GPIOC, Dmux_sel1_Pin, ((dmux_sel >> 1) & 1));
    HAL_GPIO_WritePin(GPIOF, Dmux_sel2_Pin, ((dmux_sel >> 2) & 1));
    HAL_GPIO_WritePin(GPIOJ, Dmux_sel3_Pin, ((dmux_sel >> 3) & 1));
    // Increment selection variable
    dmux_sel += 1;

    // Raster through all 32 columns (16 per ADC channel)
    mux_sel = 0;
    for (int j=0; j<16; j++)
    {
      // Use bit masking to find out which pins need to be set to high
      // But I'm hecking stupid and switched the order of the selection bits
      HAL_GPIO_WritePin(GPIOC, Mux_sel0_Pin, ((mux_sel >> 3) & 1));
      HAL_GPIO_WritePin(GPIOC, Mux_sel1_Pin, ((mux_sel >> 2) & 1));
      HAL_GPIO_WritePin(GPIOJ, Mux_sel2_Pin, ((mux_sel >> 1) & 1));
      HAL_GPIO_WritePin(GPIOF, Mux_sel3_Pin, ((mux_sel >> 0) & 1));
      // Increment selection variable
      mux_sel += 1;

      //Maybe no delay needed, I don't know how fast the reference voltage will be available
      //HAL_Delay(1);
      // Use timer to create a 5 microsecond delay
      tick = __HAL_TIM_GET_COUNTER(&htim6);
      while (__HAL_TIM_GET_COUNTER(&htim6) < (tick + 5))
      {
        // Wait for the specified delay
      }

      // Read the two channels
      status = HAL_ADC_Start(&hadc1);
      if (status != HAL_OK) return status;
      status = HAL_ADC_PollForConversion(&hadc1, 1000);
      if (status != HAL_OK) return status;
      adc_ch4_val = HAL_ADC_GetValue(&hadc1);

      status = HAL_ADC_Start(&hadc1);
      if (status != HAL_OK) return status;
      status = HAL_ADC_PollForConversion(&hadc1, 1000);
      if (status != HAL_OK) return status;
      adc_ch6_val = HAL_ADC_GetValue(&hadc1);
      // Write ADC values into the destination buffer
      pDstBuffer[i*32 + j] = adc_ch4_val;
      pDstBuffer[i*32 + j + 16] = adc_ch6_val;
    }
  }
  return HAL_OK;
}