/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dsox_reg.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BOOT_TIME 10 //ms

/* Self Test Limits */
#define MIN_ST_LIMIT_mg 50.0f
#define MAX_ST_LIMIT_mg 1700.0F
#define MIN_ST_LIMIT_mdps 150000.0f
#define MAX_ST_LIMIT_mdps 700000.0f

/* Self Test Results*/
#define ST_PASS 1U 
#define ST_FAIL 0U
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];

lsm6dsox_all_sources_t all_source;
stmdev_ctx_t dev_ctx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
void lsm6dsox_self_test(void);
void lsm6dsox_read_data_polling(void);
void lsm6dsox_orientation(void);
void lsm6dsox_setup();
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_Delay(100);
  //lsm6dsox_self_test();//Sensor Test 
  //lsm6dsox_read_data_polling();
  //lsm6dsox_orientation();
  lsm6dsox_setup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(200);
    //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);//LED
    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);//External LED
    //HAL_Delay(3000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  ISR for INT2 from LMSD6DSOX
 * 
 * @param GPIO_Pin Pin that will be checked upon entering the ISR
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == LSM6DSOX_INT2_Pin)
  {
    /* Check if 6D/4D Orientation events. */
    lsm6dsox_all_sources_get(&dev_ctx, &all_source);

    //Check the Up or Down orientation 
    if (all_source.six_d_zh) {
      sprintf((char *)tx_buffer, "ZH\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }

    if (all_source.six_d_zl) {
      sprintf((char *)tx_buffer, "ZL\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
}

/**
 * @brief Self test Function for lsm6dsox
 */
void lsm6dsox_self_test(void)
{
  uint8_t tx_buffer[1000];
  stmdev_ctx_t dev_ctx;
  float val_st_off[3];
  int16_t data_raw[3];
  float val_st_on[3];
  float test_val[3];
  uint8_t st_result;
  uint8_t whoamI;
  uint8_t drdy;
  uint8_t rst;
  uint8_t i;
  uint8_t j;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hi2c1;

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);//TODO:Change this to just be a direct call to delay() function

  /* Check device ID */
  lsm6dsox_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSOX_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);
  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Accelerometer Self Test
   */
  /* Set Output Data Rate */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_52Hz);
  /* Set full scale */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_4g);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsox_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsox_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsox_from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dsox_xl_self_test_set(&dev_ctx, LSM6DSOX_XL_ST_NEGATIVE);
  //lsm6dsox_xl_self_test_set(&dev_ctx, LSM6DSOX_XL_ST_POSITIVE);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsox_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));
  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsox_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsox_from_fs4_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  st_result = ST_PASS;

  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mg > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mg)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dsox_xl_self_test_set(&dev_ctx, LSM6DSOX_XL_ST_DISABLE);
  /* Disable sensor. */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_OFF);
  /*
   * Gyroscope Self Test
   */
  /* Set Output Data Rate */
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_208Hz);
  /* Set full scale */
  lsm6dsox_gy_full_scale_set(&dev_ctx, LSM6DSOX_2000dps);
  /* Wait stable output */
  platform_delay(100);

  /* Check if new value available */
  do {
    lsm6dsox_gy_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  lsm6dsox_angular_rate_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsox_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsox_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsox_from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  lsm6dsox_gy_self_test_set(&dev_ctx, LSM6DSOX_GY_ST_POSITIVE);
  //lsm6dsox_gy_self_test_set(&dev_ctx, LIS2DH12_GY_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(100);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm6dsox_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    lsm6dsox_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsox_from_fs2000_to_mdps(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mdps > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mdps)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  lsm6dsox_gy_self_test_set(&dev_ctx, LSM6DSOX_GY_ST_DISABLE);
  /* Disable sensor. */
  lsm6dsox_gy_data_rate_set(&dev_ctx, LSM6DSOX_GY_ODR_OFF);

  if (st_result == ST_PASS) {
    sprintf((char *)tx_buffer, "Self Test - PASS\r\n" );
  }

  else {
    sprintf((char *)tx_buffer, "Self Test - FAIL\r\n" );
  }

  tx_com(tx_buffer, strlen((char const *)tx_buffer));
}

/**
 * @brief  Read accelerometer data from lsm6dsox in Polling mode
 * 
 */
void lsm6dsox_read_data_polling(void)
{
  stmdev_ctx_t dev_ctx;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hi2c1;
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  lsm6dsox_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSOX_ID)
    while (1);

  /* Restore default configuration */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);
  //TODO :Disable the gyro here too
  /* Enable Block Data Update */
  lsm6dsox_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_12Hz5);
  /* Set full scale */
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_2g);
  /* Configure filtering chain(No aux interface)
   * Accelerometer - LPF1 + LPF2 path
   */
  lsm6dsox_xl_hp_path_on_out_set(&dev_ctx, LSM6DSOX_LP_ODR_DIV_100);
  lsm6dsox_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

  /* Read samples in polling mode (no int) */
  while (1) {
    uint8_t reg;
    /* Read output only if new xl value is available */
    lsm6dsox_xl_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read acceleration field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6dsox_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] =
        lsm6dsox_from_fs2_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
        lsm6dsox_from_fs2_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
        lsm6dsox_from_fs2_to_mg(data_raw_acceleration[2]);
      sprintf((char *)tx_buffer,
              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
}

/**
 * @brief LSM6DSOX Sensor setup 
 * 
 */
void lsm6dsox_setup()
{
  //Configure INT2
  lsm6dsox_pin_int2_route_t int2_route;

  //Initialize mems driver interface
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hi2c1;

  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);

  /* Check device ID. */
  lsm6dsox_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DSOX_ID)
  {
    while (1);
    //TODO:Implement better handling 
  }

  /* Restore default configuration. */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);
  do {
    lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface. */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);
  /* Set XL Output Data Rate to 417 Hz. */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_417Hz);
  /* Set 2g full XL scale.*/
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_2g);
  /* Set threshold to 60 degrees. */
  lsm6dsox_6d_threshold_set(&dev_ctx, LSM6DSOX_DEG_60);
  /* LPF2 on 6D/4D function selection. */
  lsm6dsox_xl_lp2_on_6d_set(&dev_ctx, PROPERTY_ENABLE);

  /* Uncomment if interrupt generation on Free Fall INT2 pin */
  lsm6dsox_pin_int2_route_get(&dev_ctx, NULL, &int2_route);
  int2_route.six_d = PROPERTY_ENABLE;
  lsm6dsox_pin_int2_route_set(&dev_ctx, NULL, int2_route);

  //sprintf((char *) tx_buffer, "Setup complete!\r\n");//TLIFT
  //tx_com(tx_buffer, strlen((char const *)tx_buffer));//TLIFT
}

/**
 * @brief Get orientation using Interrupt 2 
 * 
 */
void lsm6dsox_orientation(void)
{
  stmdev_ctx_t dev_ctx;
  /* Uncomment to configure INT 1 */
  //lsm6dsox_pin_int1_route_t int1_route;
  lsm6dsox_pin_int2_route_t int2_route;
  /* Initialize mems driver interface. */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hi2c1;
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID. */
  lsm6dsox_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSOX_ID)
    while (1);

  /* Restore default configuration. */
  lsm6dsox_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsox_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Disable I3C interface. */
  lsm6dsox_i3c_disable_set(&dev_ctx, LSM6DSOX_I3C_DISABLE);
  /* Set XL Output Data Rate to 417 Hz. */
  lsm6dsox_xl_data_rate_set(&dev_ctx, LSM6DSOX_XL_ODR_417Hz);
  /* Set 2g full XL scale.*/
  lsm6dsox_xl_full_scale_set(&dev_ctx, LSM6DSOX_2g);
  /* Set threshold to 60 degrees. */
  lsm6dsox_6d_threshold_set(&dev_ctx, LSM6DSOX_DEG_60);
  /* LPF2 on 6D/4D function selection. */
  lsm6dsox_xl_lp2_on_6d_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * To enable 4D mode uncomment next line.
   * 4D orientation detection disable Z-axis events.
   */
  //lsm6dsox_4d_mode_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Uncomment if interrupt generation on Free Fall INT1 pin
   */
  //lsm6dsox_pin_int1_route_get(&dev_ctx, &int1_route);
  //int1_route.reg.md1_cfg.int1_ff = PROPERTY_ENABLE;
  //lsm6dsox_pin_int1_route_set(&dev_ctx, &int1_route);

  /* Uncomment if interrupt generation on Free Fall INT2 pin */
  lsm6dsox_pin_int2_route_get(&dev_ctx, NULL, &int2_route);
  int2_route.free_fall = PROPERTY_ENABLE;
  lsm6dsox_pin_int2_route_set(&dev_ctx, NULL, int2_route);

  /* Wait Events. */
  while (1) {
    lsm6dsox_all_sources_t all_source;
    //Check if 6D/4D Orientation events.
    lsm6dsox_all_sources_get(&dev_ctx, &all_source);

    if (all_source.six_d) {
      sprintf((char *)tx_buffer, "6D Or. switched to ");

      if (all_source.six_d_xh) {
        strcat((char *)tx_buffer, "XH");
      }

      if (all_source.six_d_xl) {
        strcat((char *)tx_buffer, "XL");
      }

      if (all_source.six_d_yh) {
        strcat((char *)tx_buffer, "YH");
      }

      if (all_source.six_d_yl) {
        strcat((char *)tx_buffer, "YL");
      }

      if (all_source.six_d_zh) {
        strcat((char *)tx_buffer, "ZH");
      }

      if (all_source.six_d_zl) {
        strcat((char *)tx_buffer, "ZL");
      }

      strcat((char *)tx_buffer, "\r\n");
      tx_com(tx_buffer, strlen((char const *)tx_buffer));
    }
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    Sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,uint16_t len)
{
  //HAL_I2C_Mem_Write(handle, LSM6DSOX_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  HAL_I2C_Mem_Write(handle,LSM6DSOX_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp,len,1000);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    Sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read(handle, LSM6DSOX_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*
 * @brief  Delay in Milliseconds
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

/**
 * @brief Transmit over serial (UART)
 *
 * @param tx_buffer
 * @param len
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
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
  __disable_irq();
  while (1)
  {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
