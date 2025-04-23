/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  *   Questi i file che ho copiato dalla LoRaMac-node-5.0.0-branch e che ho poi
  *   modificato ed adattato.
  *
  *    lr_fhss_v1_base_types.h
  *    ral_defs.h
  *    ral_sx127x.h
  *    ral_drv.h
  *    ral.h
  *    sx127x_regs_common_defs.h
  *    sx127x_regs_gfsk_defs.h
  *    sx127x_regs_lora_defs.h
  *    sx127x.h
  *    sx127x_hal.h
  *    ral_sx127x_bsp.h
  *    loramac_radio.h
  *    radio_board.h
  *    ral_sx127x.c
  *    sx127x.c
  *    loramac_radio.c
  *    ral_sx127x_bsp.c
  *    sx127x_hal.c
  *    radio_board.c
  *  ed ora tutti questi file sono nelle cartelle di questo progetto.
  *
  *
  *  Ed anche questo ha funzionato sia in lora che in fsk.
  *  NB spesso nello slave arriva un PONG cosa che in teoria il master non strasmette.
  *     o casini nel codice o nell'hardware... sarebbe da indagare meglio !
  *
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "myvarie.h"


//REM AP #include "board.h"  queesto non la metto
//REM AP #include "../../mac/loramac_radio.h"
// ed ho dovuto aggiungere negli include path la cartella mac ed anche board.
// per i .c ho dovuto aggiungere nei "path and symbol" "source location"
// ed inoltre nei symbol ho dovuto mettere
#include "loramac_radio.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

// sarebbe il main scopiazzato dal ping-pong di esempio fornito da semtech
/*!
 * \brief Application states definition
 */
typedef enum app_states_e
{
    APP_STATE_LOW_POWER,
    APP_STATE_RX,
    APP_STATE_RX_TIMEOUT,
    APP_STATE_RX_ERROR,
    APP_STATE_TX,
    APP_STATE_TX_TIMEOUT,
} app_states_t;

/*!
 * \brief Application context definition
 */
typedef struct app_context_s
{
    app_states_t state;
    bool         is_master;
    uint16_t     buffer_size_in_bytes;
    uint8_t*     buffer;
} app_context_t;


#define  USE_MODEM_LORA
//#define  USE_MODEM_FSK

#define RX_TIMEOUT_VALUE                            1000

/*!
 * \brief RF frequency
 */

#define RF_FREQ_IN_HZ                               434000000


/*!
 * \brief RF transmission output power
 */
#define TX_RF_PWR_IN_DBM                            14


#if defined( USE_MODEM_LORA )

/*!
 * \brief LoRa modulation spreading factor
 */
#define LORA_SF                                     RAL_LORA_SF7

/*!
 * \brief LoRa modulation bandwidth
 */
#define LORA_BW                                     RAL_LORA_BW_125_KHZ

/*!
 * \brief LoRa modulation coding rate
 */
#define LORA_CR                                     RAL_LORA_CR_4_5

/*!
 * \brief LoRa preamble length
 */
#define LORA_PREAMBLE_LEN_IN_SYMB                   8

/*!
 * \brief LoRa is packet length fixed or variable
 */
#define LORA_IS_PKT_LEN_FIXED                       false
// #define LORA_IS_PKT_LEN_FIXED                       true

/*!
 * \brief LoRa is packet crc on or off
 */
#define LORA_IS_CRC_ON                              true

/*!
 * \brief LoRa is IQ inversion on or off
 */
#define LORA_IS_INVERT_IQ_ON                        false

/*!
 * \brief LoRa rx synchronization timeout
 */
#define LORA_RX_SYNC_TIMEOUT_IN_SYMB                6

/*!
 * \brief LoRa tx timeout
 */
#define LORA_TX_TIMEOUT_IN_MS                       4000

#elif defined( USE_MODEM_FSK )

/*!
 * \brief GFSK bitrate
 */
//#define GFSK_BR_IN_BPS                              50000  non andava a 50K
//                                                    a 9600 cosi si sente nelle baofeng...
#define GFSK_BR_IN_BPS                                4800

/*!
 * \brief GFSK frequency deviation
 */
//#define GFSK_FDEV_IN_HZ                             25000
#define GFSK_FDEV_IN_HZ                             10000


/*!
 * \brief GFSK bandwidth double sided
 */
//#define GFSK_BW_DSB_IN_HZ                           100000
#define GFSK_BW_DSB_IN_HZ                           40000


/*!
 * \brief GFSK preable length
 */
#define GFSK_PREABLE_LEN_IN_BITS                    40

/*!
 * \brief GFSK sync word length
 */
#define GFSK_SYNC_WORD_LEN_IN_BITS                  24

/*!
 * \brief GFSK is packet length fixed or variable
 */
#define GFSK_IS_PKT_LEN_FIXED                       false

/*!
 * \brief GFSK is packet crc on or off
 */
#define GFSK_IS_CRC_ON                              true

/*!
 * \brief GFSK rx synchronization timeout
 */
#define GFSK_RX_SYNC_TIMEOUT_IN_SYMB                6

/*!
 * \brief GFSK tx timeout
 */
#define GFSK_TX_TIMEOUT_IN_MS                       4000

#else
#error "Please select a modem under compiler options."
#endif

/*!
 * \brief Maximum application data buffer size
 *
 * \remark Please do not change this value
 */
#define PING_PONG_APP_DATA_MAX_SIZE                 255

/*!
 * \brief Application payload length
 *
 * \remark Please change this value in order to test different payload lengths
 */
#define APP_PLD_LEN_IN_BYTES                        16


// clang-format on

/*!
 * \brief Ping message string
 */
const uint8_t app_ping_msg[] = "PING";

/*!
 * \brief Pong message string
 */
const uint8_t app_pong_msg[] = "PONG";


/*!
 * \brief Application data buffer
 */
static uint8_t app_data_buffer[PING_PONG_APP_DATA_MAX_SIZE];

/*!
 * \brief Application context
 */
static app_context_t app_context = {
    .state                = APP_STATE_LOW_POWER,

    .is_master            = true,
	//.is_master            = false,

    .buffer_size_in_bytes = APP_PLD_LEN_IN_BYTES,
    .buffer               = app_data_buffer,
};

/*!
 * \brief Radio interrupt callbacks
 */
static loramac_radio_irq_t Myradio_irq_callbacks;

/*!
 * \brief Tx done interrupt callback
 */
static void irq_tx_done( void );

/*!
 * \brief Rx done interrupt callback
 *
 * \param [out] params  Pointer to the received parameters
 */
static void irq_rx_done( loramac_radio_irq_rx_done_params_t* params );

/*!
 * \brief Rx error interrupt callback
 */
static void irq_rx_error( void );

/*!
 * \brief Tx timeout interrupt callback
 */
static void irq_tx_timeout( void );

/*!
 * \brief Rx timeout interrupt callback
 */
static void irq_rx_timeout( void );

/*!
 * \brief Print the provided buffer in HEX
 *
 * \param [in] buffer Buffer to be printed
 * \param [in] size   Buffer size to be printed
 */
static void print_hex_buffer( uint8_t* buffer, uint8_t size );

/*!
 * \brief Build message to be transmitted
 *
 * \param [in]  is_ping_msg   Indicate if it is a PING or PONG message
 * \param [out] buffer        Buffer to be filled
 * \param [in]  size_in_bytes Buffer size to be filled
 */
static void app_build_message( bool is_ping_msg, uint8_t* buffer, uint8_t size_in_bytes );


static void irq_tx_done( void )
{
    loramac_radio_set_sleep( );
    app_context.state = APP_STATE_TX;
}

static void irq_rx_done( loramac_radio_irq_rx_done_params_t* params )
{
    loramac_radio_set_sleep( );

    memcpy( app_context.buffer, params->buffer, params->size_in_bytes );

    app_context.buffer_size_in_bytes = params->size_in_bytes;

//REM AP
/*---
#if defined( USE_MODEM_LORA )
    printf( "[IRQ] rx done rssi: %4d dBm, snr: %4d dB\n", params->rssi_in_dbm, params->snr_in_db );
#elif defined( USE_MODEM_FSK )
    printf( "[IRQ] rx done rssi: %4d dBm\n", params->rssi_in_dbm );
#endif
--- */

    if( app_context.buffer[1] == 'I' )
    {
        //REM AP printf( "P I N G " );
        print_hex_buffer( app_context.buffer + 4, params->size_in_bytes );
    }
    else if( app_context.buffer[1] == 'O' )
    {
    	//REM AP printf( "P O N G " );
        print_hex_buffer( app_context.buffer + 4, params->size_in_bytes );
    }
    else
    {
        print_hex_buffer( app_context.buffer, params->size_in_bytes );
    }
    app_context.state = APP_STATE_RX;
}

static void irq_rx_error( void )
{
	printf("irq_rx_error\n");
    loramac_radio_set_sleep( );
    app_context.state = APP_STATE_RX_ERROR;
}

static void irq_tx_timeout( void )
{
	printf("irq_tx_timeout\n");
    loramac_radio_set_sleep( );
    app_context.state = APP_STATE_TX_TIMEOUT;
}

static void irq_rx_timeout( void )
{
	printf("irq_rx_timeout\n");
    loramac_radio_set_sleep( );
    app_context.state = APP_STATE_RX_TIMEOUT;
}

static void print_hex_buffer( uint8_t* buffer, uint8_t size )
{
	//REM AP per ora no !
	return;

    uint8_t newline = 0;

    for( uint8_t i = 0; i < size; i++ )
    {
        if( newline != 0 )
        {
        	//REM AP printf( "\n" );
            newline = 0;
        }
        //REM AP printf( "%02X ", buffer[i] );
        if( ( ( i + 1 ) % 16 ) == 0 )
        {
            newline = 1;
        }
    }
    //REM AP printf( "\n" );
}




static void app_build_message( bool is_ping_msg, uint8_t* buffer, uint8_t size_in_bytes )
{
    uint8_t app_msg_size;

    if( is_ping_msg == true )
    {
        app_msg_size = sizeof( app_ping_msg );
        memcpy( buffer, app_ping_msg, app_msg_size );
    }
    else
    {
        app_msg_size = sizeof( app_pong_msg );
        memcpy( buffer, app_pong_msg, app_msg_size );
    }
    // Fill remaining buffer bytes
    for( uint8_t i = app_msg_size; i < size_in_bytes; i++ )
    {
        buffer[i] = i - app_msg_size;
    }
}





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Target board initialization
  //REM AP BoardInitMcu( );
  //REM AP BoardInitPeriph( );

  printf( "MyPrintf\n" );  // vedi in myvarie la gestione...

  // Radio initialization
  Myradio_irq_callbacks.loramac_radio_irq_tx_done    = irq_tx_done;
  Myradio_irq_callbacks.loramac_radio_irq_rx_done    = irq_rx_done;
  Myradio_irq_callbacks.loramac_radio_irq_rx_error   = irq_rx_error;
  Myradio_irq_callbacks.loramac_radio_irq_tx_timeout = irq_tx_timeout;
  Myradio_irq_callbacks.loramac_radio_irq_rx_timeout = irq_rx_timeout;

  printf( "radio_init\n" );
  loramac_radio_init( &Myradio_irq_callbacks );

#if defined( USE_MODEM_LORA )
  loramac_radio_lora_cfg_params_t lora_params = {
      .rf_freq_in_hz           = RF_FREQ_IN_HZ,
      .tx_rf_pwr_in_dbm        = TX_RF_PWR_IN_DBM,
      .sf                      = LORA_SF,
      .bw                      = LORA_BW,
      .cr                      = LORA_CR,
      .preamble_len_in_symb    = LORA_PREAMBLE_LEN_IN_SYMB,
      .is_pkt_len_fixed        = LORA_IS_PKT_LEN_FIXED,
      .pld_len_in_bytes        = APP_PLD_LEN_IN_BYTES,
      .is_crc_on               = LORA_IS_CRC_ON,
      .invert_iq_is_on         = LORA_IS_INVERT_IQ_ON,
      .rx_sync_timeout_in_symb = LORA_RX_SYNC_TIMEOUT_IN_SYMB,
      .is_rx_continuous        = true,
      .tx_timeout_in_ms        = LORA_TX_TIMEOUT_IN_MS,
  };
  printf( "radio_lora_set_cfg\n" );
  loramac_radio_lora_set_cfg( &lora_params );
#elif defined( USE_MODEM_FSK )
  loramac_radio_gfsk_cfg_params_t gfsk_params = {
      .rf_freq_in_hz           = RF_FREQ_IN_HZ,
      .tx_rf_pwr_in_dbm        = TX_RF_PWR_IN_DBM,
      .br_in_bps               = GFSK_BR_IN_BPS,
      .fdev_in_hz              = GFSK_FDEV_IN_HZ,
      .bw_dsb_in_hz            = GFSK_BW_DSB_IN_HZ,
      .preamble_len_in_bits    = GFSK_PREABLE_LEN_IN_BITS,
      .sync_word_len_in_bits   = GFSK_SYNC_WORD_LEN_IN_BITS,
      .is_pkt_len_fixed        = GFSK_IS_PKT_LEN_FIXED,
      .pld_len_in_bytes        = APP_PLD_LEN_IN_BYTES,
      .is_crc_on               = GFSK_IS_CRC_ON,
      .rx_sync_timeout_in_symb = GFSK_RX_SYNC_TIMEOUT_IN_SYMB,
      .is_rx_continuous        = true,
      .tx_timeout_in_ms        = GFSK_TX_TIMEOUT_IN_MS,
  };
  printf( "radio_gfsk_set_cfg\n" );
  loramac_radio_gfsk_set_cfg( &gfsk_params );
#endif

  //REM AP loramac_radio_set_rx( RAL_RX_TIMEOUT_CONTINUOUS_MODE );
  printf( "radio_set_rx\n" );
  loramac_radio_set_rx( RX_TIMEOUT_VALUE );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // la mia gestione del timeout ! e del irq sul D0
	  // e qui simulo gli irq sul DIO0  ma servirebbero pure gli altri 2 mi sa...
	  if ( IsMyDIO0_ON() )
	  {
		  MyOnDio0Irq();
	  }
	  else MyUpdTimer();

	  loramac_radio_irq_process( );

	  switch( app_context.state )
      {
      case APP_STATE_RX:
          if( app_context.is_master == true )
          {
              if( app_context.buffer_size_in_bytes > 0 )
              {
                  if( strncmp( ( const char* ) app_context.buffer, ( const char* ) app_pong_msg, 4 ) == 0 )
                  {
                	  //REM AP printf( "[APP] pong message received\n" );
                      // Send the next PING frame
                      app_build_message( true, app_context.buffer, app_context.buffer_size_in_bytes );
                      DelayMs( 1 );
                      //REM AP printf( "[APP] ping message transmission\n" );
                      printf( "radio_trasmit ping\n" );
                      loramac_radio_transmit( app_context.buffer, app_context.buffer_size_in_bytes );
                  }
                  else if( strncmp( ( const char* ) app_context.buffer, ( const char* ) app_ping_msg, 4 ) == 0 )
                  {  // A master already exists then become a slave
                      app_context.is_master = true;
                      //REM AP printf( "[APP] ping-pong slave mode\n" );

                      //REM AP loramac_radio_set_rx( RAL_RX_TIMEOUT_CONTINUOUS_MODE );
                      printf( "radio_set_rx\n" );
                      loramac_radio_set_rx( RX_TIMEOUT_VALUE );
                  }
                  else  // valid reception but neither a PING or a PONG message
                  {     // Set device as master ans start again
                      app_context.is_master = true;

                      //REM AP printf( "[APP] ping-pong master mode\n" );

                      //REM AP loramac_radio_set_rx( RAL_RX_TIMEOUT_CONTINUOUS_MODE );
                      printf( "radio_set_rx\n" );
                      loramac_radio_set_rx( RX_TIMEOUT_VALUE );

                  }
              }
          }
          else
          {
              if( app_context.buffer_size_in_bytes > 0 )
              {
                  if( strncmp( ( const char* ) app_context.buffer, ( const char* ) app_ping_msg, 4 ) == 0 )
                  {
                	  //REM AP printf( "[APP] ping message received\n" );
                      // Send the reply to the PONG string
                      app_build_message( false, app_context.buffer, app_context.buffer_size_in_bytes );
                      DelayMs( 1 );
                      //REM AP printf( "[APP] pong message transmission\n" );
                      printf( "radio_trasmit pong\n" );
                      loramac_radio_transmit( app_context.buffer, app_context.buffer_size_in_bytes );
                  }
                  else  // valid reception but not a PING as expected
                  {     // Set device as master and start again
                	  //REM AP app_context.is_master = true;
                      //REM AP printf( "[APP] ping-pong master mode\n" );

                      //REM AP loramac_radio_set_rx( RAL_RX_TIMEOUT_CONTINUOUS_MODE );
                	  printf( "radio_set_rx\n" );
                      loramac_radio_set_rx( RX_TIMEOUT_VALUE );

                  }
              }
          }
          app_context.state = APP_STATE_LOW_POWER;
          break;
      case APP_STATE_TX:

          //REM AP loramac_radio_set_rx( RAL_RX_TIMEOUT_CONTINUOUS_MODE );
    	  printf( "radio_set_rx\n" );
          loramac_radio_set_rx( RX_TIMEOUT_VALUE );


          //REM AP if( app_context.is_master == true )
          //REM AP {
        	         //REM AP printf( "[APP] ping message transmitted\n" );
          //REM AP }
          //REM AP else
          //REM AP {
        	        //REM AP printf( "[APP] pong message transmitted\n" );
          //REM AP }
          app_context.state = APP_STATE_LOW_POWER;
          break;
      case APP_STATE_RX_TIMEOUT:
      case APP_STATE_RX_ERROR:
          if( app_context.is_master == true )
          {
              // Send the next PING frame
              app_build_message( true, app_context.buffer, app_context.buffer_size_in_bytes );
              DelayMs( 1 );
              //REM AP printf( "[APP] ping message transmission\n" );
              printf( "radio_trasmit ping\n" );
              loramac_radio_transmit( app_context.buffer, app_context.buffer_size_in_bytes );
          }
          else
          {
              //REM AP loramac_radio_set_rx( RAL_RX_TIMEOUT_CONTINUOUS_MODE );
        	  printf( "radio_set_rx\n" );
              loramac_radio_set_rx( RX_TIMEOUT_VALUE );

          }
          app_context.state = APP_STATE_LOW_POWER;
          break;
      case APP_STATE_TX_TIMEOUT:

          //REM AP loramac_radio_set_rx( RAL_RX_TIMEOUT_CONTINUOUS_MODE );
    	  printf( "radio_set_rx\n" );
          loramac_radio_set_rx( RX_TIMEOUT_VALUE );

          app_context.state = APP_STATE_LOW_POWER;
          break;
      case APP_STATE_LOW_POWER:
      default:
          // Set low power
          break;
      }
      //REM AP BoardLowPowerHandler( );
      // Process Radio IRQ
      //REM AP provo a portarlo sopra loramac_radio_irq_process( );



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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MYPA4_SS_GPIO_Port, MYPA4_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PYPB1_RST_GPIO_Port, PYPB1_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MYPA4_SS_Pin */
  GPIO_InitStruct.Pin = MYPA4_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MYPA4_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MYPB0_DIO0_Pin */
  GPIO_InitStruct.Pin = MYPB0_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MYPB0_DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PYPB1_RST_Pin */
  GPIO_InitStruct.Pin = PYPB1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PYPB1_RST_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
