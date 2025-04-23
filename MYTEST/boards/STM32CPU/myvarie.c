#include "main.h"
#include "myvarie.h"
#include <stdio.h>

// ---------------------------------------------------------
// Hardware Pin change only here for new pin

#define MYPIN_DIO0_port   MYPB0_DIO0_GPIO_Port
#define MYPIN_DIO0_pin    MYPB0_DIO0_Pin

#define MYPIN_SS_Port     MYPA4_SS_GPIO_Port
#define MYPIN_SS_Pin      MYPA4_SS_Pin

#define MYSPI             hspi1

// -------------------------------------------------------


/*!
 * Safely execute call back
 */
#define ExecuteCallBack( _callback_, context ) \
    do                                         \
    {                                          \
        if( _callback_ == NULL )               \
        {                                      \
            while( 1 );                        \
        }                                      \
        else                                   \
        {                                      \
            _callback_( context );             \
        }                                      \
    }while( 0 );

// è tutta ram... non esagerare..  is in memory log of printf !
// io ho stlink-v2 che non ha la console seriale e non ho definito una seriale su
// cui redirigere le printf cosi le printf le ridirigo in questo buffer e che poi
// ispeziono con stm32cubeide.
#define   LENMyBufPrint  10000
char      MyBufPrint[LENMyBufPrint];  // circular buffer...
uint32_t  MyBufPrintIdx = 0;
bool      MyBufPrintEN = true;
bool      MyBufPrintRew = false;  // se sovrascritto.


// per gestire la printf qui arrivano tutti i caratteri delle printf...
int __io_putchar(int ch) {
	uint8_t mych = ch;
    if (MyBufPrintEN)
    {
    	if (MyBufPrintIdx >= LENMyBufPrint ) {
    		MyBufPrintRew = true;
    		MyBufPrintIdx = 0;
    	}
    	MyBufPrint[MyBufPrintIdx] = mych;
    	MyBufPrintIdx++;
    }
    return ch;
}

void EnableMyPrint(void) {
	MyBufPrintEN = true;
}

void DisableMyPrint(void) {
	MyBufPrintEN = false;
}

void my_hex_buffer( uint8_t* buffer, uint16_t size )
{
    uint8_t newline = 0;

    for( uint8_t i = 0; i < size; i++ )
    {
        if( newline != 0 )
        {
           newline = 0;
        }
        printf( "%02X ", buffer[i] );
        if( ( ( i + 1 ) % 16 ) == 0 )
        {
            newline = 1;
        }
    }
    printf( "\n" );
}

void MyIsIrqFired(void)
{
	printf("is_irq_fired\n");
}


extern SPI_HandleTypeDef MYSPI;   // è la spi definita nel main.

// i 3 oggetti timer usati.
TimerEvent_t tx_timeout_timer = {false,0,0};
TimerEvent_t rx_timeout_timer = {false,0,0};
TimerEvent_t timer_timeout = {false,0,0};

void ( *Mydio_0_irq_handler )( void* context );
void* Mydio_0_irq_handler_context;


void MyOnDio0Irq(void) {
	printf("IrqD0 go! \n");
	Mydio_0_irq_handler(Mydio_0_irq_handler_context);
	printf("IrqD0 end \n");
}


void BoardCriticalSectionBegin( uint32_t* mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t* mask )
{
    __set_PRIMASK( *mask );
}


//REM AP PROVVISORI mancano le criticalsection ed una vera gestione ad irq...
void TimerStart( TimerEvent_t *obj )
{
	if(  obj == NULL ) return;
	//REM AP if (obj->TimerEN == true) return;  da valutare se farlo...
	obj->TimerVi = HAL_GetTick();
	obj->TimerEN = true;
}

uint8_t qualetim = 0;

void MyUpdTimergo(TimerEvent_t *obj, uint32_t curTik)
{
	if (obj->TimerEN==true)
	{
		if (obj->TimerVL > 0)
		{
			if (( curTik - obj->TimerVi ) >= obj->TimerVL ) {
				obj->TimerEN = false;
				if (qualetim == 1)
					printf("tx_timeout go\n");
				else if(qualetim == 2)
				    printf("rx_timeout go\n");
				else if(qualetim == 3)
			    	printf("timeout go\n");
				ExecuteCallBack( obj->Callback, obj->Context );
				printf("timerEnd\n");
			}
		}
	}
}


void MyUpdTimer(void)
{
	uint32_t curTik = HAL_GetTick();
	qualetim = 1;
	MyUpdTimergo(&tx_timeout_timer,curTik);
	qualetim = 2;
	MyUpdTimergo(&rx_timeout_timer,curTik);
	qualetim = 3;
	MyUpdTimergo(&timer_timeout,curTik);
	qualetim = 0;
}


void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
	obj->TimerEN = false;
	obj->TimerVL = value;
}

void TimerStop( TimerEvent_t *obj )
{
	obj->TimerEN = false;
}


void TimerInit( TimerEvent_t *obj, void ( *callback )( void *context ) )
{
    obj->TimerVL = 0;
    obj->TimerVi = 0;
    obj->TimerEN = false;
    obj->Callback = callback;
    obj->Context = NULL;
}

bool TimerIsStarted( TimerEvent_t *obj )
{
	return obj->TimerEN;
}

void TimerSetContext( TimerEvent_t *obj, void* context )
{
	obj->Context = context;
}


uint32_t TimerGetCurrentTime( void )
{
	return HAL_GetTick();
}

uint32_t TimerGetElapsedTime( uint32_t past )
{
	uint32_t xcur = HAL_GetTick();
	return xcur - past;
}

void DelayMs(uint32_t timx)
{
	HAL_Delay(timx);  // DA VERIFICARE !
}



uint32_t radio_board_get_dio_1_pin_state( void )
{

 //REM AP    radio_context_t* radio_context = radio_board_get_radio_context_reference( );
 //REM AP    return GpioRead( &radio_context->dio_1 );

 // DA FARE #### ATTENZIONE CHE CE NE E' UNO ANCHE IN sx127x_hal.c
 return 0;  // per ora cosi !
}


bool IsMyDIO0_ON(void) {
	if (HAL_GPIO_ReadPin(MYPIN_DIO0_port, MYPIN_DIO0_pin) == GPIO_PIN_SET)
		 return true;
	else return false;
}


uint32_t MySx1278_hal_write(const uint16_t address, const uint8_t* data,
        const uint16_t data_length )
{
	uint8_t addr = address;

	printf("W: %02X ->", addr);
	my_hex_buffer((uint8_t*)data,data_length);

    //NSS = 0;
    HAL_GPIO_WritePin(MYPIN_SS_Port, MYPIN_SS_Pin, GPIO_PIN_RESET);

    addr = addr | 0x80;
    HAL_SPI_Transmit(&MYSPI, &addr, 1,HAL_MAX_DELAY);

    HAL_SPI_Transmit(&MYSPI, data, data_length, HAL_MAX_DELAY);

    //NSS = 1;
    HAL_GPIO_WritePin(MYPIN_SS_Port, MYPIN_SS_Pin, GPIO_PIN_SET);

    return 0;
}


uint32_t MySx1278_hal_read(const uint16_t address,  uint8_t* data,
        const uint16_t data_length )
{
	   uint8_t addr = address;

	   //NSS = 0;
	    HAL_GPIO_WritePin(MYPIN_SS_Port, MYPIN_SS_Pin, GPIO_PIN_RESET);

	    HAL_SPI_Transmit(&MYSPI, &addr, 1,HAL_MAX_DELAY);

	    HAL_SPI_Receive(&MYSPI, data, data_length, HAL_MAX_DELAY);

	    //NSS = 1;
	    HAL_GPIO_WritePin(MYPIN_SS_Port, MYPIN_SS_Pin, GPIO_PIN_SET);

		printf("R: %02X ->", addr);
		my_hex_buffer(data,data_length);

	    return 0;

}

void MySx1278_hal_reset(void)
{
	 printf("Reset \n");
	 HAL_GPIO_WritePin( PYPB1_RST_GPIO_Port , PYPB1_RST_Pin, GPIO_PIN_RESET);
	 // Wait 1 ms
	 DelayMs( 1 );
	 HAL_GPIO_WritePin( PYPB1_RST_GPIO_Port , PYPB1_RST_Pin, GPIO_PIN_SET);
	 // Wait 6 ms
     DelayMs( 6 );
}


