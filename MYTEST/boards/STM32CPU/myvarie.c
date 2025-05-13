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
extern SPI_HandleTypeDef MYSPI;   // è la spi definita nel main.
extern UART_HandleTypeDef huart1;

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
/* ----- vado su UART ----
#define   LENMyBufPrint  10000
char      MyBufPrint[LENMyBufPrint];  // circular buffer...
uint32_t  MyBufPrintIdx = 0;
bool      MyBufPrintRew = false;  // se sovrascritto.
----   */

bool      MyBufPrintEN = true;

// per gestire la printf qui arrivano tutti i caratteri delle printf...
int __io_putchar(int ch) {
	uint8_t mych = ch;
    if (MyBufPrintEN)
    {
    	/* ----- questo era via ram
    	if (MyBufPrintIdx >= LENMyBufPrint ) {
    		MyBufPrintRew = true;
    		MyBufPrintIdx = 0;
    	}
    	MyBufPrint[MyBufPrintIdx] = mych;
    	MyBufPrintIdx++;
    	----- */

    	// e questo è via UART
    	HAL_UART_Transmit(&huart1, &mych, 1, HAL_MAX_DELAY);

    }
    return ch;
}

void EnableMyPrint(void) {
	MyBufPrintEN = true;
}

void DisableMyPrint(void) {
	MyBufPrintEN = false;
}


void my_hex_buffer2( uint8_t* buffer, uint16_t size, bool nolf)
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
    if (nolf==false)
        printf( "\n" );
}

void my_hex_buffer( uint8_t* buffer, uint16_t size)
{
	my_hex_buffer2(buffer,size,false);
}

void MyIsIrqFired(void)
{
	printf("is_irq_fired\n");
}




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
	// PER ORA NON LO FACCIO NON HO INTERRUPT REALI SONO SIMILATI!!!
	// COSI NON VADO A INTERFERIRE CON la seriale....
    //*mask = __get_PRIMASK( );
    //__disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t* mask )
{
    //__set_PRIMASK( *mask );
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
	printf("DelayMs(%d)\n",timx);
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

// un prototype messo qui
void myShowReg(const uint8_t addr,  uint8_t data);


uint32_t MySx1278_hal_write(const uint16_t address, const uint8_t* data,
        const uint16_t data_length )
{
	uint8_t addr = address;
	uint8_t xi;

	if (address != 0) {  // 0 è il RegFifo ed è normale che scrive piu byte...
	   // facciamo finta che stia scrivendo un registro alla volta
	   // voglio documentare ogni registro scritto
       for (xi = 0; xi < data_length ; xi++) {
    	   myShowReg(addr+xi, *(data+xi));
       }
	}
	else
	{
		// il FIFO lo visualizza ancora cosi..
		printf("W: %02X ->", addr);
		my_hex_buffer((uint8_t*)data,data_length);
	}

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





//
// QUI ADESSO viene il bello del reverse engeniering !!!
//

bool InLoraMode = false;
bool LowFrequencyModeOn = true;  // Access Low Frequency Mode registers
                                 // (from address 0x61 to 0x73)

bool inSlepMode = false;

bool InSharedReg = false;         // access to FSK registers page located in address space
                                  // (0x0D:0x3F) while in LoRa mode


void MySx1278_hal_reset(void)
{
	 printf("Reset \n");
	 HAL_GPIO_WritePin( PYPB1_RST_GPIO_Port , PYPB1_RST_Pin, GPIO_PIN_RESET);
	 // Wait 1 ms
	 DelayMs( 1 );
	 HAL_GPIO_WritePin( PYPB1_RST_GPIO_Port , PYPB1_RST_Pin, GPIO_PIN_SET);
	 // Wait 6 ms
     DelayMs( 6 );

     // vedi sotto !!!
     InLoraMode = false;
     LowFrequencyModeOn = true;
     inSlepMode = false;
     InSharedReg = false;

}


typedef struct {
    uint8_t addr;
    const char *name;
    uint8_t flgwr;     // 1 se è il registro è stato scritto.
    uint8_t value;     // ultimo valore scritto nel registro.
} RegDef;

// questi sono i REG_LR_  presi dal MyLora07 V.5
static RegDef REGISTER_MAP[] = {
    {0x01, "LR_OPMODE", 0, 0 },
    {0x06, "LR_FRFMSB", 0, 0 },
    {0x07, "LR_FRFMID", 0, 0 },
    {0x08, "LR_FRFLSB", 0, 0 },
    {0x09, "LR_PACONFIG", 0, 0 },
    {0x0A, "LR_PARAMP", 0, 0 },
    {0x0B, "LR_OCP", 0, 0 },
    {0x0C, "LR_LNA", 0, 0 },
    {0x0D, "LR_FIFOADDRPTR", 0, 0 },
    {0x0E, "LR_FIFOTXBASEADDR", 0, 0 },
    {0x0F, "LR_FIFORXBASEADDR", 0, 0 },
    {0x10, "LR_FIFORXCURRENTADDR", 0, 0 },
    {0x11, "LR_IRQFLAGSMASK", 0, 0 },
    {0x12, "LR_IRQFLAGS", 0, 0 },
    {0x13, "LR_RXNBBYTES", 0, 0 },
    {0x14, "LR_RXHEADERCNTVALUEMSB", 0, 0 },
    {0x15, "LR_RXHEADERCNTVALUELSB", 0, 0 },
    {0x16, "LR_RXPACKETCNTVALUEMSB", 0, 0 },
    {0x17, "LR_RXPACKETCNTVALUELSB", 0, 0 },
    {0x18, "LR_MODEMSTAT", 0, 0 },
    {0x19, "LR_PKTSNRVALUE", 0, 0 },
    {0x1A, "LR_PKTRSSIVALUE", 0, 0 },
    {0x1B, "LR_RSSIVALUE", 0, 0 },
    {0x1C, "LR_HOPCHANNEL", 0, 0 },
    {0x1D, "LR_MODEMCONFIG1", 0, 0 },
    {0x1E, "LR_MODEMCONFIG2", 0, 0 },
    {0x1F, "LR_SYMBTIMEOUTLSB", 0, 0 },
    {0x20, "LR_PREAMBLEMSB", 0, 0 },
    {0x21, "LR_PREAMBLELSB", 0, 0 },
    {0x22, "LR_PAYLOADLENGTH", 0, 0 },
    {0x23, "LR_PAYLOADMAXLENGTH", 0, 0 },
    {0x24, "LR_HOPPERIOD", 0, 0 },
    {0x25, "LR_FIFORXBYTEADDR", 0, 0 },
    {0x26, "LR_MODEMCONFIG3", 0, 0 },
    {0x28, "LR_FEIMSB", 0, 0 },
    {0x29, "LR_FEIMID", 0, 0 },
    {0x2A, "LR_FEILSB", 0, 0 },
    {0x2C, "LR_RSSIWIDEBAND", 0, 0 },
    {0x2F, "LR_IFFREQ1", 0, 0 },
    {0x30, "LR_IFFREQ2", 0, 0 },
    {0x31, "LR_DETECTOPTIMIZE", 0, 0 },
    {0x33, "LR_INVERTIQ", 0, 0 },
    {0x36, "LR_HIGHBWOPTIMIZE1", 0, 0 },
    {0x37, "LR_DETECTIONTHRESHOLD", 0, 0 },
    {0x39, "LR_SYNCWORD", 0, 0 },
    {0x3A, "LR_HIGHBWOPTIMIZE2", 0, 0 },
    {0x3B, "LR_INVERTIQ2", 0, 0 },
    {0x40, "LR_DIOMAPPING1", 0, 0 },
    {0x41, "LR_DIOMAPPING2", 0, 0 },
    {0x42, "LR_VERSION", 0, 0 },
    {0x44, "LR_PLLHOP", 0, 0 },
    {0x4B, "LR_TCXO", 0, 0 },
    {0x4D, "LR_PADAC", 0, 0 },
    {0x5B, "LR_FORMERTEMP", 0, 0 },
    {0x5D, "LR_BITRATEFRAC", 0, 0 },
    {0x61, "LR_AGCREF", 0, 0 },
    {0x62, "LR_AGCTHRESH1", 0, 0 },
    {0x63, "LR_AGCTHRESH2", 0, 0 },
    {0x64, "LR_AGCTHRESH3", 0, 0 },
    {0x70, "LR_PLL", 0, 0 }
};


int16_t get_REGISTER_MAP_index(uint8_t addr) {
    for (uint16_t i = 0; i < sizeof(REGISTER_MAP) / sizeof(REGISTER_MAP[0]); i++) {
        if (REGISTER_MAP[i].addr == addr) {
            return i;
        }
    }
    return -1;  // Non trovato
}

// e questi sono i reg per FSK mode.
static RegDef REGISTER_MAP_FSK[] = {
    {0x01, "FSK_OPMODE", 0, 0 },
    {0x02, "FSK_BITRATEMSB", 0, 0 },
    {0x03, "FSK_BITRATELSB", 0, 0 },
    {0x04, "FSK_FDEVMSB", 0, 0 },
    {0x05, "FSK_FDEVLSB", 0, 0 },
    {0x06, "FSK_FRFMSB", 0, 0 },
    {0x07, "FSK_FRFMID", 0, 0 },
    {0x08, "FSK_FRFLSB", 0, 0 },
    {0x09, "FSK_PACONFIG", 0, 0 },
    {0x0A, "FSK_PARAMP", 0, 0 },
    {0x0B, "FSK_OCP", 0, 0 },
    {0x0C, "FSK_LNA", 0, 0 },
    {0x0D, "FSK_RXCONFIG", 0, 0 },
    {0x0E, "FSK_RSSICONFIG", 0, 0 },
    {0x0F, "FSK_RSSICOLLISION", 0, 0 },
    {0x10, "FSK_RSSITHRESH", 0, 0 },
    {0x11, "FSK_RSSIVALUE", 0, 0 },
    {0x12, "FSK_RXBW", 0, 0 },
    {0x13, "FSK_AFCBW", 0, 0 },
    {0x14, "FSK_OOKPEAK", 0, 0 },
    {0x15, "FSK_OOKFIX", 0, 0 },
    {0x16, "FSK_OOKAVG", 0, 0 },
    {0x17, "FSK_RES17", 0, 0 },
    {0x18, "FSK_RES18", 0, 0 },
    {0x19, "FSK_RES19", 0, 0 },
    {0x1A, "FSK_AFCFEI", 0, 0 },
    {0x1B, "FSK_AFCMSB", 0, 0 },
    {0x1C, "FSK_AFCLSB", 0, 0 },
    {0x1D, "FSK_FEIMSB", 0, 0 },
    {0x1E, "FSK_FEILSB", 0, 0 },
    {0x1F, "FSK_PREAMBLEDETECT", 0, 0 },
    {0x20, "FSK_RXTIMEOUT1", 0, 0 },
    {0x21, "FSK_RXTIMEOUT2", 0, 0 },
    {0x22, "FSK_RXTIMEOUT3", 0, 0 },
    {0x23, "FSK_RXDELAY", 0, 0 },
    {0x24, "FSK_OSC", 0, 0 },
    {0x25, "FSK_PREAMBLEMSB", 0, 0 },
    {0x26, "FSK_PREAMBLELSB", 0, 0 },
    {0x27, "FSK_SYNCCONFIG", 0, 0 },
    {0x28, "FSK_SYNCVALUE1", 0, 0 },
    {0x29, "FSK_SYNCVALUE2", 0, 0 },
    {0x2A, "FSK_SYNCVALUE3", 0, 0 },
    {0x2B, "FSK_SYNCVALUE4", 0, 0 },
    {0x2C, "FSK_SYNCVALUE5", 0, 0 },
    {0x2D, "FSK_SYNCVALUE6", 0, 0 },
    {0x2E, "FSK_SYNCVALUE7", 0, 0 },
    {0x2F, "FSK_SYNCVALUE8", 0, 0 },
    {0x30, "FSK_PACKETCONFIG1", 0, 0 },
    {0x31, "FSK_PACKETCONFIG2", 0, 0 },
    {0x32, "FSK_PAYLOADLENGTH", 0, 0 },
    {0x33, "FSK_NODEADRS", 0, 0 },
    {0x34, "FSK_BROADCASTADRS", 0, 0 },
    {0x35, "FSK_FIFOTHRESH", 0, 0 },
    {0x36, "FSK_SEQCONFIG1", 0, 0 },
    {0x37, "FSK_SEQCONFIG2", 0, 0 },
    {0x38, "FSK_TIMERRESOL", 0, 0 },
    {0x39, "FSK_TIMER1COEF", 0, 0 },
    {0x3A, "FSK_TIMER2COEF", 0, 0 },
    {0x3B, "FSK_IMAGECAL", 0, 0 },
    {0x3C, "FSK_TEMP", 0, 0 },
    {0x3D, "FSK_LOWBAT", 0, 0 },
    {0x3E, "FSK_IRQFLAGS1", 0, 0 },
    {0x3F, "FSK_IRQFLAGS2", 0, 0 },
    {0x40, "FSK_DIOMAPPING1", 0, 0 },
    {0x41, "FSK_DIOMAPPING2", 0, 0 },
    {0x42, "FSK_VERSION", 0, 0 },
    {0x44, "FSK_PLLHOP", 0, 0 },
    {0x4B, "FSK_TCXO", 0, 0 },
    {0x4D, "FSK_PADAC", 0, 0 },
    {0x5B, "FSK_FORMERTEMP", 0, 0 },
    {0x5D, "FSK_BITRATEFRAC", 0, 0 },
    {0x61, "FSK_AGCREF", 0, 0 },
    {0x62, "FSK_AGCTHRESH1", 0, 0 },
    {0x63, "FSK_AGCTHRESH2", 0, 0 },
    {0x64, "FSK_AGCTHRESH3", 0, 0 },
    {0x70, "FSK_PLL", 0, 0 }
};

int16_t get_REGISTER_MAP_FSK_index(uint8_t addr) {
    for (uint16_t i = 0; i < sizeof(REGISTER_MAP_FSK) / sizeof(REGISTER_MAP_FSK[0]); i++) {
        if (REGISTER_MAP_FSK[i].addr == addr) {
            return i;
        }
    }
    return -1;  // Non trovato
}


void myShowReg(const uint8_t addr,  uint8_t data)
{
	int16_t idx;

	if (addr == 1)
	{
        if (inSlepMode) {
        	// solo in sleep mode si puo cambiare il loramode
        	if ( (data & 128) == 128)
        		  InLoraMode = true;
        	else {
        		    if ( (data & 7) == 0) InLoraMode = false;
   		            // in pratica per toglierlo devi essere gia in sleep mode
        		    // e gli devi dare  W: 01 ->00
        	}
        }

		// intercetta quando entra ed esce da sleep mode.
		if ( (data & 7) == 0)
			inSlepMode = true;
		else
			inSlepMode = false;


		if ( InLoraMode == true && (data & 64) == 64)
			InSharedReg = true;
        else
        	InSharedReg = false;

	}

    // questa un ottimizzazione
	if (MyBufPrintEN == false) return;

    // OK diciamo che da qui in avanti
    // InLoraMode LowFrequencyModeOn InSharedReg sono validi
	// e quindi posso sapere esattamente in quale registro sta effettivamente
	// inviando il valore !!!

	printf("W ");

	if (InLoraMode) {
		if (InSharedReg && addr >= 0x0D && addr <= 0x3F) printf("SHREG ");
		else printf("LR ");
	}
	else
		printf("FSK ");

	if (addr >= 0x61 && addr <= 0x73) {
		if (LowFrequencyModeOn)
			 printf("LF ");
		else printf("HF ");
	}

	// per ora cosi...
	printf(": %02X ->", addr);
	my_hex_buffer2(&data,1,true);

	printf(" ");
	if (InLoraMode) {
	    // ricava il nome registro
		idx = get_REGISTER_MAP_index(addr);
		if (idx >=0){
			printf("REG_");
			printf(REGISTER_MAP[idx].name);
			REGISTER_MAP[idx].flgwr = 1;
			REGISTER_MAP[idx].value = data;
		}
		else printf("### noregname ###");
	}
	else {
		idx = get_REGISTER_MAP_FSK_index(addr);
		if (idx >=0){
			printf(REGISTER_MAP_FSK[idx].name);
			REGISTER_MAP_FSK[idx].flgwr = 1;
			REGISTER_MAP_FSK[idx].value = data;
		}
		else printf("### noregname ###");
	}
	printf("\n");


}


void MyReportRegModified(void)  // chiamato dal main
{
	uint16_t i;
    printf("\nReport registri usati e ultimo valore scritto \n");
	// registri FSK
    for (i = 0; i < sizeof(REGISTER_MAP_FSK) / sizeof(REGISTER_MAP_FSK[0]); i++) {
        if (REGISTER_MAP_FSK[i].flgwr != 0) {
        	printf(REGISTER_MAP_FSK[i].name);
        	printf(" 0x%02X \n",REGISTER_MAP_FSK[i].value);
        }
    }
    // registri LORA
    for (i = 0; i < sizeof(REGISTER_MAP) / sizeof(REGISTER_MAP[0]); i++) {
        if (REGISTER_MAP[i].flgwr != 0) {
        	printf("REG_");
        	printf(REGISTER_MAP[i].name);
        	printf(" 0x%02X \n",REGISTER_MAP[i].value);
        }
    }
    printf("\n");
}


void MyClearReportRegModified(void) {
	uint16_t i;
    for (i = 0; i < sizeof(REGISTER_MAP_FSK) / sizeof(REGISTER_MAP_FSK[0]); i++) {
    	REGISTER_MAP_FSK[i].flgwr = 0;
    	REGISTER_MAP_FSK[i].value = 0;
    }

    for (i = 0; i < sizeof(REGISTER_MAP) / sizeof(REGISTER_MAP[0]); i++) {
        REGISTER_MAP[i].flgwr = 0;
      	REGISTER_MAP[i].value = 0;
    }
}
