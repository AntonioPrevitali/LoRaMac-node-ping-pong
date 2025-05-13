#ifndef __MYVARIE_H__
#define __MYVARIE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

// subito con un bel casino !
#define CRITICAL_SECTION_BEGIN( ) uint32_t mask; BoardCriticalSectionBegin( &mask )
#define CRITICAL_SECTION_END( ) BoardCriticalSectionEnd( &mask )

void BoardCriticalSectionBegin( uint32_t* mask );
void BoardCriticalSectionEnd( uint32_t* mask );

void MyReportRegModified(void);
void MyClearReportRegModified(void);


void EnableMyPrint(void);
void DisableMyPrint(void);
void MyIsIrqFired(void);

typedef struct TimerEvent_s
{
	bool     TimerEN;
	uint32_t TimerVL;
    uint32_t TimerVi;
    void ( *Callback )( void* context ); //! Timer IRQ callback function
    void *Context;                       //! User defined data object pointer to pass back
}TimerEvent_t;

bool IsMyDIO0_ON(void);

void MyUpdTimer(void);

void MyOnDio0Irq(void);  // per ora il mio RA01 ha solo il pin D0...

void TimerStart( TimerEvent_t *obj );

void TimerSetValue( TimerEvent_t *obj, uint32_t value );

void TimerStop( TimerEvent_t *obj );

void TimerInit( TimerEvent_t *obj, void ( *callback )( void *context ) );

bool TimerIsStarted( TimerEvent_t *obj );

void TimerSetContext( TimerEvent_t *obj, void* context );

uint32_t TimerGetCurrentTime( void );

uint32_t TimerGetElapsedTime( uint32_t past );

void DelayMs(uint32_t timx);

uint32_t radio_board_get_dio_1_pin_state( void );

uint32_t MySx1278_hal_write(const uint16_t address, const uint8_t* data, const uint16_t data_length );
uint32_t MySx1278_hal_read(const uint16_t address,  uint8_t* data, const uint16_t data_length );
void MySx1278_hal_reset(void);



#endif // __MYVARIE_H__
