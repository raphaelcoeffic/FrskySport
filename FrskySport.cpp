/* ============================================================
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ============================================================*/

// Based loosely on:
// Atmel AVR304: Half Duplex Interrupt Driven Software UART
// Author: Mike Blandford

//
// Most of the code by Mike Blandford, stripped from openXsensor
// and slightly adapted by Raphael Coeffic.
//

#include "FrskySport.h"

// Ports and registers

#define TCCR    TCCR1A  //!< Timer/Counter Control Register
#define TCCR_P  TCCR1B  //!< Timer/Counter Control (Prescaler) Register
#define OCR     OCR1A   //!< Output Compare Register
#define EXT_IFR EIFR    //!< External Interrupt Flag Register
#define EXT_ICR EICRA   //!< External Interrupt Control Register

#define TRXDDR  DDRD
#define TRXPORT PORTD
#define TRXPIN  PIND

//Some IO, timer and interrupt specific defines.

#define SET_TX_PIN( )    ( TRXPORT |= ( 1 << PIN_SERIALTX ) )
#define CLEAR_TX_PIN( )  ( TRXPORT &= ~( 1 << PIN_SERIALTX ) )
#define GET_RX_PIN( )    ( TRXPIN & ( 1 << PIN_SERIALTX ) )

#define ENABLE_PIN_CHANGE_INTERRUPT( )       ( PCICR |= (1<<PCIE2) )
#define DISABLE_PIN_CHANGE_INTERRUPT( )      ( PCICR &= ~( 1<<PCIE2 ) )
#define CLEAR_PIN_CHANGE_INTERRUPT( )         ( PCIFR = (1<<PCIF2) )

#define ENABLE_TIMER_INTERRUPT( )       ( TIMSK1 |= ( 1<< OCIE1A ) )
#define DISABLE_TIMER_INTERRUPT( )      ( TIMSK1 &= ~( 1<< OCIE1A ) )
#define CLEAR_TIMER_INTERRUPT( )        ( TIFR1 = (1 << OCF1A) )

// This section chooses the correct timer values for the Sport protocol = 57600 baud.
// 57600 = Desired baudrate for Sport protocol = 17 micro sec per bit.
#if F_CPU == 20000000L   // 20MHz clock 
 #define TICKS2COUNTSPORT         348  // Ticks between two bits.
 #define TICKS2WAITONESPORT       348  // Wait one bit period.
 #define TICKS2WAITONE_HALFSPORT  520  // Wait one and a half bit period.
#elif F_CPU == 16000000L  // 16MHz clock
 #define TICKS2COUNTSPORT         278  // Ticks between two bits.
 #define TICKS2WAITONESPORT       278  // Wait one bit period.
 #define TICKS2WAITONE_HALFSPORT  416    // Wait one and a half bit period.
#elif F_CPU == 8000000L   // 8MHz clock
 #define TICKS2COUNTSPORT         139  // Ticks between two bits.
 #define TICKS2WAITONESPORT       139  // Wait one bit period.
 #define TICKS2WAITONE_HALFSPORT  208  // Wait one and a half bit period.
#else
 #error Unsupported clock speed
#endif

// INTERRUPT_EARLY_BIAS is to bias the sample point a bit early in case
// the Timer 0 interrupt (5.5uS) delays the start bit detection
#if F_CPU == 20000000L     // 20MHz clock
 #define INTERRUPT_EXEC_CYCL   112 // Cycles to execute interrupt routines from interrupt.
 #define INTERRUPT_EARLY_BIAS  40  // Cycles to allow of other interrupts.
#elif F_CPU == 16000000L   // 16MHz clock
 #define INTERRUPT_EXEC_CYCL   90  // Cycles to execute interrupt routines from interrupt.
 #define INTERRUPT_EARLY_BIAS  32  // Cycles to allow of other interrupts.
#elif F_CPU == 8000000L    // 8MHz clock
 #define INTERRUPT_EXEC_CYCL   45  // Cycles to execute interrupt routines from interrupt.
 #define INTERRUPT_EARLY_BIAS  16  // Cycles to allow of other interrupts.
#else
 #error Unsupported clock speed
#endif

// this section define some delays used in Aserial; values can be used by any protocol
#if F_CPU == 20000000L     // 20MHz clock
  #define DELAY_4000  ((uint16_t)4000.0 * 20.0 /16.0 )
  #define DELAY_3500  ((uint16_t)3500.0 * 20.0 /16.0 )    
  #define DELAY_2000  ((uint16_t)2000.0 * 20.0 /16.0 )
  #define DELAY_1600  ((uint16_t)1600.0 * 20.0 /16.0 )    
  #define DELAY_400  ((uint16_t)400.0 * 20.0 /16.0 )
  #define DELAY_100  ((uint16_t)100.0 * 20.0 /16.0 )
#elif F_CPU == 16000000L   // 16MHz clock
  #define DELAY_4000 ((uint16_t) (1000L * 16) )     
  #define DELAY_3500 ((uint16_t) (1000L * 16) )         
  #define DELAY_2000 ((uint16_t) (1000L * 16) )     
  #define DELAY_1600 ((uint16_t) (1000L * 16) )     
  #define DELAY_400 ((uint16_t) (400 * 16) )     
  #define DELAY_100 ((uint16_t) (100 * 16) )     
#elif F_CPU == 8000000L    // 8MHz clock
  #define  DELAY_4000 ((uint16_t)4000L * 8 )
  #define  DELAY_3500 ((uint16_t)3500L * 8 )    
  #define  DELAY_2000 ((uint16_t)2000 * 8 )
  #define  DELAY_1600 ((uint16_t)1600 * 8 )    
  #define  DELAY_400 ((uint16_t)400 * 8 )
  #define  DELAY_100 ((uint16_t)100 * 8 )    
#else
  #error Unsupported clock speed
#endif

static volatile uint8_t state;                  //!< Holds the state of the UART.
static volatile unsigned char SwUartTXData;     //!< Data to be transmitted.
static volatile unsigned char SwUartTXBitCount; //!< TX bit counter.
static volatile uint8_t SwUartRXData;           //!< Storage for received bits.
static volatile uint8_t SwUartRXBitCount;       //!< RX bit counter.
static volatile uint8_t TxCount;

// SPORT TX Byte stuffing
static uint8_t ByteStuffByte = 0;

static uint8_t LastRx;
static uint8_t TxSportData[7];
uint16_t Crc;
uint8_t volatile sportData[7];
uint8_t volatile sportDataLock;

volatile bool SportRxDataReady;
volatile uint8_t SportRxData;

static uint8_t sensorId;
static bool    sendSensorValues = false;

// sendStatus
#define TO_LOAD     0
#define LOADED      1
#define SENDING     2
#define SEND        3

uint8_t volatile sendStatus=0;

//\brief  External interrupt service routine.
//  Interrupt on Pin Change to detect change on level on SPORT signal
//  (= could be a start bit)
//
// The falling edge in the beginning of the start
//  bit will trig this interrupt. The state will
//  be changed to RECEIVE, and the timer interrupt
//  will be set to trig one and a half bit period
//  from the falling edge. At that instant the
//  code should sample the first data bit.
//
// Note: initSportUart( void ) must be called in advance.
//
// This is the pin change interrupt for port D
// This assumes it is the only pin change interrupt
// on this port

ISR(PCINT2_vect)
{
  // Pin is high = start bit (inverted)
  if ( TRXPIN & ( 1 << PIN_SERIALTX ) ) {

    DISABLE_PIN_CHANGE_INTERRUPT();
    state = RECEIVE;                 // Change state
    DISABLE_TIMER_INTERRUPT();       // Disable timer to change its registers.

    OCR1A = TCNT1
      + TICKS2WAITONE_HALFSPORT
      - INTERRUPT_EXEC_CYCL
      - INTERRUPT_EARLY_BIAS; // Count one and a half period into the future.

#if DEBUGASERIAL
    PORTC |= 1;
#endif

    SwUartRXBitCount = 0;            // Clear received bit counter
    CLEAR_TIMER_INTERRUPT();         // Clear interrupt bits
    ENABLE_TIMER_INTERRUPT();        // Enable timer1 interrupt on again
  }
}

// \brief  Timer1 interrupt service routine
//
//  Timer1 will ensure that bits are written and
//  read at the correct instants in time.
//  The state variable will ensure context
//  switching between transmit and receive.
//  If state should be something else, the
//  variable is set to IDLE. IDLE is regarded
//  as a safe state/mode.

ISR(TIMER1_COMPA_vect)
{
  switch (state) {
  case TRANSMIT : // Output the TX buffer
    if( SwUartTXBitCount < 8 ) {
      if( SwUartTXData & 0x01 ){ // If the LSB of the TX buffer is 1:
        CLEAR_TX_PIN();          //  Send a logic 1 on the TX_PIN.
      }
      else {                     // Otherwise:
        SET_TX_PIN();            //  Send a logic 0 on the TX_PIN.
      }
      SwUartTXData = SwUartTXData >> 1;    // Bitshift the TX buffer and
      SwUartTXBitCount += 1;               // increment TX bit counter.
    }
    else {             // Send stop bit.
      CLEAR_TX_PIN();  //  Output a logic 1.
      state = TRANSMIT_STOP_BIT;
      //ENABLE_TIMER0_INT(); // Allow this in now.
    }
    OCR1A += TICKS2WAITONESPORT;  // Count one period into the future.
    break;

  case TRANSMIT_STOP_BIT: // Go to idle after stop bit was sent.
    if ( ByteStuffByte || (++TxCount < 8 ) ) { // Have we sent 8 bytes?
      if ( ByteStuffByte ) {
        SwUartTXData = ByteStuffByte;
        ByteStuffByte = 0;
      }
      else {
        if ( TxCount < 7 ) { // Data (or crc)?
          SwUartTXData = TxSportData[TxCount] ;
          Crc += SwUartTXData ; //0-1FF
          Crc += Crc >> 8 ; //0-100
          Crc &= 0x00ff ;
        }
        else { // CRC
          // prepare sending check digit
          SwUartTXData = 0xFF-Crc;
        }

        if ( ( SwUartTXData == 0x7E ) || ( SwUartTXData == 0x7D ) ) {
          ByteStuffByte = SwUartTXData ^ 0x20 ;
          SwUartTXData = 0x7D ;					
        }
      }

      SET_TX_PIN();                        // Send a logic 0 on the TX_PIN.
      OCR1A = TCNT1 + TICKS2WAITONESPORT; // Count one period into the future.
      SwUartTXBitCount = 0;
      state = TRANSMIT;
      //DISABLE_TIMER0_INT(); // For the byte duration
    }
    else { // 8 bytes have been sent
      state = WAITING;
      sendStatus = SEND;
      OCR1A += DELAY_3500; // 3.5mS gap before listening
      TRXDDR &= ~( 1 << PIN_SERIALTX );  // PIN is input
      TRXPORT &= ~( 1 << PIN_SERIALTX ); // PIN is tri-stated
    }
    break;
               
  case RECEIVE:  // Start bit has been received and we will read bits of data
    // Count one period after the falling edge is trigged.
    OCR1A += TICKS2WAITONESPORT;

    //Receiving, LSB first.
    {
      uint8_t data; // Use a temporary local storage
      data = SwUartRXBitCount;

      // If 8 bits are not yet read
      if( data < 8 ) {
        SwUartRXBitCount = data + 1;
        data = SwUartRXData;

        // Shift due to receiving LSB first
        data >>= 1;

#if DEBUGASERIAL
        PORTC &= ~1; // clear PC0
#endif

        if( GET_RX_PIN( ) == 0 ) {
          // If a logical 1 is read, let the data mirror this.
          data |= 0x80;
        }

#if DEBUGASERIAL
        PORTC |= 1; // set PC0
#endif

        SwUartRXData = data;
      }
      else { // Done receiving =  8 bits are in SwUartRXData

#if DEBUGASERIAL
        PORTC &= ~1 ; // clear PC0
#endif

        if ( sendSensorValues ) {

          if ( LastRx == 0x7E ) {

            if ( SwUartRXData == sensorId ) {
              // if ( sendStatus == LOADED ) {
                if ( sportDataLock == 0 ) {
                  TxSportData[0] = sportData[0];
                  TxSportData[1] = sportData[1];
                  TxSportData[2] = sportData[2];
                  TxSportData[3] = sportData[3];
                  TxSportData[4] = sportData[4];
                  TxSportData[5] = sportData[5];
                  TxSportData[6] = sportData[6];
                }
                else { // Discard frame to be sent if data is locked
                  TxSportData[0] = 0;
                  TxSportData[1] = 0;
                  TxSportData[2] = 0;
                  TxSportData[3] = 0;
                  TxSportData[4] = 0;
                  TxSportData[5] = 0;
                  TxSportData[6] = 0;
                }

                state = TxPENDING;
                sendStatus = SENDING;

                // 400 uS gap before sending
                OCR1A += ( DELAY_400 - TICKS2WAITONESPORT);
              // }
              // else {
              //   // Wait for idle time
              //   state = WAITING;
              //   OCR1A += DELAY_3500; // 3.5mS gap before listening
              // }
            }
            else { // it is not the expected device ID
              state = WAITING;
              OCR1A += DELAY_3500; // 3.5mS gap before listening
            }
          }
          else { // Previous code is not equal to 0x7E 

            // Stop the timer interrupts.
            DISABLE_TIMER_INTERRUPT();

            // Go back to idle.
            state = IDLE;

            // clear pending interrupt
            PCIFR = ( 1<<PCIF2 );

            // pin change interrupt enabled
            PCICR |= ( 1<<PCIE2 );
          }

          LastRx = SwUartRXData;
        }
        else {
          SportRxData = SwUartRXData;
          SportRxDataReady = true;

          // Stop the timer interrupts.
          DISABLE_TIMER_INTERRUPT();

          // Go back to idle.
          state = IDLE;

          // clear pending interrupt
          PCIFR = ( 1<<PCIF2 );

          // pin change interrupt enabled
          PCICR |= ( 1<<PCIE2 );
        }
      } // End receiving  1 bit or 1 byte (8 bits)
    }
    break;

  case TxPENDING:
    // PIN is output
    TRXDDR |= ( 1 << PIN_SERIALTX );

    // Send a logic 0 on the TX_PIN.
    SET_TX_PIN();

    // Count one period into the future.
    OCR1A = TCNT1 + TICKS2WAITONESPORT;

    SwUartTXBitCount = 0;
    Crc = SwUartTXData = TxSportData[0];
    TxCount = 0;
    state = TRANSMIT;
    //DISABLE_TIMER0_INT(); // For the byte duration
    break;

  case WAITING:

    // Stop the timer interrupts.
    DISABLE_TIMER_INTERRUPT();

    // Go back to idle.
    state = IDLE;

    // clear pending interrupt
    PCIFR = ( 1<<PCIF2 );

    // pin change interrupt enabled
    PCICR |= ( 1<<PCIE2 );
    break;

  default: // Unknown state.
    // Error, should not occur. Going to a safe state.
    state = IDLE;
  } // End CASE
} // End of ISR


void setSportSensorId( uint8_t sensor_id )
{
  sensorId = sensor_id;
  sendSensorValues = true;
}


// \brief  Function to initialize the UART for Sport protocol
//  This function will set up pins to transmit and receive on.
//  Control of Timer0 and External interrupt 0.
void initSportUart()
{
  // Timer1
  TIMSK1 &= ~( 1<< OCIE1A ) ; // Disable interupt on timer 1 for compA
  TCCR1A = 0x00 ;    //Init.
  TCCR1B = 0xC1 ;    // I/p noise cancel, rising edge, Clock/1

  //PORT
  TRXDDR &= ~( 1 << PIN_SERIALTX );  // PIN is input.
  TRXPORT &= ~( 1 << PIN_SERIALTX ); // PIN is tri-stated.

  // External interrupt
#if PIN_SERIALTX == 4
  PCMSK2 |= 0x10;	 // IO4 (PD4) on Arduini mini
#elif PIN_SERIALTX == 2
  PCMSK2 |= 0x04;      // IO2 (PD2) on Arduini mini
#else
  #error "This PIN is not supported"
#endif

  PCIFR = (1<<PCIF2);	 // clear pending interrupt
  PCICR |= (1<<PCIE2); // pin change interrupt enabled

  // Internal State Variable
  state = IDLE;

#if DEBUGASERIAL
  DDRC = 0x03 ;		// PC0,1 as o/p debug
  PORTC = 0 ;
#endif
}

void setSportNewData( uint16_t id, uint32_t value )
{
  sportDataLock = 1;

  sportData[0] = 0x10;
  sportData[1] = id; // low byte
  sportData[2] = id >> 8; // hight byte
  sportData[3] = value;
  sportData[4] = value >> 8;
  sportData[5] = value >> 16;
  sportData[6] = value >> 24;

  sportDataLock = 0;
}

uint8_t returnSportState()
{
  return state;
}  
