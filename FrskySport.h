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

//
// Most of the code by Mike Blandford, stripped from openXsensor
// and slightly adapted by Raphael Coeffic.
//
#ifndef SPORT_ASERIAL_h
#define SPORT_ASERIAL_h

#include <Arduino.h>

void setSportSensorId( uint8_t sensor_id );
void setSportSensorValues( uint8_t n_values );

void disableSportSensor();
void enableSportSensor();

void initSportUart();
void setSportNewData( uint8_t idx, uint16_t id, uint32_t value );

// UART's state.
#define   IDLE               0 // Idle state, both transmit and receive possible.
#define   TRANSMIT           1 // Transmitting byte.
#define   TRANSMIT_STOP_BIT  2 // Transmitting stop bit.
#define   RECEIVE            3 // Receiving byte.
#define	  TxPENDING          4
#define	  WAITING            5

uint8_t returnSportState();

extern volatile bool    SportRxDataReady;
extern volatile uint8_t SportRxData;

// Either 2 or 4 is supported
#define PIN_SERIALTX 4

//#define DEBUGASERIAL 1

#endif
