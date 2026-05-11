/* Tested with 16MHz ATmega328p-AP, QuadX and https://github.com/gcopeland/RF24
Motors use pins 9,6,5,3 instead of 9,10,11,3
nRF24 connections (left is nRF24, right is arduino):q
  CE      7
  CSN    10
  MOSI   11
  MISO   12
  SCK    13
You can change CE and CSN in NRF24_RX.cpp
*/

#ifndef NRF24_RX_H_
#define NRF24_RX_H_

#include "config.h"
#if defined(NRF24_RX)

#include <stdint.h>

struct RadioData {
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  uint8_t arm;
};

struct RF24AckPayload {
  float gx;
  float gy;
  float gz;
  float batteryVoltage;
  uint8_t armed;
  uint8_t failsafe;
};

extern RadioData nrf24Data;
extern RF24AckPayload nrf24AckPayload;
extern int16_t nrf24_rcData[RC_CHANS];

void NRF24_Init();
void NRF24_Read_RC();

#endif
#endif