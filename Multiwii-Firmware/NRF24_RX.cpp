#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include <SPI.h>
#include <RF24.h>
#include "NRF24_RX.h"

#if defined(NRF24_RX)

RadioData nrf24Data;
RF24AckPayload nrf24AckPayload;

const byte address[6] = "DRONE";
RF24 radio(7, 8);

int16_t nrf24_rcData[RC_CHANS];

static void resetRF24Data() {
  nrf24Data.throttle = 1000;
  nrf24Data.yaw = 1500;
  nrf24Data.pitch = 1500;
  nrf24Data.roll = 1500;
  nrf24Data.arm = 0;
}

void NRF24_Init() {
  resetRF24Data();

  radio.begin();
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH); 
  radio.setAutoAck(true);
  radio.enableAckPayload();

  radio.openReadingPipe(1, address);
  radio.startListening();
}

void NRF24_Read_RC() {
  static uint32_t lastRecvTime = 0;
  uint32_t now = millis();

  // receive data
  while (radio.available()) {
    radio.read(&nrf24Data, sizeof(RadioData));
    lastRecvTime = now;
  }

  // failsafe detection
  bool failsafeActive = (now - lastRecvTime > 500);

  // apply control
  if (failsafeActive) {
    nrf24_rcData[THROTTLE] = 1000;
    nrf24_rcData[ROLL]     = 1500;
    nrf24_rcData[PITCH]    = 1500;
    nrf24_rcData[YAW]      = 1500;
    nrf24_rcData[AUX1]     = 1000; // disarm
  } else {
    nrf24_rcData[THROTTLE] = constrain(nrf24Data.throttle, 1000, 2000);
    nrf24_rcData[ROLL]     = constrain(nrf24Data.roll,     1000, 2000);
    nrf24_rcData[PITCH]    = constrain(nrf24Data.pitch,    1000, 2000);
    nrf24_rcData[YAW]      = constrain(nrf24Data.yaw,      1000, 2000);
    nrf24_rcData[AUX1]     = nrf24Data.arm ? 2000 : 1000;
  }

  // telemetry (send back to TX)
  nrf24AckPayload.gx = imu.gyroData[ROLL];
  nrf24AckPayload.gy = imu.gyroData[PITCH];
  nrf24AckPayload.gz = imu.gyroData[YAW];

  nrf24AckPayload.batteryVoltage = analog.vbat / 10.0f;
  nrf24AckPayload.armed = f.ARMED ? 1 : 0;
  nrf24AckPayload.failsafe = failsafeActive ? 1 : 0;

  radio.writeAckPayload(1, &nrf24AckPayload, sizeof(RF24AckPayload));
}
#endif