/*
  CAN bus data logger for MAZDA MX-5
*/
#include <math.h>

#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "can.h"
#include "led.h"
#include "system.h"
#include "error.h"
#include "printf.h"
#include "mazda_mx5_nc.h"


static float EngineRPM = 0;
static float Speed = 0;
static float AcceleratorPosition = 0;
static uint8_t ShiftPosition = 0;
static int16_t SteeringAngle = 0;
static float BrakePercentage = 0;
static uint8_t clutch = 0;
static uint8_t neutral = 0;

int16_t bytesToInt(uint8_t raw[], int shift, int size) {

  int16_t result = 0;

  for (int i = 0; i < size; i++) {
    // printf_("result=%04X << %d = ",result, sizeof(byte) * 8);
    result = result << (sizeof raw[0] * 8);
    // printf_("%d\n",result);
    for (int j = 0; j < sizeof raw[0] * 8; j++) {
      // printf_("%04X & %04X = %d += %04X,result=%04X\n", raw[i + shift], 1 << j,raw[i + shift] & (1 << j),result);
      // if(raw[i + shift] & (1 << j))(result += 1 << j);
      result += raw[i + shift] & (1 << j);
      //printf_("i=%d,j=%d,raw[%d]=0x%02X,result=0x%04X\n",i,j,i+shift,raw[i+shift],result);
    }
  }

  return result;

  // return &raw[shift];
}

uint16_t bytesToUint(uint8_t raw[], int shift, int size) {

  uint16_t result = 0;

  for (int i = 0; i < size; i++) {
    // printf_("result=%04X << %d = ",result, sizeof(byte) * 8);
    result = result << (sizeof raw[0] * 8);
    // printf_("%d\n",result);
    for (int j = 0; j < sizeof raw[0] * 8; j++) {
      // printf_("%04X & %04X = %d += %04X,result=%04X\n", raw[i + shift], 1 << j,raw[i + shift] & (1 << j),result);
      // if(raw[i + shift] & (1 << j))(result += 1 << j);
      result += raw[i + shift] & (1 << j);
      //printf_("i=%d,j=%d,raw[%d]=0x%02X,result=0x%04X\n",i,j,i+shift,raw[i+shift],result);
    }
  }

  return result;

  // return &raw[shift];
}

uint16_t bitToUint(uint8_t raw[], int shift) {
  uint16_t result;
  // Serial.printf("raw[%d]=%02X,>> %d = 0x%02X, & 0xFE\n",(shift - 1) / 8,raw[(shift - 1) / 8],7 - (shift - 1) % 8,(raw[(shift - 1) / 8] >> 7 - (shift - 1) % 8));

  result = (raw[(shift - 1) / 8] >> 7 - (shift - 1) % 8) & 0x01;
  /*
  printf("result0=%02X, result1=",result);
  result = result & 0x01;
  printf("%02X\n",result);
  } else {
    result = (raw[(shift - 1) / 8] & 0xFE);
  }*/
  return result;
}

uint16_t bitsToUint(uint8_t raw[], int shift, int size) {

  uint16_t result = 0;

  for (int i = shift; i < shift + size; i++) {
    result = result << 1;
    result += bitToUint(raw, i);
  }

  return result;

  // return &raw[shift];
}

void mazdaMx5EngineSpeed(uint8_t* rx_msg_data) {
  static float GearRatio;

  EngineRPM = bytesToUint(rx_msg_data, 0, 2) / 4.00;
  // printf_("%5.2f rpm\n",EngineRPM);
  Speed = (bytesToUint(rx_msg_data, 4, 2) / 100.00) - 100;
  // printf_("%3.2f km/h\n",Speed);
  // Raw speed data is given in km/h with a 100 km/h offset.
  AcceleratorPosition = bytesToUint(rx_msg_data, 6, 1) / 2.00;
  // printf_("Accel = %3.2f \%\n",AcceleratorPosition);
  // This is a percentage, increments of 0.5%
  if ((int)Speed != 0 && (int)EngineRPM != 0) {
    // printf_("%5.2f rpm, %3.2f km/h\n",EngineRPM,Speed);
    GearRatio = (EngineRPM * 60) / FINAL_RATIO * (TYRE_OUTER_DIAMETER_16 / 100000) / Speed;
  } else {
    GearRatio = 0;
  }

  if (clutch) {
    ShiftPosition = 8;
  } else {
    if (neutral) {
      ShiftPosition = 0;
    } else {
      if (GearRatio == 0) {
        ShiftPosition = 0;
      } else if (GearRatio < (GEAR_RATIO_6 + GEAR_RATIO_5) / 2) {
        ShiftPosition = 6;
      } else if (GearRatio < (GEAR_RATIO_5 + GEAR_RATIO_4) / 2) {
        ShiftPosition = 5;
      } else if (GearRatio < (GEAR_RATIO_4 + GEAR_RATIO_3) / 2) {
        ShiftPosition = 4;
      } else if (GearRatio < (GEAR_RATIO_3 + GEAR_RATIO_2) / 2) {
        ShiftPosition = 3;
      } else if (GearRatio < (GEAR_RATIO_2 + GEAR_RATIO_1) / 2) {
        ShiftPosition = 2;
      } else {
        ShiftPosition = 1;
      }
    }
  }
}

void mazdaMx5Transmission(uint8_t* rx_msg_data) {

  clutch = bitToUint(rx_msg_data, 15);
  // printf_("clutch = %d\n", clutch);
  neutral = bitToUint(rx_msg_data, 14);
  // printf_("neutral = %d\n", neutral);
}

void mazdaMx5Steering(uint8_t* rx_msg_data) {

  SteeringAngle = bytesToInt(rx_msg_data, 2, 2);
  // printf_("Steering %-d\n",SteeringAngle);
}

void mazdaMx5Brake(uint8_t* rx_msg_data) {
  // BrakePressure = (3.4518689053 * bytesToInt(rxBuf, 0, 2) - 327.27) / 1000.00;
  // BrakePercentage = min(0.2 * (bytesToInt(rxBuf, 0, 2) - 102), 100);
  BrakePercentage = 0.2 * (bytesToInt(rx_msg_data, 0, 2) - 102);
  if (100 < BrakePercentage) {
    BrakePercentage = 100;
  }
  if (BrakePercentage < 0) {
    BrakePercentage = 0;
  }
  // printf_("Brake = %3.2f \%\n",BrakePercentage);
}

void mazdaMx5OutputCsv() {
  // Serial.println("mazdaMx5OutputCsv()");
  printf_(", %.1f, %.1f, %d, %.1f, %.1f, %-.1f\n", Speed, EngineRPM, ShiftPosition, AcceleratorPosition, BrakePercentage, SteeringAngle * MAX_STEERING_ANGLE / STEERING_MAX);
}


int main(void)
{
    // Storage for status and received message buffer
    CAN_RxHeaderTypeDef rx_msg_header;
    uint8_t rx_msg_data[8] = {0};

    // Initialize peripherals
    system_init();
    can_init();
    led_init();
    usb_init();

    can_enable();

    while(1){

        // If CAN message receive is pending, process the message
        if(is_can_msg_pending(CAN_RX_FIFO0)){
            can_rx(&rx_msg_header, rx_msg_data);

            switch (rx_msg_header.StdId) {
                case CAN_ID_ENGINE_SPEED:
                    mazdaMx5EngineSpeed(rx_msg_data);
                    break;
                case CAN_ID_TRANSMISSION:
                    mazdaMx5Transmission(rx_msg_data);
                    break;
                case CAN_ID_STEERLING:
                    mazdaMx5Steering(rx_msg_data);
                    break;
                case CAN_ID_BRAKE:
                    mazdaMx5Brake(rx_msg_data);
                    break;
                case CAN_ID_ENGINE:
                    mazdaMx5OutputCsv();
                    break;
                    // default:
                    // printf_("Unexpected can frame received. rx_frame.identifier=%3x\n", rx_frame.identifier);
            }
        }
    }
}

