#include <Wire.h>
#include <SPI.h>
#include <stdlib.h>
#include "ADEXTENDER.h"
#include "AD533X_type.h"
#include "AD7124_regs.h"

#define ADC_ADDR ADS101x_DEFAULT_ADDRESS
#define DAC_ADDR AD533X_DEFAULT_ADDRESS

ADEXTENDER shield = ADEXTENDER(DAC_ADDR);
uint16_t dacValue;
uint8_t dacChannal;
uint8_t writeVaule[8];
uint8_t u8Status;
uint16_t arDAC[] = {0, 1023, 2047, 3071, 4095};
uint16_t u16Status;
uint8_t currentADCChannel; 


void printU16(uint16_t u16Value) {
  Serial.print(u16Value, HEX);
}

void printADCConfig(void) {
  // Read data back
  Serial.print("\nRead Channel config 0 : ");
  printU16(shield.ADCGetConfigChannel(AD_EXTENDER_ADC_CH_0));
 
  Serial.print("\nRead Channel config 1 : ");
  printU16(shield.ADCGetConfigChannel(AD_EXTENDER_ADC_CH_1));

  Serial.print("\nRead Channel config 2 : ");
  printU16(shield.ADCGetConfigChannel(AD_EXTENDER_ADC_CH_2));

  Serial.print("\nRead Channel config 3 : ");
  printU16(shield.ADCGetConfigChannel(AD_EXTENDER_ADC_CH_3));

  Serial.print("\nRead Channel config 4 : ");
  printU16(shield.ADCGetConfigChannel(AD_EXTENDER_ADC_CH_4));

  Serial.print("\nRead config 0 : ");
  printU16(shield.ADCGetConfig(0));

  Serial.print("\nRead config 1 : ");
  printU16(shield.ADCGetConfig(1));

  Serial.print("\nRead config 2 : ");
  printU16(shield.ADCGetConfig(2));

  Serial.print("\nRead Status Register : ");
  Serial.print(shield.ADCReadStatus(), HEX);
}

void adcSetup() {
  uint8_t u8Temp[4];
  uint16_t u16Temp;
  uint32_t u32Temp;
  uint8_t sta;

  shield.ADCReset();
  
  Serial.print("\nRead Chip ID : ");
  shield.deviceSPIRead(AD7124_ID, 1, u8Temp);
  Serial.print(u8Temp[0], HEX);

  writeVaule[0] = 0;
  writeVaule[1] = 0;
  writeVaule[2] = 0;
  shield.deviceSPIWrite(AD7124_ERREN_REG, writeVaule, 3);
  u32Temp = shield.ADCReadERROR_EN();
  Serial.print("\nRead Error EN Register : ");
  Serial.print(u32Temp, HEX);

  shield.deviceSPIRead(AD7124_Error, 3, u8Temp);
  u32Temp = (uint32_t)(u8Temp[0]) << 16;
  u32Temp |= (uint32_t)(u8Temp[1]) << 8;
  u32Temp |= (uint32_t)u8Temp[2];
   Serial.print("\nRead Error Register : ");
  Serial.print(u32Temp, HEX); 

  Serial.print("\nRead Status Register : ");
  Serial.print(shield.ADCReadStatus(), HEX);

  
  Serial.print("\nWrite adc control : ");
  Serial.print(shield.ADCReadStatus(), HEX);
  
  // Setting channel Config 0-4
  shield.ADCConfigChannel(AD_EXTENDER_ADC_CH_0, 2, AD_ENABLE);  
  shield.ADCConfigChannel(AD_EXTENDER_ADC_CH_1, 0, AD_DISABLE);
  shield.ADCConfigChannel(AD_EXTENDER_ADC_CH_2, 0, AD_DISABLE);
  shield.ADCConfigChannel(AD_EXTENDER_ADC_CH_3, 0, AD_DISABLE);
  shield.ADCConfigChannel(AD_EXTENDER_ADC_CH_4, 0, AD_DISABLE);
  

  // Setting channel Config 0-2
  shield.ADCSetConfig(0, 0, 0);   // Config 2 Vref = 1.25V PGA 1
  shield.ADCSetConfig(1, 0, 1);   // Config 2 Vref = 1.25V PGA 2
  shield.ADCSetConfig(2, 0, 3);   // Config 2 Vref = 1.25V PGA 8

  /**writeVaule[0] = 0;
  writeVaule[1] = 1;
  shield.deviceSPIWrite(AD7124_IOCon2_, writeVaule, 2);*/

  printADCConfig();

  Serial.print("\nRead ADC Control : ");
  shield.deviceSPIRead(AD7124_ADC_Control, 2, u8Temp);
  u16Temp = (uint16_t)(u8Temp[0]) << 8;
  u16Temp |= u8Temp[1];
  Serial.print(u16Temp, HEX);

}

uint8_t aDCStartConversion(E_ADEXTENDER_ADC_CH eChannel) {
  uint8_t u8Status;
  uint16_t u16ConfigValue;
  uint16_t u16CurrentActiveConfigValue;
  uint8_t u8TempBuffer[2];
  AD7124_STATUS_REGISTER *psStatus;


  // Setting configuration
  u16ConfigValue = (uint16_t)AD7124_ADC_CTRL_REG_REF_EN |
      (uint16_t)AD7124_ADC_CTRL_REG_POWER_MODE(3) |
      (uint16_t)AD7124_ADC_CTRL_REG_MODE(1);

  // Read Current active channel
  u8Status = shield.ADCReadStatus();
  psStatus = (AD7124_STATUS_REGISTER*) &u8Status;

  if (eChannel != psStatus->CH_ACTIVE) {
    // Disable current status
    shield.ADCSetChannelControl((E_ADEXTENDER_ADC_CH)psStatus->CH_ACTIVE, AD_DISABLE);
  }

  // Enable channel
  shield.ADCSetChannelControl(eChannel, AD_ENABLE);

  // Write value to ADC Control
  u8TempBuffer[0] = (uint8_t)((u16ConfigValue >> 8) & 0xFF);
  u8TempBuffer[1] = (uint8_t)(u16ConfigValue & 0xFF);
  shield.deviceSPIWrite(AD7124_ADC_Control, u8TempBuffer, 2);

  return 0;
}

void ReadVaule(uint8_t ch) {
  uint8_t u8Temp[4];
  uint16_t u16Temp;
  uint32_t u32Temp;
  uint8_t sta;
  

  sta = shield.ADCRead((E_ADEXTENDER_ADC_CH) ch, &u32Temp);
  
  if (sta == 0) {
    Serial.print("\nRead Value CH : ");
    Serial.print(ch, DEC);
    Serial.print("\t");
    Serial.print(u32Temp, HEX);
    Serial.print("\t");
    Serial.print(u32Temp, DEC);
    //Serial.print("\nRead Status Register : ");
    //Serial.print(shield.ADCReadStatus(), HEX);
  }
  else {
    Serial.print("\nTime Out");
    Serial.print("\nRead Status Register : ");
    Serial.print(shield.ADCReadStatus(), HEX);
  }

  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  dacChannal = AD533X_OUTA;
  shield.Init(DAC_ADDR);
  //Wire.begin();
  //Wire.setClock(100000);
  dacValue = 0;

  adcSetup();

  //u8Status = shield.DACUpdateVaule((AD533X_OUT_SEL)AD533X_OUTA, 4096);

  //u8Status = shield.DACSetOutByVoltage(AD_EXTENDER_OUT_CH1, AD_EXTENDER_OUT_MODE_0_5_V, 5000);
  //delay(100);

  u8Status = shield.DACSetOutByVoltage(AD_EXTENDER_OUT_CH1, AD_EXTENDER_OUT_MODE_0_5_V, 4200);
  u8Status = shield.DACSetOutByVoltage(AD_EXTENDER_OUT_CH2, AD_EXTENDER_OUT_MODE_0_5_V, 2500);

  currentADCChannel = AD_EXTENDER_ADC_CH_0;
}

void loop() {
  // put your main code here, to run repeatedly:

  ReadVaule(currentADCChannel);
  currentADCChannel = ++currentADCChannel % 5;
  delay(1000);
}
