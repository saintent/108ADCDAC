#include "ADEXTENDER.h"

ADEXTENDER shield = ADEXTENDER();

void setup() {
  uint32_t u32Temp;
  uint32_t u32TempRes;
  // put your setup code here, to run once:
  Serial.begin(115200);
  shield.Begin();

  // Setting up DAC
  shield.DACSetOutByVoltage(AD_EXTENDER_OUT_CH1, AD_EXTENDER_MODE_0_5_V, 5000);
  shield.DACSetOutByVoltage(AD_EXTENDER_OUT_CH2, AD_EXTENDER_MODE_0_10_V, 10000);

  shield.ADCRead(AD_EXTENDER_ADC_CH_4, AD_EXTENDER_MODE_0_10_V, &u32Temp, &u32TempRes);

  Serial.print("\nRead ch 4 V : ");
  Serial.print(u32Temp, DEC);
  Serial.print("\t");
  Serial.print(u32TempRes, HEX);
}

void loop() {
  uint32_t u32Temp, u32TempRes;
  
  shield.ADCRead(AD_EXTENDER_ADC_CH_0, AD_EXTENDER_MODE_TC, &u32Temp, &u32TempRes);
  Serial.print("\nRead ch 0 V : ");
  Serial.print(u32Temp, DEC);
  Serial.print("\t");
  Serial.print(u32TempRes, HEX);

  shield.ADCRead(AD_EXTENDER_ADC_CH_1, AD_EXTENDER_MODE_0_5_V, &u32Temp, &u32TempRes);
  Serial.print("\nRead ch 1 V : ");
  Serial.print(u32Temp, DEC);
  Serial.print("\t");
  Serial.print(u32TempRes, HEX);

  shield.ADCRead(AD_EXTENDER_ADC_CH_2, AD_EXTENDER_MODE_0_5_V, &u32Temp, &u32TempRes);
  Serial.print("\nRead ch 2 V : ");
  Serial.print(u32Temp, DEC);
  Serial.print("\t");
  Serial.print(u32TempRes, HEX);

  shield.ADCRead(AD_EXTENDER_ADC_CH_3, AD_EXTENDER_MODE_0_10_V, &u32Temp, &u32TempRes);
  Serial.print("\nRead ch 3 V : ");
  Serial.print(u32Temp, DEC);
  Serial.print("\t");
  Serial.print(u32TempRes, HEX);

  shield.ADCRead(AD_EXTENDER_ADC_CH_4, AD_EXTENDER_MODE_0_10_V, &u32Temp, &u32TempRes);
  Serial.print("\nRead ch 4 V : ");
  Serial.print(u32Temp, DEC);
  Serial.print("\t");
  Serial.print(u32TempRes, HEX);
  delay(2000);

}
