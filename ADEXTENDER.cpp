// ---------- SYSTEM INCLUDE --------------------------------------------------------------------- //
// ---------- EXTERNAL MODULE INCLUDE ------------------------------------------------------------ //
// N/A
// ---------- PROGRAMMING DEFINITION INCLUDE ----------------------------------------------------- //
// N/A
// ---------- EXTERN OBJECT ---------------------------------------------------------------------- //
// N/A
// ---------- PUBLIC INTERFACE METHOD ------------------------------------------------------------ //
#ifdef __cplusplus
extern "C" {
#endif
	#include <stdlib.h>
	#include <string.h>
	#include <inttypes.h>
	//#include "utility/twi.h"
	//#include "AD7124.h"
#ifdef __cplusplus
}
#endif

#include <Wire.h>
#include <Spi.h>
#include "AD7124_regs.h"
#include "ADEXTENDER.h"
// ---------- PUBLIC METHOD (FUNCTION PROTOTYPE) ------------------------------------------------- //
// N/A
// ---------- PUBLIC DATA ----------------------------------------------------------------------- //
// N/A
// ---------- PRIVATE METHOD (FUNCTION PROTOTYPE) ----------------------------------------------- //
// N/A
// ---------- PRIVATE DATA ---------------------------------------------------------------------- //
// N/A
// ---------- PRIVATE PROGRAMMING DEFINE -------------------------------------------------------- //
//#define DEBUG_LIB
#define DAC_FACTOR						100
#define CURRENT_TO_VOLTAGE_FACTOR		250
#define SS_PIN							10
#define AD7124_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */
// ---------- PRIVATE MACRO DEFINITION ---------------------------------------------------------- //
// N/A
// ---------- SOURCE FILE IMPLEMENTATION -------------------------------------------------------- //


//=========== Public Method ======================================================================//
ADEXTENDER::ADEXTENDER() {
	// TODO Auto-generated constructor stub
	Init();
}

ADEXTENDER::ADEXTENDER(/*uint8_t ADCAddress, */uint8_t DACAddress) {
	// TODO Auto-generated constructor stub
	Init(/*ADCAddress, */DACAddress);
}

ADEXTENDER::~ADEXTENDER() {
	// TODO Auto-generated destructor stub
	Wire.end();
	SPI.end();
}

uint8_t ADEXTENDER::Init(void) {
	/*u8ADCAddress = ADS101x_DEFAULT_ADDRESS;*/
	u8DACAddress = AD533X_DEFAULT_ADDRESS;
	Wire.begin();
	SPI.begin();

	pinMode(SS_PIN, OUTPUT);

	// Default Voltage 5V 1 mV per resolution
	u16Vref = 5000;
	// Calculate resolution
	u8ADCResolution = (uint8_t)(((uint32_t)(u16Vref) * DAC_FACTOR) / 4095);

	return 0;
}

uint8_t ADEXTENDER::Init(/*uint8_t ADCAddress, */uint8_t DACAddress) {
	u8DACAddress = DACAddress;
	Wire.begin();
	SPI.begin();

	pinMode(SS_PIN, OUTPUT);
	// Default Voltage 5V 1 mV per resolution
	u16Vref = 5000;
	// Calculate resolution
	u8ADCResolution = (uint8_t)(((uint32_t)(u16Vref) * DAC_FACTOR) / 4095);
	return 0;
}

uint8_t ADEXTENDER::ADCSetZero(uint16_t u16Value) {
	u16ADCZero = u16Value;
	return 0;
}

uint8_t ADEXTENDER::ADCSetSpan(uint16_t u16Value) {
	u16ADCSpan = u16Value;
	return 0;
}

uint8_t ADEXTENDER::DACSetZero(uint16_t u16Value) {
	u16DACZero = u16Value;
	return 0;
}

uint8_t ADEXTENDER::DACSetSpan(uint16_t u16Value) {
	u16DACSpan = u16Value;
	return 0;
}

uint8_t ADEXTENDER::DACUpdateVaule(
		AD533X_OUT_SEL eOutSelect,
		AD533X_POWER_DOWN_MODE ePowerMode,
		uint8_t bCLR,
		uint8_t bLDAC,
		uint16_t u16Value) {
	uint8_t u8Status;
	uint8_t u8Reg;
	uint16_t u16WriteValue;

	// Insert Pointer byte
	u8Reg = eOutSelect;
	// Insert Value
	u16WriteValue = (uint16_t)(ePowerMode << 14)
			| (uint16_t)(bCLR << 13)
			| (uint16_t)(bLDAC << 12)
			| u16Value;

	// Write value to device
	u8Status = deviceWrite(u8DACAddress, u8Reg, u16WriteValue);
	return u8Status;
}

uint8_t ADEXTENDER::DACUpdateVaule(
		AD533X_OUT_SEL eOutSelect,
		uint16_t u16Value) {
	return DACUpdateVaule(
			eOutSelect,
			AD533X_POWER_DOWN_NORMAL,
			1,
			0,
			u16Value);
}

uint8_t ADEXTENDER::DACSetOutByVoltage(E_ADEXTENDER_OUT_CHANNEL eCh,
		E_ADEXTENDER_OUT_MODE eMode, uint16_t u16Volage) {
	uint16_t u16DACValue;

	u16DACValue = ((uint32_t)u16Volage * (uint32_t)DAC_FACTOR) / u8ADCResolution;

	return DACUpdateVaule(
			(AD533X_OUT_SEL)eCh,
			AD533X_POWER_DOWN_NORMAL,
			1,
			0,
			u16DACValue);
}

uint8_t ADEXTENDER::DACSetOutByCurrent(E_ADEXTENDER_OUT_CHANNEL eCh,
		E_ADEXTENDER_OUT_MODE eMode, uint16_t u16Current) {
	uint16_t u16DACValue;
	uint16_t u16mAConvert;

	if (u16Current > 20) {
		return 255;
	}
	u16mAConvert = u16Current * CURRENT_TO_VOLTAGE_FACTOR;

	return DACSetOutByVoltage(eCh, AD_EXTENDER_OUT_MODE_0_5_V, u16mAConvert);

}

uint8_t ADEXTENDER::ADCReset(void) {

	digitalWrite(SS_PIN, LOW);

	for(int i = 0; i < 8; i++) {
		SPI.transfer(0xFF);
	}

	digitalWrite(SS_PIN, HIGH);
}

uint8_t ADEXTENDER::ADCReadStatus(void) {
	uint32_t u32ReadValue;

#ifndef DEBUG_LIB
	ADCReadRegister(AD7124_Status, 1, &u32ReadValue);
#else
	ADCReadRegisterWithPrintOut(AD7124_Status, 1, &u32ReadValue);
#endif

	return (uint8_t)u32ReadValue;

	//return deviceSPIReadU8(AD7124_STATUS_REG);
}

uint8_t ADEXTENDER::ADCReadMCLK(void) {

	return deviceSPIReadU8(AD7124_Mclk_Count);
}

uint32_t ADEXTENDER::ADCReadERROR_EN(void) {

	return deviceSPIReadU24(AD7124_ERREN_REG);
}

uint32_t ADEXTENDER::ADCReadError(void) {

	return deviceSPIReadU24(AD7124_Error);
}

uint8_t ADEXTENDER::ADCRead(E_ADEXTENDER_ADC_CH eChannel, uint32_t pOut[]) {
	uint8_t u8Ready;
	uint8_t u8Temp[4];
	uint32_t timeout;
	uint32_t u32ReadValue;

	aDCStartConversion(eChannel);

	pOut[0] = 0xFFFFFFFF;
	timeout = 10;
	u8Ready = ADCReadStatus();
	u8Ready = (u8Ready & AD7124_STATUS_REG_RDY) >> 7;

	while(u8Ready && --timeout) {
		u8Ready = ADCReadStatus();
		u8Ready = (u8Ready & AD7124_STATUS_REG_RDY) >> 7;
		delay(10);

	}

	if (timeout == 0) {
		return 255;
	}

	//pOut[0] = deviceSPIReadU24(AD7124_Data);

    /*deviceSPIRead(AD7124_Data, 3, u8Temp);

    //pOut[0] = (uint32_t)(u8Temp[0]) << 24;
    //pOut[0] |= (uint32_t)(u8Temp[1]) << 16;
    pOut[0] = (uint32_t)(u8Temp[0]) << 16;
    pOut[0] |= (uint32_t)(u8Temp[1])<< 8;
    pOut[0] |= (uint32_t)(u8Temp[2]);

    /*deviceSPIRead(AD7124_Data, 4, u8Temp);

    pOut[0] = (uint32_t)(u8Temp[0]) << 24;
    pOut[0] |= (uint32_t)(u8Temp[1]) << 16;
    pOut[0] |= (uint32_t)(u8Temp[2])<< 8;
    pOut[0] |= (uint32_t)(u8Temp[3]);*/




	ADCReadRegister(AD7124_Data, 2, pOut);

	//return (uint16_t)u32ReadValue;

	return 0;

}

uint8_t ADEXTENDER::ADCReadWithPrintOut(E_ADEXTENDER_ADC_CH eChannel, uint32_t pOut[]) {
	uint8_t u8Ready;
	uint8_t u8Temp[4];
	uint32_t timeout;
	uint32_t u32ReadValue;

	Serial.print("\nStart convertion on ch : ");
	Serial.print(eChannel, HEX);

	aDCStartConversion(eChannel);

	pOut[0] = 0xFFFFFFFF;
	timeout = 100;
	u8Ready = ADCReadStatus();
	u8Ready = (u8Ready & AD7124_STATUS_REG_RDY) >> 7;
	Serial.print("\nRead Status : ");
	Serial.print(u8Ready, HEX);

	while(u8Ready && --timeout) {
		u8Ready = ADCReadStatus();
		u8Ready = (u8Ready & AD7124_STATUS_REG_RDY) >> 7;
		Serial.print("\nRead Status : ");
		Serial.print(u8Ready, HEX);
		delay(100);

	}

	if (timeout == 0) {
		ADCReadRegisterWithPrintOut(AD7124_Data, 3, pOut);
		return 255;
	}


	ADCReadRegisterWithPrintOut(AD7124_Data, 3, pOut);


	return 0;

}

uint8_t ADEXTENDER::ADCConfigControl(uint16_t u16Value) {
	uint8_t u8Temp[2];
	uint8_t u8Status;
	uint16_t u16Temp;

	//u8Temp[0] = (uint8_t)((u16Value >> 8) & 0xFF);
	//u8Temp[1] = (uint8_t)(u16Value & 0xFF);

	//deviceSPIWrite(AD7124_ADC_Control, u8Temp, 2);

#ifndef DEBUG_LIB
	u8Status = ADCWriteRegister(AD7124_ADC_Control, (uint32_t)u16Value, 2, 1);
#else
	u8Status = ADCWriteRegisterWithPrintOut(AD7124_ADC_Control, (uint32_t)u16Value, 2, 1);
#endif

	return u8Status;
}

uint8_t ADEXTENDER::ADCSetChannelControl(E_ADEXTENDER_ADC_CH eCh, E_ADEXTENDER_STA eSta) {
	uint16_t u16CurrentActiveConfigValue;
	AD7124_CHANNEL_REGISTER *psChannelConfig;

	u16CurrentActiveConfigValue = ADCGetConfigChannel(eCh);
	psChannelConfig = (AD7124_CHANNEL_REGISTER*) &u16CurrentActiveConfigValue;
	ADCConfigChannel(
			eCh,
			(uint8_t)psChannelConfig->SETUP,
			eSta);

	return 0;
}

uint8_t ADEXTENDER::ADCConfigChannel(E_ADEXTENDER_ADC_CH eCh, uint8_t u8config, E_ADEXTENDER_STA en) {
	uint8_t u8TempBuffer[8];
	uint16_t u16Temp;
	uint8_t u8WriteChannel;
	uint8_t u8Status;

	// Write Channel configuration
	u16Temp = en <<  15 |
			AD7124_CH_MAP_REG_SETUP(u8config) |
			AD7124_CH_MAP_REG_AINP(eCh) |
			AD7124_CH_MAP_REG_AINM(17);

/*	Serial.print("\nWrite ch ");
	Serial.print(eCh, DEC);
	Serial.print(" config control : ");
	Serial.print(u16Temp, HEX);*/
	//u8TempBuffer[0] = (uint8_t)((u16Temp >> 8) & 0xFF);
	//u8TempBuffer[1] = (uint8_t)(u16Temp & 0xFF);

	u8WriteChannel = (uint8_t)(AD7124_Channel_0 + eCh);

	//deviceSPIWrite(AD7124_Channel_0 + eCh, u8TempBuffer, 2);
#ifndef DEBUG_LIB
	u8Status = ADCWriteRegister((ad7124_reg_access)u8WriteChannel, (uint32_t)u16Temp, 2, 1);
#else
	u8Status = ADCWriteRegisterWithPrintOut((ad7124_reg_access)u8WriteChannel, (uint32_t)u16Temp, 2, 1);
#endif
	return u8Status;
}

uint16_t ADEXTENDER::ADCGetConfigChannel(E_ADEXTENDER_ADC_CH eCh) {
	uint32_t u32ReadValue;
	uint8_t u8ReadEnrty;

	u8ReadEnrty = AD7124_Channel_0 + eCh;


	// Read Channel configuration
	//return deviceSPIReadU16(AD7124_Channel_0 + eCh);
#ifndef DEBUG_LIB
	ADCReadRegister((ad7124_reg_access)u8ReadEnrty, 2, &u32ReadValue);
#else
	ADCReadRegisterWithPrintOut((ad7124_reg_access)u8ReadEnrty, 2, &u32ReadValue);
#endif
	return (uint16_t)u32ReadValue;
}

uint8_t ADEXTENDER::ADCSetConfig(uint8_t u8Entry, uint8_t vrefSel, uint8_t pga) {
	uint8_t u8TempBuffer[8];
	uint16_t u16Temp;
	uint8_t u8Status;
	uint8_t u8WriteConfigEntry;

	// Write Channel configuration
	u16Temp = AD7124_CFG_REG_REF_SEL(vrefSel) |
			AD7124_CFG_REG_REF_BUFP |
			AD7124_CFG_REG_REF_BUFM |
			AD7124_CFG_REG_AIN_BUFP |
			AD7124_CFG_REG_AINN_BUFM |
			AD7124_CFG_REG_PGA(pga);

	//u8TempBuffer[0] = (uint8_t)((u16Temp >> 8) & 0xFF);
	//u8TempBuffer[1] = (uint8_t)(u16Temp & 0xFF);

	u8WriteConfigEntry = (uint8_t)(AD7124_Config_0 + u8Entry);

	//deviceSPIWrite(AD7124_Config_0 + u8Entry, u8TempBuffer, 2);

#ifndef DEBUG_LIB
	u8Status = ADCWriteRegister((ad7124_reg_access)u8WriteConfigEntry, (uint32_t)u16Temp, 2, 1);
#else
	u8Status = ADCWriteRegisterWithPrintOut((ad7124_reg_access)u8WriteConfigEntry, (uint32_t)u16Temp, 2, 1);
#endif

	return u8Status;
}

uint16_t ADEXTENDER::ADCGetConfig(uint8_t u8Entry) {
	uint32_t u32ReadValue;
	uint8_t u8ReadEnrty;
	if (u8Entry > 7) {
		return 255;
	}

	u8ReadEnrty = AD7124_Config_0 + u8Entry;


#ifndef DEBUG_LIB
	ADCReadRegisterWithPrintOut((ad7124_reg_access)u8ReadEnrty, 2, &u32ReadValue);
#else
	ADCReadRegisterWithPrintOut((ad7124_reg_access)u8ReadEnrty, 2, &u32ReadValue);
#endif
	return (uint16_t)u32ReadValue;
}

uint8_t ADEXTENDER::ADCWriteErrorEn(uint32_t u32Value) {
	uint8_t u8TempBuffer[8];
	u8TempBuffer[0] = (uint8_t)((u32Value >> 16) & 0xFF);
	u8TempBuffer[1] = (uint8_t)((u32Value >> 8) & 0xFF);
	u8TempBuffer[2] = (uint8_t)(u32Value & 0xFF);

	deviceSPIWrite(AD7124_ERREN_REG, u8TempBuffer, 3);
}


//=========== Private Method ======================================================================//

uint8_t ADEXTENDER::aDCStartConversion(E_ADEXTENDER_ADC_CH eChannel) {
	uint32_t u32Status;
	uint16_t u16ConfigValue;
	uint16_t u16CurrentActiveConfigValue;
	uint8_t u8TempBuffer[2];
	AD7124_STATUS_REGISTER *psStatus;


	// Setting configuration
	u16ConfigValue = (uint16_t)(AD7124_ADC_CTRL_REG_DOUT_RDY_DEL) |
			(uint16_t)(AD7124_ADC_CTRL_REG_REF_EN) |
			//(uint16_t)(AD7124_ADC_CTRL_REG_CONT_READ) |
			(uint16_t)(AD7124_ADC_CTRL_REG_POWER_MODE(3)) |
			(uint16_t)(AD7124_ADC_CTRL_REG_MODE(1)) |
			(uint16_t)(AD7124_ADC_CTRL_REG_CLK_SEL(1));

	// Read Current active channel
	//u8Status = ADCReadStatus();

	ADCReadRegisterWithPrintOut(AD7124_Status, 1, &u32Status);
	psStatus = (AD7124_STATUS_REGISTER*) &u32Status;
	//Serial.print("Read Active Ch : ");

	if (eChannel != psStatus->CH_ACTIVE) {
		// Disable current status
		ADCSetChannelControl((E_ADEXTENDER_ADC_CH)psStatus->CH_ACTIVE, AD_DISABLE);
	}

	// Enable channel
	ADCSetChannelControl(eChannel, AD_ENABLE);

	// Write value to ADC Control
	//u8TempBuffer[0] = (uint8_t)((u16ConfigValue >> 8) & 0xFF);
	//u8TempBuffer[1] = (uint8_t)(u16ConfigValue & 0xFF);
	//deviceSPIWrite(AD7124_ADC_Control, u8TempBuffer, 2);
	ADCConfigControl(u16ConfigValue);
	//ADCConfigControl(0x0144);

	return 0;
}

uint8_t ADEXTENDER::aDCGetConversionValue(uint16_t data[]) {
	uint8_t u8TempBuffer[4];

	deviceSPIRead(AD7124_STATUS_REG, 3, u8TempBuffer);

	return 0;

}

uint8_t ADEXTENDER::deviceWrite(uint8_t u8DevAddr,uint8_t u8Reg, uint16_t u16Value) {
	Wire.beginTransmission(u8DevAddr);
	Wire.write(u8Reg);
	Wire.write((uint8_t)(u16Value >> 8));
	Wire.write((uint8_t)u16Value);
	return Wire.endTransmission();
}

uint8_t ADEXTENDER::deviceRead(uint8_t u8DevAddr, uint8_t u8Reg, uint16_t pu16Value[]) {
	uint8_t u8ReadLength;
	uint16_t u16ReadValue;
	Wire.beginTransmission(u8DevAddr);
	Wire.write(u8Reg);
	Wire.endTransmission();
	u8ReadLength = Wire.requestFrom(u8DevAddr, (uint8_t)2);
	if (u8ReadLength > 0) {
		u16ReadValue = (uint16_t)(Wire.read()) << 8;
		u16ReadValue |= (uint16_t)(Wire.read());
	}
	pu16Value[0] = u16ReadValue;

	return u8ReadLength;
}

uint8_t ADEXTENDER::deviceSPIWrite(uint8_t u8Reg, uint8_t pu8Data[], uint8_t u8Length) {
	uint8_t i;
	uint8_t u8ComReg;

	u8ComReg = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR | AD7124_COMM_REG_RA(u8Reg);

	//SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

	digitalWrite(SS_PIN, LOW);


	SPI.transfer(u8ComReg);

	for(i = 0; i < u8Length; i++) {
		SPI.transfer(pu8Data[i]);
	}

	digitalWrite(SS_PIN, HIGH);

	//SPI.endTransaction();

	return 0;
}

uint8_t ADEXTENDER::deviceSPIRead(uint8_t u8Reg, uint8_t u8Length, uint8_t pu8DataOut[]) {
	uint8_t i;
	uint8_t u8ComReg;

	u8ComReg = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA(u8Reg);

	//SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

	digitalWrite(SS_PIN, LOW);

	SPI.transfer(u8ComReg);

	for(i = 0; i < u8Length; i++) {
		pu8DataOut[i] = SPI.transfer(0x00);
	}

	digitalWrite(SS_PIN, HIGH);

	//SPI.endTransaction();

	return 0;
}

uint8_t ADEXTENDER::ADCWriteRegister(ad7124_reg_access eRegister,
		uint32_t u32Value, uint8_t u8Size, uint8_t u8Verify) {
  uint8_t u8Msg[16];
  uint8_t i;
  uint32_t u32ReadValue;
  uint8_t u8Status;
  // Read register
  u8Msg[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR | AD7124_COMM_REG_RA(eRegister);
  if (u8Size == 3) {
    u8Msg[1] = (u32Value >> 16) & 0xFF;
    u8Msg[2] = (u32Value >> 8) & 0xFF;
    u8Msg[3] = u32Value & 0xFF;
    u8Msg[4] = computeCRC8(u8Msg, u8Size + 1);
  }
  else if (u8Size == 2) {
    u8Msg[1] =( u32Value >> 8) & 0xFF;
    u8Msg[2] = u32Value & 0xFF;
    u8Msg[3] = computeCRC8(u8Msg, u8Size + 1);
  }
  else {
    u8Msg[1] = u32Value;
    u8Msg[2] = computeCRC8(u8Msg, u8Size + 1);
  }

  digitalWrite(SS_PIN, LOW);
  for(i = 0; i < u8Size + 2; i++) {
    SPI.transfer(u8Msg[i]);

  }

  digitalWrite(SS_PIN, HIGH);

  // Calculate CRC
  Serial.print(computeCRC8(u8Msg, u8Size + 2), HEX);

  if (u8Verify == 1) {
	  u8Status = ADCReadRegister(eRegister, u8Size, &u32ReadValue);
	  if ((u8Status == 255) || (u32Value != u32ReadValue)) {
		  // Verify fail
		  return 255;
	  }
  }

  return 0;

}

uint8_t ADEXTENDER::ADCWriteRegisterWithPrintOut(ad7124_reg_access eRegister,
		uint32_t u32Value, uint8_t u8Size, uint8_t u8Verify) {
  uint8_t u8Msg[16];
  uint8_t i;
  uint32_t u32ReadValue;
  uint8_t u8Status;
  // Read register
  u8Msg[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR | AD7124_COMM_REG_RA(eRegister);
  if (u8Size == 3) {
    u8Msg[1] = (u32Value >> 16) & 0xFF;
    u8Msg[2] = (u32Value >> 8) & 0xFF;
    u8Msg[3] = u32Value & 0xFF;
    u8Msg[4] = computeCRC8(u8Msg, u8Size + 1);
  }
  else if (u8Size == 2) {
    u8Msg[1] =( u32Value >> 8) & 0xFF;
    u8Msg[2] = u32Value & 0xFF;
    u8Msg[3] = computeCRC8(u8Msg, u8Size + 1);
  }
  else {
    u8Msg[1] = u32Value;
    u8Msg[2] = computeCRC8(u8Msg, u8Size + 1);
  }

  Serial.print("\nWrite Register ");
  Serial.print(eRegister, HEX);
  Serial.print("\t\t");



  digitalWrite(SS_PIN, LOW);
  for(i = 0; i < u8Size + 2; i++) {
    Serial.print(" 0x");
    SPI.transfer(u8Msg[i]);
    Serial.print(u8Msg[i], HEX);

  }

  digitalWrite(SS_PIN, HIGH);

  //
  Serial.print("\t CRC = ");
  Serial.print(computeCRC8(u8Msg, u8Size + 2), HEX);

  if (u8Verify == 1) {
	  u8Status = ADCReadRegister(eRegister, u8Size, &u32ReadValue);
	  if ((u8Status == 255) || (u32Value != u32ReadValue)) {
		  // Verify fail
		  Serial.print("\t Verify fail ");
		  Serial.print(u32Value ,HEX);
		  Serial.print(" ");
		  Serial.print(u32ReadValue, HEX);
		  return 255;
	  }
	  Serial.print("\t Verify pass");
  }

  return 0;

}

uint8_t ADEXTENDER::ADCReadRegister(ad7124_reg_access eRegister, uint8_t u8Size, uint32_t u32Out[]) {
	uint8_t u8Msg[16];
	uint8_t i;
	uint8_t u8CRC;
	// Read register
	u8Msg[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD
			| AD7124_COMM_REG_RA(eRegister);

	digitalWrite(SS_PIN, LOW);
	SPI.transfer(u8Msg[0]);

	for (i = 0; i < u8Size + 1; i++) {
		u8Msg[1 + i] = SPI.transfer(0xFF);

	}

	digitalWrite(SS_PIN, HIGH);

	// Calculate CRC
	u8CRC = computeCRC8(u8Msg, u8Size + 2);

	if (u8CRC != 0) {
		return 255;
	}

	if (u8Size == 3) {
		u32Out[0] = (uint32_t)(u8Msg[1]) << 16;
		u32Out[0] |= (uint32_t)(u8Msg[2]) << 8;
		u32Out[0] |= (uint32_t)u8Msg[3];
	} else if (u8Size == 2) {
		u32Out[0] = (uint32_t)(u8Msg[1]) << 8;
		u32Out[0] |= (uint32_t)u8Msg[2];
	} else {
		u32Out[0] = (uint32_t)u8Msg[1];
	}

	return 0;

}

uint8_t ADEXTENDER::ADCReadRegisterWithPrintOut(ad7124_reg_access eRegister, uint8_t u8Size, uint32_t u32Out[]) {
	uint8_t u8Msg[16];
	uint8_t i;
	uint8_t u8CRC;
	// Read register
	u8Msg[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD
			| AD7124_COMM_REG_RA(eRegister);

	Serial.print("\nRead Register ");
	Serial.print(eRegister, HEX);
	Serial.print("\t\t");
	Serial.print("0x");

	digitalWrite(SS_PIN, LOW);
	SPI.transfer(u8Msg[0]);

	for (i = 0; i < u8Size + 1; i++) {
		u8Msg[1 + i] = SPI.transfer(0xFF);
		if (i == u8Size) {
			Serial.print("\t");
		}
		Serial.print(u8Msg[1 + i], HEX);
		Serial.print(" ");

	}

	digitalWrite(SS_PIN, HIGH);

	// Calculate CRC
	u8CRC = computeCRC8(u8Msg, u8Size + 2);
	Serial.print("\t CRC = ");
	Serial.print(u8CRC, HEX);

	if (u8CRC != 0) {
		return 255;
	}

	if (u8Size == 3) {
		u32Out[0] = (uint32_t)(u8Msg[1]) << 16;
		u32Out[0] |= (uint32_t)(u8Msg[2]) << 8;
		u32Out[0] |= (uint32_t)(u8Msg[3]);
	} else if (u8Size == 2) {
		u32Out[0] = (uint32_t)(u8Msg[1]) << 8;
		u32Out[0] |= (uint32_t)u8Msg[2];
	} else {
		u32Out[0] = (uint32_t)u8Msg[1];
	}

	return 0;

}

uint8_t ADEXTENDER::deviceSPIReadU8(uint8_t u8Reg) {
	uint8_t u8TempBuffer;
	uint16_t u16Temp;

	// Read Channel configuration
	deviceSPIRead(u8Reg, 1, &u8TempBuffer);

	return u8TempBuffer;
}

uint16_t ADEXTENDER::deviceSPIReadU16(uint8_t u8Reg) {
	uint8_t u8TempBuffer[4];
	uint16_t u16Temp;

	// Read Channel configuration
	deviceSPIRead(u8Reg, 2, u8TempBuffer);

	u16Temp = (uint16_t)(u8TempBuffer[0]) << 8;
	u16Temp |= u8TempBuffer[1];

	return u16Temp;
}

uint32_t ADEXTENDER::deviceSPIReadU24(uint8_t u8Reg) {
	uint8_t u8TempBuffer[4];
	uint32_t u32Temp;

	// Read Channel configuration
	deviceSPIRead(u8Reg, 3, u8TempBuffer);

	u32Temp = (uint32_t)(u8TempBuffer[0]) << 16;
	u32Temp |= (uint32_t)(u8TempBuffer[1])<< 8;
	u32Temp |= (uint32_t)(u8TempBuffer[2]);

	return u32Temp;
}

uint8_t ADEXTENDER::computeCRC8(uint8_t * pBuf, uint8_t bufSize) {
	uint8_t i   = 0;
	uint8_t crc = 0;

	while(bufSize)
	{
		for(i = 0x80; i != 0; i >>= 1)
		{
			if(((crc & 0x80) != 0) != ((*pBuf & i) != 0)) /* MSB of CRC register XOR input Bit from Data */
			{
				crc <<= 1;
				crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION;
			}
			else
			{
				crc <<= 1;
			}
		}
		pBuf++;
		bufSize--;
	}
	return crc;
}
// ---------- END OF SOURCE FILE IMPLEMENTATION ------------------------------------------------- //
