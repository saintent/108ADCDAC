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
	#include "utility/twi.h"
#ifdef __cplusplus
}
#endif

#include <Wire.h>
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
#define DAC_FACTOR						100
#define CURRENT_TO_VOLTAGE_FACTOR		250
// ---------- PRIVATE MACRO DEFINITION ---------------------------------------------------------- //
// N/A
// ---------- SOURCE FILE IMPLEMENTATION -------------------------------------------------------- //

ADEXTENDER::ADEXTENDER() {
	// TODO Auto-generated constructor stub
	Init();
}

ADEXTENDER::ADEXTENDER(uint8_t ADCAddress, uint8_t DACAddress) {
	// TODO Auto-generated constructor stub
	Init(ADCAddress, DACAddress);
}

ADEXTENDER::~ADEXTENDER() {
	// TODO Auto-generated destructor stub
	Wire.end();
}

uint8_t ADEXTENDER::Init(void) {
	u8ADCAddress = ADS101x_DEFAULT_ADDRESS;
	u8DACAddress = AD533X_DEFAULT_ADDRESS;
	Wire.begin();

	// Default Voltage 5V 1 mV per resolution
	u16Vref = 5000;
	// Calculate resolution
	u8ADCResolution = (uint8_t)(((uint32_t)(u16Vref) * DAC_FACTOR) / 4095);


	return 0;
}

uint8_t ADEXTENDER::Init(uint8_t ADCAddress, uint8_t DACAddress) {
	u8ADCAddress = ADCAddress;
	u8DACAddress = DACAddress;
	Wire.begin();

	// Default Voltage 5V 1 mV per resolution
	u16Vref = 5000;
	// Calculate resolution
	u8ADCResolution = (uint8_t)(((uint32_t)(u16Vref) * DAC_FACTOR) / 4095);
	return 0;
}

uint8_t ADEXTENDER::ADCChangeAddress(uint8_t Address) {
	u8ADCAddress = Address;
	return 0;
}

uint8_t ADEXTENDER::DACChangeAddress(uint8_t Address) {
	u8DACAddress = Address;
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

uint8_t ADEXTENDER::DACSetOutByVoltage(E_ADEXTENDER_OUT_CHANNAL eCh,
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

uint8_t ADEXTENDER::DACSetOutByCurrent(E_ADEXTENDER_OUT_CHANNAL eCh,
		E_ADEXTENDER_OUT_MODE eMode, uint16_t u16Current) {
	uint16_t u16DACValue;
	uint16_t u16mAConvert;

	if (u16Current > 20) {
		return 255;
	}
	u16mAConvert = u16Current * CURRENT_TO_VOLTAGE_FACTOR;

	return DACSetOutByVoltage(eCh, AD_EXTENDER_OUT_MODE_0_5_V, u16mAConvert);

}

uint16_t ADEXTENDER::ADCReadStatus(void) {
	uint16_t u16Status;

	deviceRead(u8ADCAddress, ADS101X_REG_CONFIG, &u16Status);

	return u16Status;
}

uint16_t ADEXTENDER::ADCRead(ADS101X_CHANNEL eChannel) {
	uint16_t u16Status;
	uint16_t u16Value;
	aDCStartConversion(eChannel);

	u16Status = ADCReadStatus();
/*	while(!u8Status) {
		u8Status = aDCGetStatus();
	}*/

	deviceRead(u8ADCAddress, ADS101X_REG_CONVERSION, &u16Value);

	return u16Value;

	//aDCGetConversionValue(&u16Value);

}

uint16_t ADEXTENDER::ADCRead(ADS101X_CHANNEL eChannel, uint16_t pu16Out[]) {
	uint16_t u16Status;
	uint16_t ConversionCmp;
	//uint16_t u16Value;
	aDCStartConversion(eChannel);

	//u8Status = aDCGetStatus();
	u16Status = ADCReadStatus();
	ConversionCmp = (u16Status & 0x8000) >> 15;
	while(!ConversionCmp) {
		u16Status = ADCReadStatus();
		ConversionCmp = (u16Status & 0x8000) >> 15;
	}
	deviceRead(u8ADCAddress, ADS101X_REG_CONVERSION, pu16Out);


	return u16Status;
	
}

uint16_t ADEXTENDER::ADCReadTC(ADS101X_CHANNEL eChannel) {
	uint16_t u16Status;
	uint16_t ConversionCmp;
	uint16_t u16ReadValue;
	
	//uint16_t u16Value;
	aDCStartConversion(eChannel, ADS101X_PGA_GAIN_16);

	//u8Status = aDCGetStatus();
	u16Status = ADCReadStatus();
	ConversionCmp = (u16Status & 0x8000) >> 15;
	while(!ConversionCmp) {
		u16Status = ADCReadStatus();
		ConversionCmp = (u16Status & 0x8000) >> 15;
	}
	deviceRead(u8ADCAddress, ADS101X_REG_CONVERSION, &u16ReadValue);


	return u16ReadValue;

}

uint8_t ADEXTENDER::aDCStartConversion(ADS101X_CHANNEL eChannel) {
	uint8_t u8Status;
	uint16_t u16ConfigValue;


	// Setting configuration
	u16ConfigValue =  (uint16_t) (1 << 15)					// Bit 15 Begin a single conversion
					| (uint16_t) (eChannel << 12)				// Bit [14:12]Input multiplexer configuration
					| (uint16_t) (ADS101X_PGA_GAIN_2P3 << 9)	// Bit [11:9] PGA configuration
					| (uint16_t) (ADS101X_MODE_SIGLE << 8)			// Bit 8 Device operating mode
			 	 	| (uint16_t) (ADS101X_DATA_RATE_128_SPS << 5)	// Bit [7:5] Data rate
					| (uint16_t) (0x03);						// Bit [1:0] Disable Comparator

	// Write value to device
	u8Status = deviceWrite(u8ADCAddress, ADS101X_REG_CONFIG, u16ConfigValue);

	return u8Status;
}

uint8_t ADEXTENDER::aDCStartConversion(ADS101X_CHANNEL eChannel, ADS101X_PGA_GAIN eScale) {
	uint8_t u8Status;
	uint16_t u16ConfigValue;


	// Setting configuration
	u16ConfigValue =  (uint16_t) (1 << 15)					// Bit 15 Begin a single conversion
					| (uint16_t) (eChannel << 12)				// Bit [14:12]Input multiplexer configuration
					| (uint16_t) (eScale << 9)	// Bit [11:9] PGA configuration
					| (uint16_t) (ADS101X_MODE_SIGLE << 8)			// Bit 8 Device operating mode
			 	 	| (uint16_t) (ADS101X_DATA_RATE_128_SPS << 5)	// Bit [7:5] Data rate
					| (uint16_t) (0x03);						// Bit [1:0] Disable Comparator

	// Write value to device
	u8Status = deviceWrite(u8ADCAddress, ADS101X_REG_CONFIG, u16ConfigValue);

	return u8Status;
}

/*uint8_t ADEXTENDER::aDCGetStatus(void) {
	uint8_t u8ReadLength;
	uint8_t status;
	uint8_t temp[2];
	uint8_t u8DeviceAddress;

	u8DeviceAddress = u8ADCAddress | I2C_WR_BIT;

	// Insert Pointer byte
	temp[0] = (uint8_t) ADS101X_REG_CONFIG;
	// Send Read pointer
	Wire.beginTransmission(u8DeviceAddress);
	Wire.write(temp[0]);
	Wire.endTransmission();
	// Read value
	u8DeviceAddress = u8ADCAddress | I2C_RD_BIT;
	u8ReadLength = Wire.requestFrom(u8DeviceAddress, (uint8_t)2);
	// Get status
	status = (temp[0] >> 7) & 1;
	return status;
}*/

uint8_t ADEXTENDER::aDCGetConversionValue(uint16_t data[]) {
	uint8_t u8ReadLength;
	uint8_t status;
	uint8_t temp[2];
	uint8_t u8DeviceAddress;
	uint16_t u16Vaule;

	u8DeviceAddress = u8ADCAddress | I2C_WR_BIT;



	// Insert Pointer byte
	temp[0] = (uint8_t) ADS101X_REG_CONVERSION;
	// Send Read pointer
	Wire.beginTransmission(u8DeviceAddress);
	Wire.write(temp[0]);
	Wire.endTransmission();
	// Read value
	u8DeviceAddress = u8ADCAddress | I2C_RD_BIT;
	u8ReadLength = Wire.requestFrom(u8DeviceAddress, (uint8_t)2);
	// Get value
	u16Vaule = (uint16_t)(temp[0]) << 4;
	u16Vaule |= (uint16_t)(temp[1] >> 4) & 0x0F;
	data[0] = u16Vaule;
	return u8ReadLength;
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

// ---------- END OF SOURCE FILE IMPLEMENTATION ------------------------------------------------- //
