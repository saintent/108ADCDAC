/*
 * ADEXTENDER.h
 *
 *  Created on: Jun 14, 2016
 *      Author: Prustya
 */

#ifndef ADEXTENDER_H_
#define ADEXTENDER_H_

// ---------- SYSTEM INCLUDE --------------------------------------------------------------------- //
#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include "AD533X_type.h"
#include "ADS101X_type.h"
#include "AD7124_regs.h"
#include <Wire.h>
#include <SPI.h>
// ---------- EXTERNAL MODULE INCLUDE ------------------------------------------------------------ //
// N/A
// ---------- PUBLIC PROGRAMMING DEFINE ---------------------------------------------------------- //
#define AD533X_DEFAULT_ADDRESS 		0x0C
#define ADS101x_DEFAULT_ADDRESS		0x48
#define I2C_WR_BIT					0
#define I2C_RD_BIT					1
// ---------- ENUMERATOR DEFINITION -------------------------------------------------------------- //
typedef enum {
	AD_EXTENDER_OUT_MODE_0_5_V = 0,
	AD_EXTENDER_OUT_MODE_0_10_V,
	AD_EXTENDER_OUT_MODE_0_20_mA,
	AD_EXTENDER_OUT_MODE_4_20_mA
}E_ADEXTENDER_OUT_MODE;

typedef enum {
	AD_EXTENDER_OUT_CH1 = 1,
	AD_EXTENDER_OUT_CH2 = 2
}E_ADEXTENDER_OUT_CHANNAL;
// ---------- TYPEDEF DATA TYPE DEFINITION ------------------------------------------------------- //
// N/A
// ---------- STRUCT OR UNION DATA TYPE DEFINITION ----------------------------------------------- //
// N/A
// ---------- PUBLIC MACRO DEFINITION ------------------------------------------------------------ //
// N/A
// ---------- EXTERN FUNCTION -------------------------------------------------------------------- //
// N/A
// ---------- EXTERN VARIABLE -------------------------------------------------------------------- //
// N/A
// ---------- CLASS DECLARATION ----------------------------------------------------------------- //

class ADEXTENDER {
public:
	ADEXTENDER(void);
	ADEXTENDER(uint8_t ADCAddress, uint8_t DACAddress);
	virtual ~ADEXTENDER();

	uint8_t Init(void);
	uint8_t Init(uint8_t ADCAddress, uint8_t DACAddress);
	uint8_t ADCChangeAddress(uint8_t Address);
	uint8_t DACChangeAddress(uint8_t Address);
	uint8_t ADCSetZero(uint16_t u16Value);
	uint8_t ADCSetSpan(uint16_t u16Value);
	uint8_t DACSetZero(uint16_t u16Value);
	uint8_t DACSetSpan(uint16_t u16Value);

	uint8_t DACUpdateVaule(
			AD533X_OUT_SEL eOutSelect,
			AD533X_POWER_DOWN_MODE ePowerMode,
			uint8_t bCLR,
			uint8_t bLDAC,
			uint16_t u16Value);

	uint8_t DACUpdateVaule(
			AD533X_OUT_SEL eOutSelect,
			uint16_t u16Value);

	uint8_t DACSetOutByVoltage(
			E_ADEXTENDER_OUT_CHANNAL eChannal,
			E_ADEXTENDER_OUT_MODE eMode,
			uint16_t u16Volage);

	uint8_t DACSetOutByCurrent(
			E_ADEXTENDER_OUT_CHANNAL eChannal,
			E_ADEXTENDER_OUT_MODE eMode,
			uint16_t u16Current);

	uint16_t ADCReadStatus(void);
	uint16_t ADCRead(ADS101X_CHANNEL eChannel);
	uint16_t ADCRead(ADS101X_CHANNEL eChannel, uint16_t out[]);
	uint16_t ADCReadTC(ADS101X_CHANNEL eChannel);
private :
	uint8_t aDCStartConversion(ADS101X_CHANNEL eChannel);
	uint8_t aDCStartConversion(ADS101X_CHANNEL eChannel, ADS101X_PGA_GAIN eScale);
	//uint8_t aDCGetStatus(void);
	uint8_t aDCGetConversionValue(uint16_t data[]);
	uint8_t deviceWrite(uint8_t u8DevAddr, uint8_t u8Reg, uint16_t u16Value);
	uint8_t deviceRead(uint8_t u8DevAddr, uint8_t u8Reg, uint16_t pu16Value[]);
	//uint8_t deviceSPIWrite(uint_t u8CMD, uint8_t pu8Data[], uint8_t u8Length);
	//uint8_t deviceSPIRead(uint_t u8CMD, uint8_t pu8Data[], uint8_t u8Length);
	uint8_t u8ADCAddress;
	uint8_t u8DACAddress;
	uint16_t u16ADCZero;
	uint16_t u16ADCSpan;
	uint16_t u16DACZero;
	uint16_t u16DACSpan;
	uint16_t u16Vref;
	uint8_t u8ADCResolution;

	ad7124_device	ad7124Device;
};

// ---------- END OF CLASS DECLARATION ---------------------------------------------------------- //
#endif /* ADEXTENDER_H_ */
