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
#include "AD7124_def.h"
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
}E_ADEXTENDER_OUT_CHANNEL;

typedef enum {
	AD_EXTENDER_ADC_CH_0 = 0,
	AD_EXTENDER_ADC_CH_1,
	AD_EXTENDER_ADC_CH_2,
	AD_EXTENDER_ADC_CH_3,
	AD_EXTENDER_ADC_CH_4
}E_ADEXTENDER_ADC_CH;

typedef enum {
	AD_DISABLE = 0,
	AD_ENABLE
}E_ADEXTENDER_STA;
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
	ADEXTENDER(/*uint8_t ADCAddress, */uint8_t DACAddress);
	virtual ~ADEXTENDER();

	uint8_t Init(void);
	uint8_t Init(uint8_t DACAddress);
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
			E_ADEXTENDER_OUT_CHANNEL eChannal,
			E_ADEXTENDER_OUT_MODE eMode,
			uint16_t u16Volage);

	uint8_t DACSetOutByCurrent(
			E_ADEXTENDER_OUT_CHANNEL eChannal,
			E_ADEXTENDER_OUT_MODE eMode,
			uint16_t u16Current);

	uint8_t ADCReset(void);

	uint8_t ADCReadStatus(void);

	uint8_t ADCRead(E_ADEXTENDER_ADC_CH eChannel, uint32_t pOut[]);

	uint8_t ADCConfigControl(uint16_t u16Value);

	uint8_t  ADCSetChannelControl(E_ADEXTENDER_ADC_CH eCh, E_ADEXTENDER_STA eSta);
	uint8_t ADCConfigChannel(E_ADEXTENDER_ADC_CH eCh, uint8_t u8config, E_ADEXTENDER_STA en);
	uint16_t ADCGetConfigChannel(E_ADEXTENDER_ADC_CH eCh);

	uint8_t ADCSetConfig(uint8_t u8Entry, uint8_t vrefSel, uint8_t pga);
	uint16_t ADCGetConfig(uint8_t u8Entry);

	uint32_t ADCReadERROR_EN(void);

private :
	uint8_t aDCStartConversion(E_ADEXTENDER_ADC_CH eChannel);

	uint8_t aDCGetConversionValue(uint16_t data[]);

	uint8_t deviceWrite(uint8_t u8DevAddr, uint8_t u8Reg, uint16_t u16Value);
	uint8_t deviceRead(uint8_t u8DevAddr, uint8_t u8Reg, uint16_t pu16Value[]);
public :
	uint8_t deviceSPIWrite(uint8_t u8Reg, uint8_t pu8Data[], uint8_t u8Length);
	uint8_t deviceSPIRead(uint8_t u8Reg, uint8_t u8Length, uint8_t pu8DataOut[]);

	uint8_t deviceSPIReadU8(uint8_t u8Reg);
	uint16_t deviceSPIReadU16(uint8_t u8Reg);
	uint32_t deviceSPIReadU24(uint8_t u8Reg);

	uint8_t computeCRC8(uint8_t pBuf[], uint8_t bufSize);

private :
	//uint8_t u8ADCAddress;
	uint8_t u8DACAddress;
	uint16_t u16ADCZero;
	uint16_t u16ADCSpan;
	uint16_t u16DACZero;
	uint16_t u16DACSpan;
	uint16_t u16Vref;
	uint8_t u8ADCResolution;

	//ad7124_device	ad7124Device;
};

// ---------- END OF CLASS DECLARATION ---------------------------------------------------------- //
#endif /* ADEXTENDER_H_ */
