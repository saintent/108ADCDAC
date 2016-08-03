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
	uint8_t ADCReadMCLK(void);

	uint32_t ADCReadError(void);

	uint8_t ADCRead(E_ADEXTENDER_ADC_CH eChannel, uint32_t pOut[]);

	uint8_t ADCConfigControl(uint16_t u16Value);

	uint8_t  ADCSetChannelControl(E_ADEXTENDER_ADC_CH eCh, E_ADEXTENDER_STA eSta);
	uint8_t ADCConfigChannel(E_ADEXTENDER_ADC_CH eCh, uint8_t u8config, E_ADEXTENDER_STA en);
	uint16_t ADCGetConfigChannel(E_ADEXTENDER_ADC_CH eCh);

	uint8_t ADCSetConfig(uint8_t u8Entry, uint8_t vrefSel, uint8_t pga);
	uint16_t ADCGetConfig(uint8_t u8Entry);

	uint8_t ADCWriteErrorEn(uint32_t u32Value);
	uint32_t ADCReadERROR_EN(void);


private :
	uint8_t aDCStartConversion(E_ADEXTENDER_ADC_CH eChannel);

	uint8_t aDCGetConversionValue(uint16_t data[]);

	uint8_t deviceWrite(uint8_t u8DevAddr, uint8_t u8Reg, uint16_t u16Value);
	uint8_t deviceRead(uint8_t u8DevAddr, uint8_t u8Reg, uint16_t pu16Value[]);
public :
	uint8_t deviceSPIWrite(uint8_t u8Reg, uint8_t pu8Data[], uint8_t u8Length);
	uint8_t deviceSPIRead(uint8_t u8Reg, uint8_t u8Length, uint8_t pu8DataOut[]);
	uint8_t ADCReadRegister(ad7124_reg_access eRegister, uint8_t u8Size, uint32_t u32Out[]);
	uint8_t ADCWriteRegister(ad7124_reg_access eRegister,
			uint32_t u32Value, uint8_t u8Size, uint8_t u8Verify);
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

//	S_AD7124_REG sAD7124Register[29] = {
//			{ 0x00, AD7124_REG_U8, 	AD7124_REG_R, 	0x00 		}, /* AD7124_Status */
//			{ 0x01, AD7124_REG_U16,	AD7124_REG_RW, 	0x0000 		}, /* AD7124_ADC_Control */
//			{ 0x02, AD7124_REG_U24,	AD7124_REG_R,	0x000000	}, /* AD7124_Data */
//			{ 0x03, AD7124_REG_U16,	AD7124_REG_RW,	0x0000  	}, /* AD7124_IOCon1 */
//			{ 0x04, AD7124_REG_U16,	AD7124_REG_RW,	0x0000  	}, /* AD7124_IOCon2 */
//			{ 0x05, AD7124_REG_U8, 	AD7124_REG_R, 	0x02		}, /* AD7124_ID */
//			{ 0x06, AD7124_REG_U24,	AD7124_REG_R,	0x0000		}, /* AD7124_Error */
//			{ 0x07, AD7124_REG_U16,	AD7124_REG_RW,	0x0044		}, /* AD7124_Error_En */
//			{ 0x08, AD7124_REG_U8, 	AD7124_REG_R, 	0x00 		}, /* AD7124_Mclk_Count */
//			{ 0x09, AD7124_REG_U16,	AD7124_REG_RW,	0x8001		}, /* AD7124_Channel_0 */
//			{ 0x0A, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_1 */
//			{ 0x0B, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_2 */
//			{ 0x0C, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_3 */
//			{ 0x0D, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_4 */
//			{ 0x0E, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_5 */
//			{ 0x0F, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_6 */
//			{ 0x10, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_7 */
//			{ 0x11, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_8 */
//			{ 0x12, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_9 */
//			{ 0x13, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_10 */
//			{ 0x14, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_11 */
//			{ 0x15, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_12 */
//			{ 0x16, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_13 */
//			{ 0x17, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_14 */
//			{ 0x18, AD7124_REG_U16,	AD7124_REG_RW,	0x0001		}, /* AD7124_Channel_15 */
//			{ 0x19, AD7124_REG_U16,	AD7124_REG_RW,	0x0860	 	}, /* AD7124_Config_0 */
//			{ 0x1A, AD7124_REG_U16,	AD7124_REG_RW,	0x0860	 	}, /* AD7124_Config_1 */
//			{ 0x1B, AD7124_REG_U16,	AD7124_REG_RW,	0x0860	 	}, /* AD7124_Config_2 */
//			{ 0x1C, AD7124_REG_U16,	AD7124_REG_RW,	0x0860	 	}, /* AD7124_Config_3 */
//			{ 0x1D, AD7124_REG_U16,	AD7124_REG_RW,	0x0860	 	}, /* AD7124_Config_4 */
//			{ 0x1E, AD7124_REG_U16,	AD7124_REG_RW,	0x0860	 	}, /* AD7124_Config_5 */
//			{ 0x1F, AD7124_REG_U16,	AD7124_REG_RW,	0x0860	 	}, /* AD7124_Config_6 */
//			{ 0x20, AD7124_REG_U16,	AD7124_REG_RW,	0x0860	 	}, /* AD7124_Config_7 */
//			{ 0x21,	AD7124_REG_U24,	AD7124_REG_RW,	0x060180	}, /* AD7124_Filter_0 */
//			{ 0x22, AD7124_REG_U24,	AD7124_REG_RW,	0x060180	}, /* AD7124_Filter_1 */
//			{ 0x23, AD7124_REG_U24,	AD7124_REG_RW,	0x060180	}, /* AD7124_Filter_2 */
//			{ 0x24, AD7124_REG_U24,	AD7124_REG_RW,	0x060180	}, /* AD7124_Filter_3 */
//			{ 0x25, AD7124_REG_U24,	AD7124_REG_RW,	0x060180	}, /* AD7124_Filter_4 */
//			{ 0x26, AD7124_REG_U24,	AD7124_REG_RW,	0x060180	}, /* AD7124_Filter_5 */
//			{ 0x27, AD7124_REG_U24,	AD7124_REG_RW,	0x060180	}, /* AD7124_Filter_6 */
//			{ 0x28, AD7124_REG_U24,	AD7124_REG_RW,	0x060180	}, /* AD7124_Filter_7 */
//			{ 0x29, AD7124_REG_U24,	AD7124_REG_RW,	0x800000	}, /* AD7124_Offset_0 */
//			{ 0x2A, AD7124_REG_U24,	AD7124_REG_RW,	0x800000	}, /* AD7124_Offset_1 */
//			{ 0x2B, AD7124_REG_U24,	AD7124_REG_RW,	0x800000	}, /* AD7124_Offset_2 */
//			{ 0x2C, AD7124_REG_U24,	AD7124_REG_RW,	0x800000	}, /* AD7124_Offset_3 */
//			{ 0x2D, AD7124_REG_U24,	AD7124_REG_RW,	0x800000	}, /* AD7124_Offset_4 */
//			{ 0x2E, AD7124_REG_U24,	AD7124_REG_RW,	0x800000	}, /* AD7124_Offset_5 */
//			{ 0x2F, AD7124_REG_U24,	AD7124_REG_RW,	0x800000	}, /* AD7124_Offset_6 */
//			{ 0x30, AD7124_REG_U24,	AD7124_REG_RW,	0x800000	}, /* AD7124_Offset_7 */
//			{ 0x31, AD7124_REG_U24,	AD7124_REG_RW,	0x500000	}, /* AD7124_Gain_0 */
//			{ 0x32, AD7124_REG_U24,	AD7124_REG_RW,	0x500000	}, /* AD7124_Gain_1 */
//			{ 0x33, AD7124_REG_U24,	AD7124_REG_RW,	0x500000	}, /* AD7124_Gain_2 */
//			{ 0x34, AD7124_REG_U24,	AD7124_REG_RW,	0x500000	}, /* AD7124_Gain_3 */
//			{ 0x35, AD7124_REG_U24,	AD7124_REG_RW,	0x500000	}, /* AD7124_Gain_4 */
//			{ 0x36, AD7124_REG_U24,	AD7124_REG_RW,	0x500000	}, /* AD7124_Gain_5 */
//			{ 0x37, AD7124_REG_U24,	AD7124_REG_RW,	0x500000	}, /* AD7124_Gain_6 */
//			{ 0x38, AD7124_REG_U24,	AD7124_REG_RW,	0x500000	}, /* AD7124_Gain_7 */
//		};
};

// ---------- END OF CLASS DECLARATION ---------------------------------------------------------- //
#endif /* ADEXTENDER_H_ */
