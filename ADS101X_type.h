//================ File Description =========================================//
//=== File name : ADS101X_type.h
//===========================================================================//


#ifndef ADS101X_TYPE_H_
#define ADS101X_TYPE_H_


//================ Include Header ===========================================//
#include "stdint.h"
//================ PULBIC DEFINE ============================================//
//
//================ PUBLIC MACRO =============================================//
//
//================ TYPEDEF DATA TYPE DEFINITION =============================//
//
//================ ENUMERATOR DEFINITION ====================================//
typedef enum {
	ADS101X_REG_CONVERSION = 0,
	ADS101X_REG_CONFIG,
	ADS101X_REG_LO_THRESH,
	ADS101X_REG_HI_THRESH
}ADS101X_REGISTER_POINTER;

typedef enum {
	ADS101X_CH1 = 4,
	ADS101X_CH2 = 5,
	ADS101X_CH3 = 6,
	ADS101X_CH4 = 7
}ADS101X_CHANNEL;

typedef enum {
	ADS101X_PGA_GAIN_2P3 = 0,			// Gain 2/3
	ADS101X_PGA_GAIN_1,					// Gain 1
	ADS101X_PGA_GAIN_2,					// Gain 2 (default)
	ADS101X_PGA_GAIN_4,					// Gain 4
	ADS101X_PGA_GAIN_8,					// Gain 8
	ADS101X_PGA_GAIN_16					// Gain 16
}ADS101X_PGA_GAIN;

typedef enum {
	ADS101X_MODE_CON = 0,				// Continuous conversion mode
	ADS101X_MODE_SIGLE					// Power-down single-shot mode (default)
}ADS101X_MODE;

typedef enum {
	ADS101X_DATA_RATE_128_SPS = 0,
	ADS101X_DATA_RATE_250_SPS,
	ADS101X_DATA_RATE_490_SPS,
	ADS101X_DATA_RATE_920_SPS,
	ADS101X_DATA_RATE_1600_SPS,
	ADS101X_DATA_RATE_2400_SPS,
	ADS101X_DATA_RATE_3300_SPS,
}ADS101X_DATA_RATE;


//================ TYPEDEF FUNCTION TYPE DEFFINITION ========================//
typedef struct {
	uint16_t		COMP_QUE:2;
	uint16_t		COMP_LAT:1;
	uint16_t		COMP_POL:1;
	uint16_t		COMP_MODE:1;
	uint16_t		DR:3;
	uint16_t		MODE:1;
	uint16_t		PGA:3;
	uint16_t		MUX:3;
	uint16_t		OS:1;
}ADS_CONFIG_BIT;
//================ TYPEDEF STRUCT/UNION =====================================//
//
//================ EXTERN FUNCTION ==========================================//
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif
//================ EXTERN FUNCTION POINTER ==================================//
//
//================ EXTERN VARIABLE ==========================================//
//
//================ EXTERN QUEUE =============================================//
//
//================ END OF FILE ==============================================//
#endif /* ADS101X_TYPE_H_ */
