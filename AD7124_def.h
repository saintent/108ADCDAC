//================ File Description =========================================//
//=== File name : AD7124_def.h
//===========================================================================//


#ifndef AD7124_DEF_H_
#define AD7124_DEF_H_


//================ Include Header ===========================================//
#include "stdint.h"
//================ PULBIC DEFINE ============================================//
//
//================ PUBLIC MACRO =============================================//
//
//================ TYPEDEF DATA TYPE DEFINITION =============================//
//
//================ ENUMERATOR DEFINITION ====================================//
//
//================ TYPEDEF FUNCTION TYPE DEFFINITION ========================//
//
//================ TYPEDEF STRUCT/UNION =====================================//
typedef struct ad7124_status_register_tag{
	uint8_t CH_ACTIVE:4;
	uint8_t POR_FLAG:1;
	uint8_t BIT_5_PADDING:1;
	uint8_t ERROR_FLAG:1;
	uint8_t RDY:1;
}AD7124_STATUS_REGISTER;

typedef struct ad7124_channel_register_tag{
	uint16_t AINM:5;
	uint16_t AINP:5;
	uint16_t PADDING:2;
	uint16_t SETUP:3;
	uint16_t EN:1;
}AD7124_CHANNEL_REGISTER;

typedef struct ad7124_config_register_tag{
	uint16_t PGA:3;
	uint16_t REF_SEL:2;
	uint16_t AIN_BUFM:1;
	uint16_t AIN_BUFP:1;
	uint16_t REF_BUFM:1;
	uint16_t REF_BUFP:1;
	uint16_t Burnout:1;
	uint16_t Bipolay:1;
	uint16_t rsv:4;
}AD7124_CONFIG_REGISTER;
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
#endif /* AD7124_DEF_H_ */
