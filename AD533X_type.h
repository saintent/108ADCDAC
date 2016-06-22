//================ File Description =========================================//
//=== File name : AD533X_type.h
//===========================================================================//


#ifndef AD533X_TYPE_H_
#define AD533X_TYPE_H_


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
	AD533X_OUTA = 1,
	AD533X_OUTB = 2,
	AD533X_OUT_ALL = 3
}AD533X_OUT_SEL;

typedef enum {
	AD533X_POWER_DOWN_NORMAL = 0,
	AD533X_POWER_DOWN_1K_TO_GND,
	AD533X_POWER_DOWN_100K_TO_GND,
	AD533X_POWER_DOWN_THREE_STATE
}AD533X_POWER_DOWN_MODE;
//================ TYPEDEF FUNCTION TYPE DEFFINITION ========================//
//
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
#endif /* AD533X_TYPE_H_ */
