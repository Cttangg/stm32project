/**
  ******************************************************************************
  * @file    common_interface.h
  * @brief   Header for common_interface.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COMMON_INTERFACE_H
#define COMMON_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef void (*Function_Pointer)(void);

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Common_SetMsp(uint32_t TopOfMainStack);
void Common_EnableIrq(void);
void Common_DisableIrq(void);
FlagStatus Common_GetProtectionStatus(void);
void Common_SetPostProcessingCallback(Function_Pointer Callback);
void Common_StartPostProcessing(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_INTERFACE_H */
