/**
 * @file pmsm_foc_KIT_XMC4800PSOC_IOT.h
 * @Firmware PMSM_FOC_SL_XMC13_XMC14_V1_5
 * @Modified date: 11-06-2024
 *
 * @cond
 ****************************************
 * PMSM FOC Motor Control Library
 *
 * Copyright (c) 2015-2024, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (dave@infineon.com).
 ******************************************
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * @endcond
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

#ifndef PMSM_FOC_CONFIGURATION_PMSM_FOC_MCUCARD_PARAMETERS_H_
#define PMSM_FOC_CONFIGURATION_PMSM_FOC_MCUCARD_PARAMETERS_H_

#include "../pmsm_foc_user_config.h"
#include <xmc_vadc.h>
#include <xmc_ccu4.h>
#include <xmc_ccu8.h>
#include <xmc_scu.h>
#include <xmc_gpio.h>
#include <xmc_wdt.h>
#include "xmc4_gpio_map.h"



#if(MCUCARD_TYPE == KIT_XMC4800PSOC_IOT)
/* ********************************************* CUSTOM_MCU *************************/
/*********************************************************************************************************************
 * CUSTOM_MCU
 * GPIO Resources Configuration
 ***************************************/
#define TRAP_PIN                  P5_0                //  CTRAP XMC48: P5_0 , XMC44: P0_7
#define INVERTER_EN_PIN        P8_9             // XMC48: P8_9 ; XMC44: P0_12

#define PHASE_U_HS_PIN        P1_15       // XMC48: P1_15- C81.OUT00    // XMC44:P0_5- C80.OUT00
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3

#define PHASE_U_LS_PIN        P1_12       //XMC48:P1_12- C81.OUT01      // XMC44: P0_2 - C80.OUT01
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3

#define PHASE_V_HS_PIN        P1_14      //XMC48: P1_14 - C81.OUT10      // XMC44: P0_4 -C80.OUT10
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3

#define PHASE_V_LS_PIN        P1_11      //XMC48: P1_11 - C81.OUT11         // XMC44: P0_1 -C80.OUT11
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3

#define PHASE_W_HS_PIN         P1_13    //XMC48:P1_13- C81.OUT20          // XMC44: P0_6  - C80.OUT30
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3

#define PHASE_W_LS_PIN         P1_10    //XMC48: P1_10 - C81.OUT21         // XMC44: P0_11 -C80.OUT31
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3

#define TEST_PIN                 P3_0

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ***************************************/
#define CCU8_MODULE          CCU81           // XMC4800- CCU81 ; XMC44 - CCU80
#define CCU8_MODULE_PHASE_U  CCU81_CC80      // XMC4800- CCU81_CC80 ; XMC44 - CCU80_CC80
#define CCU8_MODULE_PHASE_V  CCU81_CC81      // XMC4800- CCU81_CC81 ; XMC44 - CCU80_CC81
#define CCU8_MODULE_PHASE_W  CCU81_CC82       // XMC4800- CCU81_CC82 ; XMC44 - CCU80_CC83
#define CCU8_MODULE_ADC_TR   CCU81_CC83     // XMC4800- CCU81_CC83 ; XMC44 - CCU80_CC82
#define CCU8_MODULE_PRESCALER_VALUE         (0U)


/*********************************************************************************************************************
 * Secondary loop define
 ****************************************/
#define SECONDARY_LOOP_MODULE        CCU41
#define SECONDARY_LOOP_SLICE        CCU41_CC43
#define SECONDARY_LOOP_MODULE_NUM    1
#define SECONDARY_LOOP_SLICE_PRESCALER     (1U)
#define SECONDARY_LOOP_SLICE_NUM      (3U)
#define SECONDARY_LOOP_SLICE_SHADOW_TRANS_ENABLE_Msk XMC_CCU4_SHADOW_TRANSFER_SLICE_1

/* TRAP LED blinking period. */
#define LED_BLINK_PRS     (1953U >> 3U)
#define LED_BLINK_CRS1    (LED_BLINK_PRS >> 2U)
#define LED_BLINK_CRS2    (LED_BLINK_PRS >> 1U)

/*********************************************************************************************************************
 * VADC Resources Configuration
 ***************************************/
#if (CURRENT_SENSING ==  USER_THREE_SHUNT_SYNC_CONV)
/* Motor Phase U VADC define */
#define VADC_IU_GROUP         VADC_G0
#define VADC_IU_GROUP_NO      (0U)
#define VADC_IU_CHANNEL          (0U)     // XMC48: P14_0 ,G0CH0    /* XMC44: P14.1, VADC group0 channel 0 */
#define VADC_IU_RESULT_REG    (0U)

/* Motor Phase V VADC define */
#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL         (0U)     // XMC48: P14_8 ,G1CH0   /* XMC44: P14.15, VADC group1 channel 7 */
#define VADC_IV_RESULT_REG    (1U)

/* Motor Phase W VADC define */
#define VADC_IW_GROUP         VADC_G2
#define VADC_IW_GROUP_NO      (2U)
#define VADC_IW_CHANNEL       (4U)      // XMC48: P15_4 ,G2CH4  /* XMC44:  P14.5, VADC group2 channel 1 */
#define VADC_IW_RESULT_REG    (4U)



/* ADC is configured as three shunt in ASSYNC*/
#elif (CURRENT_SENSING ==  USER_THREE_SHUNT_ASSYNC_CONV)
/* Motor Phase U VADC define */
#define VADC_IU_GROUP         VADC_G0
#define VADC_IU_GROUP_NO      (0U)
#define VADC_IU_CHANNEL       (0U)        /* P14.0, VADC group0 channel 0 */
#define VADC_IU_RESULT_REG    (0U)

/* Motor Phase V VADC define */
#define VADC_IV_GROUP         VADC_G2
#define VADC_IV_GROUP_NO      (2U)
#define VADC_IV_CHANNEL       (3U)       /* P15.3, VADC group2 channel 3 */
#define VADC_IV_RESULT_REG    (3U)

/* Motor Phase W VADC define */
#define VADC_IW_GROUP         VADC_G3
#define VADC_IW_GROUP_NO      (3U)
#define VADC_IW_CHANNEL       (3U)       /* P14.9, VADC group3 channel 3 */
#define VADC_IW_RESULT_REG    (1U)

/* ADC is configured as single shunt */
#elif(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
/* single shunt current VADC define */
#define VADC_ISS_GROUP        VADC_G1
#define VADC_ISS_GROUP_NO     (1U)
#define VADC_ISS_CHANNEL      (6U)       /* P14.3 VADC group1 channel 5 */
#define VADC_ISS_RESULT_REG   (10U)
#endif

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (2U)     // XMC48: P14_2 ,G1CH2  /* XMC44: P14.9 VADC group1 channel 1 */
#define VADC_VDC_RESULT_REG   (2U)

/* DC link average current /NTC VADC define */
#define VADC_IDC_GROUP        VADC_G1
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (6U)      // XMC48: P14_6 ,G0CH6  /* XMC44: P14.14 VADC group1 channel 0 */
#define VADC_IDC_RESULT_REG   (11U)

/* Potentiometer/Thermistor VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (3U)    // XMC48: P14_3 ,G1CH3   /* XMC44: P14.13 VADC group1 channel 5 */
#define VADC_POT_RESULT_REG   (14U)


/*********************************************************************************************************************
 * UART Resources Configuration
 ***************************************/
#if(SETTING_TARGET_SPEED == BY_UART_ONLY)
#define UART_ENABLE             USIC0_CH1_P1_2_P1_3              /* 1. USIC_DISABLED_ALL, -- which enable POT ADC speed adjustment
                                                                   2. USIC0_CH0_P1_4_P1_5
                                                                   3. USIC0_CH1_P1_2_P1_3 */
#endif

/* NVIC Interrupt Resources Configuration */
/* ***************************************/


#if(UC_SERIES == XMC14)
    #define pmsm_foc_controlloop_isr            IRQ25_Handler
    #define pmsm_foc_trap_protection_irq        IRQ26_Handler
    #define pmsm_foc_secondaryloop_isr          IRQ21_Handler
    #define pmsm_foc_over_under_voltage_isr          IRQ19_Handler
    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      #if (VADC_ISS_GROUP_NO == 0)
        #define pmsm_foc_vadc_source_irqhandler              IRQ18_Handler
      #else
        #define pmsm_foc_vadc_source_irqhandler              IRQ20_Handler
      #endif
    #endif
#else
    #define pmsm_foc_controlloop_isr            CCU81_0_IRQHandler   // XMC4800 : CCU81_0IRQHandler , XMC4400 : CC80_0IRQHandler
    #define pmsm_foc_trap_protection_irq        CCU81_3_IRQHandler   // XMC4800 : CCU81_3IRQHandler , XMC4400 : CC80_3IRQHandler
    #define pmsm_foc_secondaryloop_isr          CCU41_0_IRQHandler  //change only module number
    #define pmsm_foc_over_under_voltage_isr     VADC0_G1_0_IRQHandler
    #if(CURRENT_SENSING == USER_SINGLE_SHUNT_CONV)
      #if (VADC_ISS_GROUP_NO == 0)
        #define pmsm_foc_vadc_source_irqhandler              VADC0_G0_1_IRQHandler
      #else
        #define pmsm_foc_vadc_source_irqhandler              VADC0_G1_1_IRQHandler
      #endif
    #endif
#endif



/*********************************************************************************************************************
 * DEBUGGING ENABLE Resources Configuration
 ***************************************/
#define DEBUG_PWM_0_ENABLE      (0U)        /* 1 = Enable Debug PWM P1.0, 0 = Disable Debug PWM */
#define DEBUG_PWM_1_ENABLE      (0U)        /* 1 = Enable Debug PWM P0.4, 0 = Disable Debug PWM */

#if(DEBUG_PWM_0_ENABLE == 1U || DEBUG_PWM_1_ENABLE == 1U)

  #define DEBUG_PWM_CCU4_MODULE   CCU40
  #define DEBUG_PWM_PERIOD_CNTS (400U)
  #define DEBUG_PWM_50_PERCENT_DC_CNTS  ((uint16_t)(DEBUG_PWM_PERIOD_CNTS >> 1))
  #define REVERSE_CRS_OR_0  (- Tmp_CRS) /* Tmp_CRS = 0 or (- Tmp_CRS) if Tmp_CRS < 0. */

  #if (DEBUG_PWM_0_ENABLE == 1U)
    #define DEBUG_PWM_0_SLICE                         CCU40_CC40
    #define DEBUG_PWM_0_SLICE_NUM                     (0U)
    #define DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk XMC_CCU4_SHADOW_TRANSFER_SLICE_0
    #define DEBUG_PWM_0_PORT                          XMC_GPIO_PORT1
    #define DEBUG_PWM_0_PIN                           (0U)
    #define DEBUG_PWM_0_ALT_OUT                       XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2
  #endif
  #if (DEBUG_PWM_1_ENABLE == 1U)
    #define DEBUG_PWM_1_SLICE                         CCU40_CC41
    #define DEBUG_PWM_1_SLICE_NUM                     (1U)
    #define DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk XMC_CCU4_SHADOW_TRANSFER_SLICE_1
    #define DEBUG_PWM_1_PORT                          XMC_GPIO_PORT0
    #define DEBUG_PWM_1_PIN                           (4U)
    #define DEBUG_PWM_1_ALT_OUT                       XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT4
  #endif
#endif

#endif

#endif
