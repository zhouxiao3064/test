/*
** ###################################################################
**     Processor:           S32K344_M7
**     Reference manual:    S32K3xx RM Rev.1 Draft G
**     Version:             rev. 1.3, 2020-07-22
**     Build:               b200722
**
**     Abstract:
**         Peripheral Access Layer for S32K344_M7
**
**     Copyright 1997-2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2020 NXP
**
**     NXP Confidential. This software is owned or controlled by NXP and may only be
**     used strictly in accordance with the applicable license terms. By expressly
**     accepting such terms or by downloading, installing, activating and/or otherwise
**     using the software, you are agreeing that you have read, and that you agree to
**     comply with and are bound by, such license terms. If you do not agree to be
**     bound by the applicable license terms, then you may not retain, install,
**     activate or otherwise use the software. The production use license in
**     Section 2.3 is expressly granted for this software.
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
** ###################################################################
*/

/*!
 * @file S32K344_PFLASH.h
 * @version 1.3
 * @date 2020-07-22
 * @brief Peripheral Access Layer for S32K344_PFLASH
 *
 * This file contains register definitions and macros for easy access to their
 * bit fields.
 *
 * This file assumes LITTLE endian system.
 */

/**
* @page misra_violations MISRA-C:2012 violations
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 2.3, local typedef not referenced
* The SoC header defines typedef for all modules.
*
* @section [global]
* Violates MISRA 2012 Advisory Rule 2.5, local macro not referenced
* The SoC header defines macros for all modules and registers.
*
* @section [global]
* Violates MISRA 2012 Advisory Directive 4.9, Function-like macro
* These are generated macros used for accessing the bit-fields from registers.
*
* @section [global]
* Violates MISRA 2012 Required Rule 5.1, identifier clash
* The supported compilers use more than 31 significant characters for identifiers.
*
* @section [global]
* Violates MISRA 2012 Required Rule 5.2, identifier clash
* The supported compilers use more than 31 significant characters for identifiers.
*
* @section [global]
* Violates MISRA 2012 Required Rule 5.4, identifier clash
* The supported compilers use more than 31 significant characters for identifiers.
*
* @section [global]
* Violates MISRA 2012 Required Rule 5.5, identifier clash
* The supported compilers use more than 31 significant characters for identifiers.
*
* @section [global]
* Violates MISRA 2012 Required Rule 21.1, defined macro '__I' is reserved to the compiler
* This type qualifier is needed to ensure correct I/O access and addressing.
*/

/* Prevention from multiple including the same memory map */
#if !defined(S32K344_PFLASH_H_)  /* Check if memory map has not been already included */
#define S32K344_PFLASH_H_

#include "S32K344_COMMON.h"

/* ----------------------------------------------------------------------------
   -- PFLASH Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PFLASH_Peripheral_Access_Layer PFLASH Peripheral Access Layer
 * @{
 */

/** PFLASH - Size of Registers Arrays */
#define PFLASH_PFCRI_COUNT                        3u
#define PFLASH_PFCBLKI_SPELOCK_COUNT              5u
#define PFLASH_U_PFCBLKI_SPELOCK_COUNT            1u
#define PFLASH_PFCBLKI_SSPELOCK_COUNT             4u
#define PFLASH_D_PFCBLKI_SSPELOCK_COUNT           1u
#define PFLASH_PFCBLKI_SETSLOCK_COUNT             5u
#define PFLASH_U_PFCBLKI_SETSLOCK_COUNT           1u
#define PFLASH_PFCBLKI_SSETSLOCK_COUNT            4u
#define PFLASH_D_PFCBLKI_SSETSLOCK_COUNT          1u
#define PFLASH_PFCBLKI_LOCKMASTER_S_COUNT         5u
#define PFLASH_PFCBLKI_LOCKMASTER_S_PFCBLKI_LOCKMASTER_SJ_COUNT 8u
#define PFLASH_U_PFCBLKI_LOCKMASTER_S_COUNT       1u

/** PFLASH - Register Layout Typedef */
typedef struct {
  __IO uint32_t PFCR[PFLASH_PFCRI_COUNT];          /**< Platform Flash Memory Configuration 0..Platform Flash Memory Configuration 2, array offset: 0x0, array step: 0x4 */
  uint8_t RESERVED_0[4];
  __IO uint32_t PFCR4;                             /**< Platform Flash Memory Configuration 4, offset: 0x10 */
  __IO uint32_t PFAPR;                             /**< Platform Flash Memory Access Protection, offset: 0x14 */
  uint8_t RESERVED_1[744];
  __IO uint32_t PFCPGM_PEADR_L;                    /**< Platform Flash Memory Program Erase Address Logical, offset: 0x300 */
  __I  uint32_t PFCPGM_PEADR_P;                    /**< Platform Flash Memory Program Erase Address Physical, offset: 0x304 */
  __IO uint32_t PFCPGM_XPEADR_L;                   /**< Platform Flash Memory Express Program Erase Address Logical, offset: 0x308 */
  __I  uint32_t PFCPGM_XPEADR_P;                   /**< Platform Flash Memory Express Program Erase Address Physical, offset: 0x30C */
  __IO uint32_t PFCPGM_APEADR_L;                   /**< Platform Flash Memory Alternate Program Erase Address Logical, offset: 0x310 */
  __I  uint32_t PFCPGM_APEADR_P;                   /**< Platform Flash Memory Alternate Program Erase Address Physical, offset: 0x314 */
  uint8_t RESERVED_2[40];
  __IO uint32_t PFCBLK_SPELOCK[PFLASH_PFCBLKI_SPELOCK_COUNT]; /**< Block n Sector Program Erase Lock, array offset: 0x340, array step: 0x4 */
  uint8_t RESERVED_3[4];
  __IO uint32_t PFCBLKU_SPELOCK[PFLASH_U_PFCBLKI_SPELOCK_COUNT]; /**< Block UTEST Sector Program Erase Lock, array offset: 0x358, array step: 0x4 */
  __IO uint32_t PFCBLK_SSPELOCK[PFLASH_PFCBLKI_SSPELOCK_COUNT]; /**< Block n Super Sector Program Erase Lock, array offset: 0x35C, array step: 0x4 */
  __IO uint32_t PFCDBLK_SSPELOCK[PFLASH_D_PFCBLKI_SSPELOCK_COUNT]; /**< Block n Super Sector Program Erase Lock, array offset: 0x36C, array step: 0x4 */
  uint8_t RESERVED_4[16];
  __IO uint32_t PFCBLK_SETSLOCK[PFLASH_PFCBLKI_SETSLOCK_COUNT]; /**< Block n Set Sector Lock, array offset: 0x380, array step: 0x4 */
  uint8_t RESERVED_5[4];
  __IO uint32_t PFCBLKU_SETSLOCK[PFLASH_U_PFCBLKI_SETSLOCK_COUNT]; /**< Block UTEST Set Sector Lock, array offset: 0x398, array step: 0x4 */
  __IO uint32_t PFCBLK_SSETSLOCK[PFLASH_PFCBLKI_SSETSLOCK_COUNT]; /**< Block n Set Super Sector Lock, array offset: 0x39C, array step: 0x4 */
  __IO uint32_t PFCDBLK_SSETSLOCK[PFLASH_D_PFCBLKI_SSETSLOCK_COUNT]; /**< Block n Set Super Sector Lock, array offset: 0x3AC, array step: 0x4 */
  uint8_t RESERVED_6[16];
  __I  uint32_t PFCBLK_LOCKMASTER_S[PFLASH_PFCBLKI_LOCKMASTER_S_COUNT][PFLASH_PFCBLKI_LOCKMASTER_S_PFCBLKI_LOCKMASTER_SJ_COUNT]; /**< Block a Lock Master Sectorb, array offset: 0x3C0, array step: index*0x20, index2*0x4 */
  uint8_t RESERVED_7[32];
  __I  uint32_t PFCBLKU_LOCKMASTER_S[PFLASH_U_PFCBLKI_LOCKMASTER_S_COUNT]; /**< Block UTEST Lock Master Sector, array offset: 0x480, array step: 0x4 */
  __I  uint32_t PFCBLK0_LOCKMASTER_SS0;            /**< Block m Lock Master Super Sector n, offset: 0x484 */
  __I  uint32_t PFCBLK0_LOCKMASTER_SS1;            /**< Block m Lock Master Super Sector n, offset: 0x488 */
  __I  uint32_t PFCBLK0_LOCKMASTER_SS2;            /**< Block m Lock Master Super Sector n, offset: 0x48C */
  uint8_t RESERVED_8[4];
  __I  uint32_t PFCBLK1_LOCKMASTER_SS0;            /**< Block m Lock Master Super Sector n, offset: 0x494 */
  __I  uint32_t PFCBLK1_LOCKMASTER_SS1;            /**< Block m Lock Master Super Sector n, offset: 0x498 */
  __I  uint32_t PFCBLK1_LOCKMASTER_SS2;            /**< Block m Lock Master Super Sector n, offset: 0x49C */
  uint8_t RESERVED_9[4];
  __I  uint32_t PFCBLK2_LOCKMASTER_SS0;            /**< Block m Lock Master Super Sector n, offset: 0x4A4 */
  __I  uint32_t PFCBLK2_LOCKMASTER_SS1;            /**< Block m Lock Master Super Sector n, offset: 0x4A8 */
  __I  uint32_t PFCBLK2_LOCKMASTER_SS2;            /**< Block m Lock Master Super Sector n, offset: 0x4AC */
  uint8_t RESERVED_10[4];
  __I  uint32_t PFCBLK3_LOCKMASTER_SS0;            /**< Block m Lock Master Super Sector n, offset: 0x4B4 */
  __I  uint32_t PFCBLK3_LOCKMASTER_SS1;            /**< Block m Lock Master Super Sector n, offset: 0x4B8 */
  __I  uint32_t PFCBLK3_LOCKMASTER_SS2;            /**< Block m Lock Master Super Sector n, offset: 0x4BC */
  uint8_t RESERVED_11[4];
  __I  uint32_t PFCBLK4_LOCKMASTER_SS0;            /**< Block m Lock Master Super Sector n, offset: 0x4C4 */
} PFLASH_Type, *PFLASH_MemMapPtr;

/** Number of instances of the PFLASH module. */
#define PFLASH_INSTANCE_COUNT                    (2u)

/* PFLASH - Peripheral instance base addresses */
/** Peripheral PFLASH base address */
#define PFLASH_BASE                              (0x40268000u)
/** Peripheral PFLASH base pointer */
#define PFLASH                                   ((PFLASH_Type *)PFLASH_BASE)
/** Peripheral PFLASH_ALT base address */
#define PFLASH_ALT_BASE                          (0x4026C000u)
/** Peripheral PFLASH_ALT base pointer */
#define PFLASH_ALT                               ((PFLASH_Type *)PFLASH_ALT_BASE)
/** Array initializer of PFLASH peripheral base addresses */
#define PFLASH_BASE_ADDRS                        { PFLASH_BASE, PFLASH_ALT_BASE }
/** Array initializer of PFLASH peripheral base pointers */
#define PFLASH_BASE_PTRS                         { PFLASH, PFLASH_ALT }

/* ----------------------------------------------------------------------------
   -- PFLASH Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PFLASH_Register_Masks PFLASH Register Masks
 * @{
 */

/*! @name PFCR - Platform Flash Memory Configuration 0..Platform Flash Memory Configuration 2 */
/*! @{ */
#define PFLASH_PFCR_P0_CBFEN_MASK                (0x1U)
#define PFLASH_PFCR_P0_CBFEN_SHIFT               (0U)
#define PFLASH_PFCR_P0_CBFEN_WIDTH               (1U)
#define PFLASH_PFCR_P0_CBFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P0_CBFEN_SHIFT)) & PFLASH_PFCR_P0_CBFEN_MASK)
#define PFLASH_PFCR_P1_CBFEN_MASK                (0x1U)
#define PFLASH_PFCR_P1_CBFEN_SHIFT               (0U)
#define PFLASH_PFCR_P1_CBFEN_WIDTH               (1U)
#define PFLASH_PFCR_P1_CBFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P1_CBFEN_SHIFT)) & PFLASH_PFCR_P1_CBFEN_MASK)
#define PFLASH_PFCR_P2_CBFEN_MASK                (0x1U)
#define PFLASH_PFCR_P2_CBFEN_SHIFT               (0U)
#define PFLASH_PFCR_P2_CBFEN_WIDTH               (1U)
#define PFLASH_PFCR_P2_CBFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P2_CBFEN_SHIFT)) & PFLASH_PFCR_P2_CBFEN_MASK)
#define PFLASH_PFCR_P0_DBFEN_MASK                (0x2U)
#define PFLASH_PFCR_P0_DBFEN_SHIFT               (1U)
#define PFLASH_PFCR_P0_DBFEN_WIDTH               (1U)
#define PFLASH_PFCR_P0_DBFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P0_DBFEN_SHIFT)) & PFLASH_PFCR_P0_DBFEN_MASK)
#define PFLASH_PFCR_P1_DBFEN_MASK                (0x2U)
#define PFLASH_PFCR_P1_DBFEN_SHIFT               (1U)
#define PFLASH_PFCR_P1_DBFEN_WIDTH               (1U)
#define PFLASH_PFCR_P1_DBFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P1_DBFEN_SHIFT)) & PFLASH_PFCR_P1_DBFEN_MASK)
#define PFLASH_PFCR_P2_DBFEN_MASK                (0x2U)
#define PFLASH_PFCR_P2_DBFEN_SHIFT               (1U)
#define PFLASH_PFCR_P2_DBFEN_WIDTH               (1U)
#define PFLASH_PFCR_P2_DBFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P2_DBFEN_SHIFT)) & PFLASH_PFCR_P2_DBFEN_MASK)
#define PFLASH_PFCR_P0_CPFEN_MASK                (0x10U)
#define PFLASH_PFCR_P0_CPFEN_SHIFT               (4U)
#define PFLASH_PFCR_P0_CPFEN_WIDTH               (1U)
#define PFLASH_PFCR_P0_CPFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P0_CPFEN_SHIFT)) & PFLASH_PFCR_P0_CPFEN_MASK)
#define PFLASH_PFCR_P1_CPFEN_MASK                (0x10U)
#define PFLASH_PFCR_P1_CPFEN_SHIFT               (4U)
#define PFLASH_PFCR_P1_CPFEN_WIDTH               (1U)
#define PFLASH_PFCR_P1_CPFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P1_CPFEN_SHIFT)) & PFLASH_PFCR_P1_CPFEN_MASK)
#define PFLASH_PFCR_P2_CPFEN_MASK                (0x10U)
#define PFLASH_PFCR_P2_CPFEN_SHIFT               (4U)
#define PFLASH_PFCR_P2_CPFEN_WIDTH               (1U)
#define PFLASH_PFCR_P2_CPFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P2_CPFEN_SHIFT)) & PFLASH_PFCR_P2_CPFEN_MASK)
#define PFLASH_PFCR_P0_DPFEN_MASK                (0x20U)
#define PFLASH_PFCR_P0_DPFEN_SHIFT               (5U)
#define PFLASH_PFCR_P0_DPFEN_WIDTH               (1U)
#define PFLASH_PFCR_P0_DPFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P0_DPFEN_SHIFT)) & PFLASH_PFCR_P0_DPFEN_MASK)
#define PFLASH_PFCR_P1_DPFEN_MASK                (0x20U)
#define PFLASH_PFCR_P1_DPFEN_SHIFT               (5U)
#define PFLASH_PFCR_P1_DPFEN_WIDTH               (1U)
#define PFLASH_PFCR_P1_DPFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P1_DPFEN_SHIFT)) & PFLASH_PFCR_P1_DPFEN_MASK)
#define PFLASH_PFCR_P2_DPFEN_MASK                (0x20U)
#define PFLASH_PFCR_P2_DPFEN_SHIFT               (5U)
#define PFLASH_PFCR_P2_DPFEN_WIDTH               (1U)
#define PFLASH_PFCR_P2_DPFEN(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR_P2_DPFEN_SHIFT)) & PFLASH_PFCR_P2_DPFEN_MASK)
/*! @} */

/*! @name PFCR4 - Platform Flash Memory Configuration 4 */
/*! @{ */
#define PFLASH_PFCR4_DERR_SUP_MASK               (0x1U)
#define PFLASH_PFCR4_DERR_SUP_SHIFT              (0U)
#define PFLASH_PFCR4_DERR_SUP_WIDTH              (1U)
#define PFLASH_PFCR4_DERR_SUP(x)                 (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR4_DERR_SUP_SHIFT)) & PFLASH_PFCR4_DERR_SUP_MASK)
#define PFLASH_PFCR4_BLK4_PS_MASK                (0xEU)
#define PFLASH_PFCR4_BLK4_PS_SHIFT               (1U)
#define PFLASH_PFCR4_BLK4_PS_WIDTH               (3U)
#define PFLASH_PFCR4_BLK4_PS(x)                  (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR4_BLK4_PS_SHIFT)) & PFLASH_PFCR4_BLK4_PS_MASK)
#define PFLASH_PFCR4_DMEEE_MASK                  (0x80U)
#define PFLASH_PFCR4_DMEEE_SHIFT                 (7U)
#define PFLASH_PFCR4_DMEEE_WIDTH                 (1U)
#define PFLASH_PFCR4_DMEEE(x)                    (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCR4_DMEEE_SHIFT)) & PFLASH_PFCR4_DMEEE_MASK)
/*! @} */

/*! @name PFAPR - Platform Flash Memory Access Protection */
/*! @{ */
#define PFLASH_PFAPR_M15AP_MASK                  (0x3U)
#define PFLASH_PFAPR_M15AP_SHIFT                 (0U)
#define PFLASH_PFAPR_M15AP_WIDTH                 (2U)
#define PFLASH_PFAPR_M15AP(x)                    (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M15AP_SHIFT)) & PFLASH_PFAPR_M15AP_MASK)
#define PFLASH_PFAPR_M14AP_MASK                  (0xCU)
#define PFLASH_PFAPR_M14AP_SHIFT                 (2U)
#define PFLASH_PFAPR_M14AP_WIDTH                 (2U)
#define PFLASH_PFAPR_M14AP(x)                    (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M14AP_SHIFT)) & PFLASH_PFAPR_M14AP_MASK)
#define PFLASH_PFAPR_M13AP_MASK                  (0x30U)
#define PFLASH_PFAPR_M13AP_SHIFT                 (4U)
#define PFLASH_PFAPR_M13AP_WIDTH                 (2U)
#define PFLASH_PFAPR_M13AP(x)                    (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M13AP_SHIFT)) & PFLASH_PFAPR_M13AP_MASK)
#define PFLASH_PFAPR_M12AP_MASK                  (0xC0U)
#define PFLASH_PFAPR_M12AP_SHIFT                 (6U)
#define PFLASH_PFAPR_M12AP_WIDTH                 (2U)
#define PFLASH_PFAPR_M12AP(x)                    (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M12AP_SHIFT)) & PFLASH_PFAPR_M12AP_MASK)
#define PFLASH_PFAPR_M11AP_MASK                  (0x300U)
#define PFLASH_PFAPR_M11AP_SHIFT                 (8U)
#define PFLASH_PFAPR_M11AP_WIDTH                 (2U)
#define PFLASH_PFAPR_M11AP(x)                    (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M11AP_SHIFT)) & PFLASH_PFAPR_M11AP_MASK)
#define PFLASH_PFAPR_M10AP_MASK                  (0xC00U)
#define PFLASH_PFAPR_M10AP_SHIFT                 (10U)
#define PFLASH_PFAPR_M10AP_WIDTH                 (2U)
#define PFLASH_PFAPR_M10AP(x)                    (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M10AP_SHIFT)) & PFLASH_PFAPR_M10AP_MASK)
#define PFLASH_PFAPR_M9AP_MASK                   (0x3000U)
#define PFLASH_PFAPR_M9AP_SHIFT                  (12U)
#define PFLASH_PFAPR_M9AP_WIDTH                  (2U)
#define PFLASH_PFAPR_M9AP(x)                     (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M9AP_SHIFT)) & PFLASH_PFAPR_M9AP_MASK)
#define PFLASH_PFAPR_M8AP_MASK                   (0xC000U)
#define PFLASH_PFAPR_M8AP_SHIFT                  (14U)
#define PFLASH_PFAPR_M8AP_WIDTH                  (2U)
#define PFLASH_PFAPR_M8AP(x)                     (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M8AP_SHIFT)) & PFLASH_PFAPR_M8AP_MASK)
#define PFLASH_PFAPR_M7AP_MASK                   (0x30000U)
#define PFLASH_PFAPR_M7AP_SHIFT                  (16U)
#define PFLASH_PFAPR_M7AP_WIDTH                  (2U)
#define PFLASH_PFAPR_M7AP(x)                     (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M7AP_SHIFT)) & PFLASH_PFAPR_M7AP_MASK)
#define PFLASH_PFAPR_M6AP_MASK                   (0xC0000U)
#define PFLASH_PFAPR_M6AP_SHIFT                  (18U)
#define PFLASH_PFAPR_M6AP_WIDTH                  (2U)
#define PFLASH_PFAPR_M6AP(x)                     (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M6AP_SHIFT)) & PFLASH_PFAPR_M6AP_MASK)
#define PFLASH_PFAPR_M5AP_MASK                   (0x300000U)
#define PFLASH_PFAPR_M5AP_SHIFT                  (20U)
#define PFLASH_PFAPR_M5AP_WIDTH                  (2U)
#define PFLASH_PFAPR_M5AP(x)                     (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M5AP_SHIFT)) & PFLASH_PFAPR_M5AP_MASK)
#define PFLASH_PFAPR_M4AP_MASK                   (0xC00000U)
#define PFLASH_PFAPR_M4AP_SHIFT                  (22U)
#define PFLASH_PFAPR_M4AP_WIDTH                  (2U)
#define PFLASH_PFAPR_M4AP(x)                     (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M4AP_SHIFT)) & PFLASH_PFAPR_M4AP_MASK)
#define PFLASH_PFAPR_M2AP_MASK                   (0xC000000U)
#define PFLASH_PFAPR_M2AP_SHIFT                  (26U)
#define PFLASH_PFAPR_M2AP_WIDTH                  (2U)
#define PFLASH_PFAPR_M2AP(x)                     (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M2AP_SHIFT)) & PFLASH_PFAPR_M2AP_MASK)
#define PFLASH_PFAPR_M1AP_MASK                   (0x30000000U)
#define PFLASH_PFAPR_M1AP_SHIFT                  (28U)
#define PFLASH_PFAPR_M1AP_WIDTH                  (2U)
#define PFLASH_PFAPR_M1AP(x)                     (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M1AP_SHIFT)) & PFLASH_PFAPR_M1AP_MASK)
#define PFLASH_PFAPR_M0AP_MASK                   (0xC0000000U)
#define PFLASH_PFAPR_M0AP_SHIFT                  (30U)
#define PFLASH_PFAPR_M0AP_WIDTH                  (2U)
#define PFLASH_PFAPR_M0AP(x)                     (((uint32_t)(((uint32_t)(x)) << PFLASH_PFAPR_M0AP_SHIFT)) & PFLASH_PFAPR_M0AP_MASK)
/*! @} */

/*! @name PFCPGM_PEADR_L - Platform Flash Memory Program Erase Address Logical */
/*! @{ */
#define PFLASH_PFCPGM_PEADR_L_PEADR_L_MASK       (0xFFFFFFFFU)
#define PFLASH_PFCPGM_PEADR_L_PEADR_L_SHIFT      (0U)
#define PFLASH_PFCPGM_PEADR_L_PEADR_L_WIDTH      (32U)
#define PFLASH_PFCPGM_PEADR_L_PEADR_L(x)         (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCPGM_PEADR_L_PEADR_L_SHIFT)) & PFLASH_PFCPGM_PEADR_L_PEADR_L_MASK)
/*! @} */

/*! @name PFCPGM_PEADR_P - Platform Flash Memory Program Erase Address Physical */
/*! @{ */
#define PFLASH_PFCPGM_PEADR_P_PEADR_P_MASK       (0xFFFFFFFFU)
#define PFLASH_PFCPGM_PEADR_P_PEADR_P_SHIFT      (0U)
#define PFLASH_PFCPGM_PEADR_P_PEADR_P_WIDTH      (32U)
#define PFLASH_PFCPGM_PEADR_P_PEADR_P(x)         (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCPGM_PEADR_P_PEADR_P_SHIFT)) & PFLASH_PFCPGM_PEADR_P_PEADR_P_MASK)
/*! @} */

/*! @name PFCPGM_XPEADR_L - Platform Flash Memory Express Program Erase Address Logical */
/*! @{ */
#define PFLASH_PFCPGM_XPEADR_L_XPEADR_L_MASK     (0xFFFFFFFFU)
#define PFLASH_PFCPGM_XPEADR_L_XPEADR_L_SHIFT    (0U)
#define PFLASH_PFCPGM_XPEADR_L_XPEADR_L_WIDTH    (32U)
#define PFLASH_PFCPGM_XPEADR_L_XPEADR_L(x)       (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCPGM_XPEADR_L_XPEADR_L_SHIFT)) & PFLASH_PFCPGM_XPEADR_L_XPEADR_L_MASK)
/*! @} */

/*! @name PFCPGM_XPEADR_P - Platform Flash Memory Express Program Erase Address Physical */
/*! @{ */
#define PFLASH_PFCPGM_XPEADR_P_XPEADR_P_MASK     (0xFFFFFFFFU)
#define PFLASH_PFCPGM_XPEADR_P_XPEADR_P_SHIFT    (0U)
#define PFLASH_PFCPGM_XPEADR_P_XPEADR_P_WIDTH    (32U)
#define PFLASH_PFCPGM_XPEADR_P_XPEADR_P(x)       (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCPGM_XPEADR_P_XPEADR_P_SHIFT)) & PFLASH_PFCPGM_XPEADR_P_XPEADR_P_MASK)
/*! @} */

/*! @name PFCPGM_APEADR_L - Platform Flash Memory Alternate Program Erase Address Logical */
/*! @{ */
#define PFLASH_PFCPGM_APEADR_L_APEADR_L_MASK     (0xFFFFFFFFU)
#define PFLASH_PFCPGM_APEADR_L_APEADR_L_SHIFT    (0U)
#define PFLASH_PFCPGM_APEADR_L_APEADR_L_WIDTH    (32U)
#define PFLASH_PFCPGM_APEADR_L_APEADR_L(x)       (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCPGM_APEADR_L_APEADR_L_SHIFT)) & PFLASH_PFCPGM_APEADR_L_APEADR_L_MASK)
/*! @} */

/*! @name PFCPGM_APEADR_P - Platform Flash Memory Alternate Program Erase Address Physical */
/*! @{ */
#define PFLASH_PFCPGM_APEADR_P_APEADR_P_MASK     (0xFFFFFFFFU)
#define PFLASH_PFCPGM_APEADR_P_APEADR_P_SHIFT    (0U)
#define PFLASH_PFCPGM_APEADR_P_APEADR_P_WIDTH    (32U)
#define PFLASH_PFCPGM_APEADR_P_APEADR_P(x)       (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCPGM_APEADR_P_APEADR_P_SHIFT)) & PFLASH_PFCPGM_APEADR_P_APEADR_P_MASK)
/*! @} */

/*! @name PFCBLK_SPELOCK - Block n Sector Program Erase Lock */
/*! @{ */
#define PFLASH_PFCBLK_SPELOCK_SLCK_MASK          (0xFFFFFFFFU)
#define PFLASH_PFCBLK_SPELOCK_SLCK_SHIFT         (0U)
#define PFLASH_PFCBLK_SPELOCK_SLCK_WIDTH         (32U)
#define PFLASH_PFCBLK_SPELOCK_SLCK(x)            (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK_SPELOCK_SLCK_SHIFT)) & PFLASH_PFCBLK_SPELOCK_SLCK_MASK)
/*! @} */

/*! @name PFCBLKU_SPELOCK - Block UTEST Sector Program Erase Lock */
/*! @{ */
#define PFLASH_PFCBLKU_SPELOCK_SLCK_MASK         (0x1U)
#define PFLASH_PFCBLKU_SPELOCK_SLCK_SHIFT        (0U)
#define PFLASH_PFCBLKU_SPELOCK_SLCK_WIDTH        (1U)
#define PFLASH_PFCBLKU_SPELOCK_SLCK(x)           (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLKU_SPELOCK_SLCK_SHIFT)) & PFLASH_PFCBLKU_SPELOCK_SLCK_MASK)
/*! @} */

/*! @name PFCBLK_SSPELOCK - Block n Super Sector Program Erase Lock */
/*! @{ */
#define PFLASH_PFCBLK_SSPELOCK_SSLCK_MASK        (0xFFFU)
#define PFLASH_PFCBLK_SSPELOCK_SSLCK_SHIFT       (0U)
#define PFLASH_PFCBLK_SSPELOCK_SSLCK_WIDTH       (12U)
#define PFLASH_PFCBLK_SSPELOCK_SSLCK(x)          (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK_SSPELOCK_SSLCK_SHIFT)) & PFLASH_PFCBLK_SSPELOCK_SSLCK_MASK)
/*! @} */

/*! @name PFCDBLK_SSPELOCK - Block n Super Sector Program Erase Lock */
/*! @{ */
#define PFLASH_PFCDBLK_SSPELOCK_SSLCK_MASK       (0xFU)
#define PFLASH_PFCDBLK_SSPELOCK_SSLCK_SHIFT      (0U)
#define PFLASH_PFCDBLK_SSPELOCK_SSLCK_WIDTH      (4U)
#define PFLASH_PFCDBLK_SSPELOCK_SSLCK(x)         (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCDBLK_SSPELOCK_SSLCK_SHIFT)) & PFLASH_PFCDBLK_SSPELOCK_SSLCK_MASK)
/*! @} */

/*! @name PFCBLK_SETSLOCK - Block n Set Sector Lock */
/*! @{ */
#define PFLASH_PFCBLK_SETSLOCK_SETSLCK_MASK      (0xFFFFFFFFU)
#define PFLASH_PFCBLK_SETSLOCK_SETSLCK_SHIFT     (0U)
#define PFLASH_PFCBLK_SETSLOCK_SETSLCK_WIDTH     (32U)
#define PFLASH_PFCBLK_SETSLOCK_SETSLCK(x)        (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK_SETSLOCK_SETSLCK_SHIFT)) & PFLASH_PFCBLK_SETSLOCK_SETSLCK_MASK)
/*! @} */

/*! @name PFCBLKU_SETSLOCK - Block UTEST Set Sector Lock */
/*! @{ */
#define PFLASH_PFCBLKU_SETSLOCK_SETSLCK_MASK     (0x1U)
#define PFLASH_PFCBLKU_SETSLOCK_SETSLCK_SHIFT    (0U)
#define PFLASH_PFCBLKU_SETSLOCK_SETSLCK_WIDTH    (1U)
#define PFLASH_PFCBLKU_SETSLOCK_SETSLCK(x)       (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLKU_SETSLOCK_SETSLCK_SHIFT)) & PFLASH_PFCBLKU_SETSLOCK_SETSLCK_MASK)
/*! @} */

/*! @name PFCBLK_SSETSLOCK - Block n Set Super Sector Lock */
/*! @{ */
#define PFLASH_PFCBLK_SSETSLOCK_SSETSLCK_MASK    (0xFFFU)
#define PFLASH_PFCBLK_SSETSLOCK_SSETSLCK_SHIFT   (0U)
#define PFLASH_PFCBLK_SSETSLOCK_SSETSLCK_WIDTH   (12U)
#define PFLASH_PFCBLK_SSETSLOCK_SSETSLCK(x)      (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK_SSETSLOCK_SSETSLCK_SHIFT)) & PFLASH_PFCBLK_SSETSLOCK_SSETSLCK_MASK)
/*! @} */

/*! @name PFCDBLK_SSETSLOCK - Block n Set Super Sector Lock */
/*! @{ */
#define PFLASH_PFCDBLK_SSETSLOCK_SSETSLCK_MASK   (0xFU)
#define PFLASH_PFCDBLK_SSETSLOCK_SSETSLCK_SHIFT  (0U)
#define PFLASH_PFCDBLK_SSETSLOCK_SSETSLCK_WIDTH  (4U)
#define PFLASH_PFCDBLK_SSETSLOCK_SSETSLCK(x)     (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCDBLK_SSETSLOCK_SSETSLCK_SHIFT)) & PFLASH_PFCDBLK_SSETSLOCK_SSETSLCK_MASK)
/*! @} */

/*! @name PFCBLK_LOCKMASTER_S - Block a Lock Master Sectorb */
/*! @{ */
#define PFLASH_PFCBLK_LOCKMASTER_S_LOCKMASTER_S_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK_LOCKMASTER_S_LOCKMASTER_S_SHIFT (0U)
#define PFLASH_PFCBLK_LOCKMASTER_S_LOCKMASTER_S_WIDTH (32U)
#define PFLASH_PFCBLK_LOCKMASTER_S_LOCKMASTER_S(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK_LOCKMASTER_S_LOCKMASTER_S_SHIFT)) & PFLASH_PFCBLK_LOCKMASTER_S_LOCKMASTER_S_MASK)
/*! @} */

/*! @name PFCBLKU_LOCKMASTER_S - Block UTEST Lock Master Sector */
/*! @{ */
#define PFLASH_PFCBLKU_LOCKMASTER_S_LOCKMASTER_S_MASK (0xFFU)
#define PFLASH_PFCBLKU_LOCKMASTER_S_LOCKMASTER_S_SHIFT (0U)
#define PFLASH_PFCBLKU_LOCKMASTER_S_LOCKMASTER_S_WIDTH (8U)
#define PFLASH_PFCBLKU_LOCKMASTER_S_LOCKMASTER_S(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLKU_LOCKMASTER_S_LOCKMASTER_S_SHIFT)) & PFLASH_PFCBLKU_LOCKMASTER_S_LOCKMASTER_S_MASK)
/*! @} */

/*! @name PFCBLK0_LOCKMASTER_SS0 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK0_LOCKMASTER_SS0_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK0_LOCKMASTER_SS0_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK0_LOCKMASTER_SS0_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK0_LOCKMASTER_SS0_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK0_LOCKMASTER_SS0_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK0_LOCKMASTER_SS0_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK0_LOCKMASTER_SS1 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK0_LOCKMASTER_SS1_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK0_LOCKMASTER_SS1_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK0_LOCKMASTER_SS1_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK0_LOCKMASTER_SS1_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK0_LOCKMASTER_SS1_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK0_LOCKMASTER_SS1_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK0_LOCKMASTER_SS2 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK0_LOCKMASTER_SS2_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK0_LOCKMASTER_SS2_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK0_LOCKMASTER_SS2_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK0_LOCKMASTER_SS2_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK0_LOCKMASTER_SS2_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK0_LOCKMASTER_SS2_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK1_LOCKMASTER_SS0 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK1_LOCKMASTER_SS0_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK1_LOCKMASTER_SS0_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK1_LOCKMASTER_SS0_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK1_LOCKMASTER_SS0_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK1_LOCKMASTER_SS0_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK1_LOCKMASTER_SS0_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK1_LOCKMASTER_SS1 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK1_LOCKMASTER_SS1_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK1_LOCKMASTER_SS1_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK1_LOCKMASTER_SS1_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK1_LOCKMASTER_SS1_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK1_LOCKMASTER_SS1_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK1_LOCKMASTER_SS1_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK1_LOCKMASTER_SS2 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK1_LOCKMASTER_SS2_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK1_LOCKMASTER_SS2_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK1_LOCKMASTER_SS2_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK1_LOCKMASTER_SS2_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK1_LOCKMASTER_SS2_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK1_LOCKMASTER_SS2_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK2_LOCKMASTER_SS0 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK2_LOCKMASTER_SS0_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK2_LOCKMASTER_SS0_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK2_LOCKMASTER_SS0_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK2_LOCKMASTER_SS0_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK2_LOCKMASTER_SS0_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK2_LOCKMASTER_SS0_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK2_LOCKMASTER_SS1 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK2_LOCKMASTER_SS1_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK2_LOCKMASTER_SS1_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK2_LOCKMASTER_SS1_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK2_LOCKMASTER_SS1_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK2_LOCKMASTER_SS1_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK2_LOCKMASTER_SS1_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK2_LOCKMASTER_SS2 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK2_LOCKMASTER_SS2_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK2_LOCKMASTER_SS2_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK2_LOCKMASTER_SS2_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK2_LOCKMASTER_SS2_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK2_LOCKMASTER_SS2_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK2_LOCKMASTER_SS2_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK3_LOCKMASTER_SS0 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK3_LOCKMASTER_SS0_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK3_LOCKMASTER_SS0_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK3_LOCKMASTER_SS0_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK3_LOCKMASTER_SS0_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK3_LOCKMASTER_SS0_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK3_LOCKMASTER_SS0_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK3_LOCKMASTER_SS1 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK3_LOCKMASTER_SS1_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK3_LOCKMASTER_SS1_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK3_LOCKMASTER_SS1_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK3_LOCKMASTER_SS1_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK3_LOCKMASTER_SS1_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK3_LOCKMASTER_SS1_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK3_LOCKMASTER_SS2 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK3_LOCKMASTER_SS2_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK3_LOCKMASTER_SS2_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK3_LOCKMASTER_SS2_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK3_LOCKMASTER_SS2_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK3_LOCKMASTER_SS2_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK3_LOCKMASTER_SS2_LOCKMASTER_SS_MASK)
/*! @} */

/*! @name PFCBLK4_LOCKMASTER_SS0 - Block m Lock Master Super Sector n */
/*! @{ */
#define PFLASH_PFCBLK4_LOCKMASTER_SS0_LOCKMASTER_SS_MASK (0xFFFFFFFFU)
#define PFLASH_PFCBLK4_LOCKMASTER_SS0_LOCKMASTER_SS_SHIFT (0U)
#define PFLASH_PFCBLK4_LOCKMASTER_SS0_LOCKMASTER_SS_WIDTH (32U)
#define PFLASH_PFCBLK4_LOCKMASTER_SS0_LOCKMASTER_SS(x) (((uint32_t)(((uint32_t)(x)) << PFLASH_PFCBLK4_LOCKMASTER_SS0_LOCKMASTER_SS_SHIFT)) & PFLASH_PFCBLK4_LOCKMASTER_SS0_LOCKMASTER_SS_MASK)
/*! @} */

/*!
 * @}
 */ /* end of group PFLASH_Register_Masks */

/*!
 * @}
 */ /* end of group PFLASH_Peripheral_Access_Layer */

#endif  /* #if !defined(S32K344_PFLASH_H_) */
