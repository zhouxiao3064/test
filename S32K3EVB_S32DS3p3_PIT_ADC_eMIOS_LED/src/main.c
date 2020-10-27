/*
 * Copyright (c) 2015 - 2016 , Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This is a sample project to dim LED by eMIOS based on the potentiometer
 * value converted by ADC triggered by PIT output.
 */

/* The register and bit field definitions for the S32K344 */
/* Including necessary configuration files. */
#include "S32K344.h"

/******************************************************************************/
/* Text Macro Definition                                                      */
/******************************************************************************/
#define SCB_ICIALLU     0xE000EF50
#define SCB_CCR         0xE000ED14
#define SCB_CCR_IC_Msk  0x00020000
#define SCB_CCR_DC_Msk  0x00010000
#define SCB_CSSELR      0xE000ED84
#define SCB_CCSIDR      0xE000ED80
#define SCB_CCSIDR_NUMSETS_Pos             13U                                            /*!< SCB CCSIDR: NumSets Position */
#define SCB_CCSIDR_NUMSETS_Msk             (0x7FFFUL << SCB_CCSIDR_NUMSETS_Pos)           /*!< SCB CCSIDR: NumSets Mask */
#define SCB_CCSIDR_ASSOCIATIVITY_Pos        3U                                            /*!< SCB CCSIDR: Associativity Position */
#define SCB_CCSIDR_ASSOCIATIVITY_Msk       (0x3FFUL << SCB_CCSIDR_ASSOCIATIVITY_Pos)      /*!< SCB CCSIDR: Associativity Mask */
#define CCSIDR_WAYS(x)         (((x) & SCB_CCSIDR_ASSOCIATIVITY_Msk) >> SCB_CCSIDR_ASSOCIATIVITY_Pos)
#define CCSIDR_SETS(x)         (((x) & SCB_CCSIDR_NUMSETS_Msk      ) >> SCB_CCSIDR_NUMSETS_Pos      )
#define SCB_DCISW_WAY_Pos                  30U                                            /*!< SCB DCISW: Way Position */
#define SCB_DCISW_WAY_Msk                  (3UL << SCB_DCISW_WAY_Pos)                     /*!< SCB DCISW: Way Mask */
#define SCB_DCISW_SET_Pos                   5U                                            /*!< SCB DCISW: Set Position */
#define SCB_DCISW_SET_Msk                  (0x1FFUL << SCB_DCISW_SET_Pos)                 /*!< SCB DCISW: Set Mask */
#define SCB_DCISW        0xE000EF60

/** S32_NVIC - Size of Registers Arrays */
#define S32_NVIC_ISER_COUNT                      8u
#define S32_NVIC_ICER_COUNT                      8u
#define S32_NVIC_ISPR_COUNT                      8u
#define S32_NVIC_ICPR_COUNT                      8u
#define S32_NVIC_IABR_COUNT                      8u
#define S32_NVIC_IP_COUNT                        240u

/** S32_NVIC - Register Layout Typedef */
typedef struct {
  __IO uint32_t ISER[S32_NVIC_ISER_COUNT];         /**< Interrupt Set Enable Register n, array offset: 0x0, array step: 0x4 */
       uint32_t RESERVED_0[24];//questions 2     240 IRQs are
  __IO uint32_t ICER[S32_NVIC_ICER_COUNT];         /**< Interrupt Clear Enable Register n, array offset: 0x80, array step: 0x4 */
       uint32_t RESERVED_1[24];
  __IO uint32_t ISPR[S32_NVIC_ISPR_COUNT];         /**< Interrupt Set Pending Register n, array offset: 0x100, array step: 0x4 */
       uint32_t RESERVED_2[24];
  __IO uint32_t ICPR[S32_NVIC_ICPR_COUNT];         /**< Interrupt Clear Pending Register n, array offset: 0x180, array step: 0x4 */
       uint32_t RESERVED_3[24];
  __IO uint32_t IABR[S32_NVIC_IABR_COUNT];         /**< Interrupt Active bit Register n, array offset: 0x200, array step: 0x4 */
       uint32_t RESERVED_4[56];
  __IO uint8_t IP[S32_NVIC_IP_COUNT];              /**< Interrupt Priority Register n, array offset: 0x300, array step: 0x1 */
       uint32_t RESERVED_5[644];
  __O  uint32_t STIR;                              /**< Software Trigger Interrupt Register, offset: 0xE00 */
} S32_NVIC_Type, *S32_NVIC_MemMapPtr;

 /** Number of instances of the S32_NVIC module. */
#define S32_NVIC_INSTANCE_COUNT                  (1u)

/* S32_NVIC - Peripheral instance base addresses */
/** Peripheral S32_NVIC base address */
#define S32_NVIC_BASE                            (0xE000E100u)//M7
/** Peripheral S32_NVIC base pointer */
#define S32_NVIC                                 ((S32_NVIC_Type *)S32_NVIC_BASE)

#define ADC_2_IRQn     182
#define ADC_1_IRQn     181
//#define ADC_0_IRQn    180
//#define ADC_0_IRQn     189//k2tv
/******************************************************************************/
/* Prototype Definition                                                       */
/******************************************************************************/
void Enable_Interrupt(uint8_t vector_number);
void Core_Max_Performance(void);
void Switch_to_Clock_Option_A(void);
void CM7_Cache_Enable( void );
void PORT_setup(void);
void TRGMUX_setup(void);
void ADC_setup(void);
void eMIOS_setup(void);
void PIT_setup(void);
void ADC1_Handler(void);
void ADC2_Handler(void);
/* FUNCTION ********************************************************************
 * Function Name : main
 * Description   : Main Function for CM7 Core
 * END ************************************************************************/
int main(void)
{
	Switch_to_Clock_Option_A();
	CM7_Cache_Enable();
	PORT_setup();
	TRGMUX_setup();
	ADC_setup();
	eMIOS_setup();
	PIT_setup();

    while(1){}
    return 0;
}

/* END main */
/*!
** @}
*/

/* FUNCTION ********************************************************************
 * Function Name : Enable_Interrupt
 * Description   : Change NVIC ISER to enable interrupt
 * END ************************************************************************/
void Enable_Interrupt(uint8_t vector_number)
{
  S32_NVIC->ISER[(uint32_t)(vector_number) >> 5U] = (uint32_t)(1U << ((uint32_t)(vector_number) & (uint32_t)0x1FU));
}

/* FUNCTION ********************************************************************
 * Function Name : Switch_to_Clock_Option_A
 * Description   : Configures System Clocking to Option A
 * END ************************************************************************/
void Switch_to_Clock_Option_A(void)
{
    /* Turn-on clocks for FXOSC & PLL */
    MC_ME->PRTN1_COFB1_CLKEN |= MC_ME_PRTN1_COFB1_CLKEN_REQ53(1) |  /* REQ40: 8-40 MHz Fast External Crystal Oscillator */
                                MC_ME_PRTN1_COFB1_CLKEN_REQ56(1);   /* REQ56: Frequency Modulated Phase-Locked Loop */
    MC_ME->PRTN1_PUPD        |= MC_ME_PRTN1_PUPD_PCUD_MASK;         /* PCUD=1: Trigger the hardware process */
    MC_ME->CTL_KEY = 0x5AF0; /* Enter key */
    MC_ME->CTL_KEY = 0xA50F; /* Enter inverted key */
    while (!(MC_ME->PRTN1_COFB1_STAT & MC_ME_PRTN1_COFB1_STAT_BLOCK53_MASK)) { }  /* Wait until FXOSC clock is running */
    while (!(MC_ME->PRTN1_COFB1_STAT & MC_ME_PRTN1_COFB1_STAT_BLOCK56_MASK)) { }  /* Wait until PLL clock is running */

    /* FXOSC Initialization */
    FXOSC->CTRL &= ~FXOSC_CTRL_OSC_BYP_MASK;  /* OSC_BYP=0: Internal oscillator not bypassed */
    FXOSC->CTRL |= FXOSC_CTRL_EOCV(157)   |   /* EOCV = (stabilization time in ns) / (4 * 128 * (period of clock in ns)) */
                                              /*      =           5e-3             / (4 * 128 * 62.5e-9 */
                   FXOSC_CTRL_GM_SEL(12)  |   /* GM_SEL=12: Selects the transconductance as 0.7016x */
                   FXOSC_CTRL_COMP_EN_MASK;   /* COMP_EN=1: For Crystal mode set this field to 1 */
    FXOSC->CTRL |= FXOSC_CTRL_OSCON_MASK;     /* OSCON=1: Enables FXOSC */
    while (!(FXOSC->STAT & FXOSC_STAT_OSC_STAT_MASK)) { }
    /* Wait 'till Crystal oscillator is on and providing a stable clock */

  /* PLL Initialization
   * PLLPD=0, SDMEN=0, SSCGBYP=1
   * Functional mode EPLL operates in integer-only mode:
   * Fvcoclkout = Fref * (PLLDV[MFI] / PLLDV[RDIV])
   *            = 16M  * (   60      /      1     ) = 960 MHz
   * Fvcoodiv2  = Fvcoclkout / PLLDV[ODIV2] = 960 / 2 = 480MHz
   *
   * Fphi0 = Fvcoodiv2  / (PLLODIV_0[DIV] + 1)
   *         = 480M / [ (1) x (2+1) ] = 160 MHz
   *
   * Fphi1 = Fvcoodiv2  / [ (PLLDV[ODIV2]) x (PLLODIV_1[DIV] + 1) ]
   *         = 480M / [ (1) x (1+1) ] = 240 MHz
   */
    PLL->PLLODIV[0] = 0x00000000UL;       /* Divider disabled */
    PLL->PLLODIV[1] = 0x00000000UL;       /* Divider disabled */
    PLL->PLLCR |= PLL_PLLCR_PLLPD_MASK;   /* PLLPD=1: PLL is powered down */

    PLL->PLLDV = 0x00000000UL;            /* Clear PLL divider register */
    PLL->PLLDV |= PLL_PLLDV_RDIV(0) |     /* RDIV=0: Input clock predivider Divide by 1, thus 16MHz */
                  PLL_PLLDV_MFI(60) |     /* MFI=60: Integer portion of loop divider is 60, thus VCO=16*60=960MHz */
                  PLL_PLLDV_ODIV2(2);     /* ODIV2=2: Output frequency divider Divide by 2, thus 960/2=480MHz  */
    PLL->PLLODIV[0] = PLL_PLLODIV_DIV(2); /* DIV=2: Fphi0 = Fvcoodiv2 / (DIV + 1) = 480 / (2 + 1) = 160MHz */
    PLL->PLLODIV[1] = PLL_PLLODIV_DIV(1); /* DIV=1: Fphi1 = Fvcoodiv2 / (DIV + 1) = 480 / (1 + 1) = 240MHz */
    PLL->PLLFD = 0x00000000UL;            /* Fractional Divider not used */
    PLL->PLLFM = PLL_PLLFM_SSCGBYP_MASK;  /* SSCGBYP=1: Frequency modulation (spread spectrum clock generation) bypassed */
    PLL->PLLCR &= ~PLL_PLLCR_PLLPD_MASK;  /* PLLPD=0: PLL is powered up */
    while (!(PLL->PLLSR & PLL_PLLSR_LOCK_MASK)) { }  /* Wait until PLL is locked */
    PLL->PLLODIV[0] |= PLL_PLLODIV_DE_MASK; /* DE=1: Enable PHI0 divider */
    PLL->PLLODIV[1] |= PLL_PLLODIV_DE_MASK; /* DE=0: Enable PHI1 divider */

    /* MC_CGM Initialization */
    MC_CGM->MUX_0_DIV_TRIG_CTRL |= MC_CGM_MUX_0_DIV_TRIG_CTRL_TCTL_MASK; /* TCTL=1: Switch to common trigger divider update */
    MC_CGM->MUX_0_DIV_TRIG_CTRL |= MC_CGM_MUX_0_DIV_TRIG_CTRL_HHEN_MASK; /* HHEN=1: Halt handshake protocol with AXBS is initiated */

    /* Configure System Clock Dividers */
    MC_CGM->MUX_0_DC_0 = MC_CGM_MUX_0_DC_0_DIV(0);  /* CORE_CLK      = Fphi0 / (0 + 1) = 160 MHz */
    MC_CGM->MUX_0_DC_1 = MC_CGM_MUX_0_DC_1_DIV(1);  /* AIPS_PLAT_CLK = Fphi0 / (1 + 1) =  80 MHz */
    MC_CGM->MUX_0_DC_2 = MC_CGM_MUX_0_DC_2_DIV(3);  /* AIPS_SLOW_CLK = Fphi0 / (3 + 1) =  40 MHz */
    MC_CGM->MUX_0_DC_3 = MC_CGM_MUX_0_DC_3_DIV(1);  /* HSE_CLK       = Fphi0 / (1 + 1) =  80 MHz */
    MC_CGM->MUX_0_DC_4 = MC_CGM_MUX_0_DC_4_DIV(3);  /* DCM_CLK       = Fphi0 / (3 + 1) =  40 MHz */
    MC_CGM->MUX_0_DC_5 = MC_CGM_MUX_0_DC_5_DIV(3);  /* LBIST_CLK     = Fphi0 / (3 + 1) =  40 MHz */
    MC_CGM->MUX_0_DC_6 = MC_CGM_MUX_0_DC_6_DIV(0);  /* QSPI_MEM_CLK  = Fphi0 / (0 + 1) = 160 MHz */

    MC_CGM->MUX_0_DC_0 |= MC_CGM_MUX_0_DC_0_DE_MASK; /* Enable divider 0 */
    MC_CGM->MUX_0_DC_1 |= MC_CGM_MUX_0_DC_1_DE_MASK; /* Enable divider 1 */
    MC_CGM->MUX_0_DC_2 |= MC_CGM_MUX_0_DC_2_DE_MASK; /* Enable divider 2 */
    MC_CGM->MUX_0_DC_3 |= MC_CGM_MUX_0_DC_3_DE_MASK; /* Enable divider 3 */
    MC_CGM->MUX_0_DC_4 |= MC_CGM_MUX_0_DC_4_DE_MASK; /* Enable divider 4 */
    MC_CGM->MUX_0_DC_5 |= MC_CGM_MUX_0_DC_5_DE_MASK; /* Enable divider 5 */
    MC_CGM->MUX_0_DC_6 |= MC_CGM_MUX_0_DC_6_DE_MASK; /* Enable divider 6 */

    /* Writing any value to this register provides a trigger to the dividers */
    MC_CGM->MUX_0_DIV_TRIG = 0xFFFFFFFF;
    /* Make sure the divider has sampled the new divider configuration */
    while ((MC_CGM->MUX_0_DIV_UPD_STAT & MC_CGM_MUX_0_DIV_UPD_STAT_DIV_STAT_MASK) != 0) { }

    /* Verify that we don't have switching in progress */
    while ((MC_CGM->MUX_0_CSS & MC_CGM_MUX_0_CSS_SWIP_MASK) != 0) { }

    MC_CGM->MUX_0_CSC  = MC_CGM_MUX_0_CSC_SELCTL(8);   /* SELCTL=8: PLL_PHI0_CLK */
    MC_CGM->MUX_0_CSC |= MC_CGM_MUX_0_CSC_CLK_SW_MASK; /* Request clock switch */
    while(!(MC_CGM->MUX_0_CSS & MC_CGM_MUX_0_CSS_CLK_SW_MASK)) { }  /* Wait for completion of clock switching */

    /* MC_CGM Initialization : MUX_6 - CLKOUT_RUN */
    /* Verify that we don't have switching in progress */
    while ((MC_CGM->MUX_6_CSS & MC_CGM_MUX_6_CSS_GRIP_MASK) != 0) { }
    MC_CGM->MUX_6_DC_0 &= ~MC_CGM_MUX_6_DC_0_DE_MASK;    /* DE=0: Divider is disabled */
    MC_CGM->MUX_6_CSC &= ~MC_CGM_MUX_6_CSC_SELCTL_MASK;  /* SELCTL=0 temporarily */
    MC_CGM->MUX_6_CSC |= MC_CGM_MUX_6_CSC_SELCTL(0x10);  /* SELCTL=0x10: CORE_CLK */
    MC_CGM->MUX_6_DC_0 &= ~MC_CGM_MUX_6_DC_0_DIV_MASK;   /* DEV=0 temporarily */
    MC_CGM->MUX_6_DC_0 |= MC_CGM_MUX_6_DC_0_DIV(1);      /* DEV=0: division is DIV+1=1 times */
    MC_CGM->MUX_6_DC_0 |= MC_CGM_MUX_6_DC_0_DE_MASK;     /* DE=1: Divider is enabled */
    /* Wait for clock divider update completed */
    while ((MC_CGM->MUX_5_DIV_UPD_STAT & MC_CGM_MUX_6_DIV_UPD_STAT_DIV_STAT_MASK) != 0) { }
    /* Verify that clock switch completed */
    while ((MC_CGM->MUX_6_CSS & MC_CGM_MUX_6_CSS_GRIP_MASK) != 0) { }
}

/* FUNCTION ********************************************************************
 * Function Name : CM7_Cache_Enable
 * Description   : Enable CM7 Instruction and Data Caches
 * END ************************************************************************/
void CM7_Cache_Enable( void )
{
    uint32_t ccsidr;
    uint32_t sets;
    uint32_t ways;

    /* SCB_EnableICache */
    if( ( *( uint32_t *)SCB_CCR ) & SCB_CCR_IC_Msk ) return;  /* return if ICache is already enabled */
    __asm( "DSB" );//DSBは特別なデータ同期メモリバリアとして作動します。
    //この命令以前のすべてのメモリアクセスが完了するまで、以後の命令を実行しないようにプロセッサを待機状態にします。したがって、これより以前のすべてキャッシュメンテナンス操作の完了が保証されます。
    __asm( "ISB" );//SBは命令同期バリアとして作動します。これによりプロセッサのパイプラインは破棄されます。従ってISBの後の命令は、ISBが完了した後に再びキャッシュまたはメモリから取り出されます。
    //これにより、その後の「命令フェッチ」のために命令キャッシュメンテナンス操作が見えることが保証されます。

    *( uint32_t *)SCB_ICIALLU = 0UL;         /* invalidate I-Cache */
    __asm( "DSB" );
    __asm( "ISB" );
    *( uint32_t *)SCB_CCR |=  ( uint32_t)SCB_CCR_IC_Msk;  /* enable I-Cache */
    __asm( "DSB" );
    __asm( "ISB" );

    /* SCB_EnableDCache */
    if( ( *( uint32_t *)SCB_CCR ) & SCB_CCR_DC_Msk ) return;  /* return if DCache is already enabled */
    *( uint32_t *)SCB_CSSELR = 0U;          /* select Level 1 data cache */
    __asm( "DSB" );
    ccsidr = *( uint32_t *)SCB_CCSIDR;
                                            /* invalidate D-Cache */
    sets = ( uint32_t)( CCSIDR_SETS( ccsidr ) );
    do {
      ways = ( uint32_t)( CCSIDR_WAYS( ccsidr ) );
      do {
        *( uint32_t *)SCB_DCISW = ( ( ( sets << SCB_DCISW_SET_Pos ) & SCB_DCISW_SET_Msk ) |
                                    ( ( ways << SCB_DCISW_WAY_Pos ) & SCB_DCISW_WAY_Msk ) );
      } while( ways-- != 0U );
    } while( sets-- != 0U );

    __asm( "DSB" );
    *( uint32_t *)SCB_CCR |=  ( uint32_t)SCB_CCR_DC_Msk;  /* enable D-Cache */
    __asm( "DSB" );
    __asm( "ISB" );
}



/* FUNCTION ********************************************************************
 * Function Name : PORT_setup
 * Description   : Configures pin routing and optionally pin electrical features.
 * END ************************************************************************/
void PORT_setup(void)
{
	/* Pin Configuration for PTA 29  (LED D32 RED) */
	SIUL2->MSCR[29] &= ~(SIUL2_MSCR_SSS_2_MASK |
            			 SIUL2_MSCR_SSS_1_MASK |
						 SIUL2_MSCR_SSS_0_MASK);  /* SSS=0: Choose GPIO[29] */
	SIUL2->GPDO29 &= ~SIUL2_GPDO29_PDO_n_MASK;    /* Drive low on PTA 29 to turn-off LED D32 */
	SIUL2->MSCR[29] |= SIUL2_MSCR_OBE_MASK;       /* OBE=1: GPIO Output Buffer Enabled */

	/* Pin Configuration for PTA 30 (LED D32 GREEN) */
	SIUL2->MSCR[30] &= ~(SIUL2_MSCR_SSS_2_MASK |
	            		 SIUL2_MSCR_SSS_1_MASK |
						 SIUL2_MSCR_SSS_0_MASK);  /* SSS=0: Choose GPIO[30] */
	SIUL2->GPDO30 &= ~SIUL2_GPDO30_PDO_n_MASK;    /* Drive low on PTA 30 to turn-off LED D32 */
	SIUL2->MSCR[30] |= SIUL2_MSCR_OBE_MASK;       /* OBE=1: GPIO Output Buffer Enabled */

	/* Pin Configuration for PTA31 (LED D32 BLUE) */
	SIUL2->MSCR[31] &= ~(SIUL2_MSCR_SSS_2_MASK |
	            		 SIUL2_MSCR_SSS_1_MASK |
						 SIUL2_MSCR_SSS_0_MASK);  /* SSS=0: Choose GPIO[31] */
	SIUL2->GPDO31 &= ~SIUL2_GPDO31_PDO_n_MASK;    /* Drive low on PTA 31 to turn-off LED D32 */
	SIUL2->MSCR[31] |= SIUL2_MSCR_OBE_MASK;       /* OBE=1: GPIO Output Buffer Enabled */

	/* Pin Configuration for PTB18 (LED D33 RED) */
	SIUL2->MSCR[50] &= ~(SIUL2_MSCR_SSS_2_MASK |
	            		 SIUL2_MSCR_SSS_1_MASK |
						 SIUL2_MSCR_SSS_0_MASK);  /* SSS=0: Choose GPIO[50] */
	SIUL2->GPDO50 &= ~SIUL2_GPDO50_PDO_n_MASK;    /* Drive low on PTB 18 to turn-off LED D33 */
	SIUL2->MSCR[50] |= SIUL2_MSCR_OBE_MASK;       /* OBE=1: GPIO Output Buffer Enabled */

	/* Pin Configuration for PTB25 (LED D33 GREEN) */
	SIUL2->MSCR[57] &= ~(SIUL2_MSCR_SSS_2_MASK |
	            		 SIUL2_MSCR_SSS_1_MASK |
						 SIUL2_MSCR_SSS_0_MASK);  /* SSS=0: Choose GPIO[57] */
	SIUL2->GPDO57 &= ~SIUL2_GPDO57_PDO_n_MASK;    /* Drive low on PTB 25 to turn-off LED D33 */
	SIUL2->MSCR[57] |= SIUL2_MSCR_OBE_MASK;       /* OBE=1: GPIO Output Buffer Enabled */

	/* Pin Configuration for PTE12 (LED D33 BLUE) */
	SIUL2->MSCR[140] &= ~(SIUL2_MSCR_SSS_2_MASK |
	            		  SIUL2_MSCR_SSS_1_MASK |
						  SIUL2_MSCR_SSS_0_MASK);  /* SSS=0: Choose GPIO[140] */
	SIUL2->GPDO140 &= ~SIUL2_GPDO140_PDO_n_MASK;    /* Drive low on PTE 12 to turn-off LED D33 */
	SIUL2->MSCR[140] |= SIUL2_MSCR_OBE_MASK;       /* OBE=1: GPIO Output Buffer Enabled */
}


/* FUNCTION ********************************************************************
 *
 * Function Name : TRGMUX_setup
 * Description   : Feed PIT_3 ch0 to hardware trigger of ADC_0 normal conversion
 * END ************************************************************************/
void TRGMUX_setup(void)
{

	/* Configure Clock for Trigger Multiplexing Control */
	 MC_ME->PRTN0_COFB1_CLKEN |= MC_ME_PRTN0_COFB1_CLKEN_REQ32(1) ;/* REQ32: Trigger Multiplexing Control IPS SLOT NUMBER = 0*/
	 MC_ME->PRTN0_PCONF |= MC_ME_PRTN0_PCONF_PCE(1);
     MC_ME->PRTN0_PUPD |= MC_ME_PRTN0_PUPD_PCUD(1);
     MC_ME->CTL_KEY = 0x5AF0;
	 MC_ME->CTL_KEY = 0xA50F;
	 while(!(MC_ME->PRTN0_COFB1_STAT & MC_ME_PRTN0_COFB1_STAT_BLOCK32_MASK) ) { }  /* Wait until TRGMUX clock is running */

	 /* PIT_1 ch0 source --> hardware trigger of ADC_0 normal conversion */
	 TRGMUX->TRGMUXn[TRGMUX_ADC12_1_INDEX] |= 0x7A;   /* Select PIT1_PIT1 ch0 */
	 TRGMUX->TRGMUXn[TRGMUX_ADC12_2_INDEX] |= 0x7A;   /* Select PIT1_PIT1 ch0 */

}

/* FUNCTION ********************************************************************
 *
 * Function Name : ADC_setup
 * Description   : Configures ADC for Potentiometer reading
 * END ************************************************************************/
void ADC_setup(void)
{
	/* Configure Clock for Analog-to-digital converter 0 */
	/* ADC_1 MODULE_CLK is from M7_CORE_CLK (i.e. 160MHz) */
	MC_ME->PRTN0_COFB1_CLKEN |= MC_ME_PRTN0_COFB1_CLKEN_REQ41(1) ;  /* REQ40: Analog-to-digital converter 0 */
	MC_ME->PRTN0_PCONF |= MC_ME_PRTN0_PCONF_PCE(1);        			/* PCE=1: Enable the clock to Partition #0 */
	MC_ME->PRTN0_PUPD  |= MC_ME_PRTN0_PUPD_PCUD(1);         		/* PCUD=1: Trigger the hardware process */
	MC_ME->CTL_KEY = 0x5AF0;         							    /* Valid control key for starting the hardware processes */
	MC_ME->CTL_KEY = 0xA50F;        								/* Valid invert control key for starting the hardware processes */
	while(!(MC_ME->PRTN0_COFB1_STAT & MC_ME_PRTN0_COFB1_STAT_BLOCK41_MASK)) { }  /* Wait until ADC_0 clock is running */

	/* Prepare Analog Input on PTA11  pin */
	/* ADC1_S10 */
	SIUL2->MSCR[11] &= ~SIUL2_MSCR_OBE_MASK;/* Output driver disabled */
	SIUL2->MSCR[11] &= ~SIUL2_MSCR_IBE_MASK;/* Input driver disabled */
	SIUL2->MSCR[11] &= ~SIUL2_MSCR_PUE_MASK;/* Input driver disabled */

	/* Prepare Analog Input on PTA17 pin */
	/* ADC2_S19 */
	SIUL2->MSCR[17] &= ~SIUL2_MSCR_OBE_MASK;/* Output driver disabled */
	SIUL2->MSCR[17] &= ~SIUL2_MSCR_IBE_MASK;/* Input driver disabled */
	SIUL2->MSCR[17] &= ~SIUL2_MSCR_PUE_MASK;/* Input driver disabled */

	//The prescaler must be controlled such that the AD_CLK frequency is less than or equal to 80 MHz.
	ADC_1->MCR |=ADC_MCR_PWDN_MASK;/* the analog module is requested to enter Power Down mode */
									/* ADCLKSEL is modifiable only in power-down mode */
	ADC_1->MCR |=ADC_MCR_ADCLKSEL(1);   /* AD_clk frequency is half bus clock frequency */
	/* 00b - The conversion circuit is clocked with the module clock frequency.
	   01b - The conversion circuit is clocked with half of the module clock frequency. Use this value
				during calibration when the module clock frequency is between 40 MHz and 80 MHz.
	  10b - The conversion circuit is clocked with a quarter of the module clock frequency. Use this
     value during calibration when the module clock frequency is greater than 80 MHz and at most 160 MHz.
   */

	/* Perform Calibration */
	  /* 1. Configure the ADC operating clock for 40MHz operation (program ADCLKSEL=0b */
	  /*    for 80 MHz system clock)                                                   */
	ADC_1->MCR |=ADC_MCR_ADCLKSEL(1);  /* AD_clk frequency is half bus clock frequency */

	  /* 2. Bring ADC from power down state to active conversion (program PWDN =0b) */
	ADC_1->MCR &= ~ADC_MCR_PWDN_MASK; /* Resetting this bit will start ADC transition to IDLE mode */
    //while ((ADC_0->MSR & ADC_MSR_ADCSTATUS_MASK )!= 0){}
	  /* 3. Configure the Calibration BIST Control and status register (CALBISTREG) for */
	  /*    TEST conditions. The default values are set for maximum accuracy (recommended). */
	 ADC_1->CALBISTREG = (ADC_CALBISTREG_RESN(0) |
			              ADC_CALBISTREG_TSAMP(3) |  		 /* RESN=0: 14-bit resolution selected */
						  ADC_CALBISTREG_CALSTFUL_MASK |     /* TSAMP=3: 32 cycle of ADC clk for Sample period in Calibration process */
						  ADC_CALBISTREG_NR_SMPL(3)|		 /* Enable full range (conversions start from bit 15) */
						  ADC_CALBISTREG_AVG_EN_MASK |		 /* ADC Calibration Result Average feature enabled */
						  /* 4. Start calibration (program TEST_EN =1b), calibration start immediately. */
						  ADC_CALBISTREG_TEST_EN_MASK);		 /* Enable the Calibration (self clearing) */

	  /* 5. Poll the status of C_T_BUSY for 0. (wait until it becomes '0') */
	  while( (ADC_1->CALBISTREG & ADC_CALBISTREG_C_T_BUSY_MASK) !=0 ) { }

//	  /* 6. Check the TEST_FAIL to know the final status. If '1' then calibration failed. */
//	  if ((ADC_1->CALBISTREG & ADC_CALBISTREG_TEST_FAIL_MASK )!=0 )
//	  {
//		 SIUL2->GPDO50 |= SIUL2_GPDO50_PDO_n_MASK;    /* Drive high on PTB 18 to turn-on LED D33 */
//		    while( 1 ) {}  /* Trap CPU due to ADC calibration failure */
//	  }
//
//	  /* 7. Check the status of CALIBRTD bit. If calibration is successful this bit will be '1'. */
//	  if ((ADC_1->MSR & ADC_MSR_CALIBRTD_MASK ) == 0 )
//	  {
//		  SIUL2->GPDO50 |= SIUL2_GPDO50_PDO_n_MASK;    /* Drive high on PTB 18 to turn-on LED D33 */
//		  while( 1 ) {}  /* Trap CPU due to ADC calibration failure */
//	  }


	  ADC_1->CALBISTREG |= ADC_CALBISTREG_RESN(1);	/* RESN=1: 12-bit resolution selected */
	  ADC_1->CTR0 |= ADC_CTR0_INPSAMP(0x20);	/* Set the sampling phase duration */

	  ADC_1->MCR |= ADC_MCR_MODE(0); 	     /* MODE=0: One-Shot operation mode */
	  ADC_1->MCR |= ADC_MCR_WLSIDE(0); 		 /* WLSIDE=0: The conversion data is written right aligned (bits 14 to 0) */
	  ADC_1->MCR |= ADC_MCR_TRGEN_MASK; 	 /* TRGEN=1: Enables the external trigger to start a conversion */
	  ADC_1->MCR |= ADC_MCR_EDGE(0); 		 /* EDGE=1:  selects the rising edge for the external trigger */
	  ADC_1->IMR |= ADC_IMR_MSKECH_MASK;  	 /* Enable end of Normal Chain conversion interrupt */

	  /* Enable ch34 for normal chain conversion */
	  ADC_1->NCMR1|= ADC_NCMR1_CH34_MASK; // CH34 ADC1_S10

	  Enable_Interrupt( ADC_1_IRQn );

	  ADC_1->MCR &= ~ADC_MCR_PWDN_MASK;  /* Resetting this bit will start ADC transition to IDLE mode */

	  MC_ME->PRTN0_COFB1_CLKEN |= MC_ME_PRTN0_COFB1_CLKEN_REQ42(1) ;  /* REQ40: Analog-to-digital converter 0 */
	  MC_ME->PRTN0_PCONF |= MC_ME_PRTN0_PCONF_PCE(1);        			/* PCE=1: Enable the clock to Partition #0 */
	  MC_ME->PRTN0_PUPD  |= MC_ME_PRTN0_PUPD_PCUD(1);         		/* PCUD=1: Trigger the hardware process */
	  MC_ME->CTL_KEY = 0x5AF0;         							    /* Valid control key for starting the hardware processes */
	  MC_ME->CTL_KEY = 0xA50F;        								/* Valid invert control key for starting the hardware processes */
	  while(!(MC_ME->PRTN0_COFB1_STAT & MC_ME_PRTN0_COFB1_STAT_BLOCK42_MASK)) { }  /* Wait until ADC_0 clock is running */

	  ADC_2->MCR |=ADC_MCR_ADCLKSEL(1);  /* AD_clk frequency is half bus clock frequency */

	  	  /* 2. Bring ADC from power down state to active conversion (program PWDN =0b) */
	  ADC_2->MCR &= ~ADC_MCR_PWDN_MASK; /* Resetting this bit will start ADC transition to IDLE mode */
	      //while ((ADC_0->MSR & ADC_MSR_ADCSTATUS_MASK )!= 0){}
	  	  /* 3. Configure the Calibration BIST Control and status register (CALBISTREG) for */
	  	  /*    TEST conditions. The default values are set for maximum accuracy (recommended). */
	  ADC_2->CALBISTREG = (ADC_CALBISTREG_RESN(0) |
	  			              ADC_CALBISTREG_TSAMP(3) |  		 /* RESN=0: 14-bit resolution selected */
	  						  ADC_CALBISTREG_CALSTFUL_MASK |     /* TSAMP=3: 32 cycle of ADC clk for Sample period in Calibration process */
	  						  ADC_CALBISTREG_NR_SMPL(3)|		 /* Enable full range (conversions start from bit 15) */
	  						  ADC_CALBISTREG_AVG_EN_MASK |		 /* ADC Calibration Result Average feature enabled */
	  						  /* 4. Start calibration (program TEST_EN =1b), calibration start immediately. */
	  						  ADC_CALBISTREG_TEST_EN_MASK);		 /* Enable the Calibration (self clearing) */

	  	  /* 5. Poll the status of C_T_BUSY for 0. (wait until it becomes '0') */
	  while( (ADC_2->CALBISTREG & ADC_CALBISTREG_C_T_BUSY_MASK) !=0 ) { }

	  //	  /* 6. Check the TEST_FAIL to know the final status. If '1' then calibration failed. */
	  //	  if ((ADC_1->CALBISTREG & ADC_CALBISTREG_TEST_FAIL_MASK )!=0 )
	  //	  {
	  //		 SIUL2->GPDO50 |= SIUL2_GPDO50_PDO_n_MASK;    /* Drive high on PTB 18 to turn-on LED D33 */
	  //		    while( 1 ) {}  /* Trap CPU due to ADC calibration failure */
	  //	  }
	  //
	  //	  /* 7. Check the status of CALIBRTD bit. If calibration is successful this bit will be '1'. */
	  //	  if ((ADC_1->MSR & ADC_MSR_CALIBRTD_MASK ) == 0 )
	  //	  {
	  //		  SIUL2->GPDO50 |= SIUL2_GPDO50_PDO_n_MASK;    /* Drive high on PTB 18 to turn-on LED D33 */
	  //		  while( 1 ) {}  /* Trap CPU due to ADC calibration failure */
	  //	  }


	  	  /* Configure normal chain conversion for ch54 and ch55 by hardware trigger */
	  	  ADC_2->CALBISTREG |= ADC_CALBISTREG_RESN(1);	/* RESN=1: 12-bit resolution selected */
	  	  ADC_2->CTR0 |= ADC_CTR0_INPSAMP(0x20);	/* Set the sampling phase duration */

	  	  ADC_2->MCR |= ADC_MCR_MODE(0); 	     /* MODE=0: One-Shot operation mode */
	  	  ADC_2->MCR |= ADC_MCR_WLSIDE(0); 		 /* WLSIDE=0: The conversion data is written right aligned (bits 14 to 0) */
	  	  ADC_2->MCR |= ADC_MCR_TRGEN_MASK; 	 /* TRGEN=1: Enables the external trigger to start a conversion */
	  	  ADC_2->MCR |= ADC_MCR_EDGE(0); 		 /* EDGE=1:  selects the rising edge for the external trigger */
	  	  ADC_2->IMR |= ADC_IMR_MSKECH_MASK;  	 /* Enable end of Normal Chain conversion interrupt */

	  	  /* Enable ch54 and ch55 for normal chain conversion */

	  	  ADC_2->NCMR1|= ADC_NCMR1_CH43_MASK; // CH34 ADC2_S19



	  	  Enable_Interrupt( ADC_2_IRQn );

	  	  ADC_2->MCR &= ~ADC_MCR_PWDN_MASK;  /* Resetting this bit will start ADC transition to IDLE mode */

}


/* FUNCTION ********************************************************************
 *
 * Function Name : eMIOS_setup
 * Description   : Configures eMIOS module for PWM signal generation
 *
 * END ************************************************************************/
void eMIOS_setup(void)
{
	 MC_ME->PRTN0_COFB1_CLKEN |= MC_ME_PRTN0_COFB1_CLKEN_REQ35(1) ; /* REQ34: eMIOS_1 */
	 MC_ME->PRTN0_PCONF |= MC_ME_PRTN0_PCONF_PCE(1);				/* PCE=1: Enable the clock to Partition #0 */
	 MC_ME->PRTN0_PUPD |= MC_ME_PRTN0_PUPD_PCUD(1);					/* PCUD=1: Trigger the hardware process */
	 MC_ME->CTL_KEY = 0x5AF0;										/* Valid control key for starting the hardware processes */
	 MC_ME->CTL_KEY = 0xA50F;										/* Valid invert control key for starting the hardware processes */
	 while(!(MC_ME->PRTN0_COFB1_STAT & MC_ME_PRTN0_COFB1_STAT_BLOCK35_MASK) ) { }  /* Wait until eMIOS_1 clock is running */

	  /* eMIOS_1 Global Configuration */
	 EMIOS_1->MCR |= eMIOS_MCR_GPRE(1);		/* GPRE=1: Divide 160MHz M33_CORE_CLK by 1+1 = 2 for 80MHz eMIOS clk (12.5ns) */
	 EMIOS_1->MCR &= ~eMIOS_MCR_GPREN_MASK;	/* GPREN=0: Global Prescaler disabled (no clock) and prescaler counter is cleared */
	 EMIOS_1->MCR &= ~eMIOS_MCR_GTBE_MASK;	/* GTBE=0: Global Time Base Enable Out signal negated */
	 EMIOS_1->MCR &= ~eMIOS_MCR_FRZ_MASK;	    /* FRZ=0: Exit freeze state */

	  /* Configure eMIOS_1 Channel 23: MCB, 400ns * 1000 as counter bus[A] generator  */
	 EMIOS_1->CH.UC[23].C = 0x0;					/* Disable channel pre-scaler (reset default) */
	 EMIOS_1->CH.UC[23].A = 65535;  				/* Bus[A] period is 12.5ns * 65535 = 819us (1.22kHz) */
	 EMIOS_1->CH.UC[23].C &= ~eMIOS_C_DMA_MASK;	/* DMA=0: Flag/overrun assigned to Interrupt request */
	 EMIOS_1->CH.UC[23].C |= eMIOS_C_FEN_MASK;    /* FEN=1: Enable (FLAG generates an interrupt request) */
	 EMIOS_1->CH.UC[23].C |= eMIOS_C_MODE(0x50);  /* 0x50: Modulus Counter Buffered (Up counter) */
	 EMIOS_1->CH.UC[23].C &= ~eMIOS_C_UCPRE_MASK; /* UCPRE=0: Divide Ratio 1, thus 80 / 1 = 80MHz (12.5ns) */
	 EMIOS_1->CH.UC[23].C |= eMIOS_C_UCPREN_MASK; /* Prescaler enabled */

	  /* Configure eMIOS_1 Channel 21: OPWMB, 400us 20% duty using counter bus[A] */
	  EMIOS_1->CH.UC[21].C  = 0x0;    				/* Disable channel pre-scaler (reset default) */
	  EMIOS_1->CH.UC[21].A	= 0;      				/* PWM leading edge count value */
	  EMIOS_1->CH.UC[21].B	= 10000;  				/* PWM trailing edge count value */
	  EMIOS_1->CH.UC[21].C	&= ~eMIOS_C_BSL_MASK; /* BSL=0: All channels: counter bus[A] */
	  EMIOS_1->CH.UC[21].C  |= eMIOS_C_EDPOL(1); /* Output polarity on A match */
	  EMIOS_1->CH.UC[21].C  |= eMIOS_C_MODE(0x60);/* Output Pulse Width Modulation Buf'd */

	  /* Configure eMIOS_1 Channel 5: OPWMB, 400us 20% duty using counter bus[A] */
	  EMIOS_1->CH.UC[5].C  = 0x0;    				/* Disable channel pre-scaler (reset default) */
	  EMIOS_1->CH.UC[5].A	= 0;     				/* PWM leading edge count value */
	  EMIOS_1->CH.UC[5].B	= 10000;  				/* PWM trailing edge count value */
	  EMIOS_1->CH.UC[5].C	&= ~eMIOS_C_BSL_MASK; /* BSL=0: All channels: counter bus[A] */
	  EMIOS_1->CH.UC[5].C  |= eMIOS_C_EDPOL(1); /* Output polarity on A match */
	  EMIOS_1->CH.UC[5].C  |= eMIOS_C_MODE(0x60);/* Output Pulse Width Modulation Buf'd */

	  EMIOS_1->CH.UC[0].S  |= eMIOS_S_FLAG_MASK; 	/* Write 1 to claer FLAG of eMIOS ch0 */
	  EMIOS_1->MCR         |= eMIOS_MCR_GPREN_MASK; 	/* GPREN=1: Global Prescaler enabled */
	  EMIOS_1->MCR         |= eMIOS_MCR_GTBE_MASK;	/* GTBE=1: Global Time Base Enable Out signal asserted  */

	  /* Pin Configuration for PTB25 (LED D33 GREEN) */
	  SIUL2->MSCR[57] &= ~(SIUL2_MSCR_SSS_2_MASK |
	              		   SIUL2_MSCR_SSS_1_MASK |
	  					   SIUL2_MSCR_SSS_0_MASK);
	  SIUL2->MSCR[57] |=  SIUL2_MSCR_SSS_1_MASK;   /* SSS=2: Ghoose eMIOS_1_CH[21]_Y */

	  /* Pin Configuration for PTE12 (LED D33 BLUE) */
	  SIUL2->MSCR[140] &= ~(SIUL2_MSCR_SSS_2_MASK |
	  	              		SIUL2_MSCR_SSS_1_MASK |
	  	  					SIUL2_MSCR_SSS_0_MASK);
	  SIUL2->MSCR[140] |=  SIUL2_MSCR_SSS_2_MASK;   /* SSS=4: Ghoose eMIOS_1_CH[5]_Y */
}

/* FUNCTION ********************************************************************
 *
 * Function Name : PIT_setup
 * Description   : Configures PIT_1 module to trigger ADC at every 100ms
 *
 * END ************************************************************************/
void PIT_setup(void)
{
	  /* Configure Clock for PIT_1: AIPS_SLOW_CLK @40MHz */
	 MC_ME->PRTN0_COFB1_CLKEN |= MC_ME_PRTN0_COFB1_CLKEN_REQ45(1) ; /* REQ45: PIT_1 */
	 MC_ME->PRTN0_PCONF |= MC_ME_PRTN0_PCONF_PCE(1);				/* PCE=1: Enable the clock to Partition #0 */
     MC_ME->PRTN0_PUPD |= MC_ME_PRTN0_PUPD_PCUD(1);					/* PCUD=1: Trigger the hardware process */
     MC_ME->CTL_KEY = 0x5AF0;										/* Valid control key for starting the hardware processes */
     MC_ME->CTL_KEY = 0xA50F;										/* Valid invert control key for starting the hardware processes */
     while(!(MC_ME->PRTN0_COFB1_STAT & MC_ME_PRTN0_COFB1_STAT_BLOCK45_MASK) ) { }  /* Wait until PIT_1 clock is running */

     PIT_1->MCR &= ~PIT_MCR_MDIS_MASK;		/* Clock for standard PIT timers is enabled */
     PIT_1->MCR |=PIT_MCR_FRZ_MASK;          /* Timers are stopped in Debug mode */


     PIT_1->TIMER[0].LDVAL |= PIT_RTI_LDVAL_TSV(4000000);	/* 25ns * 400000 = 10ms period */
     PIT_1->TIMER[0].TCTRL |= PIT_TCTRL_TEN_MASK;        	/* timer is enabled */
}


/* FUNCTION ********************************************************************
 * Function Name : Interrupt45_Handler
 * Description   : Interrupt Service Routine for ADC_0
 * END ************************************************************************/
void ADC1_Handler(void)
{
	if(( ADC_1->ISR & ADC_ISR_ECH_MASK) != 0 )  /* Interrupt by ECH? */
	{
		EMIOS_1->CH.UC[21].B = ( uint16_t)( ADC_1->ICDR[2] & 0x7FFF );  /* PWM trailing edge count value */
        ADC_1->ISR |= ADC_ISR_ECH_MASK;  /* Write 1 to clear ECH flag */
	}
}
void ADC2_Handler(void)
{
	if (( ADC_2->ISR & ADC_ISR_ECH_MASK) != 0 )
	{
		EMIOS_1->CH.UC[5].B  = ( uint16_t)( ADC_2->ICDR[11] & 0x7FFF );  /* PWM trailing edge count value */
		ADC_2->ISR |= ADC_ISR_ECH_MASK;  /* Write 1 to clear ECH flag */
	}
}
