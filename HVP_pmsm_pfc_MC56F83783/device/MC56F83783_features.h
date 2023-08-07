/*
** ###################################################################
**     Version:             rev. 0.1, 2020-05-06
**     Build:               b200921
**
**     Abstract:
**         Chip specific module features.
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2020 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 0.1 (2020-05-06)
**         Initial version.
**
** ###################################################################
*/

#ifndef _MC56F83783_FEATURES_H_
#define _MC56F83783_FEATURES_H_

/* SOC module features */

/* @brief CADC availability on the SoC. */
#define FSL_FEATURE_SOC_CADC_COUNT (1)
/* @brief CMP availability on the SoC. */
#define FSL_FEATURE_SOC_CMP_COUNT (4)
/* @brief COP availability on the SoC. */
#define FSL_FEATURE_SOC_COP_COUNT (1)
/* @brief CRC availability on the SoC. */
#define FSL_FEATURE_SOC_CRC_COUNT (1)
/* @brief DAC availability on the SoC. */
#define FSL_FEATURE_SOC_DAC_COUNT (2)
/* @brief DMAMUX availability on the SoC. */
#define FSL_FEATURE_SOC_DMAMUX_COUNT (1)
/* @brief EDMA availability on the SoC. */
#define FSL_FEATURE_SOC_EDMA_COUNT (1)
/* @brief EVTG availability on the SoC. */
#define FSL_FEATURE_SOC_EVTG_COUNT (1)
/* @brief EWM availability on the SoC. */
#define FSL_FEATURE_SOC_EWM_COUNT (1)
/* @brief FLEXCAN availability on the SoC. */
#define FSL_FEATURE_SOC_FLEXCAN_COUNT (1)
/* @brief FTFE availability on the SoC. */
#define FSL_FEATURE_SOC_FTFE_COUNT (1)
/* @brief GPIO availability on the SoC. */
#define FSL_FEATURE_SOC_GPIO_COUNT (7)
/* @brief I2C availability on the SoC. */
#define FSL_FEATURE_SOC_I2C_COUNT (2)
/* @brief INTC availability on the SoC. */
#define FSL_FEATURE_SOC_INTC_COUNT (1)
/* @brief MCM availability on the SoC. */
#define FSL_FEATURE_SOC_MCM_COUNT (1)
/* @brief OCCS availability on the SoC. */
#define FSL_FEATURE_SOC_OCCS_COUNT (1)
/* @brief PIT availability on the SoC. */
#define FSL_FEATURE_SOC_PIT_COUNT (2)
/* @brief PMC availability on the SoC. */
#define FSL_FEATURE_SOC_PMC_COUNT (1)
/* @brief PWM availability on the SoC. */
#define FSL_FEATURE_SOC_PWM_COUNT (2)
/* @brief QSCI availability on the SoC. */
#define FSL_FEATURE_SOC_QSCI_COUNT (2)
/* @brief QSPI availability on the SoC. */
#define FSL_FEATURE_SOC_QSPI_COUNT (1)
/* @brief SIM availability on the SoC. */
#define FSL_FEATURE_SOC_SIM_COUNT (1)
/* @brief TMR availability on the SoC. */
#define FSL_FEATURE_SOC_TMR_COUNT (2)
/* @brief XBARA availability on the SoC. */
#define FSL_FEATURE_SOC_XBARA_COUNT (1)

/* CADC module features */

/* @brief register RSLT2 count. */
#define FSL_FEATURE_CADC_RSLT2_COUNT (4)
/* @brief No auto standby (register bit PWR[ASB]). */
#define FSL_FEATURE_CADC_NO_AUTO_STANDBY (1)
/* @brief No speedmode bit (register bits PWR2[SPEEDA]). */
#define FSL_FEATURE_CADC_NO_SPEEDMODE_BIT (1)
/* @brief No sample window (register bit CTRL3[SCNT0]). */
#define FSL_FEATURE_CADC_NO_SAMPLE_WINDOW (1)
/* @brief Has unipolar differential mode (register bit CTRL3[UPDEN_L]). */
#define FSL_FEATURE_CADC_HAS_UNIPOLAR_DIFFERENTIAL_MODE (1)

/* FLEXCAN module features */

/* @brief Message buffer size */
#define FSL_FEATURE_FLEXCAN_HAS_MESSAGE_BUFFER_MAX_NUMBERn(x) (32)
/* @brief Has doze mode support (register bit field MCR[DOZE]). */
#define FSL_FEATURE_FLEXCAN_HAS_DOZE_MODE_SUPPORT (1)
/* @brief Insatnce has doze mode support (register bit field MCR[DOZE]). */
#define FSL_FEATURE_FLEXCAN_INSTANCE_HAS_DOZE_MODE_SUPPORTn(x) (1)
/* @brief Has a glitch filter on the receive pin (register bit field MCR[WAKSRC]). */
#define FSL_FEATURE_FLEXCAN_HAS_GLITCH_FILTER (1)
/* @brief Has extended interrupt mask and flag register (register IMASK2, IFLAG2). */
#define FSL_FEATURE_FLEXCAN_HAS_EXTENDED_FLAG_REGISTER (0)
/* @brief Instance has extended bit timing register (register CBT). */
#define FSL_FEATURE_FLEXCAN_INSTANCE_HAS_EXTENDED_TIMING_REGISTERn(x) (1)
/* @brief Has a receive FIFO DMA feature (register bit field MCR[DMA]). */
#define FSL_FEATURE_FLEXCAN_HAS_RX_FIFO_DMA (1)
/* @brief Instance has a receive FIFO DMA feature (register bit field MCR[DMA]). */
#define FSL_FEATURE_FLEXCAN_INSTANCE_HAS_RX_FIFO_DMAn(x) (1)
/* @brief Remove CAN Engine Clock Source Selection from unsupported part. */
#define FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE (0)
/* @brief Instance remove CAN Engine Clock Source Selection from unsupported part. */
#define FSL_FEATURE_FLEXCAN_INSTANCE_SUPPORT_ENGINE_CLK_SEL_REMOVEn(x) (0)
/* @brief Is affected by errata with ID 5641 (Module does not transmit a message that is enabled to be transmitted at a
 * specific moment during the arbitration process). */
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_5641 (0)
/* @brief Is affected by errata with ID 5829 (FlexCAN: FlexCAN does not transmit a message that is enabled to be
 * transmitted in a specific moment during the arbitration process). */
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_5829 (0)
/* @brief Is affected by errata with ID 6032 (FlexCAN: A frame with wrong ID or payload is transmitted into the CAN bus
 * when the Message Buffer under transmission is either aborted or deactivated while the CAN bus is in the Bus Idle
 * state). */
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_6032 (0)
/* @brief Is affected by errata with ID 9595 (FlexCAN: Corrupt frame possible if the Freeze Mode or the Low-Power Mode
 * are entered during a Bus-Off state). */
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_9595 (0)
/* @brief Has CAN with Flexible Data rate (CAN FD) protocol. */
#define FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE (1)
/* @brief CAN instance support Flexible Data rate (CAN FD) protocol. */
#define FSL_FEATURE_FLEXCAN_INSTANCE_HAS_FLEXIBLE_DATA_RATEn(x) (1)
/* @brief Has extra MB interrupt or common one. */
#define FSL_FEATURE_FLEXCAN_HAS_EXTRA_MB_INT (1)
/* @brief Has memory error control (register MECR). */
#define FSL_FEATURE_FLEXCAN_HAS_MEMORY_ERROR_CONTROL (0)

/* CMP module features */

/* @brief Has Trigger mode in CMP (register bit field CR1[TRIGM]). */
#define FSL_FEATURE_CMP_HAS_TRIGGER_MODE (0)
/* @brief Has Window mode in CMP (register bit field CR1[WE]). */
#define FSL_FEATURE_CMP_HAS_WINDOW_MODE (1)
/* @brief Has External sample supported in CMP (register bit field CR1[SE]). */
#define FSL_FEATURE_CMP_HAS_EXTERNAL_SAMPLE_SUPPORT (1)
/* @brief Has DMA support in CMP (register bit field SCR[DMAEN]). */
#define FSL_FEATURE_CMP_HAS_DMA (1)
/* @brief Has Pass Through mode in CMP (register bit field MUXCR[PSTM]). */
#define FSL_FEATURE_CMP_HAS_PASS_THROUGH_MODE (0)
/* @brief Has DAC Test function in CMP (register DACTEST). */
#define FSL_FEATURE_CMP_HAS_DAC_TEST (0)
/* @brief Has COUTA out of window is zero enable. */
#define FSL_FEATURE_CMP_HAS_COWZ_BIT_FIELD (1)
/* @brief Use 16 bit registers. */
#define FSL_FEATURE_CMP_USE_16BIT_REG (1)

/* EDMA module features */

/* @brief Number of DMA channels (related to number of registers TCD, DCHPRI, bit fields ERQ[ERQn], EEI[EEIn],
 * INT[INTn], ERR[ERRn], HRS[HRSn] and bit field widths ES[ERRCHN], CEEI[CEEI], SEEI[SEEI], CERQ[CERQ], SERQ[SERQ],
 * CDNE[CDNE], SSRT[SSRT], CERR[CERR], CINT[CINT], TCDn_CITER_ELINKYES[LINKCH], TCDn_CSR[MAJORLINKCH],
 * TCDn_BITER_ELINKYES[LINKCH]). (Valid only for eDMA modules.) */
#define FSL_FEATURE_EDMA_MODULE_CHANNEL (4)
/* @brief Total number of DMA channels on all modules. */
#define FSL_FEATURE_EDMA_DMAMUX_CHANNELS (4)
/* @brief Number of DMA channel groups (register bit fields CR[ERGA], CR[GRPnPRI], ES[GPE], DCHPRIn[GRPPRI]). (Valid
 * only for eDMA modules.) */
#define FSL_FEATURE_EDMA_CHANNEL_GROUP_COUNT (1)
/* @brief Has DMA_Error interrupt vector. */
#define FSL_FEATURE_EDMA_HAS_ERROR_IRQ (1)
/* @brief Number of DMA channels with asynchronous request capability (register EARS). (Valid only for eDMA modules.) */
#define FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT (4)
/* @brief Channel IRQ entry shared offset. */
#define FSL_FEATURE_EDMA_MODULE_CHANNEL_IRQ_ENTRY_SHARED_OFFSET (0)
/* @brief If 8 bytes transfer supported. */
#define FSL_FEATURE_EDMA_SUPPORT_8_BYTES_TRANSFER (0)
/* @brief If 16 bytes transfer supported. */
#define FSL_FEATURE_EDMA_SUPPORT_16_BYTES_TRANSFER (1)
/* @brief If 32 bytes transfer supported. */
#define FSL_FEATURE_EDMA_SUPPORT_32_BYTES_TRANSFER (0)

/* DMAMUX module features */

/* @brief Number of DMA channels (related to number of register CHCFGn). */
#define FSL_FEATURE_DMAMUX_MODULE_CHANNEL (4)
/* @brief Total number of DMA channels on all modules. */
#define FSL_FEATURE_DMAMUX_DMAMUX_CHANNELS (4)
/* @brief Has the periodic trigger capability for the triggered DMA channel (register bit CHCFG0[TRIG]). */
#define FSL_FEATURE_DMAMUX_HAS_TRIG (0)
/* @brief Has DMA Channel Always ON function (register bit CHCFG0[A_ON]). */
#define FSL_FEATURE_DMAMUX_HAS_A_ON (0)
/* @brief Register CHCFGn width. */
#define FSL_FEATURE_DMAMUX_CHCFG_REGISTER_WIDTH (8)

/* EWM module features */

/* @brief Has clock select (register CLKCTRL). */
#define FSL_FEATURE_EWM_HAS_CLOCK_SELECT (1)
/* @brief Has clock prescaler (register CLKPRESCALER). */
#define FSL_FEATURE_EWM_HAS_PRESCALER (1)

/* I2C module features */

/* @brief Has System Management Bus support (registers SMB, A2, SLTL and SLTH). */
#define FSL_FEATURE_I2C_HAS_SMBUS (1)
/* @brief Has DMA support (register bit C1[DMAEN]). */
#define FSL_FEATURE_I2C_HAS_DMA_SUPPORT (1)
/* @brief Has I2C bus start and stop detection (register bits FLT[SSIE], FLT[STARTF] and FLT[STOPF]). */
#define FSL_FEATURE_I2C_HAS_START_STOP_DETECT (1)
/* @brief Has I2C bus stop hold off (register bit FLT[SHEN]). */
#define FSL_FEATURE_I2C_HAS_STOP_HOLD_OFF (1)
/* @brief Has slave baud rate control (register bit C2[SBRC]). */
#define FSL_FEATURE_I2C_NO_SBRC (1)
/* @brief Maximum width of the glitch filter in number of bus clocks. */
#define FSL_FEATURE_I2C_MAX_GLITCH_FILTER_WIDTH (15)
/* @brief Has control of the drive capability of the I2C pins. */
#define FSL_FEATURE_I2C_HAS_HIGH_DRIVE_SELECTION (1)

/* PIT module features */

/* @brief If PIT use 32bit counter */
#define FSL_FEATURE_PIT_32BIT_COUNTER (0)

/* PWM module features */

/* @brief If (e)FlexPWM has module A channels (outputs). */
#define FSL_FEATURE_PWM_HAS_CHANNELA (1)
/* @brief If (e)FlexPWM has module B channels (outputs). */
#define FSL_FEATURE_PWM_HAS_CHANNELB (1)
/* @brief If (e)FlexPWM has module X channels (outputs). */
#define FSL_FEATURE_PWM_HAS_CHANNELX (1)
/* @brief If (e)FlexPWM has fractional feature. */
#define FSL_FEATURE_PWM_HAS_FRACTIONAL (1)
/* @brief If (e)FlexPWM has mux trigger source select bit field. */
#define FSL_FEATURE_PWM_HAS_MUX_TRIGGER_SOURCE_SEL (1)
/* @brief Number of submodules in each (e)FlexPWM module. */
#define FSL_FEATURE_PWM_SUBMODULE_COUNT (4U)
/* @brief Number of fault channel in each (e)FlexPWM module. */
#define FSL_FEATURE_PWM_FAULT_CH_COUNT (2)

/* QSCI module features */

/* @brief Capacity (number of entries) of the transmit/receive FIFO (or zero if no FIFO is available). */
#define FSL_FEATURE_QSCI_FIFO_SIZEn(x) (4)

/* SIM module features */

/* @brief If SIM has GPSAL register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSAL (1)
/* @brief If SIM has GPSAH register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSAH (1)
/* @brief If SIM has GPSBL register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSBL (1)
/* @brief If SIM has GPSBH register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSBH (1)
/* @brief If SIM has GPSCL register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSCL (1)
/* @brief If SIM has GPSCH register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSCH (1)
/* @brief If SIM has GPSDL register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSDL (1)
/* @brief If SIM has GPSDH register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSDH (0)
/* @brief If SIM has GPSEL register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSEL (1)
/* @brief If SIM has GPSEH register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSEH (1)
/* @brief If SIM has GPSFL register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSFL (1)
/* @brief If SIM has GPSFH register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSFH (1)
/* @brief If SIM has GPSGL register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSGL (1)
/* @brief If SIM has GPSGH register */
#define FSL_FEATURE_SIM_PINMUX_HAS_GPSGH (1)
/* @brief If SIM has NVMOPT6_LOW register */
#define FSL_FEATURE_SIM_HAS_NVMOPT6_LOW_REGISTER (1)
/* @brief If SIM has PWM_SEL register */
#define FSL_FEATURE_SIM_HAS_PWM_SEL_REGISTER (1)
/* @brief If SIM MISC0 register has MODE_STAT bit field */
#define FSL_FEATURE_SIM_MISC0_HAS_MODE_STAT_BIT_FIELD (0)
/* @brief If SIM MISC0 register has LPI2C0_TRIG_SEL bit field */
#define FSL_FEATURE_SIM_MISC0_HAS_LPI2C0_TRIG_SEL_BIT_FIELD (0)
/* @brief If SIM MISC0 register has LPI2C1_TRIG_SEL bit field */
#define FSL_FEATURE_SIM_MISC0_HAS_LPI2C1_TRIG_SEL_BIT_FIELD (0)

/* XBARA module features */

/* @brief Number of dma/interrupt requests. */
#define FSL_FEATURE_XBARA_INTERRUPT_COUNT (4)

#endif /* _MC56F83783_FEATURES_H_ */
