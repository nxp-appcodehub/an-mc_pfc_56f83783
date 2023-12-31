/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*! @name GPIOC0 (number 3), USER_LED
  @{ */
#define BOARD_USER_LED_GPIO GPIOC          /*!<@brief GPIO device name: GPIOC */
#define BOARD_USER_LED_PIN 0U              /*!<@brief GPIOC pin index: 0 */
#define BOARD_USER_LED_PIN_MASK kGPIO_Pin0 /*!<@brief PORT pin mask */
                                           /* @} */

/*! @name GPIOF7 (number 59), RELAY
  @{ */
#define BOARD_RELAY_GPIO GPIOF          /*!<@brief GPIO device name: GPIOF */
#define BOARD_RELAY_PIN 7U              /*!<@brief GPIOF pin index: 7 */
#define BOARD_RELAY_PIN_MASK kGPIO_Pin7 /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name GPIOC1 (number 4), USER_LED_2
  @{ */
#define BOARD_USER_LED_2_GPIO GPIOC          /*!<@brief GPIO device name: GPIOC */
#define BOARD_USER_LED_2_PIN 1U              /*!<@brief GPIOC pin index: 1 */
#define BOARD_USER_LED_2_PIN_MASK kGPIO_Pin1 /*!<@brief PORT pin mask */
                                             /* @} */

/*! @name GPIOF6 (number 58), BRAKE
  @{ */
#define BOARD_BRAKE_GPIO GPIOF          /*!<@brief GPIO device name: GPIOF */
#define BOARD_BRAKE_PIN 6U              /*!<@brief GPIOF pin index: 6 */
#define BOARD_BRAKE_PIN_MASK kGPIO_Pin6 /*!<@brief PORT pin mask */
                                        /* @} */

/*! @name GPIOF3 (number 40), TP22
  @{ */
#define BOARD_TP22_GPIO GPIOF          /*!<@brief GPIO device name: GPIOF */
#define BOARD_TP22_PIN 3U              /*!<@brief GPIOF pin index: 3 */
#define BOARD_TP22_PIN_MASK kGPIO_Pin3 /*!<@brief PORT pin mask */
                                       /* @} */

/*! @name GPIOF2 (number 39), TP24
  @{ */
#define BOARD_TP24_GPIO GPIOF          /*!<@brief GPIO device name: GPIOF */
#define BOARD_TP24_PIN 2U              /*!<@brief GPIOF pin index: 2 */
#define BOARD_TP24_PIN_MASK kGPIO_Pin2 /*!<@brief PORT pin mask */
                                       /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
