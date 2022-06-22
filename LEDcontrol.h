/******************************************************************************
* File Name: LEDcontrol.h
*
* Description: This file contains all the function prototypes of
*              LED data packets for SPI master.
*
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#ifndef SOURCE_LEDCONTROL_H_
#define SOURCE_LEDCONTROL_H_

#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "SpiMaster.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define LED1                    (0u)
#define LED2                    (1u)
#define LED3                    (2u)

/*******************************************************************************
* RGB LED context
*******************************************************************************/
typedef struct stc_serial_led_param
{
    uint8_t color_red, color_green, color_blue;
} stc_serial_led_param_t;


/* serial LED context structure  */
typedef struct stc_serial_led_context
{
    stc_serial_led_param_t led_num[3u];
} stc_serial_led_context_t;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void serial_led_packets(uint8_t, uint8_t, uint8_t, uint8_t);
void LED_serial_data_tranfer(uint8_t, uint8_t, uint8_t);
void serial_led_control(stc_serial_led_context_t *);
uint32_t led_byte_to_binary_conversion(uint8_t);

#endif /* SOURCE_LEDCONTROL_H_ */

/* [] END OF FILE */
