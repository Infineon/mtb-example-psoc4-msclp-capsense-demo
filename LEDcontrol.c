/*******************************************************************************
 * File Name:   LEDcontrol.c
 *
 * Description: This file contains required functions for creating required
 *              LED data packets for SPI master.
 *
 *******************************************************************************
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
 ******************************************************************************/

#include "LEDcontrol.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define NUM_OF_LEDS                 (3u)
#define NUM_OF_LED_COLORS           (3u)
#define NUM_OF_FRAME_PER_COLOR      (4u)
#define NUM_OF_A_PACKET_OF_A_LED    (NUM_OF_LED_COLORS * NUM_OF_FRAME_PER_COLOR)
#define NUM_OF_SPI_PACKET           (NUM_OF_LEDS * NUM_OF_A_PACKET_OF_A_LED)
#define LED_STATE_OFF               (8u)
#define LED_STATE_ON                (14u)
#define LED_DURATION_PULSE_SIZE     (1u)

/*******************************************************************************
* Global Definitions
*******************************************************************************/
uint8_t txBuffer[NUM_OF_SPI_PACKET + LED_DURATION_PULSE_SIZE];


/*******************************************************************************
* Function Name: serial_led_control
********************************************************************************
* Summary:
* This function creates the required data packets of three individual LEDs. It:
*  - initializes the SPI packet txBuffer,
*  - creates the required data packet of the txBuffer,
*  - initiates transfer of txBuffer through SPI master,
*  - clears the SPI Tx FIFO after the completion of data transfer
*
* Parameters:
* The pointer to the context structure stc_serial_led_context_t.
*
*******************************************************************************/
void serial_led_control(stc_serial_led_context_t * ptr_led_context)
{
    uint32_t led_index = 0u;

    txBuffer[0u] = 0u;

    for (led_index = 0u; led_index < NUM_OF_LEDS; led_index++)
    {
        serial_led_packets(led_index,
            ptr_led_context->led_num[led_index].color_red,
            ptr_led_context->led_num[led_index].color_green,
            ptr_led_context->led_num[led_index].color_blue);
    }

    send_packet(txBuffer, NUM_OF_SPI_PACKET + LED_DURATION_PULSE_SIZE);
    Cy_SCB_SPI_ClearTxFifo(CYBSP_MASTER_SPI_HW);
}


/*******************************************************************************
* Function Name: serial_led_packets
********************************************************************************
*  Summary:
*  This function creates data packet (txBuffer) of a single LED
*  based on the color combination (combination of Red, Green, Blue color)
*  of the LED. The variables red, green and blue signifies the brightness
*  of the individual color represented by number 0 - 255.
*  0 means the particular color is disabled,
*  255 means maximum brightness of the color
*
*  Parameters:
*  - LED index
*  - brightness level of three colors (Red, green and blue) each LED.
*
*******************************************************************************/
void serial_led_packets(uint8_t led_index, uint8_t brightness_red, uint8_t brightness_green, uint8_t brightness_blue)
{
    uint32_t color_index;
    uint32_t rgb_color_binary;
    uint8_t rgb_color[] = {brightness_red, brightness_green, brightness_blue};
    uint8_t *ptrBuffer;

    ptrBuffer = &txBuffer[led_index * NUM_OF_A_PACKET_OF_A_LED + LED_DURATION_PULSE_SIZE];

    for (color_index = 0u; color_index < NUM_OF_LED_COLORS; color_index++)
    {
        rgb_color_binary = led_byte_to_binary_conversion(rgb_color[color_index]);
        ptrBuffer[0u] = CY_HI8(CY_HI16(rgb_color_binary));
        ptrBuffer[1u] = CY_LO8(CY_HI16(rgb_color_binary));
        ptrBuffer[2u] = CY_HI8(CY_LO16(rgb_color_binary));
        ptrBuffer[3u] = CY_LO8(CY_LO16(rgb_color_binary));
        ptrBuffer += NUM_OF_FRAME_PER_COLOR;
    }
}


/*******************************************************************************
* Function Name: led_byte_to_binary_conversion
********************************************************************************
* Summary:
*
* This function converts brightness of the LED from uint8_t format to 
* binary format. This is stored in a uint32_t variable
*
* Parameters:
* LED brightness in uint8_t format
*
* Return:
* LED brightness in uint32_t format
*******************************************************************************/
uint32_t led_byte_to_binary_conversion(uint8_t byte_data)
{
    uint8_t i;
    uint32_t binary_data = 0u;

    /* Process the upper 7 bits of byte_data  */
    for (i = 0u; i < 7u; i++)
    {
        if (byte_data & 0x80)
        {
            binary_data =  binary_data + LED_STATE_ON; 
        }
        else
        {
            binary_data =  binary_data + LED_STATE_OFF; 
        }

        binary_data = (uint32_t) (binary_data << 4u);
        byte_data = (uint8_t) (byte_data << 1u);
    }

    /* LSB without the final shift */
    if (byte_data & 0x80)
    {
        binary_data =  binary_data + LED_STATE_ON; 
    }
    else
    {
        binary_data =  binary_data + LED_STATE_OFF; 
    }

    return binary_data;
}

/* [] END OF FILE */
