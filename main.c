/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the MSCLP Robust Low-Power
*              Liquid-Tolerant CAPSENSE&trade; code example for ModusToolbox
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
* Include header files
******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "LEDcontrol.h"
#include "SpiMaster.h"

/*******************************************************************************
* User Configurable Macros
*******************************************************************************/
/* Enable this, if Debugging needs to be enabled -
 *    See Readme for more details.
 *    SWD and Tuner cannot be enabled together as the pins are MUXed in CY8CKIT-040T */
#define SWD_DEBUG_ENABLE                        (0u)

/* Enable the serial LED feature for touch indication */
#define ENABLE_SERIAL_LED                       (1u)

/* Enable this, if Tuner needs to be enabled */
#define ENABLE_TUNER                            (1u)

/* Define the Refresh rate (in Hz) in two different application states (of CAPSENSE&trade;)*/
#define ACTIVE_MODE_REFRESH_RATE                (128u)
#define ACTIVE_LOW_REFRESH_RATE                 (32u)

/* Define timeout (in seconds) for two different states */
#define ACTIVE_MODE_TIMEOUT_IN_SEC              (5u)
#define ACTIVE_LOW_REFRESH_TIMEOUT_IN_SEC       (5u)

/* Active mode Scan time in us ~= 140us */
#define ACTIVE_MODE_FRAME_SCAN_TIME             (140u)
/* Active mode Processing time in us ~= 127us with Serial LED and Tuner disabled
 * and 288us with Serial LED and Tuner enabled */
#define ACTIVE_MODE_PROCESS_TIME                (127u)

/* ALR mode Scan time in us ~= 140us */
#define ALR_MODE_FRAME_SCAN_TIME                (140u)
/* ALR mode Processing time in us ~= 127us with Serial LED and Tuner disabled
 * and 288us with Serial LED and Tuner enabled */
#define ALR_MODE_PROCESS_TIME                   (127u)

/*******************************************************************************
* Fixed Macros
*******************************************************************************/
#define CAPSENSE_MSC0_INTR_PRIORITY             (3u)
#define CY_ASSERT_FAILED                        (0u)
#define MSCLP_CAPSENSE_WIDGET_INACTIVE          (0u)

/* Setting self-cap sensors raw-count calibration percentage to 70% */
#define CUSTOM_CSD_CALIBRATION_LEVEL            (70u)

/* setting recommended CDAC Dither scale value. Default is '0u' */
#define	CDAC_DITHER_SCALE                       (1u)

/* setting recommended CDAC Dither seed value. Default is '255u' */
#define	CDAC_DITHER_SEED                        (15u)

/* setting recommended CDAC Dither poly value. Default is '142u' */
#define	CDAC_DITHER_POLY                        (9u)

/* EZI2C interrupt priority must be higher than CAPSENSE&trade; interrupt. */
#define EZI2C_INTR_PRIORITY                     (2u)

/* Define the Reset state of Timeout counter */
#define TIMER_RESET                             (0u)

/* Disable EZI2C when SWD Debug is enabled in the User defined Macro*/
#define ENABLE_EZI2C                            (!SWD_DEBUG_ENABLE)

#define ILO_FREQ                                (40000u)
#define TIME_IN_US                              (1000000u)

#define MINIMUM_TIMER                    (TIME_IN_US / ILO_FREQ)

#if ((TIME_IN_US / ACTIVE_MODE_REFRESH_RATE) > (ACTIVE_MODE_FRAME_SCAN_TIME + ACTIVE_MODE_PROCESS_TIME))
    #define ACTIVE_MODE_TIMER            (TIME_IN_US / ACTIVE_MODE_REFRESH_RATE - \
                                            (ACTIVE_MODE_FRAME_SCAN_TIME + ACTIVE_MODE_PROCESS_TIME))
#else
    #define ACTIVE_MODE_TIMER                   (MINIMUM_TIMER)
#endif

#if ((TIME_IN_US / ACTIVE_LOW_REFRESH_RATE) > (ALR_MODE_FRAME_SCAN_TIME + ALR_MODE_PROCESS_TIME))
    #define ACTIVE_LOW_REFRESH_TIMER               (TIME_IN_US / ACTIVE_LOW_REFRESH_RATE - \
                                            (ALR_MODE_FRAME_SCAN_TIME + ALR_MODE_PROCESS_TIME))
#else
    #define ACTIVE_LOW_REFRESH_TIMER            (MINIMUM_TIMER)
#endif

/* Define Time out of different applications states depending upon
 * number of scans and corresponding refresh rate */
#define ACTIVE_MODE_TIMEOUT              (ACTIVE_MODE_REFRESH_RATE * ACTIVE_MODE_TIMEOUT_IN_SEC)
#define ACTIVE_LOW_REFRESH_TIMEOUT       (ACTIVE_LOW_REFRESH_RATE * ACTIVE_LOW_REFRESH_TIMEOUT_IN_SEC)

#define LIQUID_ACTIVE_MODE_TIMEOUT              (0u)

/* Define the conditions to check sensor status and liquid presence state */
#define SENSOR_ACTIVE                           (1u)
#define LIQUID_STATE_ACTIVE                     (1u)

/*****************************************************************************
* Finite state machine for different CAPSENSE&trade; operating states
*****************************************************************************/
typedef enum
{
    ACTIVE_STATE = 0x01u,                       /* Active mode - All the sensors are scanned in this state
                                                 * with highest refresh rate */
    ACTIVE_LOW_REFRESH_RATE_STATE = 0x02u,      /* Active-Low Refresh Rate (ALR) mode - All the sensors are
                                                 * scanned in this state with low refresh rate */
    WAKE_ON_TOUCH_STATE = 0x03u,                /* Wake on Touch (WoT) mode - Low Power sensors are scanned
                                                 * in this state with lowest refresh rate */
    LIQUID_ACTIVE_STATE = 0x04u                 /* Custom state, that indicates large volume of
                                                 * liquid present on sensors. Only the guard sensor is scanned
                                                 * in this state and other sensors are disabled
                                                 */
} CAPSENSE_STATE;

/*******************************************************************************
* Global Definitions
*******************************************************************************/
#if ENABLE_EZI2C
cy_stc_scb_ezi2c_context_t ezi2c_context;
#endif

#if ENABLE_SERIAL_LED
stc_serial_led_context_t led_context;
#endif

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void initialize_capsense(void);
static void set_dither_parameters(void);
static void capsense_msc0_isr(void);

void inactive_sensors_reconfiguration(void * context);

#if ENABLE_EZI2C
static void initialize_capsense_tuner(void);
static void ezi2c_isr(void);
#endif

#if ENABLE_SERIAL_LED
void led_control();
#endif

/* Deep Sleep Callback function */
void register_callback(void);
cy_en_syspm_status_t deep_sleep_callback(cy_stc_syspm_callback_params_t *callbackParams,
                                         cy_en_syspm_callback_mode_t mode);

/*******************************************************************************
* Global Definitions
*******************************************************************************/
#if ENABLE_EZI2C
cy_stc_scb_ezi2c_context_t ezi2c_context;
#endif

#if ENABLE_SERIAL_LED
extern cy_stc_scb_spi_context_t CYBSP_MASTER_SPI_context;
stc_serial_led_context_t led_context;
#endif

/* Call back parameters for custom, EzI2C, SPI */

#if ENABLE_EZI2C
/* Callback params for EzI2C */
cy_stc_syspm_callback_params_t ezi2cCallbackParams =
{
    .base       = SCB1,
    .context    = &ezi2c_context
};
#endif

#if ENABLE_SERIAL_LED
/* Callback params for SPI */
cy_stc_syspm_callback_params_t spiCallbackParams =
{
    .base       = SCB0,
    .context    = &CYBSP_MASTER_SPI_context
};
#endif

/* Callback params for custom callback */
cy_stc_syspm_callback_params_t deepSleepCallBackParams = {
    .base       =  NULL,
    .context    =  NULL
};

#if ENABLE_EZI2C
/* Callback declaration for EzI2C Deep Sleep callback */
cy_stc_syspm_callback_t ezi2cCallback =
{
    .callback       = (Cy_SysPmCallback)&Cy_SCB_EZI2C_DeepSleepCallback,
    .type           = CY_SYSPM_DEEPSLEEP,
    .skipMode       = 0UL,
    .callbackParams = &ezi2cCallbackParams,
    .prevItm        = NULL,
    .nextItm        = NULL,
    .order          = 0
};
#endif

#if ENABLE_SERIAL_LED
/* Callback declaration for SPI Deep Sleep callback */
cy_stc_syspm_callback_t spiCallback =
{
    .callback       = (Cy_SysPmCallback)&Cy_SCB_SPI_DeepSleepCallback,
    .type           = CY_SYSPM_DEEPSLEEP,
    .skipMode       = 0UL,
    .callbackParams = &spiCallbackParams,
    .prevItm        = NULL,
    .nextItm        = NULL,
    .order          = 1
};
#endif

/* Callback declaration for Custom Deep Sleep callback */
cy_stc_syspm_callback_t deepSleepCb =
{
    .callback       = &deep_sleep_callback,
    .type           = CY_SYSPM_DEEPSLEEP,
    .skipMode       = 0UL,
    .callbackParams = &deepSleepCallBackParams,
    .prevItm        = NULL,
    .nextItm        = NULL,
    .order          = 2
};


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - initialize CAPSENSE&trade;
*  - initialize inactive sensor states
*  - initialize tuner communication
*  - scan touch input continuously
*  - switch to different CAPSENSE&trade; states
*  - serial RGB LED for touch indication
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t timeout_counter;
    CAPSENSE_STATE capsense_state;
    uint32_t interruptStatus;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable global interrupts */
    __enable_irq();

    #if ENABLE_SERIAL_LED
    /* Initialize SPI master */
    result = init_spi_master();

    /* If SPI initialization failed, stop program execution */
    if (result != INIT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    #else

    /* SPI pins drive mode to Analog HighZ */
    Cy_GPIO_SetDrivemode(CYBSP_SERIAL_LED_PORT, CYBSP_SERIAL_LED_NUM, CY_GPIO_DM_ANALOG);
    #endif

    #if ENABLE_EZI2C
    /* Initialize EZI2C */
    initialize_capsense_tuner();
    #endif

    /* Register callbacks */
    register_callback();

    /* Initialize MSC CAPSENSE&trade; */
    initialize_capsense();

    /* Initialization of CAPSENSE&trade; state, timeout counter and MSCLP timer configuration  */
    capsense_state = ACTIVE_STATE;
    timeout_counter = TIMER_RESET;

    /* Measures the actual ILO frequency and compensate MSCLP wake up timers */
    Cy_CapSense_IloCompensate(&cy_capsense_context);

    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);

    /* Calibrate all the low power widgets */
    Cy_CapSense_CalibrateAllLpSlots(&cy_capsense_context);

/*******************************************************************************
* Start the state-machine to capture the three different states of operation.
* This is a state-machine implemented to capture 4 different states
* of CAPSENSE&trade; block for this specific application.
*******************************************************************************/
    for (;;)
    {
        switch(capsense_state)
        {
            /* CAPSENSE&trade; state: ACTIVE */
            case ACTIVE_STATE :
            {
                Cy_CapSense_ScanAllSlots(&cy_capsense_context);

                interruptStatus = Cy_SysLib_EnterCriticalSection();

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    Cy_SysPm_CpuEnterDeepSleep ();

                    Cy_SysLib_ExitCriticalSection(interruptStatus);

                    /* This is a place where all interrupt handlers will be executed */
                    interruptStatus = Cy_SysLib_EnterCriticalSection();

                }

                Cy_SysLib_ExitCriticalSection(interruptStatus);

                Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

                /* Scan, process and check the status of the all Active mode sensors */
                if (Cy_CapSense_IsAnyWidgetActive(&cy_capsense_context))
                {
                    timeout_counter = TIMER_RESET;

                    /* if Guard sensor is active, change the state to LIQUID_ACTIVE_STATE*/
                    if (LIQUID_STATE_ACTIVE ==
                            Cy_CapSense_IsWidgetActive(CY_CAPSENSE_GUARD_SENSOR_WDGT_ID, &cy_capsense_context))
                    {
                        capsense_state = LIQUID_ACTIVE_STATE;
                        timeout_counter = TIMER_RESET;
                        Cy_CapSense_InitializeAllStatuses(&cy_capsense_context);
                    }
                }

                else
                {
                    timeout_counter++;

                    /* if there is no touch and the timeout happens, change the
                    * state to ACTIVE_LOW_REFRESH_RATE*/
                    if (ACTIVE_MODE_TIMEOUT < timeout_counter)
                    {
                        capsense_state = ACTIVE_LOW_REFRESH_RATE_STATE;
                        timeout_counter = TIMER_RESET;
                        Cy_CapSense_ConfigureMsclpTimer(ACTIVE_LOW_REFRESH_TIMER, &cy_capsense_context);
                    }
                }

                #if ENABLE_SERIAL_LED
                /* Serial LED control for showing the CAPSENSE&trade; touch status (feedback) */
                led_control();
                #endif

                break;
            }
            /* end of ACTIVE_STATE */

            /* CAPSENSE&trade; state: Active Low Refresh Rate state */
            case ACTIVE_LOW_REFRESH_RATE_STATE :
            {
                Cy_CapSense_ScanAllSlots(&cy_capsense_context);

                interruptStatus = Cy_SysLib_EnterCriticalSection();

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    Cy_SysPm_CpuEnterDeepSleep ();

                    Cy_SysLib_ExitCriticalSection(interruptStatus);

                    /* This is a place where all interrupt handlers will be executed */
                    interruptStatus = Cy_SysLib_EnterCriticalSection();

                }

                Cy_SysLib_ExitCriticalSection(interruptStatus);

                Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

                /* Scan, process and check the status of the all Active mode sensors */
                if (Cy_CapSense_IsAnyWidgetActive(&cy_capsense_context))
                {
                    capsense_state = ACTIVE_STATE;
                    timeout_counter = TIMER_RESET;
                    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);
                }

                else
                {
                    timeout_counter++;

                    /* if there is no touch and the timeout happens, change the
                    * state to ACTIVE_LOW_REFRESH_RATE.*/
                    if (ACTIVE_LOW_REFRESH_TIMEOUT < timeout_counter)
                    {
                        capsense_state = WAKE_ON_TOUCH_STATE;
                        timeout_counter = TIMER_RESET;
                    }
                }

                break;
            }
            /* end of ACTIVE_LOW_REFRESH_RATE */

            /* CAPSENSE&trade; state: Wake On Touch  */
            case WAKE_ON_TOUCH_STATE :
            {
                Cy_CapSense_ScanAllLpSlots(&cy_capsense_context);

                interruptStatus = Cy_SysLib_EnterCriticalSection();

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    Cy_SysPm_CpuEnterDeepSleep();

                    Cy_SysLib_ExitCriticalSection(interruptStatus);

                    /* This is a place where all interrupt handlers will be executed */
                    interruptStatus = Cy_SysLib_EnterCriticalSection();

                }

                Cy_SysLib_ExitCriticalSection(interruptStatus);

                /* if there is any touch event, change the state to
                *  ACTIVE_STATE.*/
                if (Cy_CapSense_IsAnyLpWidgetActive(&cy_capsense_context))
                {
                    capsense_state = ACTIVE_STATE;
                    timeout_counter = TIMER_RESET;
                    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);
                }

                /* if there is no touch and the timeout happens, change the
                *  state to ACTIVE_LOW_REFRESH_RATE.*/
                else
                {
                    capsense_state = ACTIVE_LOW_REFRESH_RATE_STATE;
                    timeout_counter = TIMER_RESET;
                    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_LOW_REFRESH_TIMER, &cy_capsense_context);
                }

                break;
            }
            /* end of "WAKE_ON_TOUCH_STATE" */

            /* CAPSENSE&trade; state: LIQUID_ACTIVE_STATE
             * This is custom state for scanning guard sensor(s) only when large volume of
             * liquid present on the sensors
             *  */

            case LIQUID_ACTIVE_STATE :
            {
                /* Scan only the slot(s) of the guard sensor(s) that can indicate presence of liquid */
                Cy_CapSense_ScanSlots(CY_CAPSENSE_GUARD_SENSOR_FIRST_SLOT_ID, CY_CAPSENSE_GUARD_SENSOR_SLOT_SIZE, &cy_capsense_context);

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    Cy_SysPm_CpuEnterDeepSleep ();
                }

                /* Process only the guard sensor(s) widget to detect the presence of liquid */
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_GUARD_SENSOR_WDGT_ID, &cy_capsense_context);

                /* Scan, process and check the status of the all Active mode sensors */
                if (LIQUID_STATE_ACTIVE == Cy_CapSense_IsWidgetActive(CY_CAPSENSE_GUARD_SENSOR_WDGT_ID,
                        &cy_capsense_context))
                {
                    capsense_state = LIQUID_ACTIVE_STATE;
                }

                else
                {
                    capsense_state = ACTIVE_STATE;
                    timeout_counter = TIMER_RESET;
                }

                #if ENABLE_SERIAL_LED
                /* Serial LED control for showing the CAPSENSE&trade; touch status (feedback) */
                led_control();
                #endif

                break;
            }
            /* end of LIQUID ACTIVE_STATE */

            default:
            {
                /**  Unknown power mode state. Unexpected situation.  **/
                CY_ASSERT(CY_ASSERT_FAILED);

                break;
            }

        }
        /* end of switch */

        #if ENABLE_TUNER
        /* Establishes synchronized communication with the CAPSENSE&trade; Tuner tool */
        Cy_CapSense_RunTuner(&cy_capsense_context);
        #endif
    }
    /* end of for loop */
}
/* end of main */


/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CAPSENSE&trade; and configures the CAPSENSE&trade;
*  interrupt.
*
*******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CAPSENSE&trade; interrupt configuration MSCLP 0 */
    const cy_stc_sysint_t capsense_msc0_interrupt_config =
    {
        .intrSrc = CY_MSCLP0_LP_IRQ,
        .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };

    /* Capture the MSC HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    /* Capture the MSC HW block and initialize it to the default state. */
    cy_capsense_context.ptrInternalContext->ptrEODsInitCallback =
            inactive_sensors_reconfiguration;

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CAPSENSE&trade; interrupt for MSC 0 */
        Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
        NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

        /* Setting self-cap sensors raw-count calibration percentage to 70% */
        Cy_CapSense_SetCalibrationTarget(CUSTOM_CSD_CALIBRATION_LEVEL, CY_CAPSENSE_CSD_GROUP, &cy_capsense_context);

        /* Setting Dither parameter
        * Must be called after Cy_CapSense_Init() and before Cy_CapSense_Enable()
        */
        set_dither_parameters();

        /* Initialize the CAPSENSE&trade; firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if (status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CAPSENSE&trade; sensors are tuned
         * as per procedure give in the Readme.md file */
    }
}


/*******************************************************************************
* Function Name: capsense_msc0_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CAPSENSE&trade; MSC0 block.
*
*******************************************************************************/
static void capsense_msc0_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSCLP0_HW, &cy_capsense_context);
}


/*******************************************************************************
* Function Name: inactive_sensors_reconfiguration
********************************************************************************
* Summary:
*  This function initializes selective sensors to shield in the desired slots.
*  By default all the inactive sensors are assigned to Active shield.
*  Inactive sensor that are required to be set to some other state, the
*  corresponding slot Ids, the widget Id, the sensor Id, and the Inactive
*  sensor state to be defined in the Cy_CapSense_SlotPinState API and to be
*  initialized in initialize_capsense.
*
*******************************************************************************/
void inactive_sensors_reconfiguration(void * context)
{
    uint32_t slotId, touchpad_scan_first_slot_Id, touchpad_scan_last_slot_Id;

    /* when the Touchpad is being scanned in slot # 1 to 9.
    * proximity loop is set to ground.
    * Other Inactive sensors (Button and touchpad) are driven as
    * active shield (as set in the design.cycapsense).
    */
    touchpad_scan_first_slot_Id = 1u;
    touchpad_scan_last_slot_Id = 9u;

    (void)context;

    for (slotId = touchpad_scan_first_slot_Id; slotId <= touchpad_scan_last_slot_Id; slotId++)
    {
        Cy_CapSense_SlotPinState(slotId, &cy_capsense_context.ptrWdConfig
                [CY_CAPSENSE_GUARD_SENSOR_WDGT_ID].ptrEltdConfig[0u],
                CY_CAPSENSE_PIN_STATE_IDX_GND, &cy_capsense_context);
    }

}


#if ENABLE_EZI2C
/*******************************************************************************
* Function Name: initialize_capsense_tuner
********************************************************************************
* Summary:
* EZI2C module to communicate with the CAPSENSE&trade; Tuner tool.
*
*******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
        .intrSrc = CYBSP_EZI2C_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);

    if (status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CAPSENSE&trade; data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8_t *)&cy_capsense_tuner,
                            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
                            &ezi2c_context);

    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);
}


/*******************************************************************************
* Function Name: ezi2c_isr
********************************************************************************
* Summary:
* Wrapper function for handling interrupts from EZI2C block.
*
*******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}
#endif
/* ENABLE_EZI2C */


#if ENABLE_SERIAL_LED
/*******************************************************************************
* Function Name: led_control
*******************************************************************************
* Summary:
* Logic to control the on / off status, color and brightness of the LEDs
* based on the touch status of the CAPSENSE&trade; widgets.
*
*
*******************************************************************************/
void led_control()
{
    /* Brightness of each LED is represented by 0 to 255,
    * where 0 indicates LED in OFF state and 255 indicate maximum
    * brightness of an LED
    */
    volatile uint8_t brightness_max = 255u;
    volatile uint8_t brightness_min = 0u;

    uint8_t touchposition_x, touchposition_y ;
    cy_stc_capsense_touch_t *panelTouch =
            Cy_CapSense_GetTouchInfo(CY_CAPSENSE_TOUCHPAD_WDGT_ID, &cy_capsense_context);

    touchposition_x = panelTouch->ptrPosition->x;
    touchposition_y = panelTouch->ptrPosition->y;
/*LED1 and LED3 indicate the status of the 'CAPSENSE&trade; button' and 'Touchpad' */

/*******************************************************************************
* If CAPSENSE&trade; button is active, Turn On LED1 with color Orange (Fixed intensity)
*******************************************************************************/
    if (SENSOR_ACTIVE == Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON_WDGT_ID, &cy_capsense_context))
    {
        led_context.led_num[LED1].color_red = brightness_min;
        led_context.led_num[LED1].color_green = brightness_min;
        led_context.led_num[LED1].color_blue = brightness_max;

        led_context.led_num[LED3].color_red = brightness_min;
        led_context.led_num[LED3].color_green = brightness_min;
        led_context.led_num[LED3].color_blue = brightness_min;
    }

/*******************************************************************************
* If the Touchpad is active, Turn On LED1 and LED3
* with Green color, and vary the intensity of the LEDs as per the finger
* position reported
*******************************************************************************/
    else if (SENSOR_ACTIVE == Cy_CapSense_IsWidgetActive(CY_CAPSENSE_TOUCHPAD_WDGT_ID, &cy_capsense_context))
    {
        led_context.led_num[LED1].color_red = brightness_min;
        led_context.led_num[LED1].color_green = touchposition_x;
        led_context.led_num[LED1].color_blue = brightness_min;

        led_context.led_num[LED3].color_red = brightness_min;
        led_context.led_num[LED3].color_green = touchposition_y;
        led_context.led_num[LED3].color_blue = brightness_min;
    }

/*******************************************************************************
* Default, all the LEDs are turned off
********************************************************************************/
    else
    {
        led_context.led_num[LED1].color_red = brightness_min;
        led_context.led_num[LED1].color_green = brightness_min;
        led_context.led_num[LED1].color_blue = brightness_min;

        led_context.led_num[LED3].color_red = brightness_min;
        led_context.led_num[LED3].color_green = brightness_min;
        led_context.led_num[LED3].color_blue = brightness_min;
    }

    led_context.led_num[LED2].color_red = brightness_min;
    led_context.led_num[LED2].color_green = brightness_min;
    led_context.led_num[LED2].color_blue = brightness_min;


    serial_led_control(&led_context);
}
#endif
/* ENABLE_SERIAL_LED */


/*******************************************************************************
* Function Name: register_callback
********************************************************************************
*
* Summary:
* Register Deep Sleep callbacks for EzI2C, SPI components
*
* Parameters:
*  void
*
* Return:
* void
*
*******************************************************************************/
void register_callback(void)
{
    #if ENABLE_EZI2C
    /* Register EzI2C Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&ezi2cCallback);
    #endif

    #if ENABLE_SERIAL_LED
    /* Register SPI Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&spiCallback);
    #endif

    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&deepSleepCb);
}


/*******************************************************************************
* Function Name: deep_sleep_callback
********************************************************************************
*
* Summary:
* Deep Sleep callback implementation. Waits for the completion of SPI transaction.
* And change the SPI GPIOs to highZ while transition to deep-sleep and vice-versa
*
* Parameters:
*  callbackParams: The pointer to the callback parameters structure cy_stc_syspm_callback_params_t.
*  mode: Callback mode, see cy_en_syspm_callback_mode_t
*
* Return:
*  Entered status, see cy_en_syspm_status_t.
*
*******************************************************************************/
cy_en_syspm_status_t deep_sleep_callback(
       cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t ret_val = CY_SYSPM_FAIL;

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_CHECK_FAIL:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_BEFORE_TRANSITION:

            #if ENABLE_SERIAL_LED
            /* SPI pins drive mode to Analog HighZ */
            Cy_GPIO_SetDrivemode(CYBSP_SPI_MOSI_PORT,CYBSP_SPI_MOSI_PIN,CY_GPIO_DM_ANALOG);
            #endif

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_AFTER_TRANSITION:

            #if ENABLE_SERIAL_LED
            /* SPI pins drive mode to Strong */
            Cy_GPIO_SetDrivemode(CYBSP_SPI_MOSI_PORT,CYBSP_SPI_MOSI_PIN,CY_GPIO_DM_STRONG_IN_OFF);
            #endif

            ret_val = CY_SYSPM_SUCCESS;
            break;

        default:
            /* Don't do anything in the other modes */
            ret_val = CY_SYSPM_SUCCESS;
            break;
    }
    return ret_val;
}

/*******************************************************************************
* Function Name: set_dither_parameters
********************************************************************************
* Summary:
*  This functions sets the below CDAC Dither parameters to achive better performance
*  1. CDAC_Dither_Scale
*  		- Default value is '0'
*  		- Recommended value defined in macro 'CDAC_DITHER_SCALE'
*  2. CDAC_Dither_poly
*  		- Default value is '142'
*  		- Recommended value defined in macro 'CDAC_DITHER_POLY'
*  3. CDAC_Dither_Seed
*  		- Default value is '255'
*  		- Recommended value defined in macro 'CDAC_DITHER_SEED'
*
*  Note : Must be called after Cy_CapSense_Init() and before Cy_CapSense_Enable
*
*  Refer CE Readme for more details
*  Parameters:  void
*  Return:  void
*******************************************************************************/
static void set_dither_parameters(void)
{
    /* Set dither scale as '1' for specific widgets*/
    cy_capsense_context.ptrWdContext[CY_CAPSENSE_BUTTON_WDGT_ID].cdacDitherValue = CDAC_DITHER_SCALE;
    cy_capsense_context.ptrWdContext[CY_CAPSENSE_GUARD_SENSOR_WDGT_ID].cdacDitherValue = CDAC_DITHER_SCALE;

    /* Set dither polynomial for all widgets*/
    cy_capsense_context.ptrInternalContext->cdacDitherPoly = CDAC_DITHER_POLY;

    /* Set dither seed for all widgets*/
    cy_capsense_context.ptrInternalContext->cdacDitherSeed = CDAC_DITHER_SEED;
}

/* [] END OF FILE */
