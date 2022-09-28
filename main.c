/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the MSCLP Robust Low-Power
*              Liquid-Tolerant CAPSENSE TM code example for ModusToolbox
*
* Related Document: See README.md
*
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
/* Set this to 1 to enable SWD debug*/
#define SWD_DEBUG_ENABLE                        (0u)

/* Enable the serial LED feature for touch indication */
#define ENABLE_SERIAL_LED                       (1u)

/* Define the Refresh rate (in Hz) in two different application states (of CAPSENSE)*/
#define ACTIVE_MODE_REFRESH_RATE                (128u)
#define ACTIVE_LOW_REFRESH_RATE                 (32u)

/* Define timeout (in seconds) for two different states */
#define ACTIVE_MODE_TIMEOUT_IN_SEC              (5u)
#define ACTIVE_LOW_REFRESH_TIMEOUT_IN_SEC       (5u)

/* Measure the ILO clock frequency of the device */
#define MEASURED_ILO_IN_KHz                     (40u)

/* Define the Active state and Active low refresh rate state scan frame time  */
#define ACTIVE_MODE_SCAN_FRAME                  (0u)
#define ACTIVE_LOW_REFRESH_SCAN_FRAME           (0u)

/*******************************************************************************
* Fixed Macros
*******************************************************************************/
#define TIME_IN_MS                              (1000u)

#define CAPSENSE_MSC0_INTR_PRIORITY             (3u)
#define CY_ASSERT_FAILED                        (0u)
#define MSCLP_CAPSENSE_WIDGET_INACTIVE          (0u)

/* Setting self-cap sensors raw-count calibration percentage to 70% */
#define CUSTOM_CSD_CALIBRATION_LEVEL            (70u)

/* EZI2C interrupt priority must be higher than CAPSENSE interrupt. */
#define EZI2C_INTR_PRIORITY                     (2u)

/* Define the Reset state of Timeout counter */
#define TIMER_RESET                             (0u)

/* Disable EZI2C when SWD Debug is enabled in the User defined Macro*/
#define ENABLE_EZI2C                            (!SWD_DEBUG_ENABLE)

/* Adjustment of refresh rate based on actual measurement of ILO clock frequency */
#define EXPECTED_ILO_IN_KHz                     (40u)
#define ACTIVE_MODE_REFRESH_RATE_SCALED         (ACTIVE_MODE_REFRESH_RATE * EXPECTED_ILO_IN_KHz / MEASURED_ILO_IN_KHz)
#define ACTIVE_LOW_REFRESH_RATE_SCALED          (ACTIVE_LOW_REFRESH_RATE * EXPECTED_ILO_IN_KHz / MEASURED_ILO_IN_KHz)

#if ((TIME_IN_MS / ACTIVE_MODE_REFRESH_RATE_SCALED) > ACTIVE_MODE_SCAN_FRAME)
    #define ACTIVE_MODE_TIMER                   ((TIME_IN_MS / ACTIVE_MODE_REFRESH_RATE_SCALED) - ACTIVE_MODE_SCAN_FRAME)
#else
    #define ACTIVE_MODE_TIMER                   (0u)
#endif

#if ((TIME_IN_MS / ACTIVE_LOW_REFRESH_RATE_SCALED) > ACTIVE_LOW_REFRESH_SCAN_FRAME)
    #define ACTIVE_LOW_REFRESH_TIMER            ((TIME_IN_MS / ACTIVE_LOW_REFRESH_RATE_SCALED) - ACTIVE_LOW_REFRESH_SCAN_FRAME)
#else
    #define ACTIVE_LOW_REFRESH_TIMER            (0u)
#endif

/* Define Time out of different applications states depending upon
 * number of scans and corresponding refresh rate */
#define ACTIVE_MODE_TIMEOUT                     (ACTIVE_MODE_TIMEOUT_IN_SEC * ACTIVE_MODE_REFRESH_RATE_SCALED)
#define ACTIVE_LOW_REFRESH_TIMEOUT              (ACTIVE_LOW_REFRESH_TIMEOUT_IN_SEC * ACTIVE_LOW_REFRESH_RATE_SCALED)
#define LIQUID_ACTIVE_MODE_TIMEOUT              (0u)

/* macros indicating touch / no-touch status of all sensors */
#define SENSOR_ACTIVE                           (1u)
#define SENSOR_INACTIVE                         (0u)

/* Define the macros specific for the Guard sensors and conditions to check liquid active state */
#define GUARD_SENSOR_SELF_CAP_ACTIVE            (1u)
#define GUARD_SENSOR_MUTUAL_CAP_GANGED_ACTIVE   (2u)
#define GUARD_SENSOR_MUTUAL_CAP_LEFT_ACTIVE     (3u)
#define GUARD_SENSOR_MUTUAL_CAP_RIGHT_ACTIVE    (4u)
#define GUARD_SENSOR_MUTUAL_CAP_MID_ACTIVE      (5u)
#define LIQUID_STATE_ACTIVE                     (1u)
#define LIQUID_STATE_INACTIVE                   (0u)
#define SENSOR_CONTEXT_GUARD_MUTUAL_CAP_SENSOR  (12u)
#define FIRST_SLOT_SCAN_IN_LIQUID_ACTIVE_STATE  (CY_CAPSENSE_GUARD_LOOP_SENSOR_FIRST_SLOT_ID)
#define LAST_SLOT_SCAN_IN_LIQUID_ACTIVE_STATE   (CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW3_COL2_FIRST_SLOT_ID)
#define NUM_SLOTS_SCAN_LIQUID_ACTIVE_STATE      (LAST_SLOT_SCAN_IN_LIQUID_ACTIVE_STATE - FIRST_SLOT_SCAN_IN_LIQUID_ACTIVE_STATE + 1u)

/*****************************************************************************
* Finite state machine for different CAPSENSE operating states
*****************************************************************************/
typedef enum
{
    ACTIVE_STATE = 0x01u,                       /* State to scan sensors with 128 Hz refresh rate*/
    ACTIVE_LOW_REFRESH_RATE_STATE = 0x02u,      /* State to scan sensors with 32 Hz refresh rate */
    WAKE_ON_TOUCH_STATE = 0x03u,                /* State to scan Low Power widgets with 16 Hz refresh rate */
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
static void capsense_msc0_isr(void);

void inactive_sensors_reconfiguration(void * context);
void initialize_capsense_hardware_filter(CAPSENSE_STATE);
void hardware_filter_state_trasition(CAPSENSE_STATE);
uint32_t capsense_liquid_active_state_check();

#if ENABLE_EZI2C
static void initialize_capsense_tuner(void);
static void ezi2c_isr(void);
#endif

#if ENABLE_SERIAL_LED
void led_control(CAPSENSE_STATE);
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
*  - initialize CAPSENSE
*  - initialize inactive sensor states
*  - initialize tuner communication
*  - scan touch input continuously
*  - switch to different CAPSENSE applications(s) states
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
    uint32_t capsense_liquid_active_state_status;
    CAPSENSE_STATE capsense_state;

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

    /* Initialize MSC CAPSENSE */
    initialize_capsense();

    /* Initialization of CAPSENSE state, timeout counter and MSCLP timer configuration  */
    capsense_state = ACTIVE_STATE;
    timeout_counter = TIMER_RESET;
    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);
    capsense_liquid_active_state_status = LIQUID_STATE_INACTIVE;

    /* Initialize CIC1 and CIC2 filter for different CAPSENSE states */
    initialize_capsense_hardware_filter(capsense_state);

    /*******************************************************************************
    * Start the state-machine to capture different states of operation.
    * This is a state-machine implemented to capture 4 different states
    * of CAPSENSE block for this specific application.
    *******************************************************************************/
    for (;;)
    {
        switch(capsense_state)
        {
            /* CAPSENSE state: ACTIVE */
            case ACTIVE_STATE :
            {
                Cy_CapSense_ScanAllSlots(&cy_capsense_context);

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    Cy_SysPm_CpuEnterDeepSleep ();
                }

                Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

                /* Scan, process and check the status of the all Active mode sensors */
                if (Cy_CapSense_IsAnyWidgetActive(&cy_capsense_context))
                {
                    /* The function 'capsense_liquid_active_state_check' checks different sensor signals
                    * to identify presence of liquid on the sensors. If this returns True, the application
                    * changes from ACTIVE state to LIQUID ACTIVE state
                    */
                    capsense_liquid_active_state_status = capsense_liquid_active_state_check();

                    if(LIQUID_STATE_INACTIVE != capsense_liquid_active_state_status)
                    {
                        capsense_state = LIQUID_ACTIVE_STATE;
                        timeout_counter = TIMER_RESET;
                        Cy_CapSense_InitializeAllStatuses(&cy_capsense_context);

                        #if ENABLE_SERIAL_LED
                        /* Serial LED control for showing the CAPSENSE touch status (feedback) */
                        led_control(LIQUID_ACTIVE_STATE);
                        #endif
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
                /* Serial LED control for showing the CAPSENSE touch status (feedback) */
                led_control(ACTIVE_STATE);
                #endif

                break;
            }
            /* end of ACTIVE_STATE */

            /* CAPSENSE state: Active Low Refresh Rate state */
            case ACTIVE_LOW_REFRESH_RATE_STATE :
            {
                Cy_CapSense_ScanAllSlots(&cy_capsense_context);

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    Cy_SysPm_CpuEnterDeepSleep ();
                }

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
                    * state to WAKE_ON_TOUCH_STATE. Change the hardware filter
                    * from CIC2 to CIC1 for WAKE_ON_TOUCH_STATE mode scan */
                    if (ACTIVE_LOW_REFRESH_TIMEOUT < timeout_counter)
                    {
                        capsense_state = WAKE_ON_TOUCH_STATE;
                        hardware_filter_state_trasition(capsense_state);
                    }
                }

                break;
            }
            /* end of ACTIVE_LOW_REFRESH_RATE */

            /* CAPSENSE state: Wake On Touch  */
            case WAKE_ON_TOUCH_STATE :
            {
                Cy_CapSense_ScanAllLpSlots(&cy_capsense_context);

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    Cy_SysPm_CpuEnterDeepSleep();
                }

                /* if there is any touch event, change the state to
                *  ACTIVE_STATE. Change the hardware filter
                *  from CIC1 to CIC2 for scanning in ACTIVE_STATE */
                if (Cy_CapSense_IsAnyLpWidgetActive(&cy_capsense_context))
                {
                    capsense_state = ACTIVE_STATE;
                    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);
                }

                /* if there is no touch and the timeout happens, change the
                *  state to ACTIVE_LOW_REFRESH_RATE. Change the hardware filter
                *  from CIC1 to CIC2 for scanning in WAKE_ON_TOUCH_STATE */
                else
                {
                    capsense_state = ACTIVE_LOW_REFRESH_RATE_STATE;
                    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_LOW_REFRESH_TIMER, &cy_capsense_context);
                }

                timeout_counter = TIMER_RESET;
                hardware_filter_state_trasition(capsense_state);

                break;
            }
            /* end of "WAKE_ON_TOUCH_STATE" */

            /* CAPSENSE state: LIQUID_ACTIVE_STATE
             * This is custom state for scanning guard sensor(s) only when large volume of
             * liquid present on the sensors
             *  */

            case LIQUID_ACTIVE_STATE :
            {
                Cy_CapSense_ScanSlots(FIRST_SLOT_SCAN_IN_LIQUID_ACTIVE_STATE, NUM_SLOTS_SCAN_LIQUID_ACTIVE_STATE, &cy_capsense_context);

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    Cy_SysPm_CpuEnterDeepSleep ();
                }

                /* Process only the guard sensors widgets to detect the presence of liquid */
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_GUARD_LOOP_SENSOR_WDGT_ID, &cy_capsense_context);
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_GUARD_MUTUAL_CAP_SENSOR_WDGT_ID, &cy_capsense_context);
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW0_COL0_WDGT_ID, &cy_capsense_context);
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW3_COL0_WDGT_ID, &cy_capsense_context);
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW0_COL2_WDGT_ID, &cy_capsense_context);
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW3_COL2_WDGT_ID, &cy_capsense_context);
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW0_COL4_WDGT_ID, &cy_capsense_context);
                Cy_CapSense_ProcessWidget(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW3_COL4_WDGT_ID, &cy_capsense_context);

                /* Check the liquid active condition is still valid or not. */
                capsense_liquid_active_state_status = capsense_liquid_active_state_check();

                if(LIQUID_STATE_INACTIVE != capsense_liquid_active_state_status)
                {
                    capsense_state = LIQUID_ACTIVE_STATE;
                    timeout_counter = TIMER_RESET;
                }

                else
                {
                    timeout_counter++;

                    if (LIQUID_ACTIVE_MODE_TIMEOUT < timeout_counter)
                    {
                        capsense_state = ACTIVE_STATE;
                        timeout_counter = TIMER_RESET;
                        capsense_liquid_active_state_status = LIQUID_STATE_INACTIVE;
                    }
                }

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

        #if ENABLE_EZI2C
        /* Establishes synchronized communication with the CAPSENSE Tuner tool */
        if (CY_CAPSENSE_STATUS_RESTART_DONE == Cy_CapSense_RunTuner(&cy_capsense_context))
        {
            initialize_capsense_hardware_filter(capsense_state);
        }
        #endif
    }
    /* end of for loop */
}
/* end of main */


/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CAPSENSE and configures the CAPSENSE
*  interrupt.
*
*******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CAPSENSE interrupt configuration MSCLP 0 */
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
        /* Initialize CAPSENSE interrupt for MSC 0 */
        Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
        NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

        /* Setting self-cap sensors raw-count calibration percentage to 70% */
        Cy_CapSense_SetCalibrationTarget(CUSTOM_CSD_CALIBRATION_LEVEL, CY_CAPSENSE_CSD_GROUP, &cy_capsense_context);

        /* Initialize the CAPSENSE firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if (status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CAPSENSE sensors are tuned
         * as per procedure give in the Readme.md file */
    }
}


/*******************************************************************************
* Function Name: capsense_msc0_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CAPSENSE MSC0 block.
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
*  Parameters:
*  The pointer to the CAPSENSE&trade; context structure
*  \ref cy_stc_capsense_context_t.
*
* Return:
* void
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
                [CY_CAPSENSE_GUARD_LOOP_SENSOR_WDGT_ID].ptrEltdConfig[0u],
                CY_CAPSENSE_PIN_STATE_IDX_GND, &cy_capsense_context);
    }
}


#if ENABLE_EZI2C
/*******************************************************************************
* Function Name: initialize_capsense_tuner
********************************************************************************
* Summary:
* EZI2C module to communicate with the CAPSENSE Tuner tool.
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

    /* Set the CAPSENSE data structure as the I2C buffer to be exposed to the
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


/*******************************************************************************
* Function Name: initialize_capsense_hardware_filter
********************************************************************************
* Summary:
* This function sets the hardware filter configuration to CIC1 in Wake-On-Touch
* state, sets maximum raw-count and calibrate the low-power slots.
*  For other widgets, this sets the hardware configuration to CIC2
*
** Parameters:
*  CAPSENSE_STATE
*
* Return:
*  void
*******************************************************************************/
void initialize_capsense_hardware_filter(CAPSENSE_STATE capsense_state)
{
    /* Configure the CIC1 filter for Wake-On-Touch state*/
    hardware_filter_state_trasition(WAKE_ON_TOUCH_STATE);

    /* Sets maximum raw counts */
    CY_CAPSENSE_WAKE_ON_TOUCH_MAX_RAW_COUNT_VALUE =
            (CY_CAPSENSE_WAKE_ON_TOUCH_SNS_CLK_VALUE * CY_CAPSENSE_WAKE_ON_TOUCH_NUM_SUBCONVERSIONS_VALUE);

    /* Calibrate all the low power widgets */
    Cy_CapSense_CalibrateAllLpSlots(&cy_capsense_context);

    /* Sets CIC2 mode for active widgets only */
    if (WAKE_ON_TOUCH_STATE != capsense_state)
    {
        hardware_filter_state_trasition(ACTIVE_STATE);
    }
}


/*******************************************************************************
* Function Name: hardware_filter_state_trasition
*******************************************************************************
* Summary:
* This function changes the filter (from CIC2 to CIC1) when CAPSENSE state changes
* from any other state to wake-on-touch state,
* If the state changes from wake-on-touch state to any other state,
* hardware filter changes from CIC1 to CIC2.
*
* Parameters:
* CAPSENSE_STATE
*
* Return:
*  void
*******************************************************************************/
void hardware_filter_state_trasition(CAPSENSE_STATE capsense_state)
{
    MSCLP_Type * msclpBase;

    msclpBase = cy_capsense_context.ptrCommonConfig->ptrChConfig->ptrHwBase;

    if(WAKE_ON_TOUCH_STATE == capsense_state)
    {
        msclpBase->FILTER_CTL &= ~MSCLP_FILTER_CTL_FILTER_MODE_Msk;
    }

    else
    {
        msclpBase->FILTER_CTL |= MSCLP_FILTER_CTL_FILTER_MODE_Msk;
    }
}


/*******************************************************************************
* Function Name: capsense_liquid_active_state_check
********************************************************************************
* Summary:
*  This function implements different scenarios of liquid presence on sensors
*
*  (1) CY_CAPSENSE_GUARD_LOOP_SENSOR_WDGT_ID : Scanning of this sensor happens in
*  self-cap mode when all the neighboring electrodes are driven as shield. This
*  widget turns on when the kit is dipped inside a container of grounded liquid.
*
*  (2) CY_GUARD_MUTUAL_CAP_SENSOR_WDGT_ID : Scanning of this sensor happens in
*  mutual-cap mode when all the touchpad electrodes are ganged to make an Rx electrode
*  of the mutual-cap widget. The loop sensor is configured as Tx electrode.
*  When a floating liquid appears between the Tx and Rx electrodes, raw-count drops
*  from the baseline. The high 'low-baseline-reset' parameter of the sensor doesn't
*  allow the baseline to follow the raw-count. When the difference difference of
*  raw-count and the baseline drops below a negative threshold, indicates liquid presence.
*
*  (3) CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW0_COL0_WDGT_ID and
*  CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW0_COL4_WDGT_ID :
*  If both these two mutual cap sensors are active, will indicate the flow of thin liquid
*  stream through the left edge of the touchpad sensor.
*  Both these sensors are active when the flowing liquid is grounded.
*
*  (4 CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW0_COL2_WDGT_ID and
*  CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW3_COL2_WDGT_ID :
*  If both these two mutual cap sensors are active, will indicate the flow of thin liquid
*  stream through the middle of the touchpad sensor.
*  Both these sensors are active when the flowing liquid is grounded.
*
*  (5) CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW0_COL4_WDGT_ID and
*  CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW3_COL4_WDGT_ID :
*  If both these two mutual cap sensors are active, will indicate the flow of thin liquid
*  stream through the right edge of the touchpad sensor.
*  Both these sensors are active when the flowing liquid is grounded.
*
*  Parameters:
*   None
*
*  Return:
*   Status of one of the guard sensors whether active. Returns one of the states below:
*
*   '0' : LIQUID_STATE_INACTIVE : Not liquid detected on any of the sensor
*
*   '1' : GUARD_SENSOR_SELF_CAP_ACTIVE : The loop-sensor (self-cap) is active because of
*                                    large volume of grounded liquid
*
*   '2' : GUARD_SENSOR_MUTUAL_CAP_GANGED_ACTIVE : Floating liquid appearing between the
*                                            touchpad and proximity loop/hatch-area
*
*   '3' : GUARD_SENSOR_MUTUAL_CAP_LEFT_ACTIVE :  Thin stream of liquid flow
*                                            through the left edge of the touchpad
*
*   '4' : GUARD_SENSOR_MUTUAL_CAP_MID_ACTIVE :  Thin stream of liquid flow
*                                            through the middle of the touchpad
*
*   '5' : GUARD_SENSOR_MUTUAL_CAP_RIGHT_ACTIVE :  Thin stream of liquid flow
*                                            through the right edge of the touchpad
*
*******************************************************************************/
uint32_t capsense_liquid_active_state_check()
{
    uint32_t effective_liquid_active_status = LIQUID_STATE_INACTIVE;
    int32_t raw_data_guard_mutual_cap_sensor = 0;
    int32_t bsln_guard_mutual_cap_sensor = 0;
    int32_t negative_diff_count_guard_mutual_cap_sensor = 0;
    int32_t guard_mutual_cap_liquid_threshold = 100;

    /* Compute the drop of raw-count from the baseline (raw-count change in the negative direction)
     * when water flow happens on the sensors. */
    raw_data_guard_mutual_cap_sensor = (int32_t)(cy_capsense_tuner.sensorContext[SENSOR_CONTEXT_GUARD_MUTUAL_CAP_SENSOR].raw);
    bsln_guard_mutual_cap_sensor = (int32_t)(cy_capsense_tuner.sensorContext[SENSOR_CONTEXT_GUARD_MUTUAL_CAP_SENSOR].bsln);
    negative_diff_count_guard_mutual_cap_sensor = (bsln_guard_mutual_cap_sensor - raw_data_guard_mutual_cap_sensor);

    /* condition - 1:
     * check the status of the Self-cap guard sensor (loop sensor). This signal is high when there is large volume of
     * grounded liquid present on this sensor.
     * */
    if (SENSOR_ACTIVE == Cy_CapSense_IsWidgetActive(CY_CAPSENSE_GUARD_LOOP_SENSOR_WDGT_ID, &cy_capsense_context))
        effective_liquid_active_status = GUARD_SENSOR_SELF_CAP_ACTIVE;

    /* condition - 2:
     * check the signal level of the Mutual cap guard sensor. For ungrounded liquid flow over the sensor,
     * signal count is negative. The 'low baseline reset' parameter of the sensor is set very high. Hence the
     * baseline doesn't drop when the raw-count shifts towards the negative direction.
     * */
    else if (negative_diff_count_guard_mutual_cap_sensor >= guard_mutual_cap_liquid_threshold)
        effective_liquid_active_status = GUARD_SENSOR_MUTUAL_CAP_GANGED_ACTIVE;

    /* condition-3:
     * check the signal levels of mutual-cap button COL4-ROW0 and COL4-ROW3.
     * If diff-count of both the sensors are higher than a threshold, then it is a liquid flow
     */
    else if ((SENSOR_ACTIVE == Cy_CapSense_IsWidgetActive(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW0_COL4_WDGT_ID, &cy_capsense_context)) &&
        (SENSOR_ACTIVE == Cy_CapSense_IsWidgetActive(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW3_COL4_WDGT_ID, &cy_capsense_context)))
        effective_liquid_active_status = GUARD_SENSOR_MUTUAL_CAP_LEFT_ACTIVE;

    /* condition-4:
     * check the signal levels of mutual-cap button COL2-ROW0 and COL2-ROW3.
     * If diff-count of both the sensors are higher than a threshold, then it is a liquid flow
     */
    else if ((SENSOR_ACTIVE == Cy_CapSense_IsWidgetActive(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW0_COL2_WDGT_ID, &cy_capsense_context)) &&
        (SENSOR_ACTIVE == Cy_CapSense_IsWidgetActive(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW3_COL2_WDGT_ID, &cy_capsense_context)))
        effective_liquid_active_status = GUARD_SENSOR_MUTUAL_CAP_MID_ACTIVE;

    /* condition-5:
     * check the signal levels of mutual-cap button COL0-ROW0 and COL0-ROW3.
     * If diff-count of both the sensors are higher than a threshold, then it is a liquid flow
     */
    else if ((SENSOR_ACTIVE == Cy_CapSense_IsWidgetActive(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW0_COL0_WDGT_ID, &cy_capsense_context)) &&
        (SENSOR_ACTIVE == Cy_CapSense_IsWidgetActive(CY_CAPSENSE_GUARD_MUTUAL_CAP_ROW3_COL0_WDGT_ID, &cy_capsense_context)))
        effective_liquid_active_status = GUARD_SENSOR_MUTUAL_CAP_RIGHT_ACTIVE;

    /* default not GUARD sensor is active */
    else
        effective_liquid_active_status = LIQUID_STATE_INACTIVE;

    return effective_liquid_active_status;
}


#if ENABLE_SERIAL_LED
/*******************************************************************************
* Function Name: led_control
*******************************************************************************
* Summary:
* Logic to control the on / off status, color and brightness of the LEDs
* based on the touch status of the CAPSENSE widgets.
*
* Parameters:
* structure of different CAPSENSE application states
*
* Return:
*  void
*******************************************************************************/
void led_control(CAPSENSE_STATE capsense_state)
{
    /* Brightness of each LED is represented by 0 to 255,
    * where 0 indicates LED in OFF state and 255 indicate maximum
    * brightness of an LED
    */
    uint8_t brightness_max = 255u;
    uint8_t brightness_min = 0u;
    uint8_t touchposition_x, touchposition_y ;

    cy_stc_capsense_touch_t *panelTouch =
            Cy_CapSense_GetTouchInfo(CY_CAPSENSE_TOUCHPAD_WDGT_ID, &cy_capsense_context);

    touchposition_x = panelTouch->ptrPosition->x;
    touchposition_y = brightness_max - panelTouch->ptrPosition->y;

    /*LED1 and LED3 indicate the status of the 'CAPSENSE button' and 'Touchpad' */

    switch(capsense_state)
    {
        case ACTIVE_STATE :
        {
            /*******************************************************************************
            * If CAPSENSE button is active, Turn On LED1 with color Orange (Fixed intensity)
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

            break;

         } /* end of LED logic of 'case ACTIVE_STATE' */

        case LIQUID_ACTIVE_STATE :
        {
            /*******************************************************************************
            * All LEDs are OFF
            *******************************************************************************/
            led_context.led_num[LED2].color_red = brightness_min;
            led_context.led_num[LED2].color_green = brightness_min;
            led_context.led_num[LED2].color_blue = brightness_min;

            led_context.led_num[LED1].color_red = brightness_min;
            led_context.led_num[LED1].color_green = brightness_min;
            led_context.led_num[LED1].color_blue = brightness_min;

            led_context.led_num[LED3].color_red = brightness_min;
            led_context.led_num[LED3].color_green = brightness_min;
            led_context.led_num[LED3].color_blue = brightness_min;

            break;
        } /* end of LED logic of 'case LIQUID ACTIVE_STATE' */

        default:
        {
            led_context.led_num[LED2].color_red = brightness_min;
            led_context.led_num[LED2].color_green = brightness_min;
            led_context.led_num[LED2].color_blue = brightness_min;

            led_context.led_num[LED1].color_red = brightness_min;
            led_context.led_num[LED1].color_green = brightness_min;
            led_context.led_num[LED1].color_blue = brightness_min;

            led_context.led_num[LED3].color_red = brightness_min;
            led_context.led_num[LED3].color_green = brightness_min;
            led_context.led_num[LED3].color_blue = brightness_min;

            break;

        } /* end of 'case default' */

    } /* end of switch */

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

            #if ENABLE_EZI2C
            /* EzI2C pins drive mode to Analog HighZ */
            Cy_GPIO_SetDrivemode(CYBSP_I2C_SDA_PORT, CYBSP_I2C_SDA_PIN, CY_GPIO_DM_ANALOG);
            Cy_GPIO_SetDrivemode(CYBSP_I2C_SCL_PORT, CYBSP_I2C_SCL_PIN, CY_GPIO_DM_ANALOG);
            #endif

            #if ENABLE_SERIAL_LED
            /* SPI pins drive mode to Analog HighZ */
            Cy_GPIO_SetDrivemode(CYBSP_SPI_MOSI_PORT,CYBSP_SPI_MOSI_PIN,CY_GPIO_DM_ANALOG);
            #endif

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_AFTER_TRANSITION:

            #if ENABLE_EZI2C
            /* EzI2C pins drive mode to Analog HighZ */
            Cy_GPIO_SetDrivemode(CYBSP_I2C_SDA_PORT, CYBSP_I2C_SDA_PIN, CY_GPIO_DM_OD_DRIVESLOW);
            Cy_GPIO_SetDrivemode(CYBSP_I2C_SCL_PORT, CYBSP_I2C_SCL_PIN, CY_GPIO_DM_OD_DRIVESLOW);
            #endif

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

/* [] END OF FILE */
