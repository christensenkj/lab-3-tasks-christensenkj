/***************************************************************************//**
 * @file
 * @brief
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.
 * The software is governed by the sections of the MSLA applicable to Micrium
 * Software.
 *
 ******************************************************************************/

/*
*********************************************************************************************************
*
*                                             EXAMPLE MAIN
*
* File : ex_main.c
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*********************************************************************************************************
*/

#include  <bsp_os.h>
#include  "bsp.h"

#include  <cpu/include/cpu.h>
#include  <common/include/common.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>

#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_core.h"

#include "app.h"
/*
*********************************************************************************************************
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*********************************************************************************************************
*/

#define  EX_MAIN_START_TASK_PRIO              	21u
#define  EX_MAIN_START_TASK_STK_SIZE         	512u
#define  EX_MAIN_IDLE_TASK_PRIO              	21u
#define  EX_MAIN_IDLE_TASK_STK_SIZE			 	512u
#define  EX_MAIN_BUTTON_INPUT_TASK_PRIO         21u
#define  EX_MAIN_BUTTON_INPUT_TASK_STK_SIZE		512u
#define  EX_MAIN_SLIDER_INPUT_TASK_PRIO         21u
#define  EX_MAIN_SLIDER_INPUT_TASK_STK_SIZE		512u
#define  EX_MAIN_LED_OUTPUT_TASK_PRIO           21u
#define  EX_MAIN_LED_OUTPUT_TASK_STK_SIZE		512u

/*
*********************************************************************************************************
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*********************************************************************************************************
*/

/* Start Task Stack.                                    */
static  CPU_STK  Ex_MainStartTaskStk[EX_MAIN_START_TASK_STK_SIZE];
/* Start Task TCB.                                      */
static  OS_TCB   Ex_MainStartTaskTCB;

/* Idle Task Stack.                                    */
static  CPU_STK  Ex_MainIdleTaskStk[EX_MAIN_IDLE_TASK_STK_SIZE];
/* Idle Task TCB.                                      */
static  OS_TCB   Ex_MainIdleTaskTCB;

/* ButtonInput Task Stack.                                    */
static  CPU_STK  Ex_MainButtonInputTaskStk[EX_MAIN_BUTTON_INPUT_TASK_STK_SIZE];
/* ButtonInput Task TCB.                                      */
static  OS_TCB   Ex_MainButtonInputTaskTCB;

/* SliderInput Task Stack.                                    */
static  CPU_STK  Ex_MainSliderInputTaskStk[EX_MAIN_SLIDER_INPUT_TASK_STK_SIZE];
/* SliderInput Task TCB.                                      */
static  OS_TCB   Ex_MainSliderInputTaskTCB;

/* LedOutput Task Stack.                                    */
static  CPU_STK  Ex_MainLedOutputTaskStk[EX_MAIN_LED_OUTPUT_TASK_STK_SIZE];
/* LedOutput Task TCB.                                      */
static  OS_TCB   Ex_MainLedOutputTaskTCB;


/*
*********************************************************************************************************
*********************************************************************************************************
*                                       LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*********************************************************************************************************
*/

static  void  Ex_MainStartTask (void  *p_arg);
static  void  Ex_MainIdleTask (void  *p_arg);
static  void  Ex_MainButtonInputTask (void  *p_arg);
static  void  Ex_MainSliderInputTask (void  *p_arg);
static  void  Ex_MainLedOutputTask (void  *p_arg);
void led_drive(void);

/*
*********************************************************************************************************
*********************************************************************************************************
*                                       GLOBAL VARIABLES
*********************************************************************************************************
*********************************************************************************************************
*/
// Define pushbutton status global variables
bool PB0_status = false;
bool PB1_status = false;
// Define slider direction global variable and init to INACTIVE
uint8_t slider_pos = INACTIVE;

/*
*********************************************************************************************************
*********************************************************************************************************
*                                          GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C applications. It is assumed that your code will
*               call main() once you have performed all necessary initialization.
*
* Argument(s) : None.
*
* Return(s)   : None.
*
* Note(s)     : None.
*********************************************************************************************************
*/

int  main (void)
{
    RTOS_ERR  err;


    BSP_SystemInit();                                           /* Initialize System.                                   */
    CPU_Init();                                                 /* Initialize CPU.                                      */

    OS_TRACE_INIT();
    OSInit(&err);                                               /* Initialize the Kernel.                               */
                                                                /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    OSTaskCreate(&Ex_MainStartTaskTCB,                          /* Create the Start Task.                               */
                 "Ex Main Start Task",
                  Ex_MainStartTask,
                  DEF_NULL,
                  EX_MAIN_START_TASK_PRIO,
                 &Ex_MainStartTaskStk[0],
                 (EX_MAIN_START_TASK_STK_SIZE / 10u),
                  EX_MAIN_START_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);
    /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

    /* Start the kernel.                                    */
    OSStart(&err);
    /*   Check error code.                                  */
    APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);


    return (1);
}


/*
*********************************************************************************************************
*********************************************************************************************************
*                                           LOCAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                          Ex_MainStartTask()
*
* Description : This is the task that will be called by the Startup when all services are initializes
*               successfully.
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static  void  Ex_MainStartTask (void  *p_arg)
{
    RTOS_ERR  err;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

    Common_Init(&err);                                          /* Call common module initialization example.           */
    APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);

    BSP_OS_Init();                                              /* Initialize the BSP. It is expected that the BSP ...  */
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */
    // initialize common modules for all tasks
    cmu_open();

    OSTaskCreate(&Ex_MainIdleTaskTCB,                          /* Create the Idle Task.                               */
                 "Ex Main Idle Task",
                  Ex_MainIdleTask,
                  DEF_NULL,
                  EX_MAIN_IDLE_TASK_PRIO,
                 &Ex_MainIdleTaskStk[0],
                 (EX_MAIN_IDLE_TASK_STK_SIZE / 10u),
                  EX_MAIN_IDLE_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

    OSTaskCreate(&Ex_MainButtonInputTaskTCB,                          /* Create the Button Input Task.                               */
                 "Ex Main Button Input Task",
                  Ex_MainButtonInputTask,
                  DEF_NULL,
                  EX_MAIN_BUTTON_INPUT_TASK_PRIO,
                 &Ex_MainButtonInputTaskStk[0],
                 (EX_MAIN_BUTTON_INPUT_TASK_STK_SIZE / 10u),
                  EX_MAIN_BUTTON_INPUT_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

    OSTaskCreate(&Ex_MainSliderInputTaskTCB,                          /* Create the Slider Input Task.                               */
                 "Ex Main SliderInput Task",
                  Ex_MainSliderInputTask,
                  DEF_NULL,
                  EX_MAIN_SLIDER_INPUT_TASK_PRIO,
                 &Ex_MainSliderInputTaskStk[0],
                 (EX_MAIN_SLIDER_INPUT_TASK_STK_SIZE / 10u),
                  EX_MAIN_SLIDER_INPUT_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

    OSTaskCreate(&Ex_MainLedOutputTaskTCB,                          /* Create the Led Output Task.                               */
                 "Ex Main Led Output Task",
                  Ex_MainLedOutputTask,
                  DEF_NULL,
                  EX_MAIN_LED_OUTPUT_TASK_PRIO,
                 &Ex_MainLedOutputTaskStk[0],
                 (EX_MAIN_LED_OUTPUT_TASK_STK_SIZE / 10u),
                  EX_MAIN_LED_OUTPUT_TASK_STK_SIZE,
                  0u,
                  0u,
                  DEF_NULL,
                 (OS_OPT_TASK_STK_CLR),
                 &err);

    while (DEF_ON) {
                                                                /* Delay Start Task execution for                       */
        OSTimeDly( 10,                                        /*   1000 OS Ticks                                      */
                   OS_OPT_TIME_DLY,                             /*   from now.                                          */
                  &err);
                                                                /*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/*
*********************************************************************************************************
*                                          Ex_MainIdleTask()
*
* Description : This is the task that will be called by the Startup when all services are initializes
*               successfully.
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static  void  Ex_MainIdleTask (void  *p_arg)
{
    RTOS_ERR  err;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

//    Common_Init(&err);                                          /* Call common module initialization example.           */
//    APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);

//    BSP_OS_Init();                                              /* Initialize the BSP. It is expected that the BSP ...  */
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */

    while (DEF_ON) {
    	EMU_EnterEM1();
        /* Delay Start Task execution for                       */
		OSTimeDly( 100,                                        /*   1000 OS Ticks                                      */
					OS_OPT_TIME_DLY,                             /*   from now.                                          */
					&err);
                                                                /*   Check error code.                                  */
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/*
*********************************************************************************************************
*                                          Ex_MainButtonInputTask()
*
* Description : This is the task that will be called by the Startup when all services are initializes
*               successfully.
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static  void  Ex_MainButtonInputTask (void  *p_arg)
{
    RTOS_ERR  err;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

//    Common_Init(&err);                                          /* Call common module initialization example.           */
//    APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);

//    BSP_OS_Init();                                              /* Initialize the BSP. It is expected that the BSP ...  */
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */
    buttons_setup();
    while (DEF_ON) {
    	poll_PB0(&PB0_status);
    	poll_PB1(&PB1_status);
                                                                /*   Check error code.                                  */
        /* Delay Start Task execution for                       */
		OSTimeDly( 100,                                        /*   1000 OS Ticks                                      */
					OS_OPT_TIME_DLY,                             /*   from now.                                          */
					&err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}

/*
*********************************************************************************************************
*                                          Ex_MainSliderInputTask()
*
* Description : This is the task that will be called by the Startup when all services are initializes
*               successfully.
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static  void  Ex_MainSliderInputTask (void  *p_arg)
{
    RTOS_ERR  err;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

//    Common_Init(&err);                                          /* Call common module initialization example.           */
//    APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);

//    BSP_OS_Init();                                              /* Initialize the BSP. It is expected that the BSP ...  */
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */
    slider_setup();
    while (DEF_ON) {
    	slider_position(&slider_pos);
                                                                /*   Check error code.                                  */
        /* Delay Start Task execution for                       */
		OSTimeDly( 100,                                        /*   1000 OS Ticks                                      */
					OS_OPT_TIME_DLY,                             /*   from now.                                          */
					&err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}


/*
*********************************************************************************************************
*                                          Ex_MainLedOutputTask()
*
* Description : This is the task that will be called by the Startup when all services are initializes
*               successfully.
*
* Argument(s) : p_arg   Argument passed from task creation. Unused, in this case.
*
* Return(s)   : None.
*
* Notes       : None.
*********************************************************************************************************
*/

static  void  Ex_MainLedOutputTask (void  *p_arg)
{
    RTOS_ERR  err;


    PP_UNUSED_PARAM(p_arg);                                     /* Prevent compiler warning.                            */

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();                                /* Initialize interrupts disabled measurement.          */
#endif

//    Common_Init(&err);                                          /* Call common module initialization example.           */
//    APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE, ;);

//    BSP_OS_Init();                                              /* Initialize the BSP. It is expected that the BSP ...  */
                                                                /* ... will register all the hardware controller to ... */
                                                                /* ... the platform manager at this moment.             */
    gpio_open();
    while (DEF_ON) {
    	led_drive();
                                                                /*   Check error code.                                  */
        /* Delay Start Task execution for                       */
		OSTimeDly( 100,                                        /*   1000 OS Ticks                                      */
					OS_OPT_TIME_DLY,                             /*   from now.                                          */
					&err);
        APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), ;);
    }
}


/***************************************************************************//**
 * @brief
 *   Drives the LEDs based on the pushbutton and slider values.
 *
 * @details
 * 	 Utilizes global variables slider_pos, PB0_status, and PB1_status.
 * 	 Pushbutton operate on an XOR basis, and slider operates similarly.
 * 	 Pushbuttons and slider operate on OR basis.
 *
 * @note
 *   This function is called every time the LEDs should be set.
 *
 ******************************************************************************/
void led_drive() {
	// First determine the states of the pushbuttons combined as XOR
	bool button_activity = false;	// init button_activity to false
	if (PB0_status ^ PB1_status)
		button_activity = true;		// if the buttons are not both pressed, then set to true

	// When buttons are XOR false and the slider is either pressed on left and right or unpressed, turn off LEDs
	if (!button_activity && (slider_pos == INACTIVE)) {
		GPIO_PinOutClear(LED0_port, LED0_pin);
		GPIO_PinOutClear(LED1_port, LED1_pin);
	}
	// When buttons are XOR true and slider is LEFT
	else if (button_activity && (slider_pos == LEFT)) {
		// If pushbutton (PB0) pressed, turn on only LED0
		if (PB0_status) {
			GPIO_PinOutSet(LED0_port, LED0_pin);
			GPIO_PinOutClear(LED1_port, LED1_pin);
		}
		// if PB1 pressed, turn on both LED0 and LED1
		else {
			GPIO_PinOutSet(LED0_port, LED0_pin);
			GPIO_PinOutSet(LED1_port, LED1_pin);
		}
	}
	// If buttons are XOR true and slider is RIGHT
	else if (button_activity && (slider_pos == RIGHT)) {
		// If PB0 pressed, turn on both LEDs
		if (PB0_status) {
			GPIO_PinOutSet(LED0_port, LED0_pin);
			GPIO_PinOutSet(LED1_port, LED1_pin);
		}
		// If PB1 is pressed, only turn on LED1
		else {
			GPIO_PinOutSet(LED1_port, LED1_pin);
			GPIO_PinOutClear(LED0_port, LED0_pin);
		}
	}
	// If slider is INACTIVE (because either both sides are pressed or it's not pressed)
	// and buttons are XOR true
	else if (button_activity && (slider_pos == INACTIVE)) {
		// If PB0 is pressed, turn on only LED0
		if (PB0_status) {
			GPIO_PinOutSet(LED0_port, LED0_pin);
			GPIO_PinOutClear(LED1_port, LED1_pin);
		}
		// If PB1 is pressed, turn on LED1
		else {
			GPIO_PinOutSet(LED1_port, LED1_pin);
			GPIO_PinOutClear(LED0_port, LED0_pin);
		}
	}
	// If slider is not inactive and buttons are XOR false
	else if (!button_activity && !(slider_pos == INACTIVE)) {
		// If slider is left, turn on LED0
		if (slider_pos == LEFT) {
			GPIO_PinOutSet(LED0_port, LED0_pin);
			GPIO_PinOutClear(LED1_port, LED1_pin);
		}
		// If slider is right, turn on LED1
		else {
			GPIO_PinOutSet(LED1_port, LED1_pin);
			GPIO_PinOutClear(LED0_port, LED0_pin);
		}
	}
}


/***************************************************************************//**
 * @brief
 *   Interrupt handler for GPIO Even pins module.
 *
 *
 * @details
 * 	 Utilizes global variable PB0_status to poll the value of pushbutton 0
 * 	 using poll_PB0() function.
 *
 * @note
 *   This function is called every time PB0 is pressed.
 *
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
	__disable_irq();
	/* Get and clear all pending GPIO interrupts */
	uint32_t interruptMask = GPIO_IntGet();
	GPIO_IntClear(interruptMask);
	poll_PB0(&PB0_status);
	__enable_irq();
}

/***************************************************************************//**
 * @brief
 *   Interrupt handler for GPIO Odd pins module.
 *
 *
 * @details
 * 	 Utilizes global variable PB1_status to poll the value of pushbutton 1
 * 	 using poll_PB1() function.
 *
 * @note
 *   This function is called every time PB1 is pressed.
 *
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
	__disable_irq();
	/* Get and clear all pending GPIO interrupts */
	uint32_t interruptMask = GPIO_IntGet();
	GPIO_IntClear(interruptMask);
	poll_PB1(&PB1_status);
	__enable_irq();
}
