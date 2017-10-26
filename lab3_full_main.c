/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
/*
 * NEW FUNCTIONS or STRUCTS USED IN THE LAB3 KEY:
CS_setDCOFrequency
eUSCI_UART_Config
EUSCIA0_IRQHandler
GPIO_setAsPeripheralModuleFunctionInputPin
Interrupt_enableSleepOnIsrExit
PMAP_configurePorts
printf  (from provided printf.h, NOT from stdio.h)
Timer_A_clearCaptureCompareInterrupt
Timer_A_clearInterruptFlag
Timer_A_CompareModeConfig
Timer_A_configureUpMode
Timer_A_getCaptureCompareEnabledInterruptStatus
Timer_A_getEnabledInterruptStatus
Timer_A_initCompare
Timer_A_setCompareValue
Timer_A_startCounter
Timer_A_UpModeConfig
UART_clearInterruptFlag
UART_enableInterrupt
UART_enableModule
UART_getEnabledInterruptStatus
UART_initModule
UART_receiveData
Many functions from Lab2 are also used, of course.
 */

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "printf.h"

// TODO: Lab3: create UART, Timer_A, and CCR config structures ~~~~~~~~~~
// UART config
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                     // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};
volatile uint8_t readdata;

// Timer_A for driving RGB LED via PWM

Timer_A_UpModeConfig pwm_Config =
{
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_1,
     0xFE,
     TIMER_A_TAIE_INTERRUPT_DISABLE,
     TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
     TIMER_A_DO_CLEAR
};

// All 3 CCR configs for the 3 pins of the RGB LED

// red:

Timer_A_CompareModeConfig ccr1Config =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_1,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,
    0
};

// green:

Timer_A_CompareModeConfig ccr2Config =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_2,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,
    0
};

// blue:

Timer_A_CompareModeConfig ccr3Config =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_3,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,
    0
};

/* Lab3 general config */
#define MY_DCO_FREQUENCY_12 12000000
volatile uint32_t debounced = 0;

/* Lab3 global states */
/* parsing state */
/*  0: no light
 *  1: r
 *  2: g
 *  3: b        */
volatile int state = 0;
volatile int num_cnt = 0;
volatile char str[4];
/* watch timer */
volatile bool running = true;    //todo init state?
volatile unsigned int curr_t = 0;

void start_debounce_timer()
{
    MAP_Timer32_setCount(TIMER32_1_BASE, (MY_DCO_FREQUENCY_12)/200);   // 20ms
    MAP_Timer32_enableInterrupt(TIMER32_1_BASE);
    MAP_Timer32_startTimer(TIMER32_1_BASE, true);
    //MAP_Interrupt_enableInterrupt(TIMER32_1_INTERRUPT); // todo: testing
}

void start_watch_timer()
{
    running = true;
    MAP_Timer32_startTimer(TIMER32_0_BASE, false); // (xxx, oneshot)
}

void stop_watch_timer()
{
    running = false;
    MAP_Timer32_haltTimer(TIMER32_0_BASE);
}

extern void T32_INT0_IRQHandler(){
//    TODO: Lab2: Change LED on/off status and prepare any necessary interrupts to fire again
//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CODE FOR LAB2 LED cycling here
    MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);    //debug 1s
    curr_t++;
//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END CODE FOR LAB 2 led cycling
}


// PROVIDED: Timer_A struct for our debounce timer, triggering its interrupt when it hits its period value in CCR0
Timer_A_UpModeConfig debounce_Config =
{
     TIMER_A_CLOCKSOURCE_SMCLK, // Usually DCO clock, which in this case we set to 12MHz in main()
     TIMER_A_CLOCKSOURCE_DIVIDER_1,
     12*10^4, //10ms Debounce Delay
     TIMER_A_TAIE_INTERRUPT_ENABLE, // Should Timer_A send interrupts to Processor *at all*
     TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Should Timer_A reaching its period value (stored in CCR0) trigger an interrupt?
     TIMER_A_DO_CLEAR
};

// END CONFIG STRUCTS  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// PROVIDED: Port mapping to drive pins 2.0, 2.1, 2.0 (RGB LED) by Timer_A0's CCR1, CCR2, CCR3 respectively.
const uint8_t portMapping[] =
{
// Will be used for Port 2; In order of pin number, 0-indexed
        PMAP_TA0CCR1A,  PMAP_TA0CCR2A,  PMAP_TA0CCR3A,
        PMAP_NONE,      PMAP_NONE,      PMAP_NONE,
        PMAP_NONE,      PMAP_NONE
};


int main(void)
{
    /* Halting WDT  */
    WDT_A_holdTimer();

    /* Lab3: Setting DCO to 12MHz */
    //CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    CS_setDCOFrequency(MY_DCO_FREQUENCY_12); // Alternative which is better:

    /* prelab3 */
    //MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_2);
    /* ??? */
    //MAP_PCM_setPowerState(PCM_AM_LF_VCORE0);

//      TODO: LAB3: UART module needs to be configured to both transmit and receive. How it reacts to specific commands is elsewhere
//        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ENTER CODE FOR LAB3 UART init HERE:
    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
              GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    //![Simple UART Example]
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();
    //![Simple UART Example]

    // debug P1.0
    // Use the LED to indicate if we got an "L" through the terminal
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    // input 1.4
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);

    //rgb output
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 + GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);

//        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End CODE FOR LAB3 UART init

//      TODO: LAB3: Configure the Stopwatch (TIMER32_0) and button input (Port1) according to spec.
//           Configure Timer_A2 for debouncing, using its interrupt to prevent it from cycling forever.
//            (Struct for Timer A2 is provided, such that when it hits its CCR0 an interrupt is triggered
//          Start with the stopwatch paused, so it only starts actually counting in response to the relevant UART commands
//        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ENTER CODE FOR LAB3 stopwatch init HERE:
    // debouncing timer
    MAP_Timer32_initModule(TIMER32_1_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    // watch timer
    MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    MAP_Timer32_setCount(TIMER32_0_BASE, MY_DCO_FREQUENCY_12);
    MAP_Timer32_enableInterrupt(TIMER32_0_BASE);
    start_watch_timer();

    // buttoms (IMPORTANT P1.1 IS USED BY UART IN THIS LAB AS WELL!!!!!!!!!!!)
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);

    /* Enabling all lab2 interrupts */
    MAP_Interrupt_enableInterrupt(INT_PORT1);
    MAP_Interrupt_enableInterrupt(TIMER32_1_INTERRUPT);
    MAP_Interrupt_enableInterrupt(TIMER32_0_INTERRUPT);
//        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End CODE FOR LAB3 stopwatch init




//          TODO: LAB3, Part 2: Prepare RGB LED driven by PWM signals driven by TIMER_A0 with multiple CCRs. Note that because the processor
//            doesn't care about when specifically the timer A0 is cycling, no interrupts from it or its CCRs are needed.
//          We can initially drive them with a 100% duty cycle for testing; the UART commands can easily change the duty cycle on their own
//        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ENTER CODE FOR LAB3 RC_RGB init HERE:
    Timer_A_initCompare(TIMER_A0_BASE, &ccr1Config);
    Timer_A_initCompare(TIMER_A0_BASE, &ccr2Config);
    Timer_A_initCompare(TIMER_A0_BASE, &ccr3Config);
    Timer_A_configureUpMode(TIMER_A0_BASE, &pwm_Config);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

//        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End CODE FOR LAB3 RC_RGB init

//          PROVIDED: configure RGB pins (2.0, 2.1, 2.2) for outputting PWM function, mapped from Timer_A0's CCRs 1, 2, and 3:
            PMAP_configurePorts(portMapping, PMAP_P2MAP, 3, PMAP_DISABLE_RECONFIGURATION);  // mapping
            GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2,
                    GPIO_PRIMARY_MODULE_FUNCTION);

    /* Generic Interrupt enabling: */
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();

    while(1)
    {
        PCM_gotoLPM0InterruptSafe();
    }
}

// Helper function to clarify UART command behavior
// TODO: Lab3: Resumes, Pauses, or Restarts stopwatch depending on restart flag and the stopwatch state
//static void startStopStopwatch(bool restart){
//
//}
// END Stopwatch State Helper Function Definition ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// Helper function for sampling stopwatch. This avoids code duplication between UART and button sampling
// TODO: Lab3: Sample current stopwatch value, convert to number of elapsed milliseconds, and transmit to computer
//void stopwatchSample(){
//
//}
// END Sample Stopwatch code ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// PROVIDED: Converts a string of numeric characters into their corresponding whole number.
uint16_t atoi(volatile char* s, uint8_t l){
    uint16_t r = 0;
    uint8_t i;
    for (i = 0; i < l;i++){
        r = 10*r + (s[i]-'0');
    }
    return r;
}

///* EUSCI A0 UART ISR */
/*
extern void EUSCIA0_IRQHandler(void)
{
// TODO: Lab3: UART Command Reception logic (reacts according to the specific received character)


//    also tracks parsing state (most recently seen letter, and numeric characters in current number) to understand
//          the relevant multicharacter commands, as per spec

//    END LAB3 UART COMMAND LOGIC ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}
*/
bool is_number(uint8_t d)
{
    if (d>=48 && d<=57)
        return true;
    else
        return false;
}

void echo(char readdata)
{
    MAP_UART_transmitData(EUSCI_A0_BASE, readdata);
    MAP_UART_transmitData(EUSCI_A0_BASE,'.');
}

void show_time(void)
{
    printf(EUSCI_A0_BASE, "Current time =  %i running = %s\r\n",
           curr_t, running?"(O)":"(X)");
}

void LED(int state, volatile char* s, int cnt)
{
    /*  [state]
     *  0: no light (impossible)
     *  1: r
     *  2: g
     *  3: b
     *  num_str: "0\0" ~ "255\0"
     *  num_cnt = 1 ~ 3
     */
    if(cnt > 0) {
        //MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
        uint_fast16_t newTimerValue = atoi(s, cnt);
        uint_fast16_t invertedTimerValue = abs(255 - newTimerValue);
        uint_fast16_t newRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1 + (2*(state - 1));
        Timer_A_setCompareValue(TIMER_A0_BASE, newRegister, invertedTimerValue);
        printf(EUSCI_A0_BASE, "LED %i num %i\r\n",
               state, atoi(s, cnt));
    }
    else {
        printf(EUSCI_A0_BASE, "WRONG! SHOULDN'T BE HERE!!!\r\n\r\n\r\n");
    }

}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG) {
        readdata = MAP_UART_receiveData(EUSCI_A0_BASE);
        if (readdata == 'r') {
            if (num_cnt > 0 && state != 0 && *str) {
                LED(state, str, num_cnt);
                num_cnt = 0;
                *str = 0;
            }
            state = 1;
        } else if (readdata == 'g') {
            if (num_cnt > 0 && state != 0 && *str) {
                LED(state, str, num_cnt);
                num_cnt = 0;
                *str = 0;
            }
            state = 2;
        } else if (readdata == 'b') {
            if (num_cnt > 0 && state != 0 && *str) {
                LED(state, str, num_cnt);
                num_cnt = 0;
                *str = 0;
            }
            state = 3;
        } else if (readdata == 's') {
            if (running)
                stop_watch_timer();
            else
                start_watch_timer();
        } else if (readdata == '!') {
            stop_watch_timer();
            MAP_Timer32_setCount(TIMER32_0_BASE, MY_DCO_FREQUENCY_12);
            curr_t = 0;
        } else if (readdata == 'p') {
            show_time();
        } else if (is_number(readdata)) {
            if (state == 0) { /* no r,g,b command before */
                echo(readdata);
                goto out;
            }

            num_cnt++;
            /* save the char */
            *(str + num_cnt - 1) = readdata;    /* data */
            *(str + num_cnt) = 0;               /* maintaining end char manually */
            if (state > 0 && num_cnt >= 3 && *str ) { /* rxxx/gxxx/bxxx */
                LED(state, str, num_cnt);
                state = 0;
                num_cnt = 0;
                *str = 0;   /* clear string */
            }
        } else {
            state = 0;
            num_cnt = 0;
            *str = 0;
            echo(readdata);
        }
    }
out:
    return;
}

// TODO: Lab3: Button Interrupt, Debounce Interrupt, and Stopwatch Interrupt ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
extern void PORT1_IRQHandler(){
    //      Button 4: Samples current stopwatch value and sends it to computer (debounced)
    if(debounced == 1)
        return;
    debounced = 1;
    start_debounce_timer();

    uint32_t status;
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    if (GPIO_PIN4 & status) {
        MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
        MAP_GPIO_disableInterrupt(GPIO_PORT_P1, GPIO_PIN4);
        show_time();
        MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);
    }
}

extern void TA2_0_IRQHandler(){
//      Debounce Interrupt: Clears our debouncing flag so we listen to future button interrupts again, and
//          prevents TA2 from cycling again like it normally would.
}

extern void T32_INT1_IRQHandler(){
    //      Stopwatch Interrupt: Complains about stopwatch count running out in the far, far future
    // Jack: debouncing reset debouncing timer
    MAP_Timer32_clearInterruptFlag(TIMER32_1_BASE);
    MAP_Timer32_disableInterrupt(TIMER32_1_BASE);
    debounced = 0;
}

// END LAB3 BUTTON AND STOPWATCH INTERRUPTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
