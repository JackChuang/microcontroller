#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
/* List of all functions used in our implementation (not the only correct implementation)
 * GPIO_clearInterruptFlag
 * GPIO_enableInterrupt
 * GPIO_getEnabledInterruptStatus
 * GPIO_interruptEdgeSelect
 * GPIO_registerInterrupt
 * GPIO_setOutputLowOnPin
 * GPIO_setOutputHighOnPin
 * GPIO_toggleOutputOnPin
 * Interrupt_enableInterrupt
 * Timer32_clearInterruptFlag
 * Timer32_haltTimer
 * Timer32_initModule
 * Timer32_setCount
 * Timer32_setCountInBackground
 * Timer32_startTimer
 * Timer32_registerInterrupt
 */



static volatile uint8_t currentColor;
// TODO: LAB2: Your variables for button debouncing & tracking blink state here:
//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ENTER CODE FOR LAB2 variables HERE:
uint32_t RED_LED_ENABLED = 0;
uint32_t RED_LED_STATUS = 0;
uint32_t debounced = 0;
uint32_t count = 0;
uint32_t MODE = 0, RGB[3] = {1, 2, 4};
//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ END CODE FOR LAB2 variables here

void start_debounce_timer()
{
    MAP_Timer32_setCount(TIMER32_0_BASE,30000);
    MAP_Timer32_enableInterrupt(TIMER32_0_BASE);
    MAP_Timer32_startTimer(TIMER32_0_BASE, true);
    MAP_Interrupt_enableInterrupt(TIMER32_0_INTERRUPT);
}

void start_LED_timer()
{
    MAP_Timer32_setCount(TIMER32_1_BASE,3000000*(1+(MODE%3)));
    MAP_Timer32_enableInterrupt(TIMER32_1_BASE);
    MAP_Timer32_startTimer(TIMER32_1_BASE, true);
    MAP_Interrupt_enableInterrupt(TIMER32_1_INTERRUPT);
}

void stop_LED_timer()
{
    MAP_Timer32_clearInterruptFlag(TIMER32_1_BASE);
    MAP_Timer32_haltTimer(TIMER32_1_BASE);
    MAP_Timer32_disableInterrupt(TIMER32_1_BASE);
    MAP_Interrupt_disableInterrupt(TIMER32_1_INTERRUPT);
}

void reset_RED_LED()
{
    stop_LED_timer();
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    RED_LED_STATUS = 1;
    start_LED_timer();
    RED_LED_ENABLED = 1;
}

void handle_button_1(uint32_t status)
{
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1,status);
    MAP_GPIO_disableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

        if(RED_LED_ENABLED)
        {
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
            stop_LED_timer();
            RED_LED_STATUS = 0;
            RED_LED_ENABLED = 0;
        }
        else
        {
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            RED_LED_STATUS = 1;
            start_LED_timer();
            RED_LED_ENABLED = 1;
        }

        MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
}

void handle_button_2(uint32_t status)
{
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);
    MAP_GPIO_disableInterrupt(GPIO_PORT_P1, GPIO_PIN4);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, RGB[MODE%3]);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2,RGB[++MODE%3]);

    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);
}

extern void lab_2PORT1_IRQHandler(){
//  TODO: LAB2: Button 1: Turn on or off pulsing of red LED
//              Button 2: Cycle between colors of LED and their associated red LED pulse frequency
//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ENTER CODE FOR LAB2 buttons HERE:
    if(debounced == 1)
        return;
    debounced = 1;
    start_debounce_timer();

    count++;
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    if (GPIO_PIN1 & status)
        handle_button_1(status);
    if (GPIO_PIN4 & status)
        handle_button_2(status);

//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End CODE FOR LAB2 buttons
}

extern void T32_INT1_IRQHandler(){
//    TODO: Lab2: Change LED on/off status and prepare any necessary interrupts to fire again
//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CODE FOR LAB2 LED cycling here
    MAP_Timer32_clearInterruptFlag(TIMER32_1_BASE);
    MAP_Timer32_disableInterrupt(TIMER32_1_BASE);
    MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

    RED_LED_STATUS = RED_LED_STATUS ? 0:1;

    MAP_Timer32_setCount(TIMER32_1_BASE,3000000*(1+(MODE%3)));
    MAP_Timer32_enableInterrupt(TIMER32_1_BASE);
    MAP_Timer32_startTimer(TIMER32_1_BASE, true);

//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END CODE FOR LAB 2 led cycling
}

extern void lab2_T32_INT0_IRQHandler(){
//    TODO: Lab2: Handle Debounce Logic
//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CODE FOR LAB2 debouncing
    MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE);
    MAP_Timer32_disableInterrupt(TIMER32_0_BASE);
    debounced = 0;
//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~END CODE FOR LAB2 debouncing
}


int lab2_main(void)
{
    /*-------------------------Initialization-----------------------------*/
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Set P1.0 (LED) to output direction
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    //    Prep Color LEDs for displaying mode
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 + GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1 + GPIO_PIN4);

    SysCtl_enableSRAMBankRetention(SYSCTL_SRAM_BANK1);


//    TODO: LAB2: Button 1 & 4 init, prepare debouncing
//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ENTER CODE FOR LAB2 button interrupt config HERE:
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);

//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ END LAB2 CODE button interrupts

//  TODO: LAB2: Configure one of the Timer32 modules so the timeUp() interrupt handler is ready for use
//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ENTER CODE FOR LAB2 timer32 interrupts HERE:

    /* Configuring the TIMER32_1_BASE module of Timer32 to 3000000 (1s)
    * of MCLK in periodic mode, with a 32-bit timer */
    MAP_Timer32_initModule(TIMER32_1_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);

    /* Enabling interrupts */
    MAP_Interrupt_enableInterrupt(INT_PORT1);
    MAP_Interrupt_enableInterrupt(TIMER32_1_INTERRUPT);
    MAP_Interrupt_enableInterrupt(TIMER32_0_INTERRUPT);
    MAP_Interrupt_enableMaster();

//    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ END LAB2 CODE
    while(1){
        PCM_gotoLPM3InterruptSafe();
    }
}
