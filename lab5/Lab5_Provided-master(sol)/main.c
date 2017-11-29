/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "printf.h"

#define LEFT TIMER_A_CAPTURECOMPARE_REGISTER_1
#define RIGHT TIMER_A_CAPTURECOMPARE_REGISTER_2
int16_t SpeedVal = 0;
uint_fast16_t LeftRight = LEFT;

uint16_t atoi(volatile char* s, uint8_t l){
    uint16_t r = 0;
    uint8_t i;
    for (i = 0; i < l;i++){
        r = 10*r + (s[i]-'0');
    }
    return r;
}

/* ADC-driving timer */
const Timer_A_UpModeConfig upModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,            // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // ACLK/1 = 32Khz
        320,                                 // period = 320/32000 = 10ms => 100Hz sampling rate
        TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE, // Disable CCR0
        TIMER_A_DO_CLEAR                     // Clear Counter
};

const Timer_A_CompareModeConfig compareConfig =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR1
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_SET_RESET,               // Toggle output but
        320                                       // match Period
};

// Debounce Timer A2 config
const Timer_A_UpModeConfig debounceConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,            // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // ACLK/1 = 32Khz
        32,                                  // Period: 32/(32Khz) = 1ms
        TIMER_A_TAIE_INTERRUPT_ENABLE,      // Disable Timer ISR
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Disable CCR0
        TIMER_A_DO_CLEAR                     // Clear Counter
};

// ~~~~~ EDIT HERE ~~~~~~~ all config structs (timera & ccrs) for PWM generation from Prelab 5
// TIMER_A1 PWM upper bond
#define MAX_PWM_CNT 320
Timer_A_UpModeConfig pwm_Config =
{
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_1,
     MAX_PWM_CNT,
     TIMER_A_TAIE_INTERRUPT_DISABLE,
     TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
     TIMER_A_DO_CLEAR
};

// TIMER_A1 PWM Left
Timer_A_CompareModeConfig ccr1Config =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_1,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
    TIMER_A_OUTPUTMODE_TOGGLE_SET,
    0
};

// TIMER_A1 PWM Right
Timer_A_CompareModeConfig ccr2Config =
{
    TIMER_A_CAPTURECOMPARE_REGISTER_2,
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
    TIMER_A_OUTPUTMODE_TOGGLE_SET,
    0
};


// UART over USB config
const eUSCI_UART_Config uartConfig = // USING 3Mhz Master Clock, baud rate 115200 (values from the calculator at http://bit.ly/432UART)
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        1,                                     // BRDIV = 19
        10,                                       // UCxBRF = 8
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};
// Port remapping: Points TA1.1 and TA1.2 to pins P3.7 and P3.6, respectively.
const uint8_t portMapping[] =

{ //Port 3 remap
        PMAP_NONE,  PMAP_NONE,  PMAP_NONE,

        PMAP_NONE,      PMAP_NONE,      PMAP_NONE,

        PMAP_TA1CCR1A,      PMAP_TA1CCR2A

};


/* Statics */
static volatile bool ignoreButton;
static volatile bool sampling;
// Are we reading a number? (set true to call handleLongCommand once reading of number is complete)
static volatile bool readingNumber;
// most recently received nonnumeric character
static volatile char recentCmd;
// the number being read in over UART (accumulates digit by digit)
static volatile int16_t readValue;
// flag in case we are reading a negative value
static volatile bool negative;

int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_enableSleepOnIsrExit();

    /* Setting up clocks
     * MCLK = MCLK = 3MHz = DCO = SMCLK
     * ACLK = REFO = 32Khz */
    CS_setDCOFrequency(3000000);
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* (old) Initializing ADC (uses MCLK to sample A0 on P5.5 into MEM0 and trigger interrupt TIN_ADC14)  */
    /* (new) Initializing ADC (uses MCLK to sample)  */
    MAP_ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_NOROUTE);
    ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE1, false); //SOURCE1 = timerA0's CCR1 output per table 6-51 in 432 datasheet
    ADC14_setResolution(ADC_14BIT);

    // ~~~~~ EDIT HERE ~~~~~~~ to read in A1 and A0 (our motor driver current report pins scaled to safe values) instead of A0
    ADC14_configureSingleSampleMode(ADC_MEM0, true); //USES External voltage references!! VRef+ on P5.6, VRef- on P5.7 per http://bit.ly/432Function
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, ADC_NONDIFFERENTIAL_INPUTS);

    ADC14_configureSingleSampleMode(ADC_MEM1, true);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A1, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
        GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_TERTIARY_MODULE_FUNCTION); // per http://bit.ly/432Function

    // ~~~~~ EDIT HERE ~~~~~~~ modify the interrupts on the ADC to actually trigger for A1 & A0
    MAP_ADC14_enableInterrupt(ADC_INT0 | ADC_INT1);
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    sampling = 0; // Only start sampling at 100Hz when activated by UART command or P1.4 button press

    /* (old) Starting Timer_A0 in up mode and sourced from ACLK (32khz), sampling every 0.5s */
    /* (new) Starting Timer_A0 in up mode and sourced from ACLK (32khz), sampling every 10ms or 100Hz */
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upModeConfig);
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig);
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

    // ~~~~~ EDIT HERE ~~~~~~~  Don't forget to initialize the timerA1 and its CCRs for PWM
    //PWM
    Timer_A_initCompare(TIMER_A1_BASE, &ccr1Config);        //Left
    Timer_A_initCompare(TIMER_A1_BASE, &ccr2Config);        //Right
    Timer_A_configureUpMode(TIMER_A1_BASE, &pwm_Config);    //PWM upper bond
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);   //

    // ~~~~~ EDIT HERE ~~~~~~~ add in the GPIO pins for telling the motor driver forward or back
    // TODO: OUTPUT PIN * 2

    // SETUP UART
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION); // per http://bit.ly/432Function
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);
    MAP_UART_enableModule(EUSCI_A0_BASE);
    readingNumber = 0;
    recentCmd = 0;
    negative = 0;
    readValue = 0;

    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();

    // Setup GPIO (button & LED)
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);  // <- Not strictly necessary, safety precaution against spurious interrupt flags on boot
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);
    Interrupt_enableInterrupt(INT_PORT1);

//    Prepare debouncing using Timer A2
    ignoreButton = 0;
    Timer_A_configureUpMode(TIMER_A2_BASE, &debounceConfig);
    Timer_A_enableInterrupt(TIMER_A2_BASE);
    Timer_A_enableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    Interrupt_enableInterrupt(INT_TA2_0);

    // Redirect TA1.1 and TA1.2 outputs to pins P3.7 and P3.8, for ease of physical wiring
    PMAP_configurePorts(portMapping, PMAP_P3MAP, 2, PMAP_DISABLE_RECONFIGURATION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION); //outputs TA1.1 and TA1.2 outputs on pins 3.7 and 3.6 respectively.

    /* Going to sleep z_z */
    while (1)
    {
        MAP_PCM_gotoLPM0InterruptSafe();
    }
}

// Centralized Sampling-State-Toggle code for code cleanliness
void toggleSamplingState(){
    if (sampling){
        printf(EUSCI_A0_BASE,"\n\rSampling Halted.\n\r");
        sampling = 0;
        ADC14_disableConversion();
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    } else {
        printf(EUSCI_A0_BASE,"\n\rSampling...\n\r");
        ADC14_enableConversion();
        sampling = 1;
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }
}

// called only after a complete, multicharacter command has been received (character followed by number_
void handleLongCmd(char cmd, int16_t number){
    if(cmd == 'd'){ //double
        printf(EUSCI_A0_BASE,"\n\r%i*2 = %i\n\r",number, number*2);
    }
    // Additional Commands Here As Needed
}

// UART Interrupt (receives commands)
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        char readdata = EUSCI_A_SPI_receiveData(EUSCI_A0_BASE);
        //Always Echo back
        printf(EUSCI_A0_BASE, "%c.", readdata);

        //Handle & Accumulate numeric values; will call handleLongCmd(recentCmd,readValue) once command is complete
        if(readingNumber){
            if(readdata < '0' || readdata > '9'){ //new non-numeric character!!
                if(readdata=='-' && readValue==0){ //first character is minus sign means value should be negative, keep reading number
                    negative = 1;
                } else { //we have a nonnumeric character; have thus hit end of command.
                    handleLongCmd(recentCmd, readValue);
                    readingNumber = 0;
                    readValue = 0;
                    negative = 0;
                }
            } else { //numeric character
                readValue = 10*readValue + (negative ? -1 : 1) * (readdata - '0');
            }
        }

        //change recent command if input is not a number
        if(readdata != '-' && (readdata < '0' || readdata > '9')){
            recentCmd = readdata;
        }

        //command character logic:
        if(readdata == 's' || readdata == 'S'){ //Toggle sampling mode
            toggleSamplingState();
        }
        if(readdata == '?'){ //Help message
            printf(EUSCI_A0_BASE,"\n\rCommand:\tEffect:\n\r");
            printf(EUSCI_A0_BASE,"?\tPrint this help message\n\r"); //?
            printf(EUSCI_A0_BASE,"s\tToggle Sampling Behavior\n\r"); //s
            printf(EUSCI_A0_BASE,"d#\tDouble Input Number\n\r"); //?
            // ~~~~~ EDIT HERE ~~~~~~~ to actually use the number for the L or R command
            printf(EUSCI_A0_BASE,"l: control focus on Left only\n\r");
            printf(EUSCI_A0_BASE,"r: control focus on Right only\n\r");
            printf(EUSCI_A0_BASE,"0~9: corresponding to 0% ~ 90% dutycycles\n\r");
        }
        if(readdata == 'd'){ //double cmd
            readingNumber = true; //tells us this a long command, begins accumulating number
            // rest of logic is in handleLongCmd, called when next nonnumeric character is received.
        }

        // ~~~~~~~~~~ EDIT HERE ~~~~~~~~~~ to add L and R
        if(readdata >= '0' && readdata <= '9' ){
            SpeedVal = ((((readdata - '0')*10)*MAX_PWM_CNT)/100); // 0% to 90% duty cycle / Max counter value is 320
            Timer_A_setCompareValue(TIMER_A1_BASE, LeftRight, SpeedVal); //PWM
            printf(EUSCI_A0_BASE, "Mode %s Speed %i duty cycle\r\n",
                                   LeftRight==LEFT?"Left":"Right", ((SpeedVal*100)/MAX_PWM_CNT));
            //printf(EUSCI_A0_BASE, "%i\r\n", SpeedVal);
        }
        if(readdata == 'l') {
            LeftRight = LEFT;
            printf(EUSCI_A0_BASE,"Mode: Left\n\r");
        }
        else if(readdata == 'r') {
            LeftRight = RIGHT;
            printf(EUSCI_A0_BASE,"Mode: Right\n\r");
        }
        //Additional commands here as needed
    }
}

/* This interrupt is fired whenever a conversion is completed and placed in ADC_MEM0 */
void ADC14_IRQHandler(void)
{
    uint64_t status;
    static uint64_t cnt = 0;
    static uint16_t adcLeftVal = 0, adcRightVal = 0;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    if (status & ADC_INT0) { //A0 reading, on P5.5
        adcLeftVal = ADC14_getResult(ADC_MEM0);
        printf(EUSCI_A0_BASE, "got ADC_MEM0\n\r");
    }
    else if (status & ADC_INT1) {
        adcRightVal = ADC14_getResult(ADC_MEM1);
        printf(EUSCI_A0_BASE, "got ADC_MEM1\n\r");
    }
    cnt++;
    if(cnt>=100) {
        cnt = 0;
        printf(EUSCI_A0_BASE, "Left %u Right %u\n\r",
                               adcLeftVal, adcRightVal);  //This call is blocking!!
    }
}

void PORT1_IRQHandler(void){ //Port 1 interrupt
    uint64_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if(status & GPIO_PIN4 & ~ignoreButton){ //button 1.4
        ignoreButton = true;
        Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

        toggleSamplingState();
    }

}

void TA2_0_IRQHandler(void){ //debounce timer interrupt
    uint64_t status = Timer_A_getEnabledInterruptStatus(TIMER_A2_BASE);

    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    Timer_A_clearInterruptFlag(TIMER_A2_BASE);

    Timer_A_stopTimer(TIMER_A2_BASE);
    ignoreButton = false;
}
