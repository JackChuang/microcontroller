/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "printf.h"

// 2^14 = 16384
// adc 16383 = 3.3v (with divider) => 5v
// adc 0 = 0v => 0v
// (readdata*1000*(3.3/16384)) * (5/3.3) = REAL mA
// when no load, measured by multimeter = 0.016(0% duty) ~ 0.02(100% duty)


// lab6
volatile uint8_t lcm = 0;
volatile uint16_t gpwm = 0; // 0% ~ 100%
volatile bool isSysStart = false;
float kp = 1;
float ki = 1;
float kd = 1;

// For pwm and parsing
#define LEFT TIMER_A_CAPTURECOMPARE_REGISTER_1
#define RIGHT TIMER_A_CAPTURECOMPARE_REGISTER_2
volatile int16_t SpeedVal = 0;
volatile uint_fast16_t LeftRight = LEFT;
volatile char str[12]; // MIN = "RXXXLYYY." MAX = "R-XXXL-YYY."
volatile char strlcm[2];
volatile int num = 0;

// For encoder and rover direction
volatile uint8_t EncodeSigA = 0xff, EncodeSigB = 0xff;
volatile bool isNotClockwise = true, oldisNotClockwise = true;
volatile bool islost = false;
volatile int32_t counter = 0;
enum StatName{AA, BB, CC, DD};
volatile static enum StatName OldCurStat = AA, NewCurStat = AA;


//volatile bool IsDirFront = trun;            // is direction front
void clear(void) {
    int i;
    for (i=0; i<=12; i++)
        str[i] = 0;
    num = 0;
}
uint16_t atoi(volatile char* s, uint8_t l){
    uint16_t r = 0;
    uint8_t i;
    for (i = 0; i < l;i++){
        r = 10*r + (s[i]-'0');
    }
    return r;
}

/* duty = 0 ~ 100%*/
void rover_speed_duty(uint32_t duty) {
    Timer_A_setCompareValue(TIMER_A1_BASE, RIGHT, duty);
    Timer_A_setCompareValue(TIMER_A1_BASE, LEFT, duty);
}

void DirectionFront() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1 + GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);

}
void DirectionBack() {
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1 + GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0 + GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
}

void DirectionLeft() {
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1); //f
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // f
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2); // b
    GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1); // b

}
void DirectionRight() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1); //back
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // back
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); // forward
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1); // forward
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
#define MAX_PWM_CNT 999
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
    // P4.0    GPIO        N/A     DVSS        N/A     DVSS        A13
    // P4.2    GPIO        N/A     ACLK        TA2CLK  DVSS        A11
    // P4.3    GPIO        N/A     MCLK        N/A     RTCCLK      A10
    //http://bit.ly/432Function
    ADC14_configureSingleSampleMode(ADC_MEM0, true);
    //ADC14_configureSingleSampleMode(ADC_MEM0 | ADC_MEM1, true); //USES External voltage references!! VRef+ on P5.6, VRef- on P5.7 per
    /*
    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, ADC_NONDIFFERENTIAL_INPUTS);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A1, ADC_NONDIFFERENTIAL_INPUTS);
    ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);
*/
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0, ADC_NONDIFFERENTIAL_INPUTS);

    //ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_EXTPOS_VREFNEG_EXTNEG, ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);

    //ADC14_configureSingleSampleMode(ADC_MEM0, true);
    //ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A12, ADC_NONDIFFERENTIAL_INPUTS);
    //ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A1, ADC_NONDIFFERENTIAL_INPUTS);
    //ADC14_configureConversionMemory(ADC_MEM13, ADC_VREFPOS_EXTPOS_VREFNEG_EXTNEG, ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);

    // ADC PIN !!!!!!!!!! P4.0
    // per http://bit.ly/432Function
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
        GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0 ,
                                                   GPIO_TERTIARY_MODULE_FUNCTION);

    // ADC INT
    // ~~~~~ EDIT HERE ~~~~~~~ modify the interrupts on the ADC to actually trigger for A1 & A0 <- THIS IS NOT TRUE
    MAP_ADC14_enableInterrupt(ADC_INT0 | ADC_INT1 | ADC_INT2); //MEMx
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    sampling = 0; // Only start sampling at 100Hz when activated by UART command or P1.4 button press

    /* (old) Starting Timer_A0 in up mode and sourced from ACLK (32khz), sampling every 0.5s */
    /* (new) Starting Timer_A0 in up mode and sourced from ACLK (32khz), sampling every 10ms or 100Hz */
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upModeConfig);
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig);
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
    /*ADC done*/

    // ~~~~~ EDIT HERE ~~~~~~~  Don't forget to initialize the timerA1 and its CCRs for PWM
    //PWM
    Timer_A_initCompare(TIMER_A1_BASE, &ccr1Config);        //Left
    Timer_A_initCompare(TIMER_A1_BASE, &ccr2Config);        //Right
    Timer_A_configureUpMode(TIMER_A1_BASE, &pwm_Config);    //PWM upper bond
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);   //

    // ~~~~~ EDIT HERE ~~~~~~~ add in the GPIO pins for telling the motor driver forward or back
    // TODO: OUTPUT PIN * 2
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);

    DirectionFront();

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

    /*
    Encoder for a motor
    P4.4 - A
    P4.5 - B
    P4.6 - rising edge
    P4.7 - falling edge
    */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN4 + GPIO_PIN5 +
                                                         GPIO_PIN6 + GPIO_PIN7);
    GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN4 + GPIO_PIN5 +
                                            GPIO_PIN6 + GPIO_PIN7);

    GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN7, GPIO_HIGH_TO_LOW_TRANSITION);

    GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN6 + GPIO_PIN7);
    Interrupt_enableInterrupt(INT_PORT4);

    //Prepare debouncing using Timer A2
    ignoreButton = 0;
    Timer_A_configureUpMode(TIMER_A2_BASE, &debounceConfig);
    Timer_A_enableInterrupt(TIMER_A2_BASE);
    Timer_A_enableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    Interrupt_enableInterrupt(INT_TA2_0);

    // Redirect TA1.1 and TA1.2 outputs to pins P3.7 and P3.8, for ease of physical wiring
    PMAP_configurePorts(portMapping, PMAP_P3MAP, 2, PMAP_DISABLE_RECONFIGURATION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN6 | GPIO_PIN7,
            GPIO_PRIMARY_MODULE_FUNCTION); //outputs TA1.1 and TA1.2 outputs on pins 3.7 and 3.6 respectively.


    //setup UART (bluetooth) BT - zigbee
    /*
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, // no need for clock pin, unlike USB
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION); // per http://bit.ly/432Function
    UART_initModule(EUSCI_A2_BASE, &uartConfig);
    UART_enableModule(EUSCI_A2_BASE);

    UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA2);
     */

    // state machine init
    EncodeSigA = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN4);
    EncodeSigB = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN5);
    // state machine
    // 01 11 10 00 01 11 10
    // AA BB CC DD AA BB CC
    if ((EncodeSigA == 0x00) && (EncodeSigB == 0x01))
        OldCurStat = AA;
    else if ((EncodeSigA == 0x01) && (EncodeSigB == 0x01))
        OldCurStat = BB;
    else if ((EncodeSigA == 0x01) && (EncodeSigB == 0x00))
        OldCurStat = CC;
    else if ((EncodeSigA == 0x00) && (EncodeSigB == 0x00))
        OldCurStat = DD;

    while (1)
    {
        /* Going to sleep z_z */
        //MAP_PCM_gotoLPM0InterruptSafe();

        //printf(EUSCI_A0_BASE,"GO (dist) (L%u cm)\n\r", lcm);
        //printf(EUSCI_A0_BASE, "(%u distADC(MEM2), %i encoderL, %u% pwmL)\n\r",
        //                                      adcDistVal, counter, gpwm/10);
        if(isSysStart){
            //control
/*
            // curren
            // adcDistVal -> curlcm
            float curlcm = adcDistVal;
            static float lastInput;
            // target
            //lcm

            //errcm = curlcm - lcm;

            //gpwm = x;
            rover_speed_duty(gpwm);

            //Compute all the working error variables/
            // lcm = target
            float error = lcm - curlcm;  //curlcm: input, +:forward, -:backward
            float ITerm = lastOutput
            ITerm += (ki * error);
            // ITerm =
            // ITerm = lastOutput;
outMax = 100;
outMin = 0;
            if(ITerm > outMax) ITerm = outMax;
            else if(ITerm < outMin) ITerm= outMin;
            float dInput = (curlcm - lastInput);

            //Compute PID Output
            float output = kp * error + ITerm - kd * dInput;

            if(output > outMax) output = outMax;
            else if(output < outMin) output = outMin;
            *myOutput = output;

            //Remember some variables for next time
            lastInput = input;
            lastOutput =
*/
        }
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
    static bool isWaitNum = false, isFirstNum = false;
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        char readdata = EUSCI_A_SPI_receiveData(EUSCI_A0_BASE);
        //Always Echo back
        printf(EUSCI_A0_BASE, "%c.", readdata); //debug

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
        /* lab6
         * L##
         * G
         * K = stop
         * S = start/stop showing
         * (distADC, encoderL, pwmL)
         *
         */
        if(readdata == 'L' || readdata == 'l') {
            printf(EUSCI_A0_BASE, "COMMAND \"L\"\n\r"); //debug
            isWaitNum = true;
        } else if (readdata > '0' && readdata < '9') {
            printf(EUSCI_A0_BASE, "COMMAND \"0~9\"\n\r"); //debug
            if (isWaitNum) {
                if (isFirstNum) {
                    strlcm[1] = readdata;
                    lcm = atoi(&strlcm[0], 2);
                    printf(EUSCI_A0_BASE,"L%u cm \n\r", lcm);
                    isWaitNum = false;
                    isFirstNum = false;
               } else {
                   strlcm[0] = readdata;
               }
            }
        } else if(readdata == 'G' || readdata == 'g') {
            printf(EUSCI_A0_BASE, "COMMAND \"G\" L%u cm \n\r", lcm); //debug
            if(lcm > 0)
                isSysStart = true;
        } else if(readdata == 'K' || readdata == 'k') {
            printf(EUSCI_A0_BASE, "COMMAND \"K\"\n\r"); //debug
            isSysStart = false;
            gpwm = 10;
            rover_speed_duty(0);
        }

        if((readdata != 'L' || readdata != 'l') && !(isWaitNum && (readdata < '0' || readdata > '9'))) {
            isWaitNum = false;
        }

        /*
        // collection
        if(readdata == 'R' || readdata == 'L' || readdata == '-' ||
           (readdata >= '0' && readdata <='9')) {
            if(num >= 10) { // bad
                num = 0;
                printf(EUSCI_A0_BASE,"over length\n\r");
                clear();
            } else { // good
                str[num] = readdata;
                num++;
                //printf(EUSCI_A0_BASE, "srt = \"%s\"\n\r", str); //debug
            }
        }
        if(readdata == '.') {
            // start to parse
            // MIN = "RXXXLYYY." MAX = "R-XXXL-YYY."
            if(num != 0) {
                printf(EUSCI_A0_BASE, "\n\rCOMMAND = \"%s\"\n\r", str);

                if(num == 8) {
                    // check sanity: "RXXXLYYY."
                    if ((str[0] == 'R') &&
                        (str[1] >= '0' && str[1] <= '9') &&
                        (str[2] >= '0' && str[2] <= '9') &&
                        (str[3] >= '0' && str[3] <= '9') &&
                        (str[5] >= '0' && str[5] <= '9') &&
                        (str[6] >= '0' && str[6] <= '9') &&
                        (str[7] >= '0' && str[7] <= '9')) {
                            DirectionFront();
                            Timer_A_setCompareValue(TIMER_A1_BASE, RIGHT, atoi(&str[1], 3));
                            Timer_A_setCompareValue(TIMER_A1_BASE, LEFT, atoi(&str[5], 3));
                            printf(EUSCI_A0_BASE, "***** Forward R %i L %i *****\r\n",
                                                   atoi(&str[1], 3), atoi(&str[5], 3));
                    } else {
                        printf(EUSCI_A0_BASE, "wrong command\n\r", str);
                    }
                }
                else if(num == 10) {
                    // check sanity: "R-XXXL-YYY."
                    if ((str[0] == 'R') &&
                        (str[1] == '-')&&(str[6] == '-') &&
                        (str[5] == 'L') &&
                        (str[2] >= '0' && str[2] <= '9') &&
                        (str[3] >= '0' && str[3] <= '9') &&
                        (str[4] >= '0' && str[4] <= '9') &&
                        (str[7] >= '0' && str[7] <= '9') &&
                        (str[8] >= '0' && str[8] <= '9') &&
                        (str[9] >= '0' && str[9] <= '9')) {
                            DirectionBack();
                            Timer_A_setCompareValue(TIMER_A1_BASE, RIGHT, atoi(&str[2], 3));
                            Timer_A_setCompareValue(TIMER_A1_BASE, LEFT, atoi(&str[7], 3));
                            printf(EUSCI_A0_BASE, "***** Backward R %i L %i *****\r\n",
                                                   atoi(&str[2], 3), atoi(&str[7], 3));
                    } else {
                        printf(EUSCI_A0_BASE, "wrong command\n\r", str);
                    }
                } else if(num == 9) { //BONUS points!!!!!!!!!!!!!!
                    // check sanity: "RXXXL-YYY." or "R-XXXLYYY."
                    if ((str[0] == 'R') &&
                        (str[1] == '-') &&
                        (str[2] >= '0' && str[2] <= '9') &&
                        (str[3] >= '0' && str[3] <= '9') &&
                        (str[4] >= '0' && str[4] <= '9') &&
                        (str[5] == 'L') &&
                        (str[6] >= '0' && str[6] <= '9') &&
                        (str[7] >= '0' && str[7] <= '9') &&
                        (str[8] >= '0' && str[8] <= '9')) {
                            DirectionRight();
                            Timer_A_setCompareValue(TIMER_A1_BASE, RIGHT, atoi(&str[2], 3));
                            Timer_A_setCompareValue(TIMER_A1_BASE, LEFT, atoi(&str[6], 3));
                            printf(EUSCI_A0_BASE, "***** Turning Right R %i L %i *****\r\n",
                                                   atoi(&str[2], 3), atoi(&str[6], 3));
                    } else if ((str[0] == 'R') &&
                            (str[1] >= '0' && str[1] <= '9') &&
                            (str[2] >= '0' && str[2] <= '9') &&
                            (str[3] >= '0' && str[3] <= '9') &&
                            (str[4] == 'L') &&
                            (str[5] == '-') &&
                            (str[6] >= '0' && str[6] <= '9') &&
                            (str[7] >= '0' && str[7] <= '9') &&
                            (str[8] >= '0' && str[8] <= '9')) {
                                DirectionLeft();
                                Timer_A_setCompareValue(TIMER_A1_BASE, RIGHT, atoi(&str[1], 3));
                                Timer_A_setCompareValue(TIMER_A1_BASE, LEFT, atoi(&str[6], 3));
                                printf(EUSCI_A0_BASE, "***** Turning Left R %i L %i *****\r\n",
                                                       atoi(&str[1], 3), atoi(&str[6], 3));
                    }
                }
                else
                    printf(EUSCI_A0_BASE, "*****wrong command*****\n\r", str);
                clear();
            } else {
                printf(EUSCI_A0_BASE, "\n\rclear\n\r");
            }
        }
        //Additional commands here as needed
         */
    }
}

/* This interrupt is fired whenever a conversion is completed and placed in ADC_MEM0 */
void ADC14_IRQHandler(void)
{
    uint32_t status;
    static uint32_t cntLeft = 0, cntRigh = 0, cntADC = 0;
    static uint16_t adcLeftVal = 0, adcRightVal = 0;
    static uint16_t SumLeft = 0, SumRight = 0;
    static uint32_t SumADCDistVal = 0;
    static uint16_t adcDistVal = 0;
    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);
    // printf(EUSCI_A0_BASE, "ADC status %x\n\r", status);  // debug

    if (status & ADC_INT0) { //A0 reading, on P5.5
        adcLeftVal = ADC14_getResult(ADC_MEM0); //debug
        SumLeft += (adcLeftVal*1000*5/16384);
        cntLeft++;
        //if(cnt>=100)
            //printf(EUSCI_A0_BASE, "got ADC_MEM0\n\r"); //debug
        printf(EUSCI_A0_BASE, "peeking ADC(MEM0) %n %n(sum)\n\r", ADC14_getResult(ADC_MEM0), SumLeft);  // debug
    }
    else if (status & ADC_INT1) {
        adcRightVal = ADC14_getResult(ADC_MEM1); //debug
        SumRight += (adcRightVal*1000*5/16384);
        cntRigh++;
        //if(cnt>=100)
            //printf(EUSCI_A0_BASE, "got ADC_MEM1\n\r"); //debug
    }
    else if (status & ADC_INT2) //A13/MEM2 reading, on P4.0
    {
        SumADCDistVal += ADC14_getResult(ADC_MEM2);
        cntADC++;
        //if (cntADC > 10)
            printf(EUSCI_A0_BASE, "peeking ADC(MEM2) %n %n(sum)\n\r", ADC14_getResult(ADC_MEM2), SumADCDistVal);  // debug
    }
    if(cntRigh >= 100 || cntLeft >= 100 || cntADC >= 10 ) {
        if (cntLeft > 0 && cntRigh > 0)
            printf(EUSCI_A0_BASE, "Left %u mA (MEM0), Right %u mA(MEM1)\n\r",
                                       SumLeft/cntLeft,
                                       SumRight/cntRigh);
        if (cntADC > 0) {
            adcDistVal = SumADCDistVal/cntADC;
            printf(EUSCI_A0_BASE, "(%n / %u)\n\r", SumADCDistVal, cntADC);   // debug
        }
        printf(EUSCI_A0_BASE, "(%u distADC(MEM2) | %i encoderL | %u% pwmL)\n\r",
                                       adcDistVal, counter, gpwm);


        cntLeft = 0;
        cntRigh = 0;
        cntADC = 0;
        SumADCDistVal = 0;
        SumLeft=0;
        SumRight=0;
    }
}

void PORT1_IRQHandler(void){ //Port 1 interrupt
    uint64_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, status);
    //GPIO_disableInterrupt(GPIO_PORT_P1, GPIO_PIN4);

    if(status & GPIO_PIN4 & ~ignoreButton){ //button 1.4
        ignoreButton = true;
        Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);

        toggleSamplingState();
    }
    //GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);
}

void PORT4_IRQHandler(void){ //Port 4 interrupt
    static int cnt = 0;
    uint64_t status = GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
    if((status & GPIO_PIN6) || (status & GPIO_PIN7)) {
        EncodeSigA = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN4);
        EncodeSigB = GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN5);
        // state machine
        // 01 11 10 00 01 11 10
        // AA BB CC DD AA BB CC
        // Forward <--==Rover==---> backward
        switch (OldCurStat) {
            case DD:
                if ((EncodeSigA == 0x00) && (EncodeSigB == 0x01)) {
                    NewCurStat = AA; counter--;
                    isNotClockwise = true; islost = false;
                    if (isNotClockwise != oldisNotClockwise)
                        printf(EUSCI_A0_BASE, "***Direction changed***\n\r");
                } else if ((EncodeSigA == 0x01) && (EncodeSigB == 0x00)) {
                    NewCurStat = CC; counter++;
                    isNotClockwise = false; islost = false;
                    if (isNotClockwise != oldisNotClockwise)
                        printf(EUSCI_A0_BASE, "***Direction changed***\n\r");
                } else {
                    islost = true;
                    printf(EUSCI_A0_BASE, "lost steps\n\r");
                }
                break;
            case AA:
                // 01 11 10 00 01 11 10
                // AA BB CC DD AA BB CC
                // Forward <--==Rover==---> backward
                if ((EncodeSigA == 0x01) && (EncodeSigB == 0x01)) {
                    NewCurStat = BB; counter--;
                    isNotClockwise = true; islost = false;
                    if (isNotClockwise != oldisNotClockwise)
                        printf(EUSCI_A0_BASE, "***Direction changed***\n\r");
                } else if ((EncodeSigA == 0x00) && (EncodeSigB == 0x00)) {
                    NewCurStat = DD; counter++;
                    isNotClockwise = false; islost = false;
                    if (isNotClockwise != oldisNotClockwise)
                        printf(EUSCI_A0_BASE, "***Direction changed***\n\r");
                } else {
                    islost = true;
                    printf(EUSCI_A0_BASE, "lost steps\n\r");
                }
                break;
            case BB:
                // 01 11 10 00 01 11 10
                // AA BB CC DD AA BB CC
                // Forward <--==Rover==---> backward
                if ((EncodeSigA == 0x01) && EncodeSigB == 0x00) {
                    NewCurStat = CC; counter--;
                    isNotClockwise = true; islost = false;
                    if (isNotClockwise != oldisNotClockwise)
                        printf(EUSCI_A0_BASE, "***Direction changed***\n\r");
                } else if ((EncodeSigA == 0x00) && (EncodeSigB == 0x01)) {
                    NewCurStat = AA; counter++;
                    isNotClockwise = false; islost = false;
                    if (isNotClockwise != oldisNotClockwise)
                        printf(EUSCI_A0_BASE, "***Direction changed***\n\r");
                } else {
                    islost = true;
                    printf(EUSCI_A0_BASE, "lost steps\n\r");
                }
                break;
            case CC:
                // 01 11 10 00 01 11 10
                // AA BB CC DD AA BB CC
                // Forward <--==Rover==---> backward
                if ((EncodeSigA == 0x00) && (EncodeSigB == 0x00)) {
                    NewCurStat = DD; counter--;
                    isNotClockwise = true; islost = false;
                    if (isNotClockwise != oldisNotClockwise)
                        printf(EUSCI_A0_BASE, "***Direction changed***\n\r");
                } else if ((EncodeSigA == 0x01) && (EncodeSigB == 0x01)) {
                    NewCurStat = BB; counter++;
                    isNotClockwise = false; islost = false;
                    if (isNotClockwise != oldisNotClockwise)
                        printf(EUSCI_A0_BASE, "***Direction changed***\n\r");
                } else {
                    islost = true;
                    printf(EUSCI_A0_BASE, "lost steps\n\r");
                }
                break;
        }

        if ((isNotClockwise != oldisNotClockwise) || islost) {
            cnt = 0;
        }
        else {
            cnt++;
            if(cnt >= 333) {
                cnt = 0;
                printf(EUSCI_A0_BASE, "***isForwarding (%s) counter %l***\n\r",
                                       isNotClockwise?"NOT":"YES", counter);
            }
        }
        OldCurStat = NewCurStat;
        oldisNotClockwise = isNotClockwise;
        //printf(EUSCI_A0_BASE, "A %x (4.6), B %x (4.7) (hex)\n\r",
        //                          EncodeSigA, EncodeSigB); //debug
    }
}

void TA2_0_IRQHandler(void){ //debounce timer interrupt
    uint64_t status = Timer_A_getEnabledInterruptStatus(TIMER_A2_BASE);

    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    Timer_A_clearInterruptFlag(TIMER_A2_BASE);

    Timer_A_stopTimer(TIMER_A2_BASE);
    ignoreButton = false;
}

/* BT - zigbee
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        char readdata = UART_receiveData(EUSCI_A2_BASE);

        printf(EUSCI_A0_BASE, "%c.", readdata);

        // toggle LED1.0 w/ every incoming character on bluetooth
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }

}
*/
