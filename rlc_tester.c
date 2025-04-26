// Serial Example
// Jasmine Proctor

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Blue LED:
//   PF2 drives an NPN
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

// Measure LR:
//   PB4 drives an NPN transistor that when on can measure L or R
// High-side R:
//   PA6 drives an NPN transistor that when on measures C
// Measure C:
//   PB6 drives an NPN transistor that when on measures C
// Low-side R:
//   PB7 drives an NPN transistor that when on measures L
// Integrate:
//   PC6 drives an NPN transistor that when on measures R
// Switch:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// Analog:
//   PC7

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "wait.h"

//-----------------------------------------------------------------------------
// Maximum number of Characters that can be accepted
//-----------------------------------------------------------------------------

//#define DEBUG
#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;
//-----------------------------------------------------------------------------

// Bitband aliases

//Port A
#define HIGHSIDE_R   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))

//Port B
#define MEAS_LR      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))
#define MEAS_C       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define LOWSIDE_R    (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 7*4)))

//Port C
#define INTEGRATE    (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define ANALOG       (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))

//Port F
#define SWITCH       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

//MASKS

//Port A masks
#define HIGHSIDE_R_MASK 64

//Port B masks
#define MEAS_LR_MASK 16
#define MEAS_C_MASK 64
#define LOWSIDE_R_MASK 128

//Port C masks
#define INTEGRATE_MASK 64
#define ANALOG_MASK 128

//Port F masks
#define SWITCH_MASK 16
#define GREEN_LED_MASK 8
#define RED_LED_MASK 2
#define ANALOG_O_MASK 1
#define BLUE_LED_MASK 4

//----------------------------------------------------
//GLOBAL VARIABLES

uint32_t time;
int8_t r,c,l;
bool complete = false;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure (them) to be digital outputs
    GPIO_PORTA_DIR_R |= HIGHSIDE_R_MASK;
    GPIO_PORTB_DIR_R |= MEAS_LR_MASK;
    GPIO_PORTB_DIR_R |= MEAS_C_MASK;
    GPIO_PORTB_DIR_R |= LOWSIDE_R_MASK;
    GPIO_PORTC_DIR_R |= INTEGRATE_MASK;
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK | BLUE_LED_MASK;
    //analog???

    // Configure SW1  digital inputs w either pull-up or pull-down enabled as needed for proper orientation
    GPIO_PORTF_DIR_R &= ~SWITCH_MASK;


    GPIO_PORTA_DEN_R |= HIGHSIDE_R_MASK;
    GPIO_PORTC_DEN_R |= INTEGRATE_MASK ; // dont config analog_input
    GPIO_PORTB_DEN_R |= MEAS_LR_MASK | MEAS_C_MASK | LOWSIDE_R_MASK;
    GPIO_PORTF_DEN_R |= SWITCH_MASK | GREEN_LED_MASK | RED_LED_MASK | BLUE_LED_MASK;


    GPIO_PORTF_PUR_R |= SWITCH_MASK;

    //ANALOG COMPARATOR
    SYSCTL_RCGCACMP_R |= SYSCTL_RCGCACMP_R0;
    _delay_cycles(3);
    // confirm that clock in on for port c (PC7 & PC^ & PF0)

    //enable gpio pin associated with CO- input signal(PC7)
    GPIO_PORTC_DIR_R &= ~(ANALOG_MASK);
    GPIO_PORTC_AFSEL_R |= ANALOG_MASK;  // i dont think i turn this on...
    GPIO_PORTC_DEN_R &= ~(ANALOG_MASK);
    GPIO_PORTC_AMSEL_R |= ANALOG_MASK;

    //enable gpio pin associated with CO+ input signal (PC6)
//    GPIO_PORTC_DIR_R &= ~(ANALOG_P_MASK);
//    GPIO_PORTC_AFSEL_R |= ANALOG_P_MASK;
//    GPIO_PORTC_DEN_R &= ~(ANALOG_P_MASK);
//    GPIO_PORTC_AMSEL_R |= ANALOG_P_MASK;

    //gpio output signal
    GPIO_PORTF_LOCK_R =  0x4C4F434B; //unlocks portf gpio commit
    GPIO_PORTF_CR_R = ANALOG_O_MASK;
    GPIO_PORTF_DIR_R |= ANALOG_O_MASK;
    GPIO_PORTF_AFSEL_R |= ANALOG_O_MASK;
    GPIO_PORTF_DEN_R |= ANALOG_O_MASK;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF0_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF0_C0O;


    //configure comparator
    COMP_ACREFCTL_R = 0x20F;
    COMP_ACCTL0_R |= COMP_ACCTL0_ISLVAL | COMP_ACCTL0_CINV | COMP_ACCTL0_ASRCP_REF;
    waitMicrosecond(10);

    NVIC_EN0_R = 1 << (INT_COMP0 -  16);



    //configure timer 1
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACDIR; //one shot mode and count up
    //TIMER1_TAILR_R = 0xFFFFFFFF;
    TIMER1_IMR_R = 0;
    TIMER1_TAV_R = 0;   // zero counter

    // turn on timer
    //TIMER1_CTL_R |= TIMER_CTL_TAEN;


}

void analog0Isr(void){
    if(r == 1){
    time = TIMER1_TAR_R;
    MEAS_LR = 0;
    LOWSIDE_R = 1;
    INTEGRATE = 1;
    //toggle red led
    RED_LED = 1;
    //disable ACINTEN
    COMP_ACINTEN_R &= ~(COMP_ACINTEN_IN0);
    // Clear the interrupt flag for Comparator 0
    //review
    COMP_ACMIS_R = COMP_ACRIS_IN0;
    complete = true;
    }
    if(c){
        time = TIMER1_TAR_R;
        MEAS_C = 1;
        LOWSIDE_R = 1;
        HIGHSIDE_R = 0;
        //toggle green led
        GREEN_LED = 1;
        //disable ACINTEN
        COMP_ACINTEN_R &= ~(COMP_ACINTEN_IN0);
        // Clear the interrupt flag for Comparator 0
        //review
        COMP_ACMIS_R = COMP_ACRIS_IN0;
        complete = true;
    }
    if (l){
        time = TIMER1_TAR_R;
                MEAS_LR = 0;
                HIGHSIDE_R = 0;
                //toggle blue led
                BLUE_LED = 1;
                //disable ACINTEN
                COMP_ACINTEN_R &= ~(COMP_ACINTEN_IN0);
                // Clear the interrupt flag for Comparator 0
                //review
                COMP_ACMIS_R = COMP_ACRIS_IN0;
                complete = true;
    }
}

//blokcing function that waits for Isr

void waitforIsr(void){
    while(!complete);
}


// GetsUART

void getsUart0(USER_DATA *data)
{
    int count = 0;
    //start loop
    //char c = getcUart0();
    while(true)
    {
        char c = getcUart0();
        if(c == 127 || c == 8)
        {
            if(count > 0){
                count--;
        }
            //else go back to getC
        }
        else if(c == 10 || c == 13)
        {
            data->buffer[count] = '\n';
            break;
        }
        else if(c >= 32)
        {
            data->buffer[count++] = c;
            if (count == MAX_CHARS){
                data->buffer[count] = '\n';
                break;
            }
        }
    }

}

void parseFields(USER_DATA *data)
{
    // alpha = 'a' (A-Z and a-z)
    // numeric = 'n' (0-9)
    // delimiter = 'd' (everything else)

    //assume that the previous character is a delimiter

    //go through buffer from left to right looking for the start of a field
    int delim = 0;
    data->fieldCount = 0;
    int i;
    for(i = 0; (data->buffer[i] != '\n'); i++){
        //data.fieldCount
        if (i == 0){
            //check fieldType

            //alpha
            if( (data->buffer[i] >= 65 && data->buffer[i]>= 90) || (data->buffer[i] >= 97 && data->buffer[i] <= 122 ))
            {
                data->fieldType[data->fieldCount] = 'a';
                data->fieldCount++;
            }

            //numeric
            else if( (data->buffer[i] >= 48 && data->buffer[i] <=57))
            {
                data->fieldType[data->fieldCount] = 'n';
                data->fieldCount++;
            }

            //delimiter
            else
            {
                //data->fieldType[data->fieldCount] = 'd';
                //data->fieldCount++;
                data->buffer[i] = 0;
                delim = 1;
            }
        }

        //for rest of the buffer
        else
        {
                //alpha
                if( (data->buffer[i] >= 65 && data->buffer[i]>= 90) || (data->buffer[i] >= 97 && data->buffer[i] <= 122 ))
                {
                    if (delim == 1){
                    data->fieldType[data->fieldCount] = 'a';
                    data->fieldPosition[data->fieldCount] = i;
                    data->fieldCount++;
                    delim = 0;
                    }
                    //else

                }

                //numeric
                else if( (data->buffer[i] >= 48 && data->buffer[i] <=57))
                {
                    if(delim == 1){
                    data->fieldType[data->fieldCount] = 'n';
                    data->fieldPosition[data->fieldCount] = i;
                    data->fieldCount++;
                    delim = 0;
                    }
                }

                //delimiter
                else
                {
                    data->buffer[i] = 0;
                    delim = 1;
                }
          }

        }
    }

char* getFieldString(USER_DATA* data, uint8_t fieldNumber){

   char str[MAX_CHARS+1];
   //return if fieldNumber is out of bounds
   if(fieldNumber > data->fieldCount){
       return NULL;
   }
    //return the string
   if(fieldNumber <= data->fieldCount){
       uint8_t i = 0;

       while ((data->buffer[(data->fieldPosition[fieldNumber-1])+i]) != '\0'  && (data->buffer[(data->fieldPosition[fieldNumber-1])+i]) != '\n'){
           str[i++] = data->buffer[data->fieldPosition[fieldNumber-1]+i];
       }
        //add terminating null character at the end of the string
        str[i] = '\0';
        return str;
   }
   return NULL;
}

 int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber){
     if(fieldNumber > data->fieldCount){
         return 0;
     }

     if(data->fieldType[fieldNumber] == 'n'){
         return ((int32_t)atoi(&data->buffer[data->fieldPosition[fieldNumber]]));
     }
     return 0;
 }

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments){
    //get string of first field
    char *str = getFieldString(data, 1);
    //compare the strings
    int val = (data->fieldCount)-1;
    int check = strcmp(str, strCommand);

    if(!check && val >= minArguments){
        //if(((data->(fieldCount)-1)>= minArguments)){
            return true;
    //    }
    //    return false;
    }
    return false;
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
    {
    USER_DATA data;
    initHw();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    // Command evaluation
    bool valid;
    while(true){
    RED_LED = 0;
    GREEN_LED = 0;
    BLUE_LED = 0;
    putsUart0("type (r) (l) (c)");
    putcUart0('\n');
    memset(data.buffer, '\0', sizeof(data.buffer));
    memset(data.fieldPosition, '\0', sizeof(data.fieldPosition));
    memset(data.fieldType, '\0', sizeof(data.fieldType));
    getsUart0(&data);
    parseFields(&data);
    putsUart0(data.buffer);
    putcUart0('\n');
    if(isCommand(&data, "r", 0)){
        // discharge capacitor
        r = 1;
        c = 0;
        l = 0;
        MEAS_LR = 0;
        MEAS_C = 0;
        LOWSIDE_R = 1;
        INTEGRATE = 1;
        HIGHSIDE_R = 0;
        waitMicrosecond(3000000);
        //review
        //COMP_ACMIS_R = COMP_ACRIS_IN0;
        valid = true;
        MEAS_LR = 1;
        TIMER1_TAV_R = 0;
        _delay_cycles(3);
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
        LOWSIDE_R = 0;
        INTEGRATE = 1;
        COMP_ACMIS_R = COMP_ACRIS_IN0;
        COMP_ACINTEN_R |= COMP_ACINTEN_IN0;
        waitforIsr();
        putsUart0("reading resistor value");
        putcUart0('\n');
        waitMicrosecond(2000000);
        char string[50] = {0};
        snprintf(string, sizeof(string), "ticks: %"PRIu32" \n", time);
        putsUart0(string);
        snprintf(string, sizeof(string), "time: %"PRIu32" (us)\n", time/40);
        putsUart0(string);
        snprintf(string, sizeof(string), "resistance: %f \n", round((time/40.0)/(1.379)));
        putsUart0(string);
        putcUart0('\n');

    }
    else if(isCommand(&data, "c", 0)){
            valid = true;
            c = 1;
            r = 0;
            l = 0;
            MEAS_LR = 0;
            MEAS_C = 1;
            LOWSIDE_R = 1;
            INTEGRATE = 0;
            HIGHSIDE_R = 0;
            waitMicrosecond(2000000);
            LOWSIDE_R = 0;

            TIMER1_TAV_R = 0;
            _delay_cycles(3);
            HIGHSIDE_R = 1;
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            COMP_ACMIS_R = COMP_ACRIS_IN0;
            COMP_ACINTEN_R |= COMP_ACINTEN_IN0;
            waitforIsr();
            putsUart0("reading capacitor value");
            putcUart0('\n');
            waitMicrosecond(2000000);
            char string[50] = {0};
            snprintf(string, sizeof(string), "ticks: %"PRIu32" \n", time);
            putsUart0(string);
            snprintf(string, sizeof(string), "time: %"PRIu32" (us)\n", time/40);
            putsUart0(string);
            snprintf(string, sizeof(string), "capacitance: %f (uF)\n", (round)(time/40.0)/(137900));
            putsUart0(string);
            putcUart0('\n');
        }
    else if(isCommand(&data, "l", 0)){
            valid = true;
            l = 1;
            r = 0;
            c = 0;
            MEAS_LR = 0;
            MEAS_C = 0;
            LOWSIDE_R = 1;
            INTEGRATE = 0;
            HIGHSIDE_R = 0;
            waitMicrosecond(2000000);
            TIMER1_TAV_R = 0;
            _delay_cycles(3);
            TIMER1_CTL_R |= TIMER_CTL_TAEN;
            MEAS_LR = 1;
            COMP_ACMIS_R = COMP_ACRIS_IN0;
            COMP_ACINTEN_R |= COMP_ACINTEN_IN0;
            waitforIsr();
            putsUart0("reading inductor value");
            putcUart0('\n');
            waitMicrosecond(2000000);
            char string[50] = {0};
            snprintf(string, sizeof(string), "ticks: %"PRIu32" \n", time);
            putsUart0(string);
            snprintf(string, sizeof(string), "time: %"PRIu32" (us)\n", time/40);
            putsUart0(string);
            snprintf(string, sizeof(string), "inductance: %f (uH)\n", (round)(time/1.68));
            putsUart0(string);
            putcUart0('\n');
        }

    // Look for error
    if (!valid){
    putsUart0("invalid command");
    putcUart0('\n');
    }
    }
    //return 0;

}
