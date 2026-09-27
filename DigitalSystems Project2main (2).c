/*
 * Project2_us.c
 *
 * Created: 27/04/2025
 * Author : Ros.Walker
 * Student Id : 23361719
 */ 

 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <avr/cpufunc.h>
 
 #define F_CPU 20000000
 #define USART3_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)
 #define TARGET_BAUD_RATE 115200
 
 #define TCB_INTERVAL 1000000
 #define TCB_TOP 50000
 #define TCB_COUNT (TCB_INTERVAL/TCB_TOP)
 #define TICK_DURATION_US 0.1
 
 #include <util/delay.h>
 #include <stdio.h>

 /* Use a struct to make the association between PORTs and bits connected to the LED array more explicit */
 struct LED_BITS
 {
	PORT_t *LED_PORT;
	uint8_t bit_mapping;
 };

 struct LED_BITS LED_Array[10] = {
 {&PORTC, PIN5_bm}, {&PORTC, PIN4_bm}, {&PORTA, PIN0_bm}, {&PORTF, PIN5_bm}, {&PORTC, PIN6_bm}, {&PORTB, PIN2_bm}, {&PORTF, PIN4_bm}, {&PORTA, PIN1_bm}, {&PORTA, PIN2_bm}, {&PORTA, PIN3_bm}
 };
 
/* Global variables */
 unsigned char queue[50];
 uint8_t newDistanceData, newTimeData, newADC0Data, qcntr = 0, sndcntr = 0, ServoFollowADC;
 uint16_t adc_reading, Pulse_Width, step_delay = 1000, servo_pos = 1500;		
 extern volatile uint16_t high_time, low_time; 
 volatile uint16_t pulseTime = 0;
 volatile uint16_t periodTime = 1;
 volatile uint16_t high_time = 1;
 volatile uint16_t low_time = 1;
 
void CLOCK_init (void);
void InitialiseLED_PORT_bits(void);
void Initialise_TCA0_SS_PWM(void);
void Initialise_EVSYS (void);
void Initialise_TCB0_ICP_PW(void);
void Initialise_TCB1_ICP_PW(void);
void Initialise_TCB2_ICP_PWFRQ(void);
void Set_Clear_Ports(uint8_t set);
void USART3_init(void);
void ADC0_init(void);
void sendmsg(char* s); 

 void CLOCK_init (void)
 {
	/* Disable CLK_PER Prescaler */
	ccp_write_io( (void *) &CLKCTRL.MCLKCTRLB , (0 << CLKCTRL_PEN_bp));
	/* If set from the fuses during device programming, the CPU will now run at 20MHz (default is /6) */
 }

 void InitialiseLED_PORT_bits()
 {
	PORTC.DIRSET = PIN6_bm | PIN5_bm | PIN4_bm;  /*(1<<6) | (1<<5) | (1<<4); 0x70;*/		/* PC4-UNO D1 (TXD1), PC5-UNO D0 (RXD1), PC6 - UNO D4  */
	PORTA.DIRSET = PIN3_bm | PIN2_bm | PIN1_bm | PIN0_bm; /*(1<<1) | (1<<0);   0x0f; */      /* PA1-UNO D7, PA0 - UNO D2, PA2- LED8, PA3 - LED9  */
	PORTB.DIRSET = PIN2_bm; /*0x04;*/		/* PB2 - UNO D5 */
	PORTF.DIRSET = PIN5_bm | PIN4_bm; /*(1<<5) | (1<<4);   0x30; */		/* PF5 - UNO D3, PF4 UNO D6 */
 }

void USART3_init(void) {
	PORTB.DIRCLR = PIN5_bm;  
	PORTB.DIRSET = PIN4_bm;  
	USART3.BAUD = (uint16_t)USART3_BAUD_RATE(TARGET_BAUD_RATE); 
	USART3.CTRLB = (USART_TXEN_bm | USART_RXEN_bm);  
	PORTMUX.USARTROUTEA |= PORTMUX_USART3_ALT1_gc; 
	USART3.CTRLA = USART_TXCIE_bm;
}

void Initialise_TCA0_SS_PWM()
{
	PORTA.DIRSET = PIN0_bm;  /* Set PA0 as input */
	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;  /* Set TCA0 to Single Slope PWM (CTRLB) */
	TCA0.SINGLE.PER = 24999;  /* Set TCA0.SINGLE.PER or PERBUF for 50Hz PWM frequency (24999) */
	TCA0.SINGLE.CMP0BUF = 1250;  /* Set TCA0.SINGLE.CMP0 for nominal -90degrees initial position – On time = 1ms */
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;  /* Timer/Counter TCA0 Clock Source: CLK_PER divided by 16 and TCA0 enabled (CTRLA) */
}

 void Initialise_EVSYS()
 {
	/* EVSYS.CHANNEL0: connect the ultrasonic sensor to event channel 0 */
	EVSYS.CHANNEL0 = EVSYS_GENERATOR_PORT1_PIN0_gc;  /* Connect user to event channel 0  */
    EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL0_gc;      /* TCB0 is the Channel 0 User */
	
	/* EVSYS.CHANNEL4: connect the 555 timer input to Event channel 4 */
	EVSYS.CHANNEL4 = EVSYS_GENERATOR_PORT0_PIN3_gc;  /* PORTE Pin 3 -> Channel 4 */
    EVSYS.USERTCB2 = EVSYS_CHANNEL_CHANNEL4_gc;      /* TCB2 uses Channel 4  */
	EVSYS.USERTCB1 = EVSYS_CHANNEL_CHANNEL4_gc;      /* TCB1 uses channel 4*/
	
	/* EVSYS.CHANNEL2: TCB3 overflow triggers ADC conversions */
	EVSYS.CHANNEL2 = EVSYS_GENERATOR_TCB3_CAPT_gc;  /* TCB3 overflow goes to channel 2 */
    EVSYS.USERADC0 = EVSYS_CHANNEL_CHANNEL2_gc;     /* ADC0 uses channel 2 as trigger source  */
}

 void Initialise_TCB0_ICP_PW()
 {
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  /* Enable TCB0 and set CLK_PER divider to 2: Timer clock = 10MHz now */
	TCB0.CTRLB = TCB_CNTMODE_PW_gc;                      /* Configure TCB0 in Input Capture Pulse Width mode */
 	TCB0.INTCTRL = TCB_CAPT_bm;                          /* Enable Capture or Timeout interrupt */
 	TCB0.EVCTRL = TCB_CAPTEI_bm;                         /* Enable Event Input and Event Edge, Rising Edge selected */
 }
 
 void Initialise_TCB1_ICP_PWFRQ()
 {
	 TCB1.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  /* CLK_PER/2 and enable TCB1 */
	 TCB1.CTRLB = TCB_CNTMODE_FRQPW_gc;                   /* Frequency and Pulse Width measurement mode */
	 TCB1.INTCTRL = TCB_CAPT_bm;                          /* Enable Capture interrupt */
	 TCB1.EVCTRL = TCB_CAPTEI_bm | TCB_EDGE_bm;           /* Capture Event Input, Rising edge */
 }
 
 void Initialise_TCB2_ICP_PWFRQ()
 {
	 TCB2.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  /* Enable TCB2 and set CLK_PER divider to 2: Timer clock = 10MHz now */
	 TCB2.CTRLB = TCB_CNTMODE_FRQPW_gc;                   /* Configure TCB0 in Input Capture Clock Frequency Measurement mode */
	 TCB2.INTCTRL = TCB_CAPT_bm;                          /* Enable Capture or Timeout interrupt */
	 TCB2.EVCTRL = TCB_CAPTEI_bm;                         /* Enable Event Input and Event Edge, Rising Edge selected */
 }
 
 void TCB3_init(void)
 {
	  /* enable overflow interrupt */
	 TCB3.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  /* PER divided by 2 and Enable the TCB0 */
	 TCB3.CTRLB = TCB_CNTMODE_INT_gc;                     /* Periodic Interrupt Mode */
	 TCB3.CCMP = (F_CPU / 2) / 200;                       /* Set TCB3.CCMP for 5ms interrupt rate */
	 TCB3.INTCTRL = TCB_CAPT_bm;                          /* Enable the interrupt */ 
 }
 
 void ADC0_init(void)
 {
	 ADC0.CTRLA = 0b00000000;    /* CTRLA: 10-bit resolution selected, Free Running Mode NOT selected, ADC0 not enabled yet */
	 ADC0.CTRLB = 0b00000000;    /* CTRLB: Simple No Accumulation operation selected, this line could be omitted */
	 ADC0.CTRLC = 0b01010110;    /* CTRLC: SAMPCAP=1; REFSEL: VDD; PRESC set to DIV128 */
	 ADC0.CTRLD = 0b00100000;    /* CTRLD: INITDLY set to 16 CLK_ADC cycles */
	 ADC0.MUXPOS = 0b00000011;   /* MUXPOS: Select AIN3 (shared with PORTD3), decision based on the Shield and adapters we use */
	 
	 ADC0.EVCTRL = 0b00000001;   /* EVCTRL: STARTEI set to 1  */
	 ADC0.INTCTRL = 0b00000001;  /* INTCTRL: Enable an interrupt when conversion complete (RESRDY) */
	 ADC0.CTRLA |= 0b00000001;	 /* Enable ADC0 and leave the other CTRLA bits unchanged, note |= */
}
 
 int main(void)
{
    char ch;
    char str_buffer[60];
    uint8_t continuousDistance = 0;
    uint8_t continuousTime = 0;
    uint8_t continuousVolts = 0;

    ServoFollowADC = 0;
    newDistanceData = 0;
    newTimeData = 0;
    newADC0Data = 0;

    /////////Initialization Section/////////

    CLOCK_init();                 /* Sets system clock to 20 MHz */
    InitialiseLED_PORT_bits();    /* set UNO D0-D7 to all outputs, also LED8 and LED9 */
    Set_Clear_Ports(0);           /* Initialize LEDS to all OFF */
	
    Initialise_TCA0_SS_PWM();     /* set up TCA0 for PWM*/
    Initialise_EVSYS();           /* set up event system for ADC and timers*/
    USART3_init();                /* Initialize USAET3 for serial communication*/
	Initialise_TCB0_ICP_PW();     /* Set up TCA0 for ultrasonic sensor*/
	Initialise_TCB1_ICP_PWFRQ();  /* Set up TCB1 for 555 timeout detection*/
    Initialise_TCB2_ICP_PWFRQ();  /* Set up TCB2 for frequency and pulse width*/ 
    TCB3_init();                  /* Set up TCB3 for 5ms periodic timer*/
    ADC0_init();                  /* Initialize ADC0 for voltage input*/

    sei();                        /* Enable Global Interrupts */

    while (1) {
        if (USART3.STATUS & USART_RXCIF_bm) {  // USART Character Handling
            ch = USART3.RXDATAL;

            switch (ch) {
                case 'a':  // Reports the ADC0 raw value
                case 'A':
                    sprintf(str_buffer, "ADC0 RES = %d\n", adc_reading);
                    sendmsg(str_buffer);
                    break;
                case 'v':  // Report the ADC conversion result
                case 'V':
				{
	                uint16_t mV = ((uint32_t)adc_reading * 5000) / 1023;
	                sprintf(str_buffer, "Voltage = %u mV\n", mV);
	                sendmsg(str_buffer);
	                break;
                }
                case 't':  // Reports the period of the 555 timer
                case 'T':
				{
                        if (periodTime > 0) {
	                        uint8_t duty = (pulseTime * 100UL) / periodTime;
							periodTime = periodTime / 10;
	                        sprintf(str_buffer, "555 Timer:\nPeriod = %u us\nPulse = %u us\nDuty = %u%%\n",
	                        periodTime, pulseTime, duty);
	                        sendmsg(str_buffer);
	                    } else {
	                        sendmsg("555 Timer: Invalid period\n");
                        }
                }
                    break;
                case 'H':  // Reports the high pulse of the 555 timer
                case 'h':
                    {
	                    uint16_t high_us = high_time / 10;  // Convert ticks to us
	                    char msg[32];
	                    snprintf(msg, sizeof(msg), "High Time: %u µs\n", high_us);
	                    sendmsg(msg);
	                    break;
                    }
                    break;
                case 'L':  // Reports the low pulse of the 555 timer
                case 'l':
                    {
	                    uint16_t low_us = low_time / 10;
	                    char msg[32];
	                    snprintf(msg, sizeof(msg), "Low Time: %u µs\n", low_us);
	                    sendmsg(msg);
	                    break;
                    }
                    break;
                case 'd':  // Reports the distance detected by ultrasonic sensor
                case 'D':
				{
                    
                    uint16_t distance_cm = pulseTime / 58;
                    sprintf(str_buffer, "Distance = %u mm\n", distance_cm);
                    sendmsg(str_buffer);
                    break;
				}
                case 's':  // Continuously reports distance of ultrasonic sensor
                case 'S':
                    continuousDistance = 1;
                    sprintf(str_buffer, "Continuous Distance ON\n");
                    sendmsg(str_buffer);
                    break;
                case 'u':  // Stops continuously reporting the distance
                case 'U':
                    continuousDistance = 0;
                    sprintf(str_buffer, "Continuous Distance OFF\n");
                    sendmsg(str_buffer);
                    break;
                case 'c':  // Continuously reports the timer input period
                case 'C':
                    continuousTime = 1;
                    sprintf(str_buffer, "Continuous Time ON\n");
                    sendmsg(str_buffer);
                    break;
                case 'e':  // Stops continuously reporting the timer input period
                case 'E':
                    continuousTime = 0;
                    sprintf(str_buffer, "Continuous Time OFF\n");
                    sendmsg(str_buffer);
                    break;
                case 'm':  // Continuously report the ADC result
                case 'M':
                    continuousVolts = 1;        
					sprintf(str_buffer, "Continuous Volts ON\n");    
                    sendmsg(str_buffer);
                    break;
                case 'n':  // Stops continuously reporting the ADC result
                case 'N':
                    continuousVolts = 0;
                    sprintf(str_buffer, "Continuous Volts OFF\n");
                    sendmsg(str_buffer);
                    break;
                case 'f':  // Set the Servo mode to follow ADC0 RES
                case 'F':
                    ServoFollowADC = 1;
					sprintf(str_buffer, "Servo set to follow mode\n");
					sendmsg(str_buffer);
                    break;
                case 'g':  // Set the Servo mode move at a user selected speed
                case 'G':
                    ServoFollowADC = 0;
					sprintf(str_buffer, "Servo set to step mode\n");
					sendmsg(str_buffer);
                    break;
                case '0':  // No movement
                    step_delay = 0xFFFF;
                    break;
                case '1':  // 1.0s per step
                    step_delay = 200;
                    break;
                case '2':  // 0.75s per step
                    step_delay = 150;
                    break;
                case '3':  // 0.5s per step
                    step_delay = 100;
                    break;
                case '4':  // 0.4s per step
                    step_delay = 80;
                    break;
                case '5':  // 0.25s per step
                    step_delay = 50;
                    break;
                case '6':  // 0.2s per step
                    step_delay = 40;
                    break;
                case '7':  // 0.15s per step
                    step_delay = 30;
                    break;
                case '8':  // 0.10s per step
                    step_delay = 20;
                    break;
                case '9':  // 0.05s per step
                    step_delay = 10;
                    break;
                    break;
                default:
                    sprintf(str_buffer, "Unrecognized input: %c\n", ch);
                    sendmsg(str_buffer);
                    break;
            }
        }
        if (continuousDistance && newDistanceData) {  /* Report ultrasonic sensor distance if new data available*/
	        newDistanceData = 0;
	        uint16_t distance_cm = pulseTime / 58;
	        sprintf(str_buffer, "Pulse Width = %u us\nDistance = %u mm\n", pulseTime, distance_cm);
	        sendmsg(str_buffer);
        }

        else if (continuousTime && newTimeData) {  /* Report 555 timer pulse period/frequency if new data available*/
	        newTimeData = 0;
	        if (periodTime > 0) {
		        uint8_t duty = (pulseTime * 100UL) / periodTime;
				periodTime = periodTime / 10;
		        sprintf(str_buffer,
		        "555 Timer:\nPeriod = %u us\nPulse = %u us\nDuty = %u%%\n",
		        periodTime, pulseTime, duty);
		        sendmsg(str_buffer);
		        } else {
		        sendmsg("555 Timer: Invalid period\n");
	        }
        }
        else if (continuousVolts && newADC0Data) {  /* Report ADC voltage if new data available*/
	        newADC0Data = 0;
	        uint16_t mV = ((uint32_t)adc_reading * 5000) / 1023;
	        sprintf(str_buffer, "ADC0 = %u (%u mV)\n", adc_reading, mV);
	        sendmsg(str_buffer);
        }
    }
}

/* sendmsg function */
void sendmsg(char* s) {
	 if (qcntr == sndcntr) {
		 qcntr = 0;
		sndcntr = 1;
		while (*s)
			queue[qcntr++] = *s++;
		USART3.TXDATAL = queue[0];
	 }
 }
 
 
/* Function to set or clear all LED port bits, 1 - set, 0 - clear */
void Set_Clear_Ports(uint8_t set) {
 
	for (int i = 0; i < 10; i++)
	{
		if (set)
			LED_Array[i].LED_PORT->OUTSET = LED_Array[i].bit_mapping;
		else
			LED_Array[i].LED_PORT->OUTCLR = LED_Array[i].bit_mapping;
	}
 }
 
/* ****************************************************************/ 
/* Interrupt Service Routines */
/* ****************************************************************/  

ISR(TCB0_INT_vect)
 {
	TCB0.INTFLAGS = TCB_CAPT_bm;  /* Clear the interrupt flag */
 
	pulseTime = TCB0.CCMP;        /* capture the width of the pulse which id proportional to the distance*/ 
	
	newDistanceData = 1;          /* signal that new distance data is available*/
 }
 
 ISR(TCB1_INT_vect)
 {
	TCB1.INTFLAGS = TCB_CAPT_bm;    /* Clear interrupt flag*/

	 uint16_t clocksT = TCB1.CNT;  /* Total period (clocks)*/
	 uint16_t clocksP = TCB1.CCMP;   /* High pulse width (clocks)*/

	 pulseTime = clocksP;
	 periodTime = clocksT;
	 high_time = clocksP;
	 low_time = clocksT - clocksP;
	 
	 newTimeData = 1;               /* signal that new timer data is available*/
	 
	 if (periodTime == 0) {         /* if period is 0, the 555 timer stops oscillating*/
		 PORTF.OUTSET = PIN4_bm;    /* Turn ON LED (PORTF pin 4)*/
	 } else {
		 PORTF.OUTCLR = PIN4_bm;    /* Otherwise, turn OFF the LED*/
	 }
 }
 
 ISR(TCB2_INT_vect)
 {
	 uint16_t clocksP, clocksT, Time_Period;

	 TCB2.INTFLAGS = TCB_CAPT_bm;  /* Clear interrupt flag*/

	 clocksT = TCB2.CNT;           /* In PW - Freq mode, CNT stops on the trailing edge until CCMP is read.*/
	 clocksP = TCB2.CCMP;          /* Capture the value when the pulse ends (on the trailing edge)*/

	 /* Convert the captured counts to microseconds based on the system clock*/
	 Pulse_Width = clocksP / 10;   
	 Time_Period = clocksT / 10;  

	 newTimeData = 1;              /* Signal that new data is available */

	 /* LED5 ON if period > 300us, otherwise OFF*/
	 if (Time_Period > 300) {
		 PORTB.OUTSET = (1 << 2);  /* Turn on LED5*/
		 } else {
		 PORTB.OUTCLR = (1 << 2);  /* Turn off LED5*/
	 }

	 PORTF.OUTCLR = (1 << 4);      /* Turn off LED at PORTF Pin 4 (bit 4)*/
 }

ISR(TCB3_INT_vect) {
	TCB3.INTFLAGS = TCB_CAPT_bm;   /* Clear interrupt*/

	static uint8_t trigger_counter = 0;
	
	/* Send a 10µs pulse every 50ms*/
	if (trigger_counter++ >= 10) {
		trigger_counter = 0;
		PORTC.OUTSET = PIN6_bm;    /* Trigger pin high*/
		_delay_us(10);
		PORTC.OUTCLR = PIN6_bm;    /* Trigger pin low*/
	} 

	/* If not in ADC follow mode, servo moves based on step timing*/
	if (ServoFollowADC == 0) {
		static uint16_t servo_counter = 0;
		static uint8_t direction = 1;
		
		if (++servo_counter >= step_delay) {
			servo_counter = 0;

			/* Oscillating between 0 and 25 steps*/
			if (direction) {
				servo_pos++;
				if (servo_pos >= 25) direction = 0;  /* Reverse at top*/
				} else {
				if (servo_pos > 0) servo_pos--;
				if (servo_pos == 0) direction = 1;   /* Reverse at bottom*/
			}


			/* Map servo_pos (0–25 steps) to PWM (1250–2500 µs)*/
			TCA0.SINGLE.CMP0BUF = 1250 + (servo_pos * 50);  
		}
		
	}
}

 ISR(ADC0_RESRDY_vect)
 {
	 adc_reading = ADC0.RES; /* Store the latest ADC result*/ 
	 newADC0Data = 1;        /* Signal new data is available*/
	 
	 /* Set the LED[7] on/off based on the adc_reading */
	 if (adc_reading > 716){
		 LED_Array[7].LED_PORT->OUTSET = LED_Array[7].bit_mapping;
	 } else {
		 LED_Array[7].LED_PORT->OUTCLR = LED_Array[7].bit_mapping;
	 }
	 
	 /* If in ADC follow mode, update servo PWM position*/
	 if (ServoFollowADC) {
		uint16_t cmp = 1250 + (adc_reading * 5) / 4;
		TCA0.SINGLE.CMP0BUF = cmp;
	}
}
 
ISR(USART3_TXC_vect) {
	  USART3.STATUS |= USART_TXCIF_bm;
	  if (qcntr != sndcntr)
	  USART3.TXDATAL = queue[sndcntr++];
 }

