#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

#define EV_ENABLE_PORT_DIR  DDRD
#define EV_ENABLE_STATE     PORTD
#define EV_ENABLE_PIN       PIN7

#define EV_IN1_PORT_DIR     DDRB
#define EV_IN1_STATE        PORTB
#define EV_IN1_PIN          PIN0

#define EV_IN2_PORT_DIR     DDRB
#define EV_IN2_STATE        PORTB
#define EV_IN2_PIN          PIN1




volatile uint8_t nbrIntTIMER2 = 0;
volatile uint8_t nbr_ms = 0;
volatile uint8_t nbr_sec = 0;
volatile uint8_t nbr_min = 0;
volatile uint8_t nbr_heure = 0;

volatile uint8_t ticks = 0; // 0 0 0 0 0 0 0 0
//                                      h m s ms

volatile uint8_t ev_state = 0x00; // 0x00 : close / 0x01 : open

//use for RTC clock

inline void initTimer2(void){
    //Disable timer2 interrupts
    TIMSK2  = 0x00;
    //set initial counter value
    TCNT2=0x00;
    //set prescaller 1024
    TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20);

    //clear interrupt flags
    TIFR2 = (1<<TOV2);
    
    //enable TOV2 interrupt
    TIMSK2 |= (1<<TOIE2);
}



void openEV(void){
    EV_ENABLE_STATE |= (1<<EV_ENABLE_PIN);     // 1 : wakeup
    _delay_us(40);
    //Break
    EV_IN1_STATE |= (1<<EV_IN1_PIN);           // 1 
    EV_IN2_STATE |= (1<<EV_IN2_PIN);           // 1  
    _delay_ms(15);
    EV_ENABLE_STATE &=~ (1<<EV_ENABLE_PIN);    // 0 : sleep
    ev_state = 0X01;
    
}
void closeEV(void){
    EV_ENABLE_STATE |= (1<<EV_ENABLE_PIN);     // 1 : wakeup
    _delay_us(40);
    //reverse
    EV_IN1_STATE &=~ (1<<EV_IN1_PIN);          // 0 
    EV_IN2_STATE |= (1<<EV_IN2_PIN);           // 1
    _delay_ms(15);
    EV_ENABLE_STATE &=~ (1<<EV_ENABLE_PIN);    // 0 : sleep
    ev_state = 0x00;
}

ISR(WDT_vect){
    
    nbr_ms++;
    ticks ^= (1<<0x01);
    
    if (nbr_ms == 1) {
        nbr_ms = 0x00;
        nbr_sec++;
        ticks ^= (1<<0x02);
        if (nbr_sec == 60) {
            nbr_sec = 0;
            nbr_min++;
            ticks ^= (1<<0x04);
            if (nbr_min == 60) {
                nbr_min = 0;
                nbr_heure++;
                ticks ^= (1<<0x08);
                if (nbr_heure == 24) {
                    nbr_heure =0;
                }
            }
        }
    }
}

ISR(TIMER2_OVF_vect){
    

}
void goto_sleep(void){
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
}

void initWDT(void){
    //disable interrupts
    cli();
    //reset watchdog
    wdt_reset();
    //set up WDT interrupt
    WDTCSR = (1<<WDCE)|(1<<WDE);
    //Start watchdog timer with 1s prescaller
    WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP2)|(1<<WDP1);
}

void initIO(void) {
	DDRB = 0x00;
    DDRC = 0x00;
    DDRD = 0x00;
    
    PORTB = 0X00;
    PORTC = 0X00;
    PORTD = 0X00;
    
    EV_ENABLE_PORT_DIR |= (1<<EV_ENABLE_PIN);
    EV_IN1_PORT_DIR |= (1<<EV_IN1_PIN);
    EV_IN2_PORT_DIR |= (1<<EV_IN2_PIN);
    
    EV_ENABLE_STATE &=~ (1<<EV_ENABLE_PIN); // 0 : sleep
    EV_IN1_STATE |= (1<<EV_IN1_PIN); // 1
    EV_IN2_STATE |= (1<<EV_IN2_PIN); // 1
    
    PRR = (1<<PRTWI) | (1<<PRTIM0) | (1<<PRTIM1) | (1<<PRTIM2) | (1<<PRSPI) | (1<<PRUSART0) | (1<<PRADC);
    
}


int main(void) {
	initIO();
    initWDT();
    closeEV();
	
    sei();
	while (1) {
        
        if (ticks & (1<<0X02)) {
            if (ev_state) {
                closeEV();
            }
            else if (!ev_state){
                openEV();
            }
        }
        
        goto_sleep();
	}
	return 0; // never reached
}
