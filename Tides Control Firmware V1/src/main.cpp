#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include <Arduino.h>
#include <Bounce2.h>

#define BUTTON_DEBOUNCE_DELAY   20   // [ms]

#define  MOTOR    4
#define  PWR_BTN  1
#define  PWR_LED  0
#define  CTR_BTN  2

#define BTN 1
#define timer_init() (TIMSK |= (1 << OCIE0A))
#define BTN_HOLD_MS 500    // Press button for 1 second

Bounce POWER_BTN  = Bounce();
Bounce STRT_BTN   = Bounce();

void run();
void check_btn();
void power_off();
void adc_setup();

int pwr_state = 1;
int run_state = 0;

int motor_spd = 100;
int min_motor_spd = 50;

byte saved_ADCSRA = 0;;

#define BTN 1
#define BTN_STATUS !((PINB >> BTN) & 0x01)



enum Btn_Status
{
    BTN_UP,
    BTN_DOWN,
    BTN_IGNORE
};

volatile unsigned int timer;  // milliseconds counter
Btn_Status btn_status;        // Status of the button

enum Device_Status
{
    POWER_OFF,
    RUNNING
};

Device_Status status = RUNNING; // Set start ON or OFF when power is connected

void setup() {
  // put your setup code here, to run once:

  sei();                  // Enable interrupts
  PORTB |= (1 << BTN);    // Enable PULL_UP resistor
  GIMSK |= (1 << PCIE);   // Enable Pin Change Interrupts
  PCMSK |= (1 << BTN);    // Use PCINTn as interrupt pin (Button I/O pin)
  TCCR0A |= (1 << WGM01); // Set CTC mode on Timer 1
  TIMSK |= (1 << OCIE0A); // Enable the Timer/Counter0 Compare Match A interrupt
  TCCR0B |= (1 << CS01);  // Set prescaler to 8
  OCR0A = 125;            // Set the output compare reg so tops at 1 ms

  //adc_setup();

  btn_status = BTN_UP;

  pinMode(MOTOR, OUTPUT);
  pinMode(PWR_LED, OUTPUT);

  //POWER_BTN.attach( PWR_BTN ,  INPUT_PULLUP );
  //POWER_BTN.interval(5); // interval in ms

  STRT_BTN.attach( CTR_BTN ,  INPUT );
  STRT_BTN.interval(5); // interval in ms

}

void loop() {
  // put your main code here, to run repeatedly:

  if (btn_status == BTN_DOWN)
        {
            if (timer > BTN_HOLD_MS) // Check if button has been pressed enough
            {
                if (status == RUNNING)
                    status = POWER_OFF;
                else
                {
                    status = RUNNING;
                    // setup of the device here if needed;
                    ADCSRA |= _BV(ADEN);   
                }
                btn_status = BTN_IGNORE; // If status already changed don't swap it again
            }
        }
        else
        {
            if (status) // Is status RUNNING?
            {
                /* main code here */

                PORTB |= (1 << PB0); // Pin 0 ON
                

                run();

                /* -------------- */
            }
            else
            {
                PORTB &= ~(1 << PB0); // Pin 0 OFF
                analogWrite(MOTOR, 0);
                motor_spd = 150;
                run_state = 0;
                power_off();
            }
        }

  /*POWER_BTN.update();

  if ( POWER_BTN.changed() ) {

    int deboucedInput = POWER_BTN.read();
    
    if ( deboucedInput == LOW ) {

      pwr_state = (pwr_state + 1)%2;

    } else {

    }
  }

  */
  /*
  if (pwr_state == 1) {

    run();
    digitalWrite(PWR_LED, HIGH);

  } else {

    //go to deep sleep until woken up.
    digitalWrite(PWR_LED, LOW);

    //turn motor off and go to sleep.
    analogWrite(MOTOR, 0);
    run_state = 0;

  }
  */

}


void run() {

  check_btn();

  if (run_state == 1) {

    //run the motor at a given speed.
    analogWrite(MOTOR, motor_spd);

    //change speed based on buttons.

    int button_reading = analogRead(A1);

    if ((button_reading >= 0) && (button_reading < 441)) {

    } else if ((button_reading >= 441) && (button_reading < 603)) { 
      motor_spd += 1;
      if (motor_spd >= 255) motor_spd = 255;
    } else if ((button_reading >= 603) && (button_reading < 766)) {
      motor_spd -= 1;
      if (motor_spd <= min_motor_spd) motor_spd = min_motor_spd +1;;
    } else {

    }

    delay(10);
    
  } else {

    //stop the motor.

    //potentially ramp down here.
    analogWrite(MOTOR, 0);
    motor_spd = 100;

  }

}

void check_btn() {

  STRT_BTN.update();

  if ( STRT_BTN.changed() ) {

    int deboucedInput = STRT_BTN.read();
    
    if ( deboucedInput == LOW ) {

      run_state = (run_state + 1)%2;

    } else {

    }
  }
  
}

void adc_setup()
{
 DDRB|=(1<<PB1);         //PB1 as output to activate LED
 ADCSRA|=(1<<ADEN);      //Enable ADC module
 ADMUX=0x01; // configuring PB2 to take input
 ADCSRB=0x00;           //Configuring free running mode
 ADCSRA|=(1<<ADSC)|(1<<ADATE);   //Start ADC conversion and enabling Auto trigger
 }


void power_off()
{
    cli();                               // Disable interrupts before next commands
    wdt_disable();                       // Disable watch dog timer to save power
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep mode power down
    saved_ADCSRA = ADCSRA;
    //ADCSRA = 0;            // turn off ADC
    ADCSRA &= ~_BV(ADEN);
    sleep_enable();
    sleep_bod_disable(); // Disable brown-out detector
    sei();               // Enable interrupts
    sleep_cpu();
    sleep_disable();
}

ISR(PCINT0_vect)
{
    if (!((PINB >> BTN) & 0x01)) // Check if button is down
    {
        btn_status = BTN_DOWN;
        timer_init();
        timer = 0;
    }
    else
        btn_status = BTN_UP;
}

ISR(TIM0_COMPA_vect)
{
    timer++;
}