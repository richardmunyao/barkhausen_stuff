int BUSY_PIN = 7; //PC23
//int CHIP_SELECT_PIN = 3; //PC28
int CHIP_SELECT_PIN = 4;
int READ_PIN = 2; //PB25
int RESET_PIN = 6; //PC24
int START_PIN = 5; //PC25
int OVERSAMPLING_PIN_0 = 10; //PC29;PA28
int OVERSAMPLING_PIN_1 = 9; //PC21
int OVERSAMPLING_PIN_2 = 8; //PC22
uint32_t buff [4];
byte task = 1;


void setup() {
SerialUSB.begin(0);
// Serial.begin(9600);

//initialize important pins:
pinMode(RESET_PIN, OUTPUT);
pinMode(BUSY_PIN, INPUT);
pinMode(CHIP_SELECT_PIN, OUTPUT);
pinMode(START_PIN, OUTPUT);
pinMode(OVERSAMPLING_PIN_0, OUTPUT);
pinMode(OVERSAMPLING_PIN_1, OUTPUT);
pinMode(OVERSAMPLING_PIN_2, OUTPUT);
pinMode(READ_PIN, OUTPUT);
pinMode(12, OUTPUT);

for(int pin = 33; pin < 42; pin++){
pinMode(pin, INPUT);
}

for(int pin = 44; pin < 51; pin++){
pinMode(pin, INPUT);
}

//no over-sampling:
digitalWrite(OVERSAMPLING_PIN_0, LOW);
digitalWrite(OVERSAMPLING_PIN_1, LOW);
digitalWrite(OVERSAMPLING_PIN_2, LOW);

//turn others off too:
digitalWrite(RESET_PIN, LOW);
digitalWrite(BUSY_PIN, LOW);
digitalWrite(CHIP_SELECT_PIN, LOW);
digitalWrite(START_PIN, LOW);
digitalWrite(READ_PIN, LOW);

//do reset: The RESET high pulse should typically be 50 ns wide.
digitalWrite(RESET_PIN, HIGH);
delay(1);
digitalWrite(RESET_PIN, LOW);

/************** Generate clock pulses for CONV_START **************/

  /*Initialization of the timer 6 */
  pmc_set_writeprotect(false);  //Turn off write protection on Power Management Controller
  pmc_enable_periph_clk(TC6_IRQn);  // Enable the  peripheral clock by IRQ for Timer 2 channel 0
  pmc_enable_periph_clk(TC7_IRQn);  // Enable the  peripheral clock by IRQ for Timer 2 channel 1
  
  /* Configuration of timer TC2 channel 0 => timer 6 */
  TcChannel * t6 = &(TC2->TC_CHANNEL)[0] ;    // pointer creation on register TC0 channel 0
                                             
  t6->TC_CCR = TC_CCR_CLKDIS ;  // deactivation of the clocks during setting time
  t6->TC_IDR = 0xFFFFFFFF ;     // disable interrupts
  t6->TC_SR ;                   // read status for timer reset
  t6->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |   // use TCLK1 (division by 2, F = 42MHz)
              TC_CMR_WAVE |         // curve mode
              TC_CMR_WAVSEL_UP_RC | // PWM counter using the RC register
              TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
              TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;
              
int val = (42000/100);// Frequency definition: 42 MHz / TC_RC. Divisor is freq in KHz
  t6->TC_RC = val;     
  t6->TC_RA = (val * 0.95);      // Definition of the duty ratio, with RA = 0.95 RC
 
  t6->TC_CMR = (t6->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ; // set and reset flags for comparison of RA and RC registers                    
  t6->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // reactivation of the clock.
  /* Configuration of I/O for Timer6 : */
  int ulPin6 = 5; // Timer6 -> PWM Pin 5 (START_PIN)
  PIO_Configure(g_APinDescription[ulPin6].pPort,
  g_APinDescription[ulPin6].ulPinType,
  g_APinDescription[ulPin6].ulPin,
  g_APinDescription[ulPin6].ulPinConfiguration);
/************** End Generate clock pulses for CONV_START **************/


/************** Generate clock pulses for READ_PIN **************/

/* Configuration du timer TC0 channel 0 => timer 0 */
  TcChannel * t0 = &(TC2->TC_CHANNEL)[1] ;    // creation du pointeur sur le registre TC0 channel 0
                                             
  t0->TC_CCR = TC_CCR_CLKDIS ;  // désactivation des horloges le temps des reglages
  t0->TC_IDR = 0xFFFFFFFF ;     // desactivation des interruptions
  t0->TC_SR ;                   // lecture status pour reinitialisation du timer
  t0->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |   // utilisation TCLK1 (division par 2, F = 42MHz)
              TC_CMR_WAVE |         // mode courbe
              TC_CMR_WAVSEL_UP_RC | // compteur PWM utilisant le registre RC
              TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
              TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;
             
  t0->TC_RC = val;     // Definition de la fréquence : 42 MHz / TC_RC
  t0->TC_RA = (val * 0.015);      // Definition du rapport cyclique, avec RA = 1/2 RC
 
  t0->TC_CMR = (t0->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ; // set et reset des flags pour la comparaison des registres RA et RC

  t6->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // reactivation de l horloge.  
  int ulPin0 = 3; // Timer0 -> PWM Pin 2
  t0->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // reactivation de l horloge.   !!!THIS!!  
  PIO_Configure(g_APinDescription[ulPin0].pPort,
  g_APinDescription[ulPin0].ulPinType,
  g_APinDescription[ulPin0].ulPin,
  g_APinDescription[ulPin0].ulPinConfiguration); 

  

  /************** End Generate clock pulses for READ_PIN **************/

}

void loop() {
// put your main code here, to run repeatedly:

while(1){
  if( !!(g_APinDescription[BUSY_PIN].pPort -> PIO_PDSR & g_APinDescription[BUSY_PIN].ulPin) == 1  ){
//g_APinDescription[12].pPort -> PIO_SODR = g_APinDescription[12].ulPin; //Turn on pin 12; for troubleshooting
// -------------- read and send loop starts here -------------//
if (task == 1){
//REG_PIOB_SODR |= (0x01 << 25); //high (READ_PIN (pin 2 -> B25))
//g_APinDescription[2].pPort -> PIO_CODR = g_APinDescription[2].ulPin;
//REG_PIOB_CODR |= (0x01 << 25); //low
g_APinDescription[12].pPort -> PIO_SODR = g_APinDescription[12].ulPin; //Turn on pin 12; for troubleshooting
buff[0] = 0x80000000;
buff[0] |= ( (REG_PIOC_PDSR << 1) >> 1);

//REG_PIOB_SODR |= (0x01 << 25); //high (READ_PIN (pin 2 -> B25))
//g_APinDescription[2].pPort -> PIO_CODR = g_APinDescription[2].ulPin;
buff[1] = 0x80000000;
buff[1] |= ( (REG_PIOC_PDSR << 1) >> 1);
//
//REG_PIOB_SODR |= (0x01 << 25); //high (READ_PIN (pin 2 -> B25))
//g_APinDescription[2].pPort -> PIO_CODR = g_APinDescription[2].ulPin;
buff[2] = 0x80000000;
buff[2] |= ( (REG_PIOC_PDSR << 1) >> 1);

//REG_PIOB_SODR |= (0x01 << 25); //high (READ_PIN (pin 2 -> B25))
//g_APinDescription[2].pPort -> PIO_CODR = g_APinDescription[2].ulPin;
buff[3] = 0x80000000;
buff[3] |= ( (REG_PIOC_PDSR << 1) >> 1);
//
//REG_PIOB_SODR |= (0x01 << 25); //high (READ_PIN (pin 2 -> B25))
//g_APinDescription[2].pPort -> PIO_CODR = g_APinDescription[2].ulPin;
//buff[4] = 0x80000000;
//buff[4] |= ( (REG_PIOC_PDSR << 1) >> 1);
//
//REG_PIOB_SODR |= (0x01 << 25); //high (READ_PIN (pin 2 -> B25))
//g_APinDescription[2].pPort -> PIO_CODR = g_APinDescription[2].ulPin;
//buff[5] = 0x80000000;
//buff[5] |= ( (REG_PIOC_PDSR << 1) >> 1);
//
//REG_PIOB_SODR |= (0x01 << 25); //high (READ_PIN (pin 2 -> B25))
//g_APinDescription[2].pPort -> PIO_CODR = g_APinDescription[2].ulPin;
//buff[6] = 0x80000000;
//buff[6] |= ( (REG_PIOC_PDSR << 1) >> 1);
//
//REG_PIOB_SODR |= (0x01 << 25); //high (READ_PIN (pin 2 -> B25))
//g_APinDescription[2].pPort -> PIO_CODR = g_APinDescription[2].ulPin;
//buff[7] = 0x80000000;
//buff[7] |= ( (REG_PIOC_PDSR << 1) >> 1);

SerialUSB.write( (uint8_t*)buff, sizeof(buff) );
SerialUSB.write(0x2A); // asterisk *
//REG_PIOB_SODR |= (0x01 << 25); //high (READ_PIN (pin 2 -> B25))
//g_APinDescription[2].pPort -> PIO_CODR = g_APinDescription[2].ulPin;
//delayMicroseconds(1);
task = 0;
g_APinDescription[12].pPort -> PIO_CODR = g_APinDescription[12].ulPin; //Turn on pin 12; for troubleshooting
}
else{
  //no task
  }
}
else{
  //reset task
  task = 1;
  }

// --------------- end read and send loop -------------//
//g_APinDescription[12].pPort -> PIO_CODR = g_APinDescription[12].ulPin;  //Turn off pin 12; for troubleshooting
  }

}

