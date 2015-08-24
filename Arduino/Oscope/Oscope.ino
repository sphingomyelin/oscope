// Oscilloscope
// Input defined by ANALOG_IN
// SIG_OUT true puts a 1Hz wave on DIGITAL_OUT for testing.

// defined for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define ANALOG_IN 1
#define DIGITAL_OUT 13
bool SIG_OUT = true;

void setup() {
  Serial.begin(2000000);
  
  pinMode(ANALOG_IN, INPUT);

  // initialize ADC, default is a 128 prescaler
  noInterrupts();
  // set prescaler to 64 - 250kHz ADC clock
  /*sbi(ADCSRA,ADPS2); // 1 1 0
  sbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);*/
  // set prescaler to 32 - 500kHz ADC clock, for better precision
  sbi(ADCSRA,ADPS2); // 1 0 1
  cbi(ADCSRA,ADPS1);
  sbi(ADCSRA,ADPS0);
  // set prescaler to 16 - 1MHz ADC clock, still works, but precision decaying
  /*sbi(ADCSRA,ADPS2); // 1 0 0
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);*/
  interrupts();
  
  // Generate a signal to examine (testing)
  if(SIG_OUT){
    pinMode(DIGITAL_OUT, OUTPUT);
    
    // initialize timer1 
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
  
    OCR1A = 31250;            // compare match register 16MHz/256/1Hz / 2 (toggle)
    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS12);    // 256 prescaler 
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    interrupts();             // enable all interrupts
  }
}

// Interrupt based
ISR(TIMER1_COMPA_vect){
  digitalWrite(DIGITAL_OUT, digitalRead(DIGITAL_OUT) ^ 1);
}

void loop() {
  int val = analogRead(ANALOG_IN);
  long time = micros();
  Serial.write( 0xff );
  Serial.write( (val >> 8) & 0xff );
  Serial.write( val & 0xff );
  Serial.write( (time >> 24) & 0xff );
  Serial.write( (time >> 16) & 0xff );
  Serial.write( (time >>  8) & 0xff );
  Serial.write( time & 0xff );
//  Serial.write( 0x00 ); // Not necessarily needed. Do stats to know. --> didn't happen in 30s operation
//  delayMicroseconds(10);
}

