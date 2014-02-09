#define SONAR_PIN         4 

#define SONAR_INACTIVE    0
#define SONAR_ACTIVE      1
#define SONAR_PULSE_START 2
#define SONAR_DATA_AVAIL  3

#define SMOOTH_SONAR_DATA 1
#ifdef SMOOTH_SONAR_DATA
  #define SMOOTH_FACTOR   0.96
#endif

//#define DEBUG             1

volatile unsigned long start_time;
volatile unsigned long echo_time;
volatile float new_distance;
volatile short sonar_state;

float gdt_ms; /* In microseconds */
unsigned long g_last_us;

unsigned int sonar_period_timer_ms;
unsigned long sonar_last_ms;

float sonar_distance;

void setup()
{
  sonar_init();
  Serial.begin(115200);
  
  sonar_state = SONAR_INACTIVE;
  g_last_us = micros();
  sonar_last_ms = millis();
}

void loop()
{
  gdt_ms = (micros() - g_last_us)/1000.0;
  g_last_us = micros();
  
  sonar_period_timer_ms = millis() - sonar_last_ms;
  
  /* Sonar update */
  if(sonar_state == SONAR_INACTIVE || sonar_period_timer_ms > 15)
  {
    sonar_pulse();
    sonar_last_ms = millis();
  } 
  else if(sonar_state == SONAR_DATA_AVAIL) /* Output available */
  {
    #ifdef SMOOTH_SONAR_DATA
      sonar_distance = SMOOTH_FACTOR*sonar_distance + (1 - SMOOTH_FACTOR)*new_distance;
    #else
      sonar_distance = new_distance;
    #endif
    
    Serial.println(sonar_distance);
    
    sonar_state = SONAR_INACTIVE;
  }
  
  //Serial.println(gdt_ms);
}

void sonar_pulse()
{
  pinMode(SONAR_PIN, OUTPUT);
  PORTD &= ~(1 << SONAR_PIN); /* Drive pin LOW */
  delayMicroseconds(2);
  PORTD |= (1 << SONAR_PIN);  /* Drive pin HIGH */
  delayMicroseconds(5);
  PORTD &= ~(1 << SONAR_PIN); /* Drive pin LOW */ 
  pinMode(SONAR_PIN, INPUT);
    
  sonar_state = SONAR_ACTIVE;
}

void sonar_init()
{
  PCMSK2 = _BV(PCINT20);
  PCICR = _BV(PCIE2); 
}

ISR(PCINT2_vect)
{
  if(((PIND >> SONAR_PIN) & 0x1) > 0 && (sonar_state == SONAR_ACTIVE))
  {
    start_time = micros();
    sonar_state = SONAR_PULSE_START; 
  }
  else if(((PIND >> SONAR_PIN) & 0x1) == 0 && (sonar_state == SONAR_PULSE_START))
  {
    echo_time = micros() - start_time;
    new_distance = echo_time / 58;
  
    sonar_state = SONAR_DATA_AVAIL; /* Measurement found */
  }
}

