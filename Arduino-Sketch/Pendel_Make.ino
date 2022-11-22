#include "DCF77.h"
#include <TimeLib.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define REED 2                  // Pin for the reed relay (counts pendulum ticks)
#define DCF_DATA 3              // Connection pin to DCF 77 device
#define DCF_POWER 4             // Pin to power DCF
#define SECONDS_PER_TICK  2     // Duration of a complete pendulum cycle
#define DAY_SECONDS 86400       // Number of seconds a day
#define DAY_TICKS 86091         // Number of ticks a day (usually = DAY_SECONDS, but not on my watch)
#define CORR_THRESHOLD 5        // Threshold in seconds, before correction starts
#define DCF_INTERRUPT 1         // Interrupt number associated with pin

#define UP_B   B00000111        // Drive PortB 0,1,2 with High - the rest with Low
#define UP_C   B00001100        // Drive PortC 2,3 with High   - the rest with Low     
#define DOWN_B B00111000        // Drive PortB 3,4,5 with High - the rest with Low
#define DOWN_C B00000011        // Drive PortC 0,1 with High   - the rest with Low
#define OFF    B00000000        // Drive all Ports B and C with Low

#define UPTIME    1200          // ms needed, to drive the weight all the way upwards
#define DOWNTIME  1050          // ms needed, to drive the weight all the way downwards

time_t time;
DCF77 DCF = DCF77(DCF_DATA,DCF_INTERRUPT);
time_t DCFtime;

enum State { Sleep, Clock, Adjust };

volatile State state = Sleep;
volatile bool MotorDir = true;
volatile unsigned long NextCycle;
volatile unsigned long PendulumTicks = 0;
volatile int wdtCounter = 0;
volatile int Preller = 0;

unsigned long PendulumTicksOld = 0;
long TimeInSeconds = 0;
long TimeInSecondsOld = 0;
//int SyncSeconds;
float AllTimeDiff = 0;
float AllTimeDiffOld = 0;
int MissedSeconds = 0;

int synchDCF()
{
  bool synched = false;
  int SyncSeconds = 0;
  unsigned long SyncTime;
  digitalWrite(DCF_POWER, HIGH);          // Start DCF

  DCF.Start();
  SyncTime = millis();
  while (!synched)
  {
    time_t DCFtime = DCF.getUTCTime();    // Check if new DCF77 time is available use only UTC, to avoid Summertime<->Wintertime huddle
    if (((millis() - SyncTime) % 1000) == 0)  // This is millis() overflow proof!
    {
      SyncSeconds++;                      // Every 1000ms it took another second for sync
      Serial.print(".");                  // "Live" Signal
      if ((SyncSeconds % 60) == 0)
        Serial.println();
      delay(2);                           // This is done only once a second
    }
    if (DCFtime!=0)
    {                                     // We are in sync!
      Serial.println();                   // check DCF time as often as possible (2ms)
      Serial.println("DCF Time valid");
      setTime(DCFtime);
      synched = true;                     // leave sync function
    }
  }
  digitalClockDisplay();  
  DCF.Stop();
  digitalWrite(DCF_POWER, LOW);           // Stop DCF
  return SyncSeconds;
}

// the setup function runs once when you press reset or power the board
void setup() 
{
  int SyncSeconds;
  int delta = 4;
  Serial.begin(9600);
  Serial.println("Waiting for DCF77 time ... ");
  Serial.println("This will take at least 2 minutes.");

  cli();
  pinMode (REED, INPUT);
  pinMode (DCF_POWER, OUTPUT);
  digitalWrite(DCF_POWER, LOW);           // Disable DCF
    
  DDRB = B00111111;   // PortB 0,1,2,3,4,5 are output
  DDRC = B00001111;   // PortC 0,1,2,3 are output
  PORTB = OFF;        // Motor Off (Output = Low)
  PORTC = OFF;        // Motor Off (Output = Low)
                      // Enable Watchdog IR with 1s duration, to catch missed pendulum ticks.
  wdt_reset();        // Reset Watchdog Timer
  MCUSR &= 0xF7;      // Enable access to WD
  WDTCSR = 0x18;      // Watchdog change enable
  WDTCSR = 0x06;      // Cycle = 1s
  WDTCSR |= 0x40;     // Watchdog Interrupt enable

  NextCycle = DAY_TICKS;                   // Default value before first DCF result
  attachInterrupt(digitalPinToInterrupt(REED), pendelISR, FALLING);
  ADCSRA = ADCSRA & B01111111;              // ADC abschalten, ADEN bit7 zu 0
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  PendulumTicks = 0;                      // start counter at 0 immediately, when DCF time is taken
  sei();
    
  SyncSeconds = synchDCF();               // Time it took, to synchronize
  cli();                                  // Do not interrupt copy process
  PendulumTicksOld = PendulumTicks;       // Save counter immediately, when DCF is taken
  PendulumTicks = 0;                      // start counter at 0 immediately, when DCF time is taken
  sei();
  NextCycle = DAY_TICKS - SyncSeconds;    // Next sync in 24h - DCF sync time
  TimeInSeconds = (((hour() * (unsigned long)60) + minute()) * (unsigned long)60) + second();   // Time in seconds since 0:00

  Serial.print("Sync Seconds    ");         
  Serial.println(SyncSeconds);            // print Sync Seconds
  Serial.print("Pendulum ticks  ");         
  Serial.println(PendulumTicksOld);       // print pendulum counter
  Serial.print("Time in Seconds ");
  Serial.println(TimeInSeconds);          // print Time in seconds. (Überlauf bei 65536 Ticks durch Serial.print)
  Serial.print("Missing Reed: ");
  Serial.println(MissedSeconds);

  delay(100);                             // Wait for Serial.Print, to finish
}

// the loop function runs over and over again forever
void loop() 
{
  static float deltaSeconds;
  int SyncSeconds;
  bool MotorOn;
  
  switch(state)
  {
    case Sleep:
    {
      if (PendulumTicks > NextCycle)
      {
        state = Clock;
        break;
      }
      sleep_enable();
      sleep_mode();
      break;
    }
    case Clock:
    {
      NextCycle = DAY_TICKS;
      SyncSeconds = synchDCF();             // Start DCF again
      cli();                                // Do not interrupt copy process
      PendulumTicksOld = PendulumTicks;     // Save counter immediately, when DCF is taken
      PendulumTicks = 0;                    // and restart counter
      sei();
      NextCycle = DAY_TICKS - SyncSeconds;  // Next sync in 24h - DCF sync time
      Serial.print("Next Cycle: ");
      Serial.println(NextCycle);
      TimeInSecondsOld = TimeInSeconds;     // Save old DCFTime
      TimeInSeconds = (((hour() * (unsigned long)60) + minute()) * (unsigned long)60) + second();   // Time in seconds since 0:00
      deltaSeconds = TimeInSeconds - TimeInSecondsOld + DAY_SECONDS;
      
      Serial.print("DCF Delta sec.  ");
      Serial.println(deltaSeconds);
      Serial.print("Pendulum ticks  ");         
      Serial.println(PendulumTicksOld);
      Serial.print("Missing Reed: ");
      Serial.println(MissedSeconds);
      Serial.print("Preller: ");
      Serial.println(Preller);

      
      deltaSeconds = deltaSeconds - ((3600 * PendulumTicksOld)/3587.142857);    // convert ticks to seconds (on my clock only)
      AllTimeDiffOld = AllTimeDiff;
      AllTimeDiff += deltaSeconds;
      Serial.print("AllTimeDiff  ");         
      Serial.println(AllTimeDiff);
      
      if (deltaSeconds < 0)
      {                                         // Pendulum too slow
        Serial.print("too fast ");
        Serial.println(deltaSeconds);
      }
      else
      {                                         // Pendulum too fast
        Serial.print("too slow ");
        Serial.println(deltaSeconds);
      }
      state = Adjust;
      break;
    }
    case Adjust:
    {
      int delta;
      delta = abs(AllTimeDiff) - abs(AllTimeDiffOld);
      MotorOn = false;                          // Motor default is off
      Serial.print("AllTimeDiffOld: ");
      Serial.println(AllTimeDiffOld);
      Serial.print("DeltaSeconds: ");
      Serial.println(deltaSeconds);

      if (abs(AllTimeDiff) >= CORR_THRESHOLD)   // if time difference gets too big, motor will start correction run
      {
        MotorOn = true;
        if((AllTimeDiff * AllTimeDiffOld > 0) && (delta < 0))
        {
          MotorOn = false;                      // but not, if time difference is decreasing
        }
      }

      if (MotorOn)
      {
        int duration;
        Serial.print("Abs-Delta: ");
        Serial.println(delta);
        if (deltaSeconds < 0)
        {
          Serial.println("Down Start");
          PORTB = DOWN_B;
          PORTC = DOWN_C;
          duration = (1050/72) * abs(deltaSeconds);
          Serial.println(duration);
          if (duration > 1050)            // limit the Motor on time
            duration = 1050;
          delay(duration);
          PORTC = OFF;
          PORTB = OFF;
          Serial.println("Down Ende");
        }
        if (deltaSeconds > 0)
        {
          Serial.println("Up Start");
          PORTB = UP_B;
          PORTC = UP_C;
          duration = (1200/72) * abs(deltaSeconds);
          Serial.println(duration);
          if (duration > 1200)            // limit the Motor on time
            duration = 1200;
          delay(duration);
          PORTC = OFF;
          PORTB = OFF;
          Serial.println("Up Ende");
        }
      }
      delay(200);
      state = Sleep;
      break;
    }
  }
}

ISR(WDT_vect)                                               // A watchdog IR will take care of missed or bounced pendulum ticks
{
  if (wdtCounter == 2)                                      // Three seconds after Pendulum Trigger
  {
    MissedSeconds += SECONDS_PER_TICK;                      // Because WDT was not reset by Pendulum, must be a missed one
    wdtCounter++;                                           // Count missed ones every 2 seconds now!
  }
  wdtCounter = (wdtCounter + 1 ) % 3;                       // Each wdt-second, increase modulo 3
}


void pendelISR()                                            // this is the interrupt from the pendulum trigger reed relais
{
  if (wdtCounter !=0)                                       // debounce reed switch using WDT
  {
    PendulumTicks += SECONDS_PER_TICK;                      // don't increase before WDT > 0
    wdt_reset();                                            // Reset watchdog timer
    wdtCounter = 0;                                         // Reset number of wdtCounts
  }
  else
  {
    Preller++;
  }
  sleep_disable();
}


void digitalClockDisplay()
{                             // utility function to output DCF time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits)
{                             // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
