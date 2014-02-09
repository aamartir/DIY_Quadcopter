#include "Sonar.h"
#include "AB_Filter.h"

Sonar sonar;
float dt;
long last_time;

void setup()
{
  Serial.begin(115200);
  pinMode(4, INPUT);
  
  sonar.sonarInit();
  Serial.println( "Waiting for Sonar data..." ); 
}

void loop()
{
  dt = (micros() - last_time)/1000000.0f;
  last_time = micros();
  
  if( sonar.isSonarDataAvailable() )
  {
    Serial.print( sonar.getSonarAltitude( 1.0f, IN_PER_US ) );  
    Serial.print( "\t" );
    Serial.println( sonar.getSonarROF() );
  }

}
