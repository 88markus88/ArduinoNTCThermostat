#define PROGNAME  "NTC_Thermostat.cpp"
#define PROGVERSION "V0.03"
#define PROGDATE "2021-11-10"

#include <Arduino.h>
#include <avr/wdt.h>

#undef isPlotter
#ifdef isPlotter
  #include "Plotter.h"
  Plotter p;
  double x;
#endif

#define isDIPSwitch
// dip switches are used to control the temperature setting for the thermostat
// the values for all set switches are added. e.g. 2&4&6 ON: 2.5+10+40 = 52.5 °C
// same for hysteresis. 8 ON = 5°C, 8&9 ON = 15 °C Hysteresis
#ifdef isDIPSwitch
  // new binary values for PCB
  int noDIPSwitches = 7;  // temperature settings control for switches. 
  int switchArray[7]=   {2, 3, 4,  5,  6,  7,   8};   // digital ports for switches
  double switchValue[7]={2, 4, 8, 16, 32, 64, 128};   // value for these digital ports  
  // old values from prototype
  // int noDIPSwitches = 7;  // temperature settings control for switches. 
  // int switchArray[6]=   {2,   3,  4,  5,  6,  7};   // digital ports for switches
  // double switchValue[6]={2.5, 5, 10, 20, 40, 80};   // value for these digital ports  

  // new binary values for PCB
  // old values from prototype
  int noHysSwitches = 3;               // hysteresis control for heater
  int hysArray[3]=   {9,   10,  11};   // digital ports for switches
  double hysValue[3]={2,    4,   8};   // value for these digital ports  

  #define MODE_HEATING  0       
  #define MODE_COOLING  1
  int switchForMode = 0;               // digital port for switch heating (open) to cooling (closed)
#endif

#define isRelay
#ifdef isRelay  // relais connected to GPIO 12
  #define RELAYPIN1 12 //not 13, that is connected to LED and switched during start
#endif  

// port for a heartbeat LED
#undef isLEDHeartbeat
#ifdef isLEDHeartbeat
  #define HEARTBEATPIN 11
#endif

#define isDisplay
#ifdef isDisplay
  // OLED Display
  // stuff for SSD1306 ASCII library. Used instead of adafruit since better memory usage
  // https://github.com/greiman/SSD1306Ascii 
  #include <Wire.h>
  #include "SSD1306Ascii.h"
  #include "SSD1306AsciiWire.h"
  // 0X3C+SA0 - 0x3C or 0x3D
  #define I2C_ADDRESS 0x3C
  // Define proper RST_PIN if required.
  #define RST_PIN -1
#endif  // isDisplay

#ifdef isDisplay
  // SSD1306ascii - oled object 
  SSD1306AsciiWire oled;
#endif  // isDisplay

// state machine states & variable for heater control
#define stHEATING 1
#define stOVERHEATED 2
#define stFAIL 3
#define stCOOLING 4
#define stOVERCOOLED 5

int heaterState = 0;

// thermostatat mode: heating or cooling
int thermostatMode = 0;      // 0= heating, dip open. 1=cooling, dip closed

// target temperature and hysteresis with defaults
float targetTemp = 28.0;     // when this temp is reached: overheated
float tempHysteresis = 5.0;  // hysteresis: switching back to heating if targetTemp-tempHysteresis

// variables for text output on serial (usb) port
char printstring[120];
char number1[12], number2[12];

/**************************************************!
  @brief    get the actual state of the system: heating or overheated, and set the state accordingly
  @details  is using global variables: 
              heaterState     store the actual state
              targetTemp      thermostat target temperature
              tempHysteresis  temperature hysteresis. switching on again if targetTemperature - tempHysteresis reached
            States:
            stHEATING     : normal heating, relay is on
            stOVERHEATED  : over targetTemp, and not yet below targetTemp-tempHysteresis
            stFAIL        : failure, e.g. NTC sensor broken (open)

  @param    float actTemp
  @note     uses global variables to store information
  @return   int 
***************************************************/
int getHeaterState(float actTemp)
{
  if(thermostatMode == MODE_HEATING)
  {
    switch(heaterState){
      case (stHEATING):
        if(actTemp >= targetTemp)
          heaterState = stOVERHEATED;
        if(actTemp < -300)
          heaterState = stFAIL;
        break;
      case (stOVERHEATED):
        if(actTemp < targetTemp - tempHysteresis)
          heaterState = stHEATING;
        if(actTemp < -300)
          heaterState = stFAIL;  
        break;
      case (stFAIL):
        {
          if(actTemp > -300)  // no longer in fail, go to overheated
            heaterState = stOVERHEATED;
          break;
        }  
      default:
        heaterState = stOVERHEATED;
    } // switch heaterState
  }

  if(thermostatMode == MODE_COOLING)
  {
    switch(heaterState){
      case (stCOOLING):
        if(actTemp <= targetTemp)
          heaterState = stOVERCOOLED;
        if(actTemp < -300)
          heaterState = stFAIL;
        break;
      case (stOVERCOOLED):
        if(actTemp > targetTemp + tempHysteresis)
          heaterState = stCOOLING;
        if(actTemp < -300)
          heaterState = stFAIL;  
        break;
      case (stFAIL):
        {
          if(actTemp > -300)  // no longer in fail, go to overheated
            heaterState = stOVERCOOLED;
          break;
        }  
      default:
        heaterState = stOVERCOOLED;
    } // switch heaterState
  }
  return(heaterState);
}

/**************************************************!
  @brief    get the thermostat mode (heating or cooling) from jumper
  @details  this function evaluates jumper setting to determine the 
            thermostat mode, either for heating (jumper open or for cooling (jumper closed)
            is using global variables: 
              none
  @param   none
  @return  double int mode
***************************************************/
int getThermostatModeFromDIP()
{
  int val,mode;

  val=digitalRead(switchForMode);
  if(val==0)  // jumper is closed
    mode = MODE_COOLING; // cooling
  else  
    mode = MODE_HEATING; // heating

  sprintf(printstring,"PIN: %d Val: %d Thermostat Mode: %d \n", switchForMode, val, mode);
  Serial.print(printstring);

  return(mode);    
}

/**************************************************!
  @brief    get the target temperature in °C from dip switch settings 
  @details  this function evaluates dip switch settings to determine the 
            thermostat target temperature
            is using global variables: 
              switchArray[]     store the dip switch ports used
              switchValue[]     assigns temp values to each dip switch
  @param   none
  @return  double tempTarget: Target temperature for thermostat in °C
***************************************************/
double getTargetTempFromDIP()
{
  int i, val;
  double tempTarget = 0;

  for(i=0;i<noDIPSwitches;i++){
    val = digitalRead(switchArray[i]);
    if(val == 0)
      tempTarget += switchValue[i];
    // dtostrf(switchValue[i],4,2,number1);  
    // sprintf(printstring,"%d %d %s / ",switchArray[i],val, number1);  
    // Serial.print(printstring);
  }
  return(tempTarget);
}

/**************************************************!
  @brief    get the temp hysteresis in °C from dip switch settings
  @details  this function evaluates dip switch settings to determine the 
            temperature hysteresis for the thermostat
            is using global variables: 
              hysArray[]     store the dip switch ports used
              hysValue[]     assigns temp values to each dip switch
  @param   none
  @return  hysTarget: Target Hysteresis in °C
***************************************************/
double getHysteresisFromDIP()
{
  int i, val;
  double hysTarget = 0;

  for(i=0;i<noHysSwitches;i++){
    val = digitalRead(hysArray[i]);
    if(val == 0)
      hysTarget += hysValue[i];
    // dtostrf(hysValue[i],4,2,number1);  
    // sprintf(printstring,"%d %d %s / ",hysArray[i],val, number1);  
    // Serial.print(printstring);
  }
  return(hysTarget);
}


// NTC temperature calculation data
double U_0 = 4.750;     // reference voltage for measurement at Arduino Nano
double R_ref = 10000;  // resistor in series with NTC
double T_0 = 298.15;   // NTC reference temperature: 25°C  
double B = 3977;       // NTC B value//--- calculation routine for temperature from voltage over ntc in mV
/**************************************************!
  @brief    calculation routine for temperature from voltage over ntc in mV
  @details  this function gets the measured voltage across the NTC in mV
            and calculates the temperature in °C from it.
            It uses global variables for the calculation:
            double U_0 = 4.75;     // reference voltage [V] over whole resistor bridge
            double R_ref = 10000;  // resistor in series with NTC in [Ohm]
            double T_0 = 298.15;   // NTC reference temperature in [K]: 25°C  
            double B = 3977;       // NTC B value 
  @param   double U_Meas:   measured voltage at NTC in [mV]
  @return  double T_meas:   measured temperature in [°C]
***************************************************/
double calcTemp(double U_meas)
{
  double T_meas, R_meas;

  R_meas = R_ref/(1000*U_0/U_meas - 1.0);
  if(R_meas < 500000)
    T_meas = (1/(log(R_meas/R_ref)/B + 1/T_0)) - 273.15;
  else
    T_meas = -999;  // error condition if resistance too high, likely broken
  
  #ifndef isPlotter
    dtostrf(R_meas,4,2,number1);    // use this since %f does not work in sprintf on arduino
    dtostrf(T_meas,4,2,number2);    // use this since %f does not work in sprintf on arduino
    sprintf(printstring,"R_meas: %s [Ohm] T_meas %s [°C] ",number1, number2);
    Serial.print(printstring);
  #endif
  return(T_meas);
}

// setup routine
/**************************************************!
  @brief    setup routine
  @details  set serial mode for communiction with terminal
          set pinMOde for analog input from resistor measuring bridge
          prepares plotter using "processing" with listener script and plotter.h lib
          evalates dip switches for target temperature and hysteresis
          sets port for relay, and before that sets it to safe "LOW"
  @param   none
  @return  void
***************************************************/
void setup() {
  int i;

  Serial.begin(115200);
  delay(1000);

  wdt_enable(WDTO_4S);   // Watchdog auf 4 s stellen. http://www.netzmafia.de/skripten/hardware/Arduino/Watchdog/index.html 
  
  pinMode(A0, INPUT);   // pin for analog input from resistor bridge with NTC

  #ifdef isPlotter
    p.Begin();
    p.AddTimeGraph( "Temp Graph [3600 points]", 3600, "NTC Temp [C]", x );
  #endif

  #ifdef isDIPSwitch
    // set input digital ports for target temperature dip switches. 
    for(i=0;i<noDIPSwitches;i++){
      delay(10);
       pinMode(switchArray[i], INPUT_PULLUP);
      }
    // set input digital ports for hysteresis dip switches. 
    for(i=0;i<noHysSwitches;i++){
      delay(10);
       pinMode(hysArray[i], INPUT_PULLUP);
      }  
    // set input digital port for heating / cooling mode
    pinMode(switchForMode, INPUT_PULLUP);
  #endif

  // set digital port as output for Relay
  #ifdef isRelay
    digitalWrite(RELAYPIN1, LOW);   // vor dem Modus setzen auf LOW, um relais klackern zu vehindern
    pinMode(RELAYPIN1,OUTPUT);
    delay(10);
  #endif

  // Set digital Output as output for LED
  #ifdef isLEDHeartbeat
    pinMode(HEARTBEATPIN, OUTPUT); // Digital-Pin 0 as input
  #endif   

  #ifdef isDisplay
    // ssd1306ascii stuff
    Wire.begin();
    Wire.setClock(400000L);

    #if RST_PIN >= 0
      oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
    #else // RST_PIN >= 0
      oled.begin(&Adafruit128x64, I2C_ADDRESS);
    #endif // RST_PIN >= 0
    oled.setFont(System5x7);
    oled.clear();
  #endif 
} // setup

// ========= Main LOOP =========
/**************************************************!
  @brief    ========= Main LOOP =========
  @details  main loop            
  @param   none
  @return  void
***************************************************/
void loop() {
  int U_read, i;
  double tempC, U_ntc, U_vcc, tmp=0.0;

  // test
  /*
  digitalWrite(RELAYPIN1, HIGH);
  delay(200);
  digitalWrite(RELAYPIN1, LOW);
  */

  wdt_reset(); // keep watchdog happy

  // get thermostat mode from switch. can be MODE_HEATING or MODE_COOLING
  thermostatMode = getThermostatModeFromDIP();
  // sprintf(printstring,"Thermostat Mode: %d \n", thermostatMode);
  // Serial.print(printstring);

  // get the target temperature and hysteresis settings
  targetTemp = getTargetTempFromDIP();
  dtostrf(targetTemp,4,1,number1);
  tempHysteresis = getHysteresisFromDIP();
  dtostrf(tempHysteresis,4,1,number2);
  sprintf(printstring,"Target: %s [C] Hysteresis: %s [C] --- ", number1, number2 );
  #ifndef isPlotter
    Serial.print(printstring);
  #endif  

  // read analog value on A0
  U_read = 0;
  for(i=0; i<10;i++)
    U_read += analogRead(A0);
  tmp = (float)U_read/10;  
  U_vcc = U_0;                // Actual Arduino 5V supply voltage in mV
  U_ntc= 1000*(U_vcc * (tmp)/1023);  // ntc voltage in mV. Basis: VCC=4.750 V. (Was number 4750.0)
  dtostrf(tmp,4,2,number1);    // use this since %f does not work in sprintf on arduino
  dtostrf(U_ntc,4,2,number2);    // use this since %f does not work in sprintf on arduino
  sprintf(printstring,"U_read: %s U_ntc: %s  [mV] \n", number1, number2);
  #ifndef isPlotter
    Serial.print(printstring);
  #endif  

  tempC = calcTemp(U_ntc);
  if(tempC<-300){    // error condition: -999
    sprintf(printstring," NTC broken, high resitance ");
    #ifndef isPlotter
      Serial.print(printstring);
    #endif  
  }

  // get and update the heater state in state machine
  getHeaterState(tempC);
  #ifndef isPlotter
    sprintf(printstring," Thermostat mode: %d Heater state: %d  \n",thermostatMode, heaterState);
    Serial.print(printstring);
  #endif  

  // set LED according to heater state
  #ifdef isLEDHeartbeat
    if(heaterState == stHEATING || heaterState==stCOOLING)
      digitalWrite (HEARTBEATPIN, HIGH);
    else
      digitalWrite (HEARTBEATPIN, LOW); 
  #endif  

  // set relay according to heater state
  #ifdef isRelay
    if(heaterState == stHEATING || heaterState==stCOOLING)
      digitalWrite(RELAYPIN1, HIGH);
    else
      digitalWrite(RELAYPIN1, LOW);
  #endif

  // output to OLED display
  #ifdef isDisplay
    // stuff for ssd1306ascii
    // oled.clear();
    oled.setCursor(0,0);
    oled.setInvertMode(true);
    if(thermostatMode == MODE_HEATING)
      sprintf(printstring," Thermostat: Heating  ");
    else
      sprintf(printstring," Thermostat: Cooling  "); 
    oled.print(printstring);
    oled.setInvertMode(false);

    oled.setCursor(0,2);
    dtostrf(targetTemp,4,1,number1); 
    dtostrf(tempHysteresis,4,1,number2); 
    sprintf(printstring,"Limit:%s Hys:%s C  ",number1,number2);
    // Serial.print(printstring);
    oled.print(printstring);

    oled.setCursor(0,3);
    dtostrf(U_ntc,4,2,number1); 
    sprintf(printstring,"U: %s [mV]   ",number1);
    // Serial.print(printstring);
    oled.print(printstring);

    oled.setCursor(0,5);
    oled.set2X();
    dtostrf(tempC,5,2,number1); 
    sprintf(printstring," %s [C]   ",number1);
    // Serial.print(printstring);
    oled.print(printstring);
    oled.set1X();
    
    oled.setCursor(0,7);
    if(heaterState==stHEATING)    
      sprintf(printstring," htrState: %d HEATING   ",heaterState);
    if(heaterState==stOVERHEATED) 
      sprintf(printstring," htrState: %d OVERHEAT  ",heaterState);
    if(heaterState==stCOOLING)    
      sprintf(printstring," htrState: %d COOLING   ",heaterState);
    if(heaterState==stOVERCOOLED) 
      sprintf(printstring," htrState: %d OVERCOOL  ",heaterState);  
    if(heaterState==stFAIL) 
      sprintf(printstring," htrState: %d FAIL      ",heaterState);
    // sprintf(printstring,"htrState: %d OVERHEAT   ",heaterState);
    // Serial.print(printstring);
    if(heaterState==stHEATING || heaterState==stCOOLING)  
      oled.setInvertMode(true);
    oled.print(printstring);
    oled.setInvertMode(false);


  #endif  

  #ifdef isPlotter
    // x = 10*sin( 2.0*PI*( millis() / 5000.0 ) );
    x = tempC;
    p.Plot(); // usually called within loop()
  #endif

  delay(900);
}