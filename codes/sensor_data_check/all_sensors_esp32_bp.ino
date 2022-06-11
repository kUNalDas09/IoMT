  //I2C
  #include <Wire.h>
  
  //UART
  #define RXD2 16
  #define TXD2 17
  
  //BP
  char sbuffer[30], ch;
  unsigned char pos;
  unsigned char sys, dia, pulse;
  #define bp_trigger_pin 5
  int bp_checked = 0;
  
  
  //MAX30100 pulse sensor
  #include "MAX30100_PulseOximeter.h"
  #define REPORTING_PERIOD_MS     7000
  PulseOximeter pox;
  uint32_t tsLastReport = 0;
  
  //MAX30205 temperature sensor
  #include "ClosedCube_MAX30205.h"
  ClosedCube_MAX30205 max30205;
  
  //GSR
  const int GSR = 34;
  int sensorValue=0;
  int gsr_average=0;
  
  //DHT11
  #include "DHT.h"
  #define DHTPIN 4
  #define DHTTYPE DHT11
  DHT dht(DHTPIN,DHTTYPE);
  
  //AD8232
  int LOplus = 41, LOminus = 40;
  #define adb_outputpin A0
  int adb_output;  //ad8232
  
  //required variables
  int tp, ts = 0, set_value = 7000, btn_state;
  
  #define pulse_time 30000
  
  
  //function for beat detection. Used by pulse sensor.
  void onBeatDetected()
  {
      Serial.println("Beat!");
      if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
          Serial.print("Heart rate:");
          Serial.print(pox.getHeartRate());
          Serial.print("bpm / SpO2:");
          Serial.print(pox.getSpO2());
          Serial.println("%");
   
          tsLastReport = millis();
      }
  }
  
  //function for BP monitor. Used by BP monitor device
  char mygetchar(void)
  { //receive Serial character from sensor (blocking while nothing received)
   while (!Serial2.available())
   Serial.println("Checking BP");
  // Serial.print(Serial.read());
   return Serial2.read();
  }
  
  
  
  void setup() {
  
  //bp triggering pin
  pinMode(bp_trigger_pin, OUTPUT);
  
  //AD8232 pin
  pinMode(LOplus, INPUT); 
  pinMode(LOminus, INPUT);
  
  //  serial monitor initialization.
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  //Trigger the BP monitor
//  digitalWrite(bp_trigger_pin, HIGH);
//  delay(500);
//  digitalWrite(bp_trigger_pin, LOW);
//  delay(500);
  
  //initializing pulse sensor.
  Serial.print("Initializing pulse oximeter..");  
  if (!pox.begin()) {
      Serial.println("FAILED");
        
  } else {
      Serial.println("SUCCESS");
  }
  
  //// Register a callback for the beat detection
   pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
   
  //initializing temperature sensor.
  Serial.println("Initializing temperature sensor...");
  max30205.begin(0x49);
  
  //initializing DHT11
  Serial.println("Initializing DHT11...");
  dht.begin();
  }
  
  void loop() {
  
//  Serial.print(bp_checked);
   
  if(ts<21){
    
  //Check pulse & spo2
    while(millis()<= pulse_time){
          pox.update();
          pox.setOnBeatDetectedCallback(onBeatDetected);
  //      Serial.println("pulse state");
    }
  if(bp_checked == 0){
    while (bp_checked == 0)
    {
  
       ch = mygetchar(); //loop till character received
  
   if(ch== 0x0A) // if received character is , 0x0A, 10 then process buffer
   {
       pos = 0; // buffer position reset for next reading
  
       // extract data from Serial1 buffer to 8 bit integer value
       // convert data from ASCII to decimal
       sys = ((sbuffer[1]-'0')*100) + ((sbuffer[2]-'0')*10) +(sbuffer[3]-'0');
       dia = ((sbuffer[6]-'0')*100) + ((sbuffer[7]-'0')*10) +(sbuffer[8]-'0');
       pulse = ((sbuffer[11]-'0')*100) + ((sbuffer[12]-'0')*10) +(sbuffer[13]-'0');
  
  
       // example: send demo output to Serial1 monitor on "Serial1"
       Serial.print("Sys: ");     Serial.print(sys);
       Serial.print("\t Dia: ");  Serial.print(dia);
       Serial.print("\t Pulse: ");Serial.print(pulse);
       Serial.println();
  
       bp_checked = 1;
       
   } else { 
    //store Serial1 data to buffer
       sbuffer[pos] = ch;
       pos++;
   }
  }
  }
      
      float temp = max30205.readTemperature();     //body temp
      float roomTemp = dht.readTemperature();      // surrounding temp
      float humidity = dht.readHumidity();         // surrounding humidity
  
      long sum=0;                                  //gsr value
        for(int i=0;i<100;i++)           
        {
          sensorValue=analogRead(GSR);
          sum += sensorValue;
          delay(5);
        }      
      gsr_average = sum/100;
  
        tp = (millis() - ts)/1000;
  
  ////    body temperature 
        Serial.print("BODY TEMPERATURE: ");
        Serial.print(temp,2);
        Serial.println("°C" );
        
  //    gsr value
        Serial.print("\tGSR_VALUE: ");
        Serial.print("\tHUMIDITY: ");
        Serial.print(humidity);
        Serial.println("%");
  
  ////  room temperature
        Serial.print("\tRoom_Temp: ");
        Serial.print(roomTemp);
        Serial.println("°C" );  
  
        ts = 900;
      }
      
        if((digitalRead(LOminus) == 1)||(digitalRead(LOplus) == 1)){
            Serial.println('!');
        }
        else{
          Serial.print("ECG_Values: ");
          adb_output = analogRead(A0);
          Serial.println(adb_output);
        }
        delay(21);
        
  }
