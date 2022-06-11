  #include <WiFi.h>  
  #include <WebServer.h>  
  #include <AutoConnect.h>
  #include <PubSubClient.h>
  #include <Wire.h>
    
  WebServer Server;                 
  AutoConnect Portal(Server);
  
  void rootPage() {
    char content[] = "IOMT";
    Server.send(200, "text/plain", content);
  }
  #define RXD2 16
  #define TXD2 17
  
  char sbuffer[30], ch;
  unsigned char pos;
  unsigned char sys, dia, pulse;
  #define bp_trigger_pin 5
  int bp_checked = 0;

  #include "MAX30100_PulseOximeter.h"
  #define REPORTING_PERIOD_MS     7000
  PulseOximeter pox;
  uint32_t tsLastReport = 0;
  
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
  
  //required variables
  int tp, ts = 0, set_value = 7000, btn_state, ecg_last_report = 0, ecg_set_time = 60000;
  
  #define pulse_time 14000

  //MQTT_PART
  //server
  const IPAddress controller_ip(192, 168, 1, 122);
  //credentials
  const char* mqtt_username = "iomt";
  const char* mqtt_pswd = "iomt2021";
  //topics
  const char* bodyTemp_topic = "iomt/patient/body_temp";
  const char* gsr_topic = "iomt/patient/gsrReader";
  const char* roomTemp_topic = "iomt/patient/room_temp";  
  const char* humidity_topic = "iomt/patient/humidity_";
  const char* pulse_topic = "iomt/patient/pulsedata";
  const char* spo2_topic = "iomt/patient/spo2_info";
  const char* ecg_topic = "iomt/patient/ecg_infor";
  const char* sys_topic = "iomt/patient/systolic_";
  const char* dia_topic = "iomt/patient/diastolic";
  
  //clientID
  const char* ClientID = "client_ID";
  
  WiFiClient wifiClient;
  PubSubClient client(controller_ip,1883,wifiClient);

  void connect_MQTT()
  {   
    if(client.connect(ClientID,mqtt_username,mqtt_pswd)){  
          Serial.println("Connected to the broker successfully.");
    
    }else Serial.println("Error!! Connection to broker failed.");
    
  }
  
  //function for beat detection. Used by pulse sensor.
  void onBeatDetected()
  {

      int spo2 =  pox.getSpO2();
      
      Serial.println("Beat!");
      if (millis() - tsLastReport > pulse_time) {
        
        if(client.publish(spo2_topic, String(spo2).c_str()))
        {
          Serial.print("spO2 sent");
        }
        else
        {
          Serial.println("spO2 value failed to sent. Trying to reconnect.");
          client.connect(ClientID,mqtt_username,mqtt_pswd);
          delay(50);
          client.publish(spo2_topic, String(spo2).c_str());
        } 
   
          tsLastReport = millis();
      }
  }
  
  char mygetchar(void)
  { 
   while (!Serial2.available())
   Serial.println("Checking BP");
  // Serial.print(Serial.read());
   return Serial2.read();
  }
  
  
  
  void setup() {
       
  pinMode(bp_trigger_pin, OUTPUT);
  
  pinMode(LOplus, INPUT); 
  pinMode(LOminus, INPUT);
  
    Serial.begin(9600);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    Serial.println("Not Connected to any network !!");
    Server.on("/", rootPage);
    if (Portal.begin()) {
      Serial.println("WiFi connected: " + WiFi.localIP().toString());
    }

    connect_MQTT();

  digitalWrite(bp_trigger_pin, HIGH);
  delay(500);
  digitalWrite(bp_trigger_pin, LOW);
  delay(500);
    
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

  Portal.handleClient();
 
   if(ts<21){
  //Check pulse & spo2
    while(millis()<= pulse_time){
          pox.update();
          pox.setOnBeatDetectedCallback(onBeatDetected);
    }
     
  if(bp_checked == 0){
  
       ch = mygetchar();
         
   if(ch== 0x0A) 
   {
       pos = 0; // buffer position reset for next reading
  
       // extract data from Serial1 buffer to 8 bit integer value
       // convert data from ASCII to decimal
       sys = ((sbuffer[1]-'0')*100) + ((sbuffer[2]-'0')*10) +(sbuffer[3]-'0');
       dia = ((sbuffer[6]-'0')*100) + ((sbuffer[7]-'0')*10) +(sbuffer[8]-'0');
       pulse = ((sbuffer[11]-'0')*100) + ((sbuffer[12]-'0')*10) +(sbuffer[13]-'0');
      

       if(client.publish(sys_topic, String(sys).c_str()))
        {
          Serial.print("sent");
        }
        else
        {
          Serial.println("Sys value failed to sent. Trying to reconnect.");
          client.connect(ClientID,mqtt_username,mqtt_pswd);
          delay(50);
          client.publish(sys_topic, String(sys).c_str());
        }

        if(client.publish(dia_topic, String(dia).c_str()))
        {
          Serial.print("sent");
        }
        else
        {
          Serial.println("Dia value failed to sent. Trying to reconnect.");
          client.connect(ClientID,mqtt_username,mqtt_pswd);
          delay(50);
          client.publish(dia_topic, String(dia).c_str());
        }

        if(client.publish(pulse_topic, String(pulse).c_str()))
        {
          Serial.print("Pulse sent");
        }
        else
        {
          Serial.println("Pulse value failed to sent. Trying to reconnect.");
          client.connect(ClientID,mqtt_username,mqtt_pswd);
          delay(50);
          client.publish(pulse_topic, String(pulse).c_str());
        } 
        
       bp_checked = 1;
       
   } else { 
    //store Serial1 data to buffer
       sbuffer[pos] = ch;
       Serial.println("waiting...");
       pos++;
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

        if(client.publish(bodyTemp_topic, String(temp).c_str()))
        {
          Serial.print("sent");
        }
        else
        {
          Serial.println("Bodytemp value failed to sent. Trying to reconnect.");
          client.connect(ClientID,mqtt_username,mqtt_pswd);
          delay(50);
          client.publish(bodyTemp_topic, String(temp).c_str());
        }
        
       if(client.publish(gsr_topic, String(gsr_average).c_str()))
        {
          Serial.print("GSR sent");
        }
        else
        {
          Serial.println("GSR value failed to sent. Trying to reconnect.");
          client.connect(ClientID,mqtt_username,mqtt_pswd);
          delay(50);
          client.publish(gsr_topic, String(gsr_average).c_str());
        }

        if(client.publish(humidity_topic, String(humidity).c_str()))
        {
          Serial.print("Humidity sent");
        }
        else
        {
          Serial.println("Humidity value failed to sent. Trying to reconnect.");
          client.connect(ClientID,mqtt_username,mqtt_pswd);
          delay(50);
          client.publish(humidity_topic, String(humidity).c_str());
        } 
 
        if(client.publish(roomTemp_topic, String(roomTemp).c_str()))
        {
          Serial.print("Room Temp sent");
        }
        else
        {
          Serial.println("Room Temp value failed to sent. Trying to reconnect.");
          client.connect(ClientID,mqtt_username,mqtt_pswd);
          delay(50);
          client.publish(roomTemp_topic, String(roomTemp).c_str());
        }   
 
  
        ts = 900;
      }
         
          if((digitalRead(LOminus) == 1)||(digitalRead(LOplus) == 1)){
            Serial.println('!');
          }
          else{
             Serial.println(analogRead(adb_outputpin));
          }

   delay(21);
  }
