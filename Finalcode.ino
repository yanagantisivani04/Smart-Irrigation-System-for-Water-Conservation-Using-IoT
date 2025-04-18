/*!
 * @file  readData.ino
 * @brief  This example describes the method of using this module to test the collected rainfall within one hour.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license    The MIT License (MIT)
 * @author     [fary](feng.yang@dfrobot.com)
 * @version    V1.0
 * @date       2023-02-28
 * @url        https://github.com/DFRobot/DFRobot_RainfallSensor
 */
#define BLYNK_TEMPLATE_ID "TMPL3FpQyJ16U"
#define BLYNK_TEMPLATE_NAME "IoT irrigation system"
#define BLYNK_AUTH_TOKEN "LQ9AJPsUH7vkGu5mRxNY0EE7VxncFYU-"
#include "DFRobot_RainfallSensor.h"
#include "DHT.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
const char *ssid =  "ECE_Lab";     // Enter your WiFi Name
const char *pass =  "pscmr@ece"; // Enter your WiFi Password
const char *auth = BLYNK_AUTH_TOKEN;
#define TIMER_ZERO 0
#define TIMER_ONE 1
unsigned long howLongToWait_MQTT = 7000;
unsigned long howLongToWait_WiFi = 25000;
unsigned long time_now_read = 0;
float temp_dht = 0.0;
float humid_dht = 0.0;
float temp_dht_final = 0.0;
float humid_dht_final = 0.0;
long currentMillis = 0;
long previousMillis = 0;
float calibrationFactor = 4.5;
int interval = 1000;
volatile byte pulseCount;
bool moisture_flag = false;
bool motor_state_flag = false;
void IRAM_ATTR MOTOR_state_control(); 
byte pulse1Sec = 0;
bool relay_flag = false;
float flowRate;
int relay_pin = 19;
#define DHTTYPE DHT11
const int MOISTURE_READ_1 = 34;
const int MOISTURE_READ_2 = 35;
#define DHT11PIN 26
#define FLOW_SENSE_PIN 25
/I2C_DATA = 21    ,    I2C_CLK = 22/
unsigned int moist_sense_1 = 0;
unsigned int moist_sense_percentage_1 = 0;
unsigned int moist_sense_2 = 0;
unsigned int moist_sense_percentage_2 = 0;
unsigned int time_delay = 0;
float rain_fall = 0.0;
float rain_fall_hour = 0.0;
DFRobot_RainfallSensor_I2C Sensor(&Wire);
TaskHandle_t sense_collect;
TaskHandle_t sense_publish;
SemaphoreHandle_t sema_sensor_access;
hw_timer_t *timer = NULL;
hw_timer_t *timer_one = NULL;
DHT dht_irrigation(DHT11PIN, DHT11);
void IRAM_ATTR PulseCounter();
WiFiClient client;
void setup(void)
{
  pinMode(FLOW_SENSE_PIN,INPUT);
  pinMode(relay_pin,OUTPUT);
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  dht_irrigation.begin();
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSE_PIN),PulseCounter, HIGH);
  delay(1000);
  while(!Sensor.begin()){
    Serial.println("Sensor init err!!!");
    delay(1000);
  }
  Serial.print("vid:\t");
  Serial.println(Sensor.vid,HEX);
  Serial.print("pid:\t");
  Serial.println(Sensor.pid,HEX);
  Serial.print("Version:\t");
  Serial.println(Sensor.getFirmwareVersion());
  //Set the cumulative rainfall value in units of mm.
  //Sensor.setRainAccumulatedValue(0.2794);
  sema_sensor_access = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(
                          Task_SENSE_COLLECT,
                          "sensorcollect",  // Task name
                           25000,             // Stack size (bytes)
                           NULL,             // Parameter
                              1,                // Task priority
                           &sense_collect,             // Task handle
                           0);
  delay(500);
  xTaskCreatePinnedToCore(
                    Task_PUBLISH,   /* Task function. */
                    "task_publish",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    0,           /* priority of the task */
                    &sense_publish,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */ 
  delay(500);
  Serial.println();
  delay(500);
  Serial.println();
  Serial.println("return the clock speed of the CPU.");
  // return the clock speed of the CPU.
  uint8_t cpuClock = ESP.getCpuFreqMHz();
  Serial.print("cpu clock:::");
  Serial.println(cpuClock);
  delay(500);
  Serial.println("Timer one Begin");
  timer_one = timerBegin(TIMER_ONE,cpuClock, true);
  Serial.println();
  Serial.println("Attach scroll function to our timer one.");
  timerAttachInterrupt(timer_one, &MOTOR_state_control, true);
  delay(500);
  Serial.println();
  Serial.println("Set alarm to call scroll function.");
  // Set alarm to call SCROLL function.
  timerAlarmWrite(timer_one, 10000, true);
  delay(500);
  Serial.println();
  Serial.println("Start an alarm for scrolling.");
  // Start an alarm.
  timerAlarmEnable(timer_one);
  delay(500);
}
void IRAM_ATTR MOTOR_state_control() 
{
  motor_state_flag = true;
}
void Task_SENSE_COLLECT(void * parameter)
{
  Serial.print("Task_sense_collect running on core:");
   Serial.println(xPortGetCoreID());
   //xSemaphoreGive ( sema_sensor_access );
   for(;;)
   {
      if(xSemaphoreTake( sema_sensor_access, portMAX_DELAY) == pdPASS)
      {
         Serial.println("pass success to mqtt loop");
         Serial.print("Sensor WorkingTime:\t");
         Serial.print(Sensor.getSensorWorkingTime());
         Serial.println(" H");
        //Get the cumulative rainfall during the sensor operating time.
        Serial.print("Rainfall:\t");
        rain_fall = Sensor.getRainfall();
        Serial.println(rain_fall);
        //Here is an example function that calculates the cumulative rainfall in a specified hour of the system. The function takes an optional argument, which can be any value between 1 and 24.
        Serial.print("1 Hour Rainfall:\t");
        rain_fall_hour = Sensor.getRainfall(1);
        Serial.print(rain_fall_hour);
        Serial.println(" mm");
        //Get the raw data, which is the tipping bucket count of rainfall, in units of counts.
        Serial.print("rainfall raw:\t");
        Serial.println(Sensor.getRawData());
        moist_sense_1 = analogRead(MOISTURE_READ_1);
        moist_sense_percentage_1 = map(moist_sense_1,2651,0,0,100);
        Serial.print("Moisture_sense_1:\t");
        Serial.print(moist_sense_percentage_1);
        Serial.print("%");
        Serial.println(moist_sense_1);
        moist_sense_2 = analogRead(MOISTURE_READ_2);
        moist_sense_percentage_2 = map(moist_sense_2,2651,0,0,100);
        Serial.print("Moisture_sense_2:\t");
        Serial.print(moist_sense_percentage_2);
        Serial.print("%");
        Serial.println(moist_sense_2);
        if((moist_sense_percentage_2 <=10)&&(moist_sense_percentage_1 <= 10))
        {
          moisture_flag = false;
        }
        else
        if((moist_sense_percentage_2 >= 40)&&(moist_sense_percentage_1 >= 40))
        {
          moisture_flag = true;
        }
        temp_dht = dht_irrigation.readTemperature();
        if(isnan(temp_dht))
        {
          temp_dht_final = 27.0;
        }
        else
        {
          temp_dht_final = temp_dht;
        }
        delay(10);
        humid_dht = dht_irrigation.readHumidity();
        if(isnan(humid_dht))
        {
          humid_dht_final = 50.0;
        }
        else
        {
          humid_dht_final = humid_dht;
        }
        Serial.print("temp:\t");
        Serial.print(temp_dht_final);
        Serial.print("humidity:\t");
        Serial.print(humid_dht_final);
        pulse1Sec = pulseCount;
        pulseCount = 0;
        flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
        previousMillis = millis();
        Serial.print("Flow rate: ");
        Serial.print(float(flowRate));  // Print the integer part of the variable
        Serial.print("L/min");
        Serial.print("\n"); 
     }
     xSemaphoreGive ( sema_sensor_access );
     vTaskDelay(2000/portTICK_PERIOD_MS);
   }
}
void Task_PUBLISH(void * parameter)
{
   Serial.print("Task_sensor_publish running on core:");
   Serial.println(xPortGetCoreID());
   xSemaphoreGive ( sema_sensor_access );
    for(;;)
    {
      if(xSemaphoreTake( sema_sensor_access, portMAX_DELAY) == pdPASS)
      {
          Serial.println("pass success zero verified for publishing");
          Blynk.virtualWrite(V0,temp_dht_final);
          Blynk.virtualWrite(V1,humid_dht_final);
          Blynk.virtualWrite(V2,moist_sense_percentage_1);
          Blynk.virtualWrite(V3,moist_sense_percentage_2);
          Blynk.virtualWrite(V4,rain_fall);
          Blynk.virtualWrite(V5,flowRate);
          if(moisture_flag == true)
          {
            Blynk.virtualWrite(V6,"OFF");
          }
          else
          if(moisture_flag == false)
          {
            Blynk.virtualWrite(V6,"ON");
          }
          
      }
      xSemaphoreGive ( sema_sensor_access );  
      vTaskDelay(20000/portTICK_PERIOD_MS);  
    }
}
void loop()
{
  if(millis() >= time_delay+1000)
  {
    time_delay += 1000;
      if(moisture_flag == true)
      {
        digitalWrite(relay_pin,LOW);
        Serial.println("relay off");
      }
      else
      if(moisture_flag == false)
      {
        digitalWrite(relay_pin,HIGH);
        Serial.println("relay on");
      }
   }
   Blynk.run();
}
void IRAM_ATTR PulseCounter()
{
  pulseCount++;
}

