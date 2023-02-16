#define Program_Version "V1.0"
#include <EEPROM.h>
#include <SPI.h>                                               //the SX127X device is SPI based so load the SPI library                                         
#include <SX127XLT.h>   
#include <string>
#include <iostream>
#include <sstream>
#include <ArduinoJson.h>
#include "Settings.h" 

using std::cout; using std::cin;
using std::endl; using std::string;

SX127XLT LT;                                                   //create a library class instance called LT

TaskHandle_t Normal;  //Normal function
TaskHandle_t Anomaly;  //Anomaly detection 
// ISR DEFINITION ##########################################################################################
void IRAM_ATTR pulseCounter_1() {
  // Increment the pulse counter for sensor 1
  sensors[0].pulseCount++;sensors[0].check = 1;  
}
void IRAM_ATTR pulseCounter_2() {
  // Increment the pulse counter for sensor 2
  sensors[1].pulseCount++;sensors[1].check = 1;
}
void IRAM_ATTR pulseCounter_3() {
  // Increment the pulse counter for sensor 3
  sensors[2].pulseCount++;sensors[2].check = 1;
}
void IRAM_ATTR pulseCounter_4() {
  // Increment the pulse counter for sensor 4
  sensors[3].pulseCount++;sensors[3].check = 1;
}
typedef void (*pulseCounter_fn)();
pulseCounter_fn pulseCounters[FLOWSENSORS] = {pulseCounter_1,pulseCounter_2,pulseCounter_3,pulseCounter_4};
void setup_lora(){
  //SPI.begin();                                     //default setup can be used be used
  SPI.begin(SCK, MISO, MOSI, NSS);                   //alternative format for SPI3, VSPI; SPI.begin(SCK,MISO,MOSI,SS)
  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    //led_Flash(2, 125);                                   //two further quick LED flashes to indicate device found
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (!LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
    {
      //led_Flash(50, 50); //long fast speed LED flash indicates device error
    }
  }
  LT.setMode(MODE_STDBY_RC);                              //got to standby mode to configure device
  LT.setPacketType(PACKET_TYPE_LORA);                     //set for LoRa transmissions
  LT.setRfFrequency(Frequency, Offset);                   //set the operating frequency
  LT.calibrateImage(0);                                   //run calibration after setting frequency
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, LDRO_AUTO);  //set LoRa modem parameters
  LT.setBufferBaseAddress(0x00, 0x00);                    //where in the SX buffer packets start, TX and RX
  LT.setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);  //set packet parameters
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);              //syncword, LORA_MAC_PRIVATE_SYNCWORD = 0x12, or LORA_MAC_PUBLIC_SYNCWORD = 0x34
  LT.setHighSensitivity();                                //set for highest sensitivity at expense of slightly higher LNA current
  LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_TX_DONE, 0, 0);   //set for IRQ on RX done
  //***************************************************************************************************
  Serial.print(F("Transmitter ready"));
  Serial.println();

}
// PACKET HEALTH CHECK #####################################################################################
void packet_is_OK(uint8_t * buff)                       //sending check ok
{
  //if here packet has been sent OK
  uint16_t localCRC;

  Serial.print(F("  BytesSent,"));
  Serial.print(TXPacketL);                             //print transmitted packet length
  localCRC = LT.CRCCCITT(buff, TXPacketL, 0xFFFF);
  Serial.print(F("  CRC,"));
  Serial.print(localCRC, HEX);                              //print CRC of sent packet
  Serial.print(F("  TransmitTime,"));
  Serial.print(endmS - startmS);                       //print transmit time of packet
  Serial.print(F("mS"));
  Serial.print(F("  PacketsSent,"));
  Serial.print(TXPacketCount);                         //print total of packets sent OK
}

void packet_is_Error()                                // sending check error
{
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                  //read the the interrupt register
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);                         //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                    //print IRQ status
  LT.printIrqStatus();                             //prints the text of which IRQs set
}
// SEND OVER LORA ################################################################################################
void sendDataToGW(int index)            
{
  DynamicJsonDocument doc(256);
  JsonObject data = doc.createNestedObject(F("data"));

  //write the date
  doc["device"] = sensors[index].id;
  data["consumption"] = sensors[index].totalMilliLitres;
  data["period"] = sensors[index].total_duration;
  char out[128];
  size_t n = serializeJson(doc, out);
  int size = n ;    
  uint8_t DATA_BUFF[size+1];
  
  memcpy(DATA_BUFF, out, sizeof(out));

  TXPacketL = sizeof(DATA_BUFF);                                   //set TXPacketL to length of array
  Serial.print("size of buff= ");
  Serial.print(TXPacketL);

  Serial.print(TXpower);                                        //print the transmit power defined
  Serial.print(F(" dBm "));
  Serial.print(F("Packet> "));
  Serial.flush();
  LT.printASCIIPacket(DATA_BUFF, TXPacketL);                         //print the buffer (the sent packet) as ASCII

  digitalWrite(LED1, HIGH);
  startmS =  millis();                                         //start transmit timer
  if (LT.transmit(DATA_BUFF, TXPacketL, 5000, TXpower, WAIT_TX))    //will return packet length sent if OK, otherwise 0 if transmit error
  {
    endmS = millis();                                          //packet sent, note end time
    TXPacketCount++;
    packet_is_OK(DATA_BUFF);
  }
  else
  {
    packet_is_Error();                                 //transmit packet returned 0, there was an error
  }

  digitalWrite(LED1, LOW);
  Serial.println();
  delay(packet_delay);    
}
void SEND_ANOMALY2GW(int index){
   //Prepare the Json object to be sent
  DynamicJsonDocument doc(256);
  
  doc["device"] = ANML_DET[index].id;

  doc["type"] = ANML_DET[index].type;
// Sending the JSON body
  char out[128];
  size_t n = serializeJson(doc, out);
  Serial.println("Sending anomaly message..");
  // Send the data over LoRa
  int size = n;
  uint8_t ANML_BUFF[size+1];
  memcpy(ANML_BUFF, out, sizeof(out));
  TXPacketL = sizeof(ANML_BUFF);
  startmS =  millis();
  if (LT.transmit(ANML_BUFF, TXPacketL, 5000, TXpower, WAIT_TX)) {
    endmS = millis();
    TXPacketCount++;
    packet_is_OK(ANML_BUFF);
  }
  else {packet_is_Error();}  
  Serial.println("-------------");
}
void calculateWaterFlow(int index){
  sensors[index].pulse1Sec = sensors[index].pulseCount;
  sensors[index].flowRate = ((1000.0 / (millis() - sensors[index].oldTime)) * sensors[index].pulseCount) / calibrationFactor;
  sensors[index].flowMilliLitres = (sensors[index].flowRate / 60) * 1000;
  sensors[index].frac = (sensors[index].flowRate - int(sensors[index].flowRate)) * 10;
  sensors[index].totalMilliLitres += sensors[index].flowMilliLitres;
  sensors[index].totalLitres = sensors[index].totalMilliLitres / 1000;
  sensors[index].total_duration ++ ;
  sensors[index].oldTime = millis(); 
  sensors[index].previousMillis = millis();
  sensors[index].pulseCount = 0;
}

void printData(int index){
  Serial.print(F("Flow rate: "));
  Serial.print(int(sensors[index].flowRate)); // Print the integer part of the variable
  Serial.print(F(".")); // Print the decimal point
  Serial.print(sensors[index].frac, DEC); // Print the fractional part of the variable
  Serial.print(F("L/min"));// Print the number of litres flowed in this second
  Serial.print(F("  Current Liquid Flowing: "));
  Serial.print(sensors[index].flowMilliLitres);
  Serial.print(F("mL/Sec"));
  Serial.print(F("  Output Total Water: "));
  Serial.print(sensors[index].totalMilliLitres);
  Serial.println(F("mL"));
  Serial.print(F("Duration= "));
  Serial.println(sensors[index].total_duration);
  Serial.print(F("pulse number: "));
  Serial.println(sensors[index].pulse1Sec);
}
//###############################################################################################################
// ################################# DEFINING NORMAL FUNCTION TASK ##############################################
//###############################################################################################################

void normal_task(int index){
      if ((millis() - sensors[index].oldTime) > 1000 && sensors[index].check == 1) // Only process counters once per second
    {
        detachInterrupt(sensors[index].pin);
        digitalWrite(sensors[index].pin, HIGH); //bring pullup resistor to avoid noisy input data
        calculateWaterFlow(index);
        printData(index);
        sensors[index].check = 0; // Reset to 0 and wait if there is Input Signal to run the Loop Again
        // Enable the interrupt again now that we've finished sending output
        
        attachInterrupt(sensors[index].pin, pulseCounters[index], FALLING);           
    } 
            sensors[index].currentMillis = millis();
        
        if ((sensors[index].check == 0) && (sensors[index].previousMillis != 0) && ((sensors[index].currentMillis - sensors[index].previousMillis) >= interval))
        {
          sendDataToGW(index);
          sensors[index].previousMillis = 0;
          sensors[index].totalMilliLitres = 0;
          sensors[index].total_duration = 0;
          esp_deep_sleep_start();
        }
}


//Task1code: normal function
void Task1code( void * pvParameters ){
  //const TickType_t xDelay = 500;
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for (int index ; index < 5 ; index++){
    normal_task(index);  
  }
  vTaskDelay(10);
}
void anomaly_task(int index){
      if ((ANML_DET[index].total_duration_threshold == config[index].duration) && (config[index].id == sensors[index].id)) {     
        ANML_DET[index].id = sensors[index].id;
        ANML_DET[index].type ="duration";
        Serial.println();
        Serial.println();
        Serial.println("!!!!!!!!!!!! Anomaly Detected: duration !!!!!!!!!!!!");
        Serial.println();
        Serial.println();
        SEND_ANOMALY2GW(index);
        ANML_DET[index].total_duration_threshold = 0;
      }

        if ((ANML_DET[index].total_consumption_threshold > config[index].consumption) && (config[index].id == sensors[index].id) && (ANML_DET[index].sent_anomaly == true)) {
        
        ANML_DET[index].type ="consumption";

        ANML_DET[index].id = sensors[index].id;
        
        Serial.println();
        Serial.println();
        Serial.println("!!!!!!!!!!!! Anomaly Detected: consumption !!!!!!!!!!!!");
        Serial.println();
        Serial.println();
        SEND_ANOMALY2GW(index);
        ANML_DET[index].total_consumption_threshold = 0;
        ANML_DET[index].sent_anomaly = false;
        //esp_deep_sleep_start(); 
      }    
  }
//Task2code: anomaly detection
void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
    for (int index ; index < 5 ; index++){
    anomaly_task(index);  
  }
  vTaskDelay(10);
}
void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}
void setup()
{
  Serial.begin(115200);
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  pinMode(LED1, OUTPUT);                               //setup pin as output for indicator LED
  led_Flash(2, 125);                                   //two quick LED flashes to indicate program start
  if (VCCPOWER >= 0)
  {
    pinMode(VCCPOWER, OUTPUT);                  //For controlling power to external devices
    digitalWrite(VCCPOWER, LOW);                //VCCOUT on. lora device on
  }
  Serial.println();
  Serial.print(F(__TIME__));
  Serial.print(F(" "));
  Serial.println(F(__DATE__));
  Serial.println(F(Program_Version));
  Serial.println();
  Serial.print("CPU Freq: ");
  Serial.println(getCpuFrequencyMhz());
  Serial.println(setCpuFrequencyMhz(40));
  setup_lora();
 for (int index = 0;index < FLOWSENSORS ; index ++){
    Serial.println(index);
    pinMode(sensors[index].pin, INPUT_PULLDOWN);
    //digitalWrite(sensors[index].pin, HIGH);
    attachInterrupt(digitalPinToInterrupt(sensors[index].pin), pulseCounters[index], FALLING);
    esp_sleep_enable_ext0_wakeup(sensors[index].pin,!digitalRead(sensors[index].pin));
 }
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,            /* Task function. */
                    "Task1 : Normal",     /* name of task. */
                    10000,                /* Stack size of task */
                    NULL,                 /* parameter of the task */
                    1,                    /* priority of the task */
                    &Normal,              /* Task handle to keep track of created task */
                    0);                   /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,             /* Task function. */
                    "Task2 : Anomaly",     /* name of task. */
                    10000,                 /* Stack size of task */
                    NULL,                  /* parameter of the task */
                    1,                     /* priority of the task */
                    &Anomaly,              /* Task handle to keep track of created task */
                    1);                    /* pin task to core 1 */
    delay(500); 

}


void loop()
{
   
  /*for (int i = 0; i < 4; i++) {
    if (millis() < sensors[i].oldTime){sensors[i].oldTime = 0;}
    sensor(i);
}*/
}