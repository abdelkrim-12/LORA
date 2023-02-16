
#define NSS  5
#define SCK  18
#define MISO  19
#define MOSI  23
#define NRESET  27
#define LED1  -1
#define DIO0  35
#define DIO1  -1
#define DIO2  -1
#define VCCPOWER  -1
#define ENA  2

#define LORA_DEVICE DEVICE_SX1278                   //we need to define the device we are using

constexpr unsigned long uS_TO_S_FACTOR = 1000000;    /* Conversion factor for micro seconds to seconds */
constexpr unsigned long TIME_TO_SLEEP =   10;        /* Time ESP32 will go to sleep (in seconds) */
constexpr unsigned long interval = 5000;
constexpr float calibrationFactor = 9.22;
constexpr int threshold_water = 150;
constexpr int threshold_duration = 10;
RTC_DATA_ATTR int bootCount = 0;

uint8_t TXPacketL;
uint32_t TXPacketCount, startmS, endmS;
#define FLOWSENSORS 4

//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
constexpr uint32_t Frequency = 434000000;           //frequency of transmissions in hertz
constexpr uint32_t Offset = 0;                      //offset frequency for calibration purposes

constexpr uint8_t Bandwidth = LORA_BW_125;          //LoRa bandwidth
constexpr uint8_t SpreadingFactor = LORA_SF7;       //LoRa spreading factor
constexpr uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
constexpr uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto

constexpr int8_t TXpower = 20;                      //LoRa transmit power in dBm

constexpr uint16_t packet_delay = 5000;             //mS delay between packets

//constexpr int watersensorPin[4] = {33,34,35,36};

//const String sensor_id[4] = {"lavabo_1","lavabo_2","lavabo_3","lavabo_4"};

struct Config
{
    int duration;
    int consumption;
    String id;
};

Config config[4];

struct Sensor {
  gpio_num_t pin;
  String id;
  volatile byte pulseCount;
  float flowRate;
  float flowMilliLitres;
  float totalMilliLitres;
  float totalLitres;
  byte pulse1Sec;
  unsigned long previousMillis;
  unsigned long currentMillis;
  unsigned long oldTime;
  unsigned int frac;
  int duration;
  int total_duration;
  volatile int check;
};

Sensor sensors[] = {
  { GPIO_NUM_36, "Lavabo_1", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { GPIO_NUM_39, "Lavabo_2", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { GPIO_NUM_34, "Lavabo_3", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { GPIO_NUM_32, "Lavabo_4", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

struct detection {
  bool sent_anomaly;
  String id;
  String type;
  unsigned long total_duration_threshold;
  unsigned long total_consumption_threshold;
  //unsigned long SentTotal;
};

detection ANML_DET[4];