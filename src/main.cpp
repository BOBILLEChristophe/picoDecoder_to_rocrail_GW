/*----------------------------------------------------------------------------------------------------------------

Ce programme est une passerelle entre un bus de rétro signalisation CAN et un logiciel de gestion de réseau
comme Rocrail, iTrain ou JMRI et quelques autres.

Le transfert des données peut s'opérer soit en mode TCP, (Ethernet ou WiFi) ou encore sur un bus CAN

----------------------------------------------------------------------------------------------------------------

This program is a gateway between an CAN feedback bus and a network management software
like Rocrail, iTrain, or JMRI, among others.

Data transfer can occur either via TCP (Ethernet or WiFi) or over a CAN bus.

----------------------------------------------------------------------------------------------------------------*/

/*
/!\ ATTENTION : Pour une utilisation de la bibliothèque Server.h soit en WiFi soit en Ethernet, il faut modifier le fichier Server.h
/Users/xxxxxx/.platformio/packages/framework-arduinoespressif32/cores/esp32/Server.h:30:18

    virtual void begin() = 0; // For Ethernet
    virtual void begin(uint16_t port=0) = 0; // For WiFi

    voir ici : https://github.com/arduino-libraries/Ethernet/issues/88
*/

#define PROJECT "PicoDecoder gateway for Rocrail"
#define VERSION "0.0.7"
#define AUTHOR "Christophe BOBILLE - www.locoduino.org"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

struct Module
{
  uint16_t ID;
  uint8_t value;
};

struct Message
{
  uint16_t module;
  uint16_t sensor;
  uint8_t value;
};

const uint8_t NBRE_MODULES = 1;
Module module[NBRE_MODULES];

//----------------------------------------------------------------------------------------
//  Select a communication mode
//----------------------------------------------------------------------------------------
// Uncomment the following line if you're using a WiFi or Ethernet
// Comment out if you are using WiFi
#define ETHERNET
// Comment out if you are using Ethernet
// #define WIFI

//----------------------------------------------------------------------------------------
//  Ethernet et WIFI
//----------------------------------------------------------------------------------------
#if defined(ETHERNET) || defined(WIFI)
IPAddress ip(192, 168, 1, 208);
const uint port = 15731;
#endif

//----------------------------------------------------------------------------------------
//  Ethernet
//----------------------------------------------------------------------------------------
#if defined(ETHERNET)
#include <Ethernet.h>
#include <SPI.h>
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
EthernetServer server(port);
EthernetClient client;

//----------------------------------------------------------------------------------------
//  WIFI
//----------------------------------------------------------------------------------------
#elif defined(WIFI)
#include <WiFi.h>
const char *ssid = "**********";
const char *password = "**********";
IPAddress gateway(192, 168, 1, 1);  // passerelle par défaut
IPAddress subnet(255, 255, 255, 0); // masque de sous réseau
WiFiServer server(port);
WiFiClient client;

#endif

//----------------------------------------------------------------------------------------
//  CAN
//----------------------------------------------------------------------------------------
#include <ACAN_ESP32.h>                                  // https://github.com/pierremolinaro/acan-esp32.git
static const uint32_t DESIRED_BIT_RATE = 250UL * 1000UL; // Marklin CAN baudrate = 250Kbit/s

//----------------------------------------------------------------------------------------
//  This gateway hash + Rocrail hash
//----------------------------------------------------------------------------------------
const uint16_t hash = 0x1811; // this gateway hash

//----------------------------------------------------------------------------------------
//  Queues
//----------------------------------------------------------------------------------------
QueueHandle_t canToTcpQueue;
QueueHandle_t tcpToCanQueue;
QueueHandle_t debugQueue; // Queue for debug messages

//----------------------------------------------------------------------------------------
//  Buffers  : Rocrail always send 13 bytes
//----------------------------------------------------------------------------------------
static const uint8_t BUFFER_SIZE = 13;
byte cBuffer[BUFFER_SIZE]; // CAN buffer
byte sBuffer[BUFFER_SIZE]; // Serial buffer

//----------------------------------------------------------------------------------------
//  Task
//----------------------------------------------------------------------------------------
void CANReceiveTask(void *pvParameters);
void TCPSendTask(void *pvParameters);
#if defined(ETHERNET)
void ethernetMonitorTask(void *pvParameters);
#elif defined(WIFI)
void wifiMonitorTask(void *pvParameters);
#endif

//----------------------------------------------------------------------------------------
//  SETUP
//----------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(100);
  }

#if !defined(ETHERNET) && !defined(WIFI) && !defined(CAN)
  Serial.print("Select a communication mode.");
  while (1)
  {
  }
#endif

#if defined(ETHERNET)
  Serial.println("Waiting for Ethernet connection : ");
  // Ethernet initialization
  Ethernet.init(5); // MKR ETH Shield (change depending on your hardware)
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("IP address = ");
  Serial.println(Ethernet.localIP());
  Serial.printf("Port = %d\n", port);

#elif defined(WIFI)
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, password);
  Serial.print("Waiting for WiFi connection : \n\n");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address : ");
  Serial.println(WiFi.localIP());
  Serial.printf("Port = %d\n", port);
  server.begin();
#endif

  Serial.println("Configure ESP32 CAN");
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  settings.mDriverReceiveBufferSize = 100;
  // settings.mDriverTransmitBufferSize = 50;
  settings.mRxPin = GPIO_NUM_21; // Optional, default Tx pin is GPIO_NUM_4
  settings.mTxPin = GPIO_NUM_22; // Optional, default Rx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);
  if (errorCode)
  {
    Serial.print("Configuration error 0x");
    Serial.println(errorCode, HEX);
    return;
  }
  else
    Serial.print("Configuration CAN OK\n\n");

  // Create queues
  canToTcpQueue = xQueueCreate(50, sizeof(CANMessage));
  tcpToCanQueue = xQueueCreate(50, BUFFER_SIZE * sizeof(byte));
  debugQueue = xQueueCreate(50, sizeof(CANMessage)); // Create debug queue

  // Create tasks
  xTaskCreatePinnedToCore(CANReceiveTask, "CANReceiveTask", 4 * 1024, NULL, 3, NULL, 0); // priority 3 on core 0
  xTaskCreatePinnedToCore(TCPSendTask, "TCPSendTask", 4 * 1024, NULL, 5, NULL, 1);       // priority 5 on core 1
#if defined(ETHERNET)
  xTaskCreatePinnedToCore(ethernetMonitorTask, "Ethernet Monitor", 4 * 1024, NULL, 1, NULL, 1); // priority 1 on core 1
#elif defined(WIFI)
  xTaskCreatePinnedToCore(wifiMonitorTask, "WiFi Monitor", 4 * 1024, NULL, 1, NULL, 1); // priority 1 on core 1
#endif

} // end setup

//----------------------------------------------------------------------------------------
//  LOOP
//----------------------------------------------------------------------------------------

void loop()
{
  vTaskDelete(NULL);
} // Nothing to do

//----------------------------------------------------------------------------------------
//   CANReceiveTask
//----------------------------------------------------------------------------------------

void CANReceiveTask(void *pvParameters)
{
  CANMessage frameIn;
  Message message;
  while (true)
  {
    if (ACAN_ESP32::can.receive(frameIn))
    {
      byte idModule = frameIn.id;

      if (module[idModule].value != frameIn.data16[0])
      {
        message.module = idModule;
        for (byte i = 0; i < 16; i++)
        {
          {
            message.sensor = i;
            message.value = ((frameIn.data16[0] & (1 << i)) >> i);
            xQueueSend(canToTcpQueue, &message, portMAX_DELAY);
          }
        }
        module[idModule].value = frameIn.data16[0];
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
} // end loop

//----------------------------------------------------------------------------------------
//   TCPSendTask
//----------------------------------------------------------------------------------------

void TCPSendTask(void *pvParameters)
{
  Message message;
  byte sBuffer[BUFFER_SIZE];
  const uint8_t command = 0x22;
  const bool response = true;
  const uint8_t dlc = 8;

  while (true)
  {
    if (xQueueReceive(canToTcpQueue, &message, portMAX_DELAY))
    {
      if (client || client.connected())
      {
        sBuffer[0] = 0x00;
        sBuffer[1] = command | response;
        sBuffer[2] = (hash & 0xFF00) >> 8; // hash
        sBuffer[3] = hash & 0x00FF;
        sBuffer[4] = dlc;
        sBuffer[5] = (message.module & 0xFF00) >> 8;
        sBuffer[6] = message.module & 0x00FF;
        sBuffer[7] = (message.sensor & 0xFF00) >> 8;
        sBuffer[8] = message.sensor & 0x00FF;
        sBuffer[9] = 0x00;
        sBuffer[10] = message.value;
        sBuffer[11] = 0x00;
        sBuffer[12] = 0x0F;
        client.write(sBuffer, BUFFER_SIZE);
      }
    }
  }
}

#if defined(ETHERNET)

//----------------------------------------------------------------------------------------
//   ethernetMonitorTask
//----------------------------------------------------------------------------------------

void ethernetMonitorTask(void *parameter)
{
  while (true)
  {
    if (!client.connected())
    {
      Serial.println("Connexion au serveur TCP perdue. Tentative de reconnexion...");
      client = server.available();
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // Vérifier toutes les 5 secondes
  }
}

#elif defined(WIFI)

//----------------------------------------------------------------------------------------
//   wifiMonitorTask
//----------------------------------------------------------------------------------------

// Tâche pour surveiller la connexion WiFi et la reconnecter si nécessaire (Core 1)
void wifiMonitorTask(void *parameter)
{
  while (true)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      // xSemaphoreTake(tcpMutex, portMAX_DELAY);
      Serial.println("Connexion au WiFi perdue. Tentative de reconnexion...");
      WiFi.begin(ssid, password);
    }
    // xSemaphoreGive(tcpMutex);
    vTaskDelay(pdMS_TO_TICKS(5000)); // Vérifier toutes les 5 secondes
  }
}

#endif