#include <Slave_esp_wifi.h>
#include <PID_controller.h>

float Timer = 0;

void TimeCount()
{
  Timer = micros();
}

// MAC address of the sender - Middle ESP32
uint8_t masterAddress[] = {0x48, 0xE7, 0x29, 0x9F, 0x3F, 0x44};

// New Slave MAC
uint8_t New_MAC_Address[] = {0x48, 0xE7, 0x29, 0x96, 0x77, 0x44};

// PMK and LMK keys
static const char* PMK_KEY_STR = "_A_H_L_T_T_T_ED3";
static const char* LMK_KEY_STR = "_SON_DINH_VU_ED3";

// Define variables to store data received from ESPNOW
int PWM;
byte X_value;
byte Y_value;
byte leftB;
byte rightB;

float PRate;
float IRate;
float DRate;

float PAngle;
float IAngle;
float DAngle;

// Define a ESPNOW received message structure
typedef struct {
  int P;
  byte XJS;
  byte YJS;
  byte LB;
  byte RB;

  float PR;
  float IR;
  float DR;

  float PA;
  float IA;
  float DA;
} Wifi_receivedMessage;
 
// Create a structured object for monitor incoming data
Wifi_receivedMessage controllerData;

// Define a wifi joystick message structure
typedef struct {
  float time;
  float voltage;
  float k_picth;
  float k_roll;
} Wifi_sentMessage;
 
// Create a structured object for joystick incoming data
Wifi_sentMessage sensorData;

// Create a middle peer object
esp_now_peer_info_t masterPeer;

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&controllerData, incomingData, sizeof(controllerData));

  PWM = controllerData.P;
  X_value = controllerData.XJS;
  Y_value = controllerData.YJS;
  leftB = controllerData.LB;
  rightB = controllerData.RB;

  PRate = controllerData.PR;
  IRate = controllerData.IR;
  DRate = controllerData.DR;

  PAngle = controllerData.PA;
  IAngle = controllerData.IA;
  DAngle = controllerData.DA;
}

void init_ESPNOW_Slave()
{
  // Set Slave ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Change MAC
  esp_wifi_set_mac(WIFI_IF_STA, New_MAC_Address);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Set the PMK key for Slave ESP32
  esp_now_set_pmk((uint8_t *)PMK_KEY_STR);

  // Register Middle ESP32 peer
  memcpy(masterPeer.peer_addr, masterAddress, 6);
  masterPeer.channel = 0;

    ///*** Set the Middle ESP32's LMK ***///
    for (uint8_t i = 0; i < 16; i++)
    {
      masterPeer.lmk[i] = LMK_KEY_STR[i];
    }

  masterPeer.encrypt = true; // Only middle peer is accessible

  // Add middle peer   
  if (esp_now_add_peer(&masterPeer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register callback function for received data
  esp_now_register_recv_cb(OnDataRecv);
  // Register callback function for sent data
  // esp_now_register_send_cb(OnDataSent); 
}

void sendingData()
{
  sensorData.time = Timer;
  sensorData.voltage = in_voltage;
  sensorData.k_picth = KalmanAngleRoll;
  sensorData.k_roll = KalmanAnglePitch;

  esp_err_t result = esp_now_send(masterAddress, (uint8_t *) &sensorData, sizeof(sensorData));

}

void Print_PID_Value (){
  Serial.print(" [");
  Serial.printf("%.4f", PRate);
  Serial.print (" ");
  Serial.print("");
  Serial.printf("%.4f", IRate);
  Serial.print ("  ");
  Serial.printf("%.4f", DRate);
  Serial.print("] \n");
}

