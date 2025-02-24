/*
  Description: This is the firmware code for the robotic fish 'Mish'.
  Date: 2025-02-18 20:31:31
  Creator: Xiaozhu Lin (linxzh@shanghaitech.edu.cn)
  Last Modification Date: 2025-02-23 16:57:33
  Last Editors: Xiaozhu Lin (linxzh@shanghaitech.edu.cn)
  Version: 0.1.0
*/


#include <Arduino.h>

#include <math.h>
#include <Ticker.h>
#include <WiFi.h>

#include <AsyncUDP.h>

#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "LobotSerialServo.h"


const float rad2deg = 180 / PI;
const float deg2rad = PI / 180;

const float deg2bit = 1000 / 240;
const float bit2deg = 240 / 1000;

const float rad2bit = rad2deg * deg2bit;
const float bit2rad = bit2deg * deg2rad;


const gpio_num_t led_pin = GPIO_NUM_21;
float led_value = 0;

const gpio_num_t servo_tx_pin = GPIO_NUM_4;
const gpio_num_t servo_rx_pin = GPIO_NUM_5;
const int servo_nums = 3;
typedef struct
{
  int id;
  int position;
  int central_position;
  int vin;
  int temp;
  int alive;
} servo_info_t;
servo_info_t servo_info[servo_nums] = {{1, 0, 510, 0, 0, true}, {2, 0, 520, 0, 0, true}, {3, 0, 510, 0, 0, true}};

typedef struct
{
  float theta[servo_nums];
  float speed[servo_nums];
} downward_info_t;
downward_info_t downward_info = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

typedef struct
{
  int id[servo_nums];
  float theta[servo_nums];  // rad \in [-1.047, 1.047]
  float vin[servo_nums];
  int temp[servo_nums];
} upward_info_t;
upward_info_t upward_info = {{0, 0, 0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0, 0, 0}};

const char *ssid = "MAgIC_Fish_Project";
const char *password = "123456789";

AsyncUDP UdpSocket;
unsigned int local_port = 2333;
unsigned int remote_port = 6060;
IPAddress remote_ip(192, 168, 1, 101);

Ticker upward_timer;


void setup()
{
  Serial.begin(115200);

  pinMode(led_pin, OUTPUT);
  pinMode(beep_pin, OUTPUT);

  Serial1.begin(115200, SERIAL_8N1, servo_rx_pin, servo_tx_pin);
  
  Serial.println();
  Serial.print("[SERVO] ID: ");
  for (int i = 0; i < servo_nums; i++)
  {
    servo_info[i].alive = LobotSerialServoReadLoadOrUnload(Serial1, servo_info[i].id) >= 0;
    if (servo_info[i].alive)  // for read function stuck avoid
    {
      Serial.print(servo_info[i].id);
      Serial.print(", ");
    }
  }
  Serial.println("is ALIVE!");
  delay(1000);

  for (int i = 0; i < servo_nums; i++)
  {
    LobotSerialServoMove(Serial1, servo_info[i].id, servo_info[i].central_position, 1000);
  }
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("[WIFI] Connecting to: ");
  Serial.println(ssid);
  while (!WiFi.isConnected())
  {
    delay(1000);  // ms
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("[WIFI] IP is: ");
  Serial.println(WiFi.localIP());
  delay(3000);

  while (!UdpSocket.listen(local_port));
  UdpSocket.onPacket(downward_callback_func);

  upward_timer.attach_ms(20, upward_timing_func);

  ArduinoOTA.begin();  // NOT DELETE IT
}

void loop()
{
  ArduinoOTA.handle();  // NOT DELETE IT

  // int servo_index = 0, set_position = 0;

  // adjust the central value of the servomotor through serial input
  // if(Serial.available() > 0)
  // {
  //   int value = Serial.parseInt();
  //   servo_info[servo_index].central_position = value;
  //   Serial.readStringUntil('\n');
  // }
  // LobotSerialServoMove(Serial1, servo_info[servo_index].id, servo_info[servo_index].central_position, 0);
  
  // generate sine periodic signal to check the working status of the servomotor
  // set_position = sin(2 * PI * millis() / 1000) * 100 + 500;
  // LobotSerialServoMove(Serial1, servo_info[servo_index].id, set_position, 0);

  // Serial.print("set_position:");
  // Serial.print(set_position);
  // Serial.print(", ");
  
  // Serial.print("servo_info[");
  // Serial.print(servo_index);
  // Serial.print("].central_position:");
  // Serial.print(servo_info[servo_index].central_position);
  // Serial.print(", ");
  
  // Serial.print("servo_info[");
  // Serial.print(servo_index);
  // Serial.print("].position:");
  // Serial.print(servo_info[servo_index].position);
  // Serial.print(", ");
  
  // Serial.print("servo_info[");
  // Serial.print(servo_index);
  // Serial.print("].vin:");
  // Serial.print(servo_info[servo_index].vin);
  // Serial.print(", ");
  
  // Serial.print("servo_info[");
  // Serial.print(servo_index);
  // Serial.print("].temp:");
  // Serial.print(servo_info[servo_index].temp);
  // Serial.println();
  
  // delay(20);
}

void upward_timing_func()
{
  digitalWrite(led_pin, LOW);
  
  for (int i = 0; i < servo_nums; i++)  // cost 2.5ms * 3
  {
    if (servo_info[i].alive)
    {
      servo_info[i].position = LobotSerialServoReadPosition(Serial1, servo_info[i].id);
    }
  }
  
  if (servo_info[0].alive)
  {
    servo_info[0].vin = LobotSerialServoReadVin(Serial1, servo_info[0].id);  // cost 2.5ms
  }
  servo_info[1].vin = servo_info[0].vin;
  servo_info[2].vin = servo_info[0].vin;

  if (servo_info[0].alive)
  {
    servo_info[0].temp = LobotSerialServoReadTemp(Serial1, servo_info[0].id);  // cost 2.5ms
  }
  servo_info[1].temp = servo_info[0].temp;
  servo_info[2].temp = servo_info[0].temp;

  for (int i = 0; i < servo_nums; i++)
  {
    upward_info.id[i] = servo_info[i].id;
    upward_info.theta[i] = (servo_info[i].position - servo_info[i].central_position) * bit2rad;  // rad
    upward_info.vin[i] = servo_info[i].vin / 1000.0;
    upward_info.temp[i] = servo_info[i].temp;
  }

  uint8_t packet[sizeof(upward_info_t)];
  memcpy(packet, &upward_info, sizeof(packet));

  UdpSocket.connect(remote_ip, remote_port);
  UdpSocket.write(packet, sizeof(upward_info_t));
  UdpSocket.close();

  digitalWrite(led_pin, HIGH);
}

void downward_callback_func(AsyncUDPPacket packet)
{
  digitalWrite(led_pin, LOW);

  memcpy(&downward_info, packet.data(), sizeof(downward_info));
  
  for (int i = 0; i < servo_nums; i++)  // servo control
  {
    downward_info.theta[i] = fclamp(downward_info.theta[i], -1.047, 1.047);  // rad
    int desired_position = round(servo_info[i].central_position + downward_info.theta[i] * rad2bit);  // bit
    
    int cost_time;
    if (downward_info.speed[i] > 0)  // ATTENTION !!!
    {
      downward_info.speed[i] = fclamp(downward_info.speed[i], 0.05, 5.23);  // rad/s
      int current_theta = (servo_info[i].position - servo_info[i].central_position) * bit2rad;  // rad
      cost_time = abs(round((current_theta - downward_info.theta[i]) * 1000.0 / downward_info.speed[i]));  // ms
    }
    else
    {
      cost_time = 0;  // ms
    }
    LobotSerialServoMove(Serial1, servo_info[i].id, desired_position, cost_time);
  }

  digitalWrite(led_pin, HIGH);
}


float fclamp(float val, float O_Min, float O_Max) {
  return min(max(val, O_Min), O_Max);
}

float fmap(float val, float I_Min, float I_Max, float O_Min, float O_Max) {
  return (((val - I_Min) * ((O_Max - O_Min) / (I_Max - I_Min))) + O_Min);
}
