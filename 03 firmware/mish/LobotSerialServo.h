#define GET_LOW_BYTE(A) (uint8_t)((A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 以A为高八位 B为低八位 合并为16位整形

#define ID_ALL 254  //ID为254时，会向所有舵机进行广播，可用于读取未知ID的舵机信息

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36


//校验和
byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

int LobotSerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  unsigned long startTime = millis();
  while (millis() - startTime < 1000)
  {
    if (SerialX.available() > 0)
    {
      rxBuf = SerialX.read();
      
      if (!frameStarted)
      {      
        if (rxBuf == LOBOT_SERVO_FRAME_HEADER)
        {
          frameCount++;
          if (frameCount == 2)
          {
            frameCount = 0;
            frameStarted = true;
            dataCount = 1;
          }
        }
        else
        {
          frameStarted = false;
          dataCount = 0;
          frameCount = 0;
        }
      }

      if (frameStarted)
      {
        recvBuf[dataCount] = (uint8_t)rxBuf;
        if (dataCount == 3)
        {
          dataLength = recvBuf[dataCount];
          if (dataLength < 3 || dataCount > 7)
          {
            dataLength = 2;
            frameStarted = false;
          }
        }
        dataCount++;
        if (dataCount == dataLength + 3)
        {
          if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1])
          {
            frameStarted = false;
            memcpy(ret, recvBuf + 4, dataLength);
            return 1;
          }
          return -1;
        }
      }
    }
  }
  
  return -2;
}

// //解析接收到的数据包信息，并返回
// int LobotSerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
// {
//   bool frameStarted = false;
//   bool receiveFinished = false;
//   byte frameCount = 0;
//   byte dataCount = 0;
//   byte dataLength = 2;
//   byte rxBuf;
//   byte recvBuf[32];
//   byte i;

//   // delayMicroseconds(100);  // add 2025/02/19 by linxzh


//   while (SerialX.available())
//   {
//     rxBuf = SerialX.read();
//     // delayMicroseconds(100);
    
//     if (!frameStarted)
//     {      
//       if (rxBuf == LOBOT_SERVO_FRAME_HEADER)
//       {
//         frameCount++;
//         if (frameCount == 2)
//         {
//           frameCount = 0;
//           frameStarted = true;
//           dataCount = 1;
//         }
//       }
//       else
//       {
//         frameStarted = false;
//         dataCount = 0;
//         frameCount = 0;
//       }
//     }

//     if (frameStarted)
//     {
//       recvBuf[dataCount] = (uint8_t)rxBuf;
//       if (dataCount == 3)
//       {
//         dataLength = recvBuf[dataCount];
//         if (dataLength < 3 || dataCount > 7)
//         {
//           dataLength = 2;
//           frameStarted = false;
//         }
//       }
//       dataCount++;
//       if (dataCount == dataLength + 3)
//       {
//         if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1])
//         {
//           frameStarted = false;
//           memcpy(ret, recvBuf + 4, dataLength);
//           return 1;
//         }
//         return -1;
//       }
//     }
//   }
//   return -2;
// }

//写入舵机ID
void LobotSerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  SerialX.write(buf, 7);
}

//控制舵机转动
void LobotSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  SerialX.write(buf, 10);
}

//读取ID
int LobotSerialServoReadID(HardwareSerial &SerialX)
{
  int count = 10000;
  int ret;
  byte buf[6];
  
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = ID_ALL;  //ID_ALL为254，表示向所有舵机进行广播，可用于读取未知ID的舵机信息
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_ID_READ;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);

  
  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2048;
  return ret;
}

//读取舵机位置
int LobotSerialServoReadPosition(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
  
  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -1;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;
  return ret;
}

//读取偏差
int LobotSerialServoReadDev(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6],i;

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_ANGLE_OFFSET_READ;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
  
  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    //ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
    {
      if(buf[1]>>7 == 0)
      {
        ret = (int16_t)buf[1];
      }
      else{
        i = ~buf[1]+1;
        ret = (int16_t) -i;
      }
    }
  else
    ret = -2048;
  return ret;
}

//读取转动范围
int retL;
int retH;
int LobotSerialServoReadAngleRange(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_ANGLE_LIMIT_READ;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
  
  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    {
      retL = (int16_t)BYTE_TO_HW(buf[2], buf[1]); 
      retH = (int16_t)BYTE_TO_HW(buf[4], buf[3]);
    }
  else
    ret = -2048;
  return ret;
}

//读取电压
int LobotSerialServoReadVin(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2049;
    
  return ret;
}

//读取电压范围
int vinL;
int vinH;
int LobotSerialServoReadVinLimit(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_LIMIT_READ;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
  
  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    {
      vinL = (int16_t)BYTE_TO_HW(buf[2], buf[1]); 
      vinH = (int16_t)BYTE_TO_HW(buf[4], buf[3]);
    }
  else
    ret = -2048;
  return ret;
}

//读取温度报警阈值
int LobotSerialServoReadTempLimit(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_TEMP_MAX_LIMIT_READ;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2049;
    
  return ret;
}

//读取温度
int LobotSerialServoReadTemp(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_TEMP_READ;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2049;
    
  return ret;
}

//读取舵机状态
int LobotSerialServoReadLoadOrUnload(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_READ;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2049;
    
  return ret;
}

//停止转动
void LobotSerialServoStopMove(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
}

//设置舵机模式
void LobotSerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);

  SerialX.write(buf, 10);
}

//舵机上电
void LobotSerialServoLoad(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  
  SerialX.write(buf, 7);
}

//舵机掉电
void LobotSerialServoUnload(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  
  SerialX.write(buf, 7);
}



