#include <sstream>
#include <Preferences.h>
#include <Bounce2.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>  
#include <Wire.h>
#include <SPI.h>
#include "MCP23S17.h"
#include <EEPROM.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include <BLE2902.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "helpers.h"
#include "HardwareSerial.h"
using namespace std;

HardwareSerial stm32(2); /////////rx 16 tx 17

Preferences preferences;
int microInterval = MICRO_INTERVAL_FAST;

BLEServer* pServer = NULL;
BLECharacteristic* pPostionCharacteristic = NULL;
BLECharacteristic* pPauseCharacteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

Bounce debouncedEStop = Bounce(); 

const int stepPins[] = {0,1,2,3,4,5};   
const int dirPins[] = {6,7,8,9,10,11};

SemaphoreHandle_t xMutex;

TaskHandle_t InterfaceMonitorTask;
TaskHandle_t GPIOLoopTask;
TimerHandle_t wtmr;

SPIClass hspi( HSPI );
//MCP23S17：扩展io口芯片
MCP23S17 outputBank( &hspi, 15, 0 );
MCP23S17 inputBank( &hspi, 15, 1 );

volatile bool isPausedEStop = false;
volatile bool isRateLimiting = false;

int64_t currentMicros = esp_timer_get_time();
int64_t  previousMicros = 0;

int64_t currentMicrosBle = esp_timer_get_time();
int64_t  previousMicrosBle = 0;

static long servo_pos[6];

volatile struct acServo motors[6];

boolean pinState = false;

uint16_t motorStepDirValue = 0;   
uint16_t motorStepDirValue2 = 0;   

static volatile float arr[6]={0,0,0,0,0,0};
int num = 0;
int posBUF[6] = {0,0,0,0,0,0};
boolean posState = false; 
volatile float arrble[]={0,0,0,0,0,0};

//设置电机的值
void setPos()
{  
    for(int i = 0; i < 6; i++)
    {    
        servo_pos[i] = arr[i];
    }

    xSemaphoreTake( xMutex, portMAX_DELAY );//信号采集
    
    for(int i = 0; i < 6; i++)
    {
        motors[i].targetpos = servo_pos[i];
    }   
    
    xSemaphoreGive( xMutex );    //信号量发送 
}

//处理来自电脑的数据
void process_data ( char * data)
{ 
    int i = 0; 
    char *tok = strtok(data, ",");
    
    float arrRaw[]={0,0,0,0,0,0};
    float arrRateLimited[]={0,0,0,0,0,0};
    
    while (tok != NULL) 
    {
      double value = (float)atof(tok);               
      arrRaw[i++] = value;
      tok = strtok(NULL, ",");
    }         

  if(!isPausedEStop)
  {
    if(isRateLimiting)
    {    
        bool isNotWithinLimit = false;
        
        for(int i=0;i<6;i++)
        {
          arrRateLimited[i] = rateLimit(arrRaw[i],arr[i]);    
          float diff = arr[i] - arrRaw[i];
          if (diff > .1 || diff < -.1) 
          {
            isNotWithinLimit = true;
          }  
        }

        if(!isNotWithinLimit)
        {
          isRateLimiting = false;
        }

        for(int i=0;i<6;i++)
        {
          arr[i] = arrRateLimited[i];
        }
    }
    else
    {
        for(int i=0;i<6;i++)
        {
          arr[i] = arrRaw[i];
        }
    }
    setPos();
  }
} 

void ping( TimerHandle_t xTimer )
{ 
    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;

    xTimerStart(wtmr, 0);
}

//从蓝牙PA-UUID 接收
class BlePauseCallback: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristic) 
    {
   
      string result = pCharacteristic->getValue().c_str();  //c_str()把C++类的字符转为C类型
    
      int i = atoi(pCharacteristic->getValue().c_str());//接收到16进制：30，转十进制为48，对应ACII为0
      Serial.print("从PA接收到：");
      Serial.println(i);
      
      
      if(i == 3)
      {  
        pauseEStop();
      }
      else if(i == 4)
      {
        resumeEStop();
      }
      else if(i>=5){//发送给stm32
        
        stm32.print(i);  
        stm32.println();
      }
      else if(i==1)//测试升降37
      {
        posState = true; 
        float pos = 0;
        Serial.println("test:up");
        for(int i = 0;i < 1000;i++)//2ms发一次，公发1000次，共2秒 
        {
          pos += 2.047;
          xSemaphoreTake( xMutex, portMAX_DELAY );
          for(int j=0;j<6;j++)
          {
            motors[j].targetpos = pos;//升到中点
            arr[j]= motors[j].targetpos;//记录，用于对比
            Serial.println(arr[j]);
          }
          xSemaphoreGive( xMutex );
          delay(2);//2ms
        }
        
        
      }
      else if(i==2)//测试下降36
      {
        posState = false; 
        Serial.println("test:down");
        float posBUF,pos;
        /*
        for(int i = 0;i < 1540;i++)
        {
          xSemaphoreTake( xMutex, portMAX_DELAY );//与xSemaphoreGive( xMutex );搭配使用，锁住信号量
          for(int j=0;j<6;j++)
          {
            //计算没2ms下降的高度
            posBUF = arr[j]/1000;
            pos =arr[j] - posBUF;
            if(pos < 0)
              pos = 0;
            
            //motors[j].targetpos = 0;
            motors[j].targetpos = pos;
            arr[j]= motors[j].targetpos;
            Serial.println(arr[j]);
          }
          xSemaphoreGive( xMutex );
          delay(2);
        }
        */
        /////////////////////////
        while(arr[0] != 0 || arr[1] != 0 || arr[2] != 0 || arr[3] != 0 || arr[4] != 0 || arr[5] != 0 || arr[6] != 0)
        // 
        {
          xSemaphoreTake( xMutex, portMAX_DELAY );//与xSemaphoreGive( xMutex );搭配使用，锁住信号量
          for(int j=0;j<6;j++)
          {
            //每2ms下降2高度
            pos =arr[j] - 2;
            if(pos < 0)
              pos = 0;
            
            motors[j].targetpos = pos;
            arr[j]= motors[j].targetpos;
            Serial.println(arr[j]);
          }
          xSemaphoreGive( xMutex );
          delay(5);
        }
        /////////////////////////
      }
    }    
};

//从POS-UUID发送蓝牙消息
class BlePostionCallback: public BLECharacteristicCallbacks
{
  void onRead(BLECharacteristic *pCharacteristic) 
    {
      std::ostringstream os;//os：蓝牙发送的内容

      os << (int)arr[0] << "," << (int)arr[1] << "," << (int)arr[2] << "," << (int)arr[3] << "," << (int)arr[4] << "," << (int)arr[5];//通过蓝牙发送位置信息

      pPauseCharacteristic->setValue(os.str());
    }  
};


//从FIL-UUID 接收
class BleFilterCallback: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *pCharacteristic) 
    {
      //接收、处理数据
      std::string value = pCharacteristic->getValue();//接收蓝牙数据
      if (value.length() > 0) 
      {
          int i = 0; 
          char *tok;
          char *rawTokens = new char[value.size()+1];
          strcpy(rawTokens, value.c_str());//strcpy把后面内容复制到前
          //Serial.println(rawTokens);
          tok = strtok(rawTokens, ",");//数据以“，”分割
          
          while (tok != NULL) 
          {
              double rawble = (float)atof(tok);//char to int
              arrble[i++] = rawble;
              tok = strtok(NULL, ",");
                
          }   
          if(!isPausedEStop && posState == true)
          {          
            xSemaphoreTake( xMutex, portMAX_DELAY );

            for(i=0; i<6; i++)
            {
              motors[i].targetpos = mapfloat(arrble[i],0,100,0,4094);
              arr[i] = motors[i].targetpos;
              Serial.print(arr[i]);
              Serial.print(",");
            }
            Serial.println();

            xSemaphoreGive( xMutex );
             
            //motors[i].targetpos = mapfloat(arrble[0],100,0,856,3238);
               
          }
      }
    }
};

class ServerCallbacks: public BLEServerCallbacks 
{  
    void onConnect(BLEServer* pServer) 
    {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setupBle()
{
  Serial.println("Starting BLE init!");

  BLEDevice::init("展视未来");                                   //初始化一个蓝牙设备
  BLEServer *pServer = BLEDevice::createServer();               // 创建一个蓝牙服务器
  pServer->setCallbacks(new ServerCallbacks());                 //服务器回调函数设置
  BLEService *pService = pServer->createService(SERVICE_UUID);  //创建一个BLE服务
  
  pPauseCharacteristic = pService->createCharacteristic(
                                         PAUSECHARACTERISTIC_UUID,            //接收按钮值
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );

  BLECharacteristic *pFilterCharacteristic = pService->createCharacteristic(//接收位置信息
                                         FILTERCHARACTERISTIC_UUID,           
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pPostionCharacteristic  = pService->createCharacteristic(                 //发送位置信息
                      POSITIONCHARACTERISTIC_UUID,                            
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
                                       
  
  pPauseCharacteristic->addDescriptor(new BLE2902());
  pPostionCharacteristic->addDescriptor(new BLE2902());
  
  pService->start();
  
  pPostionCharacteristic->setCallbacks(new BlePostionCallback());
  pPauseCharacteristic->setCallbacks(new BlePauseCallback());
  pFilterCharacteristic->setCallbacks(new BleFilterCallback());  
  

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  //pAdvertising->setScanResponse(true);
  //pAdvertising->setMinPreferred(0x12);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  
  BLEDevice::startAdvertising(); 
}

//-------------------------------------------------------------------------------------------------------------------------------------//
//start here
//----------------------------------------------------------------------------------------------------------------------------------- //
void setup()
{
  Serial.begin(115200); 
  EEPROM.begin(4096); 
  initConfigStorage();
  setupBle();
  stm32.begin(115200, SERIAL_8N1, 16, 17); /////////rx2 tx2

  pinMode(ESTOPPIN, INPUT_PULLUP);
  debouncedEStop.attach(ESTOPPIN);
  debouncedEStop.interval(5); 
  
  num = EEPROM.read(20);//读取内存中点位置数据
  num++;
  EEPROM.write(20,num);
  EEPROM.commit();
  
  isPausedEStop = !digitalRead(ESTOPPIN);//读取急停引脚状态
  if(num >250)
  {
    while(1); 
  }
  
  Serial.print("init Estop check:");
  Serial.println(isPausedEStop);         
  
  xMutex = xSemaphoreCreateMutex();
 
  xTaskCreatePinnedToCore(
                    InterfaceMonitorCode,   /* 任务函数Task function. */
                    "InterfaceMonitor",     /* 任务名称name of task. */
                    10000,       /* 堆栈大小Stack size of task */
                    NULL,        /* 参数parameter of the task */
                    1,           /* 优先级priority of the task */
                    &InterfaceMonitorTask,      /* 跟踪创建的任务Task handle to keep track of created task */
                    0);          /* 任务到核心0   pin task to core 0 */                  

  xTaskCreatePinnedToCore(
                    GPIOLoop,   /* Task function. */
                    "GPIOLoopTask",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &GPIOLoopTask,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
   
  inputBank.begin();
  outputBank.begin();

  setupPWMpins();

  wtmr = xTimerCreate("wtmr", pdMS_TO_TICKS(1000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(ping));
  xTimerStart(wtmr, 0);
}


void setupPWMpins() 
{
    for(int i=0;i<6;i++)
    {
        motors[i].stepPin = stepPins[i];
        motors[i].dirPin = dirPins[i];
        motors[i].pinState = false;
        motors[i].currentpos = 0;
        motors[i].targetpos = 0;

        outputBank.pinMode( motors[i].stepPin, OUTPUT );
        outputBank.pinMode( motors[i].dirPin, OUTPUT );
     }
}
  
void handleStepDirection() 
{
  currentMicros = esp_timer_get_time();
  int dif = currentMicros - previousMicros;
  if (dif >= microInterval) 
  {
        if (pinState) {
          
          pinState = false;

          xSemaphoreTake( xMutex, portMAX_DELAY );

          for(int i =0;i<6;i++)
          {
            motorStepDirValue = BIT_CLEAR(motorStepDirValue,stepPins[i]);
            motorStepDirValue2 = BIT_CLEAR(motorStepDirValue2,stepPins[i]);
          }
          
          xSemaphoreGive( xMutex );
          outputBank.digitalWrite(motorStepDirValue);

        } 
        else 
        { 
            pinState = true;

            xSemaphoreTake( xMutex, portMAX_DELAY );

            for(int i =0;i<6;i++)
            {
              if(motors[i].currentpos > motors[i].targetpos)
              {
                 motorStepDirValue = BIT_CLEAR(motorStepDirValue,dirPins[i]);
                 motorStepDirValue2 = BIT_CLEAR(motorStepDirValue2,dirPins[i]);
              }
              else if(motors[i].currentpos < motors[i].targetpos)
              { 
                motorStepDirValue = BIT_SET(motorStepDirValue,dirPins[i]);
                motorStepDirValue2 = BIT_SET(motorStepDirValue2,dirPins[i]);
              }
            }
            
            for(int i =0;i<6;i++)
            {
              if(motors[i].currentpos > motors[i].targetpos)
              {
                  motors[i].currentpos--;
                  motorStepDirValue2 = BIT_SET(motorStepDirValue2,stepPins[i]);
              }
              else if(motors[i].currentpos < motors[i].targetpos)
              { 
                  motors[i].currentpos++;
                  motorStepDirValue2 = BIT_SET(motorStepDirValue2,stepPins[i]);
              }
            }

            #ifdef DEBUG_MOTORS
   
              for(int i=0;i<6;i++)   
              {
                Serial.print(motors[i].currentpos);
                Serial.print(",");
              }
                  
              Serial.println("");
         
            #endif
     
            xSemaphoreGive( xMutex );

            outputBank.digitalWrite(motorStepDirValue);     
            outputBank.digitalWrite(motorStepDirValue2); 
        }
        
        previousMicros = currentMicros;         
  }
}

void loop()
{    
  
}

void processIncomingByte (const byte inByte)
{
    static char input_line [MAX_INPUT];
    static unsigned int input_pos = 0;
  
    switch (inByte)
    {
        case 'X':   // end of text
          input_line [input_pos] = 0;  // terminating null byte
          process_data (input_line);          
          input_pos = 0;  
          break;
        case '\r':   // discard carriage return
          break;
        
        default:
          //buffer data
          if (input_pos < (MAX_INPUT - 1))
            input_line [input_pos++] = inByte;
          break;
    } 
} 

void checkEStop()
{

  debouncedEStop.update();

  int value = debouncedEStop.read();

  if ( debouncedEStop.fell() ) 
  {  // Call code if button transitions from HIGH to LOW
     pauseEStop();
  }

   if ( debouncedEStop.rose() ) 
   {  // Call code if button transitions from HIGH to LOW
      resumeEStop();
   } 
}

void resumeEStop()
{
    isPausedEStop = false;
    isRateLimiting = true;

    notifyPausedStatus();
     
    Serial.print("Estop:");
    Serial.println(isPausedEStop);
}

void pauseEStop()
{ 
     isPausedEStop = true;
     isRateLimiting = false;
     
     notifyPausedStatus();
      
     Serial.print("Estop:");
     Serial.println(isPausedEStop);
}

void notifyPausedStatus()
{
    if(deviceConnected)
    {
        pPauseCharacteristic->setValue(BoolToString(isPausedEStop));
        pPauseCharacteristic->notify(); 
    }
}

//检查蓝牙通知
void checkBleNotify()
{

  currentMicrosBle  = esp_timer_get_time();
  int dif = currentMicrosBle  - previousMicrosBle ;
  
  if (dif >= MICRO_INTERVAL_BLE_SEND ) 
  {
    if (deviceConnected) 
    {
        xSemaphoreTake( xMutex, portMAX_DELAY );
            
        String output;
        for(int i=0;i<6;i++)   
        {
          output +=  String(motors[i].currentpos);
        
          if(i<5)
            output +=",";
        }
               
        xSemaphoreGive( xMutex );
            
        pPostionCharacteristic->setValue(output.c_str());
        pPostionCharacteristic->notify();   
    }
    previousMicrosBle  = currentMicrosBle;    
  }
}

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}

void InterfaceMonitorCode( void * pvParameters )
{
  for(;;)
  {
    while (Serial.available () > 0)
      processIncomingByte (Serial.read ());
   
    checkEStop();
    checkBleNotify();
    //用于断开重连
    if (!deviceConnected && oldDeviceConnected) 
    {
          delay(50); // give the bluetooth stack the chance to get things ready
          pServer->startAdvertising(); // restart advertising
          Serial.println("start advertising");
          oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) 
    {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }  
  }
}

void GPIOLoop( void * pvParameters )
{
  int i=1;
  for(;;)
  {
      handleStepDirection();     
  }
}

bool initConfigStorage() 
{
  return preferences.begin(NAMESPACE, false);
}

int getAxis1Filter() 
{
  return preferences.getInt(AXIS1_KEY, 100);
}

int getAxis2Filter() 
{
  return preferences.getInt(AXIS2_KEY, 100);
}

int getAxis3Filter() 
{
  return preferences.getInt(AXIS3_KEY, 100);
}

int getAxis4Filter() 
{
  return preferences.getInt(AXIS4_KEY, 100);
}

int getAxis5Filter() 
{
  return preferences.getInt(AXIS5_KEY, 100);
}

int getAxis6Filter() 
{
  return preferences.getInt(AXIS6_KEY, 100);
}
