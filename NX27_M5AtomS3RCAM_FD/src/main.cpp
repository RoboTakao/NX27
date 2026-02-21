#include <M5Unified.h>
#include <Wire.h>          // I2C setting
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "camera_pins.h"
#include <esp_camera.h>
#include <fb_gfx.h>
#include "human_face_detect_msr01.hpp"

#define SERVICE_UUID "1010"
#define CHRX_UUID "1012"
#define CHTX_UUID "1011"

#define SLAVE_ADDR 0x08

const float Pi = 3.141593;

byte joyLX=100, joyLY=100, joyRX=100, joyRY=100, joyLSW, joyRSW, joyLDistance, joyRDistance, joyPitch;

int xf=160, yf=120;
int xfpre=160, yfpre=120;
int CamX, CamY;

BLEServer* pServer = NULL;
BLECharacteristic* pCharTx = NULL;
BLECharacteristic* pCharRx = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

const uint8_t Srv0 = 5; //GPIO Right Leg Upper
const uint8_t Srv1 = 6; //GPIO Right Leg Under
const uint8_t Srv2 = 7; //GPIO Left leg Upper
const uint8_t Srv3 = 8; //GPIO Left leg Under
const uint8_t Srv4 = 38; //GPIO Neck

const uint8_t srv_CH0 = 0, srv_CH1 = 1, srv_CH2 = 2, srv_CH3 = 3, srv_CH4 = 4; //チャンネル
const double PWM_Hz = 50;   //PWM周波数
const uint8_t PWM_level = 14; //PWM 14bit(0～16384)

int pulseMIN = 410;  //0deg 500μsec 50Hz 14bit : PWM周波数(Hz) x 2^14(bit) x PWM時間(μs) / 10^6
int pulseMAX = 2048;  //180deg 2500μsec 50Hz 14bit : PWM周波数(Hz) x 2^14(bit) x PWM時間(μs) / 10^6

int cont_min = 0;
int cont_max = 180;

int angZero[] = {92, 89, 83, 96, 95}; //Trimming
int ang0[5];
int ang1[5];
int ang_b[5];
char ang_c[5];
float ts=100;  //100msごとに次のステップに移る
float td=10;   //10回で分割

float amp_r = 6.6;
float amp_l = 6.9;
float amp_rt = 4.0;
float amp_lt = 4.3;
int nee = 25;
int fws = 13;
int fws_t = 13;
int head_p = 0;
int nee_p = 0;
int nee_r = 0;

float angleX, angleZ;
float acc_angle_x, acc_angle_z;
float preAngleZ = 0;

float interval_pid, preInterval_pid, Tpid;

float eZ = 0, eZ_pre = 0;
float deZ = 0;
float ieZ = 0;
float uZ = 0;

float Kpf = 1.2;
float Kdf = 0.04;
float Kif = 0.0;

float target_angle = 12.75*1.8;

int g_flag = 0;

int mode_flag = 1;

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    if (value.length()>0) {
      joyLX=value[0];
      joyLY=value[1];
      joyRX=value[2];
      joyRY=value[3];
      joyLSW=value[4];
      joyRSW=value[5];
      joyPitch=value[6];
    }
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void setupBLE() {
  BLEDevice::init("NX27_M5AtomS3R");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharTx = pService->createCharacteristic(CHTX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharRx = pService->createCharacteristic(CHRX_UUID, BLECharacteristic::PROPERTY_WRITE_NR);
  pCharRx ->setCallbacks(new MyCallbacks());
  pCharTx->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void checkBLE() {
    // notify changed value
  if (deviceConnected) {
      pCharTx->setValue((uint8_t*)&value, 6);
      pCharTx->notify();
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}

// Forward Step
int f_s[20][6]={
  {fws ,nee                        ,fws ,-nee                        ,0 ,0},
  {fws ,nee+int(amp_r*sin(Pi/8))   ,fws ,-nee+int(amp_r*sin(Pi/8))   ,0 ,1},
  {fws ,nee+int(amp_r*sin(Pi/4))   ,fws ,-nee+int(amp_r*sin(Pi/4))   ,0 ,1},
  {fws ,nee+int(amp_r*sin(Pi*3/8)) ,fws ,-nee+int(amp_r*sin(Pi*3/8)) ,0 ,0},
  {fws ,nee+int(amp_r*sin(Pi/2))   ,fws ,-nee+int(amp_r*sin(Pi/2))   ,0 ,0},
  {0   ,nee+int(amp_r*sin(Pi/2))   ,0   ,-nee+int(amp_r*sin(Pi/2))   ,0 ,0},
  {-fws,nee+int(amp_r*sin(Pi/2))   ,-fws,-nee+int(amp_r*sin(Pi/2))   ,0 ,-1},
  {-fws,nee+int(amp_r*sin(Pi*5/8)) ,-fws,-nee+int(amp_r*sin(Pi*5/8)) ,0 ,-1},
  {-fws,nee+int(amp_r*sin(Pi*3/4)) ,-fws,-nee+int(amp_r*sin(Pi*3/4)) ,0 ,0},
  {-fws,nee+int(amp_r*sin(Pi*7/8)) ,-fws,-nee+int(amp_r*sin(Pi*7/8)) ,0 ,0},
  {-fws,nee                        ,-fws,-nee                        ,0 ,0},
  {-fws,nee+int(amp_l*sin(Pi*9/8)) ,-fws,-nee+int(amp_l*sin(Pi*9/8)) ,0 ,-1},
  {-fws,nee+int(amp_l*sin(Pi*5/4)) ,-fws,-nee+int(amp_l*sin(Pi*5/4)) ,0 ,-1},
  {-fws,nee+int(amp_l*sin(Pi*11/8)),-fws,-nee+int(amp_l*sin(Pi*11/8)),0 ,0},
  {-fws,nee+int(amp_l*sin(Pi*3/2)) ,-fws,-nee+int(amp_l*sin(Pi*3/2)) ,0 ,0},
  {0   ,nee+int(amp_l*sin(Pi*3/2)) ,0   ,-nee+int(amp_l*sin(Pi*3/2)) ,0 ,0},
  {fws ,nee+int(amp_l*sin(Pi*3/2)) ,fws ,-nee+int(amp_l*sin(Pi*3/2)) ,0 ,1},
  {fws ,nee+int(amp_l*sin(Pi*13/8)),fws ,-nee+int(amp_l*sin(Pi*13/8)),0 ,1},
  {fws ,nee+int(amp_l*sin(Pi*7/4)) ,fws ,-nee+int(amp_l*sin(Pi*7/4)) ,0 ,0},
  {fws ,nee+int(amp_l*sin(Pi*15/8)),fws ,-nee+int(amp_l*sin(Pi*15/8)),0 ,0}};

// Back Step
int b_s[20][6]={
  {-fws,nee                        ,-fws,-nee                        ,0 ,0},
  {-fws,nee+int(amp_r*sin(Pi/8))   ,-fws,-nee+int(amp_r*sin(Pi/8))   ,0 ,1},
  {-fws,nee+int(amp_r*sin(Pi/4))   ,-fws,-nee+int(amp_r*sin(Pi/4))   ,0 ,1},
  {-fws,nee+int(amp_r*sin(Pi*3/8)) ,-fws,-nee+int(amp_r*sin(Pi*3/8)) ,0 ,0},
  {-fws,nee+int(amp_r*sin(Pi/2))   ,-fws,-nee+int(amp_r*sin(Pi/2))   ,0 ,0},
  {0   ,nee+int(amp_r*sin(Pi/2))   ,0   ,-nee+int(amp_r*sin(Pi/2))   ,0 ,0},
  {fws ,nee+int(amp_r*sin(Pi/2))   ,fws ,-nee+int(amp_r*sin(Pi/2))   ,0 ,-1},
  {fws ,nee+int(amp_r*sin(Pi*5/8)) ,fws ,-nee+int(amp_r*sin(Pi*5/8)) ,0 ,-1},
  {fws ,nee+int(amp_r*sin(Pi*3/4)) ,fws ,-nee+int(amp_r*sin(Pi*3/4)) ,0 ,0},
  {fws ,nee+int(amp_r*sin(Pi*7/8)) ,fws ,-nee+int(amp_r*sin(Pi*7/8)) ,0 ,0},
  {fws ,nee                        ,fws ,-nee                        ,0 ,0},
  {fws ,nee+int(amp_l*sin(Pi*9/8)) ,fws ,-nee+int(amp_l*sin(Pi*9/8)) ,0 ,-1},
  {fws ,nee+int(amp_l*sin(Pi*5/4)) ,fws ,-nee+int(amp_l*sin(Pi*5/4)) ,0 ,-1},
  {fws ,nee+int(amp_l*sin(Pi*11/8)),fws ,-nee+int(amp_l*sin(Pi*11/8)),0 ,0},
  {fws ,nee+int(amp_l*sin(Pi*3/2)) ,fws ,-nee+int(amp_l*sin(Pi*3/2)) ,0 ,0},
  {0   ,nee+int(amp_l*sin(Pi*3/2)) ,0   ,-nee+int(amp_l*sin(Pi*3/2)) ,0 ,0},
  {-fws,nee+int(amp_l*sin(Pi*3/2)) ,-fws,-nee+int(amp_l*sin(Pi*3/2)) ,0 ,1},
  {-fws,nee+int(amp_l*sin(Pi*13/8)),-fws,-nee+int(amp_l*sin(Pi*13/8)),0 ,1},
  {-fws,nee+int(amp_l*sin(Pi*7/4)) ,-fws,-nee+int(amp_l*sin(Pi*7/4)) ,0 ,0},
  {-fws,nee+int(amp_l*sin(Pi*15/8)),-fws,-nee+int(amp_l*sin(Pi*15/8)),0 ,0}};

// Left Turn_Step
int l_s[12][6]={
  {0      ,nee                        ,0       ,-nee                        ,0 ,0},
  {0      ,nee+int(amp_rt*sin(Pi/4))  ,0       ,-nee+int(amp_rt*sin(Pi/4))  ,0 ,1},
  {0      ,nee+int(amp_rt*sin(Pi/2))  ,0       ,-nee+int(amp_rt*sin(Pi/2))  ,0 ,1},
  {fws_t/2,nee+int(amp_rt*sin(Pi/2))  ,-fws_t/2,-nee+int(amp_rt*sin(Pi/2))  ,0 ,0},
  {fws_t  ,nee+int(amp_rt*sin(Pi/2))  ,-fws_t  ,-nee+int(amp_rt*sin(Pi/2))  ,0 ,-1},
  {fws_t  ,nee+int(amp_rt*sin(Pi*3/4)),-fws_t  ,-nee+int(amp_rt*sin(Pi*3/4)),0 ,-1},
  {fws_t  ,nee                        ,-fws_t  ,-nee                        ,0 ,0},
  {fws_t  ,nee+int(amp_lt*sin(Pi*5/4)),-fws_t  ,-nee+int(amp_lt*sin(Pi*5/4)),0 ,-1},
  {fws_t  ,nee+int(amp_lt*sin(Pi*3/2)),-fws_t  ,-nee+int(amp_lt*sin(Pi*3/2)),0 ,-1},
  {fws_t/2,nee+int(amp_lt*sin(Pi*3/2)),-fws_t/2,-nee+int(amp_lt*sin(Pi*3/2)),0 ,0},
  {0      ,nee+int(amp_lt*sin(Pi*3/2)),0       ,-nee+int(amp_lt*sin(Pi*3/2)),0 ,1},
  {0      ,nee+int(amp_lt*sin(Pi*7/4)),0       ,-nee+int(amp_lt*sin(Pi*7/4)),0 ,1}};

// Right Turn Step
int r_s[12][6]={
  {0      ,nee                         ,0       ,-nee                        ,0 ,0},
  {0      ,nee-int(amp_lt*sin(Pi/4))   ,0       ,-nee-int(amp_lt*sin(Pi/4))  ,0 ,-1},
  {0      ,nee-int(amp_lt*sin(Pi/2))   ,0       ,-nee-int(amp_lt*sin(Pi/2))  ,0 -1},
  {fws_t/2,nee-int(amp_lt*sin(Pi/2))   ,-fws_t/2,-nee-int(amp_lt*sin(Pi/2))  ,0 ,0},
  {fws_t  ,nee-int(amp_lt*sin(Pi/2))   ,-fws_t  ,-nee-int(amp_lt*sin(Pi/2))  ,0 ,1},
  {fws_t  ,nee-int(amp_lt*sin(Pi*3/4)) ,-fws_t  ,-nee-int(amp_lt*sin(Pi*3/4)),0 ,1},
  {fws_t  ,nee                         ,-fws_t  ,-nee                        ,0 ,0},
  {fws_t  ,nee-int(amp_rt*sin(Pi*5/4)) ,-fws_t  ,-nee-int(amp_rt*sin(Pi*5/4)),0 ,1},
  {fws_t  ,nee-int(amp_rt*sin(Pi*3/2)) ,-fws_t  ,-nee-int(amp_rt*sin(Pi*3/2)),0 ,1},
  {fws_t/2,nee-int(amp_rt*sin(Pi*3/2)) ,-fws_t/2,-nee-int(amp_rt*sin(Pi*3/2)),0 ,0},
  {0      ,nee-int(amp_rt*sin(Pi*3/2)) ,0       ,-nee-int(amp_rt*sin(Pi*3/2)),0 ,-1},
  {0      ,nee-int(amp_rt*sin(Pi*7/4)) ,0       ,-nee-int(amp_rt*sin(Pi*7/4)),0 ,-1}};

// Home Position
int h_p[6]={0,nee,0,-nee,0,0};

void Srv_drive(int srv_CH,int SrvAng){
  SrvAng = map(SrvAng, cont_min, cont_max, pulseMIN, pulseMAX);
  ledcWrite(srv_CH, SrvAng);
}

void servo_set(){
  int a[5],b[5];
  
  for (int j=0; j <=4 ; j++){
      a[j] = ang1[j] - ang0[j];
      b[j] = ang0[j];
      ang0[j] = ang1[j];
  }

  for (int k=0; k <=td ; k++){

      Srv_drive(srv_CH0, a[0]*float(k)/td+b[0]);
      Srv_drive(srv_CH1, a[1]*float(k)/td+b[1]);
      Srv_drive(srv_CH2, a[2]*float(k)/td+b[2]);
      Srv_drive(srv_CH3, a[3]*float(k)/td+b[3]);
      Srv_drive(srv_CH4, a[4]*float(k)/td+b[4]);

      delay(ts/td);
  }
}

void IMU_data(){
  float fK = 0.5;
  auto imu_update = M5.Imu.update();
    if (imu_update) {
        auto data = M5.Imu.getImuData();

        acc_angle_z = atan2(data.accel.x, -data.accel.y) * 180 / Pi -90; //M5AtomS3RCAMの場合は-90入れる

        //ローパスフィルター
        angleZ = fK*acc_angle_z+(1.0-fK)*preAngleZ;

        preAngleZ = angleZ;
    }
  //Serial.printf("angleZ:%f\r\n", angleZ);
}

void PDI_cal()
{
  interval_pid = millis() - preInterval_pid;
  preInterval_pid = millis();
  Tpid = interval_pid * 0.001;

  IMU_data();

  eZ = target_angle-abs(angleZ);
  deZ = (eZ - eZ_pre)/Tpid;
  ieZ = ieZ + (eZ + eZ_pre)*Tpid/2;
  uZ = Kpf*eZ + Kif*ieZ + Kdf*deZ;
  eZ_pre = eZ;

  if(g_flag == -1){
     ang1[4] = ang1[4] + int(uZ);
  }
  if(g_flag == 1){
     ang1[4] = ang1[4] + int(-uZ);
  }
}

void forward_step()
{
  for (int i=0; i <=19 ; i++){
    for (int j=0; j <=4 ; j++){
      ang1[j] = angZero[j] + f_s[i][j];
    }
    g_flag = f_s[i][5];
    PDI_cal();
    servo_set();
  }
}

void back_step()
{
  for (int i=0; i <=19 ; i++){
    for (int j=0; j <=4 ; j++){
      ang1[j] = angZero[j] + b_s[i][j];
    }
    g_flag = b_s[i][5];
    PDI_cal();
    servo_set();
  }
}

void right_step()
{
  for (int i=0; i <=11 ; i++){
    for (int j=0; j <=4 ; j++){
      ang1[j] = angZero[j] + r_s[i][j];
    }
    g_flag = r_s[i][5];
    PDI_cal();
    servo_set();
  }
}

void left_step()
{
  for (int i=0; i <=11 ; i++){
    for (int j=0; j <=4 ; j++){
      ang1[j] = angZero[j] + l_s[i][j];
    }
    g_flag = l_s[i][5];
    PDI_cal();
    servo_set();
  }
}

void head_position()
{
  for (int j=0; j <=4 ; j++){
    ang1[j] = angZero[j] + h_p[j];
  }
  ang1[1] = angZero[1] + nee + nee_p + nee_r;
  ang1[3] = angZero[3] - nee - nee_p + nee_r;
  ang1[4] = angZero[4] + head_p;
  servo_set();
}

void home_position()
{
  for (int j=0; j <=4 ; j++){
    ang1[j] = angZero[j] + h_p[j];
  }
  servo_set();
}

void Transit_I2c()
{
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(joyLX + CamX);
  Wire.write(joyLY + CamY);
  Wire.endTransmission();
  Serial.printf("CamX:%d CamY:%d\r\n", CamX, CamY);
}

static camera_config_t camera_config = {
    .pin_pwdn     = PWDN_GPIO_NUM,
    .pin_reset    = RESET_GPIO_NUM,
    .pin_xclk     = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7       = Y9_GPIO_NUM,
    .pin_d6       = Y8_GPIO_NUM,
    .pin_d5       = Y7_GPIO_NUM,
    .pin_d4       = Y6_GPIO_NUM,
    .pin_d3       = Y5_GPIO_NUM,
    .pin_d2       = Y4_GPIO_NUM,
    .pin_d1       = Y3_GPIO_NUM,
    .pin_d0       = Y2_GPIO_NUM,

    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href  = HREF_GPIO_NUM,
    .pin_pclk  = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565,
    //.pixel_format = PIXFORMAT_GRAYSCALE,
    //.pixel_format = PIXFORMAT_JPEG,
    .frame_size   = FRAMESIZE_QVGA,   // QVGA(320x240)
    .jpeg_quality = 0,
    .fb_count     = 1,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST,
    .sccb_i2c_port = 0,
};

esp_err_t camera_init(){

    //initialize the camera
    M5.In_I2C.release();
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.println("Camera Init Failed");
        return err;
    } else {
        Serial.println("Camera Init Success");
    }

    return ESP_OK;
}

static void face_coordinate(fb_data_t *fb, std::list<dl::detect::result_t> *results, int face_id)
{
    int x, y, w, h;
    
    int i = 0;
    for (std::list<dl::detect::result_t>::iterator prediction = results->begin(); prediction != results->end(); prediction++, i++)
    {
        // rectangle box
        x = (int)prediction->box[0];
        y = (int)prediction->box[1];

        // yが負の数のときにfb_gfx_drawFastHLine()でメモリ破壊してリセットする不具合の対策
        if(y < 0){
           y = 0;
        }

        w = (int)prediction->box[2] - x + 1;
        h = (int)prediction->box[3] - y + 1;

        if((x + w) > fb->width){
            w = fb->width - x;
        }
        if((y + h) > fb->height){
            h = fb->height - y;
        }

        xf = x + w/2;
        yf = y + h/2;

        //Serial.printf("xf:%d yf:%d\r\n", xf, yf);
    }
}

esp_err_t camera_capture_and_face_detect(){
  //acquire a frame
  M5.In_I2C.release();
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera Capture Failed");
    return ESP_FAIL;
  }

  int face_id = 0;

  HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
  std::list<dl::detect::result_t> &results = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});

  if (results.size() > 0) {

      fb_data_t rfb;
      rfb.width = fb->width;
      rfb.height = fb->height;
      rfb.data = fb->buf;
      rfb.bytes_per_pixel = 2;
      rfb.format = FB_RGB565;

      face_coordinate(&rfb, &results, face_id);
  }
  
  xf = int((0.3 * float(xfpre)) + (0.7 * float(xf)));
  yf = int((0.3 * float(yfpre)) + (0.7 * float(yf)));
  xfpre = xf;
  yfpre = yf;

  esp_camera_fb_return(fb);

  return ESP_OK;
}

void face_detect(void *pvParameters)
{
  while (1) {
    camera_capture_and_face_detect();
    CamX = map(xf,0,320,200,-200);
    CamY = map(yf,0,240,-200,200);
    Transit_I2c();
  }
}

void setup() 
{ 
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  M5.Imu.begin();

  Wire.begin(2, 1); //SDA-2, SCL-1

  M5.Lcd.setRotation(2);
  
  pinMode(Srv0, OUTPUT);
  pinMode(Srv1, OUTPUT);
  pinMode(Srv2, OUTPUT);
  pinMode(Srv3, OUTPUT);
  pinMode(Srv4, OUTPUT);

  //モータのPWMのチャンネル、周波数の設定
  ledcSetup(srv_CH0, PWM_Hz, PWM_level);
  ledcSetup(srv_CH1, PWM_Hz, PWM_level);
  ledcSetup(srv_CH2, PWM_Hz, PWM_level);
  ledcSetup(srv_CH3, PWM_Hz, PWM_level);
  ledcSetup(srv_CH4, PWM_Hz, PWM_level);

  //モータのピンとチャンネルの設定
  ledcAttachPin(Srv0, srv_CH0);
  ledcAttachPin(Srv1, srv_CH1);
  ledcAttachPin(Srv2, srv_CH2);
  ledcAttachPin(Srv3, srv_CH3);
  ledcAttachPin(Srv4, srv_CH4);

  home_position();

  pinMode(POWER_GPIO_NUM, OUTPUT);
  digitalWrite(POWER_GPIO_NUM, LOW);
  delay(500);
  camera_init();

  xTaskCreatePinnedToCore(face_detect, "face_detect", 4096, NULL, 1, NULL, 1);

  setupBLE();
} 
 
void loop()  
{   
  checkBLE();

  if (joyRSW == 1)
    {
      mode_flag = mode_flag  * -1;
      delay(500);
    }

  if (joyRY > 150)
    {
      forward_step();
    }
    
  if (joyRY < 50)
    {
      back_step();
    }
    
  if (joyRX < 50)
    {
      right_step();
    }

  if (joyRX > 150)
    {
      left_step();
    }

  if ((joyLX > 150)||(joyLX < 50)||(joyLY > 150)||(joyLY < 50))
    {
      head_p = map(joyLX,0,200,-45,45);
      nee_p = map(joyLY,50,150,10,-10);
      head_position();
    }

  if (mode_flag == -1)
    {
      nee_r = map(joyPitch,0,200,6,-6);
      head_position();
    }
  
  if (mode_flag == 1)
    {
      if ((joyRY <= 150) && (joyRY >= 50) && (joyRX <= 150) && (joyRX >= 50) && (joyLX <= 150) && (joyLX >= 50) && (joyLY <= 150) && (joyLY >= 50))
      {
        head_p = map(CamX, -100, 100, -30, 30);
        head_position();
      }
    }

} 
