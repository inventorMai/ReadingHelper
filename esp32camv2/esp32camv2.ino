/*
   2022-12-07
   QQ交流群：566565915
   官网https://bemfa.com

  分辨率默认配置：config.frame_size = FRAMESIZE_UXGA;
  其他配置：
  FRAMESIZE_UXGA （1600 x 1200）
  FRAMESIZE_QVGA （320 x 240）
  FRAMESIZE_CIF （352 x 288）
  FRAMESIZE_VGA （640 x 480）
  FRAMESIZE_SVGA （800 x 600）
  FRAMESIZE_XGA （1024 x 768）
  FRAMESIZE_SXGA （1280 x 1024）

  config.jpeg_quality = 10;（10-63）越小照片质量最好
  数字越小表示质量越高，但是，如果图像质量的数字过低，尤其是在高分辨率时，可能会导致ESP32-CAM崩溃

  支持发布订阅模式，当图片上传时，订阅端会自动获取图片url地址，可做图片识别，人脸识别，图像分析
*/

//需要在arduino IDE软件中---工具-->管理库-->搜索arduinojson并安装
//需要在arduino IDE软件中---工具-->管理库-->搜索arduinojson并安装
//需要在arduino IDE软件中---工具-->管理库-->搜索arduinojson并安装
//建议使用esp32 sdk 2.0版本
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_client.h"
#include <ArduinoJson.h>
#include <HardwareSerial.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#define PIN 15
#define NUMPIXELS 8
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD

#include "camera_pins.h"


// ===========================
// Enter your WiFi credentials
// ===========================

/*********************需要修改的地方**********************/
const char* ssid = "";                          //WIFI名称
const char* password = "";                     //WIFI密码
int capture_interval = 5 * 1000;                       // 默认20秒上传一次，可更改（本项目是自动上传，如需条件触发上传，在需要上传的时候，调用take_send_photo()即可）
const char* uid = "";  //用户私钥，巴法云控制台获取
const char* topic = "ReadingHelper";                   //主题名字，可在控制台新建
char wechatMsg[50] = "";                               //如果不为空，会推送到微信，可随意修改，修改为自己需要发送的消息
const char* wecomMsg = "";                             //如果不为空，会推送到企业微信，推送到企业微信的消息，可随意修改，修改为自己需要发送的消息
const char* urlPath = "";                              //如果不为空，会生成自定义图片链接，自定义图片上传后返回的图片url，url前一部分为巴法云域名，第二部分：私钥+主题名的md5值，第三部分为设置的图片链接值。
bool flashRequired = false;                            //闪光灯，true是打开闪光灯
const int brightLED = 4;                               //默认闪光灯引脚

/********************************************************/

const char* post_url = "http://images.bemfa.com/upload/v1/upimages.php";  // 默认上传地址
static String httpResponseString;                                         //接收服务器返回信息
bool internet_connected = false;
long current_millis;
long last_capture_millis = 0;
unsigned long previousMillis = 0;  //用于计时断网重连
unsigned long interval = 10000;    //如果断网，重连时间间隔10秒

// Bright LED (Flash)
const int ledFreq = 50;        // PWM settings
const int ledChannel = 15;     // camera uses timer1
const int ledRresolution = 8;  // resolution (8 = from 0 to 255)
                               // change illumination LED brightness
HardwareSerial MySerial(1);

// WifiClient
//最大字节数
#define MAX_PACKETSIZE 512
//设置心跳值30s
#define KEEPALIVEATIME 5 * 1000
//tcp客户端相关初始化，默认即可

WiFiClient TCPclient;
String TcpClient_Buff = "";
unsigned int TcpClient_BuffIndex = 0;
unsigned long TcpClient_preTick = 0;
unsigned long preHeartTick = 0;     //心跳
unsigned long preTCPStartTick = 0;  //连接
bool preTCPConnected = false;

#define TCP_SERVER_ADDR "bemfa.com"
//服务器端口，tcp创客云端口8344
#define TCP_SERVER_PORT "8344"

//********************需要修改的部分*******************//

#define DEFAULT_STASSID ""                 //WIFI名称，区分大小写，不要写错
#define DEFAULT_STAPSW ""                 //WIFI密码
String UID = "";  //用户私钥，可在控制台获取,修改为自己的UID
String TOPIC = "ReadingHelper";                   //主题名字，可在控制台新建
#define upDataTime 2 * 1000


/*
  *发送数据到TCP服务器
 */
void sendtoTCPServer(String p) {

  if (!TCPclient.connected()) {
    Serial.println("Client is not readly");
    return;
  }
  TCPclient.print(p);
  Serial.println(p);
  preHeartTick = millis();  //心跳计时开始，需要每隔60秒发送一次数据
}

/*
  *初始化和服务器建立连接
*/
void startTCPClient() {
  if (TCPclient.connect(TCP_SERVER_ADDR, atoi(TCP_SERVER_PORT))) {
    Serial.print("\nConnected to server:");
    Serial.printf("%s:%d\r\n", TCP_SERVER_ADDR, atoi(TCP_SERVER_PORT));

    String tcpTemp = "";                                        //初始化字符串
    tcpTemp = "cmd=1&uid=" + UID + "&topic=" + TOPIC + "\r\n";  //构建订阅指令
    sendtoTCPServer(tcpTemp);                                   //发送订阅指令
    tcpTemp = "";                                               //清空

    preTCPConnected = true;
    TCPclient.setNoDelay(true);
  } else {
    Serial.print("Failed connected to server:");
    Serial.println(TCP_SERVER_ADDR);
    TCPclient.stop();
    preTCPConnected = false;
  }
  preTCPStartTick = millis();
}

String serial_info = "-1";
String last_serial_info = serial_info;
void doTCPClientTick() {
  //检查是否断开，断开后重连
  if (WiFi.status() != WL_CONNECTED) return;

  if (!TCPclient.connected()) {  //断开重连
    if (preTCPConnected == true) {

      preTCPConnected = false;
      preTCPStartTick = millis();
      Serial.println();
      Serial.println("TCP Client disconnected.");
      TCPclient.stop();
    } else if (millis() - preTCPStartTick > 1 * 1000)  //重新连接
      startTCPClient();
  }
  if (TCPclient.available()) {  //收数据
    char c = TCPclient.read();
    TcpClient_Buff += c;
    TcpClient_BuffIndex++;
    TcpClient_preTick = millis();

    if (TcpClient_BuffIndex >= MAX_PACKETSIZE - 1) {
      TcpClient_BuffIndex = MAX_PACKETSIZE - 2;
      TcpClient_preTick = TcpClient_preTick - 200;
    }
    preHeartTick = millis();
  }
  if (millis() - preHeartTick >= upDataTime) {  //上传数据
    preHeartTick = millis();
    while (MySerial.available() > 0) {
      serial_info = MySerial.readStringUntil('@');
      Serial.print("Serial input:");
      Serial.println(serial_info);
      if (serial_info == "Cam") {
        take_send_photo();
      } else {
        handle_pixel(serial_info);
      }
    }
    /*********************数据上传*******************/
    /*
      数据用#号包裹，以便app分割出来数据，&msg=#23#80#on#\r\n，即#温度#湿度#按钮状态#，app端会根据#号分割字符串进行取值，以便显示
      如果上传的数据不止温湿度，可在#号后面继续添加&msg=#23#80#data1#data2#data3#data4#\r\n,app字符串分割的时候，要根据上传的数据进行分割
    */
    String upstr = "";
    upstr = "cmd=2&uid=" + UID + "&topic=" + TOPIC + "&msg=" + serial_info + "\r\n";
    sendtoTCPServer(upstr);
    upstr = "";
    last_serial_info = serial_info;
  }
  if ((TcpClient_Buff.length() >= 1) && (millis() - TcpClient_preTick >= 200)) {  //data ready
    TCPclient.flush();
    Serial.println("Buff");
    Serial.println(TcpClient_Buff);
    //////字符串匹配，检测发了的字符串TcpClient_Buff里面是否包含&msg=on，如果有，则打开开关
    TcpClient_Buff = "";  //清空字符串，以便下次接收
    TcpClient_BuffIndex = 0;
  }
}


void brightLed(byte ledBrightness) {
  ledcWrite(ledChannel, ledBrightness);  // change LED brightness (0 - 255)
  Serial.println("Brightness changed to " + String(ledBrightness));
}


// ----------------------------------------------------------------
//       set up PWM for the illumination LED (flash)
// ----------------------------------------------------------------
// note: I am not sure PWM is very reliable on the esp32cam - requires more testing
void setupFlashPWM() {
  ledcSetup(ledChannel, ledFreq, ledRresolution);
  ledcAttachPin(brightLED, ledChannel);
  brightLed(0);
}


void setup() {
  Serial.begin(115200);
  pinMode(brightLED, OUTPUT);    // flash LED
  digitalWrite(brightLED, LOW);  // led off = Low
  MySerial.begin(9600, SERIAL_8N1, 12, 13);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;

    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t* s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }


#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");


  Serial.print("Camera Ready!:");
  Serial.print(WiFi.localIP());

  setupFlashPWM();  // configure PWM for the illumination LED

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif

  pixels.begin();
}

int brightness_level = 0;
constexpr int MAX_BRIGHTNESS_LEVEL = 6;

void handle_pixel(String& info) {
  if (info == "LED_ON") {
    brightness_level = MAX_BRIGHTNESS_LEVEL;
  } else if (info == "LED_OFF") {
    brightness_level = 0;
  } else if (info == "BRIGHTER") {
    brightness_level += 1;
    if (brightness_level >= MAX_BRIGHTNESS_LEVEL) {
      brightness_level = MAX_BRIGHTNESS_LEVEL;
    }
  } else if (info == "DARKER") {
    brightness_level -= 1;
    if (brightness_level < 0) {
      brightness_level = 0;
    }
  }
  for(int i=0; i<NUMPIXELS; i++) {
    pixels.setPixelColor(i,
                        pixels.Color((int)(241.0 * brightness_level / MAX_BRIGHTNESS_LEVEL),
                                     (int)(229.0 * brightness_level / MAX_BRIGHTNESS_LEVEL),
                                     (int)(201.0 * brightness_level / MAX_BRIGHTNESS_LEVEL)));
  }
  pixels.show();
}



/********http请求处理函数*********/
esp_err_t _http_event_handler(esp_http_client_event_t* evt) {
  if (evt->event_id == HTTP_EVENT_ON_DATA) {
    httpResponseString.concat((char*)evt->data);
  }
  return ESP_OK;
}



static esp_err_t take_send_photo() {
  if (flashRequired) {
    brightLed(255);
    delay(300);
  }
  Serial.println("take_send_photo...");
  camera_fb_t* fb = NULL;
  esp_err_t res = ESP_OK;
  fb = esp_camera_fb_get();
  if (flashRequired) brightLed(0);  // change LED brightness back to previous state

  if (!fb) {
    Serial.println("Camera capture failed...");
    return ESP_FAIL;
  }


  httpResponseString = "";
  esp_http_client_handle_t http_client;
  esp_http_client_config_t config_client = { 0 };
  config_client.url = post_url;
  config_client.event_handler = _http_event_handler;
  config_client.method = HTTP_METHOD_POST;
  http_client = esp_http_client_init(&config_client);
  esp_http_client_set_post_field(http_client, (const char*)fb->buf, fb->len);  //设置http发送的内容和长度
  esp_http_client_set_header(http_client, "Content-Type", "image/jpg");        //设置http头部字段
  esp_http_client_set_header(http_client, "Authorization", uid);               //设置http头部字段
  esp_http_client_set_header(http_client, "Authtopic", topic);                 //设置http头部字段
  esp_http_client_set_header(http_client, "wechatmsg", wechatMsg);             //设置http头部字段
  esp_http_client_set_header(http_client, "wecommsg", wecomMsg);               //设置http头部字段
  esp_http_client_set_header(http_client, "picpath", urlPath);                 //设置http头部字段
  esp_err_t err = esp_http_client_perform(http_client);                        //发送http请求
  if (err == ESP_OK) {
    Serial.println(httpResponseString);  //打印获取的URL
    //json数据解析
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, httpResponseString);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
    }
    String url = doc["url"];
    Serial.println(url);  //打印获取的URL
  }

  Serial.println("Taking picture END");
  esp_camera_fb_return(fb);
  esp_http_client_cleanup(http_client);


  return res;
}
void loop() {
  // TODO check Wifi and reconnect if needed 断网重连
  if ((WiFi.status() != WL_CONNECTED) && (millis() - previousMillis >= interval)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = millis();
  }

  //定时发送
  current_millis = millis();
  // if (current_millis - last_capture_millis > capture_interval) {  // Take another picture
  //   last_capture_millis = millis();
  //   Serial.println("Capturing!");
  //   take_send_photo();
  // }
  doTCPClientTick();
}
