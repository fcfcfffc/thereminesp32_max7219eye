#include <Arduino.h>
#include "driver/pcnt.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "LedControl.h"
#include "binary.h"
#include <SPI.h>
// ======= 引脚定义 =======
#define PCNT_INPUT_SIG_IO   1      // 差频输入
#define BUTTON_PIN          19      // 按钮用于设定基准频率
#define PWM_OUT_PIN         2      // 本地PWM输出
#define PCNT_UNIT           PCNT_UNIT_0

// ======= ESP-NOW 接收端 MAC 地址（替换为你的）======
uint8_t peerAddress[] = {0xF0, 0x9E, 0x9E, 0xAD, 0x77, 0x98}; // 替换为接收端 MAC 地址

typedef struct {
  uint8_t pwm;
} pwm_packet_t;

int potVal;
int looking;
// DIN connects to pin 6
// CLK connects to pin 5
// CS connects to pin 4
LedControl lc=LedControl(6,5,4,8);
byte LeftEyeOpen[] =
{
    B00000000,
		B00111100,
		B00100100,
		B00110100,
		B00110100,
		B00100100,
		B00111100,
		B00000000
};

byte LeftEyeBlink[] =
{
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00000000
};

byte RightEyeOpen[] =
{
B00000000,
		B00111100,
		B00100100,
		B00110100,
		B00110100,
		B00100100,
		B00111100,
		B00000000
};

byte RightEyeBlink[] =
{
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00000000
};

byte eyeopen[8]= {
B00111100,
B01000010,
B10000001,
B10011001,
B10011001,
B10000001,
B01000010,
B00111100
	};

// eyes closed
byte eyeclosed[8]= {
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00011000,
		B00000000};

// eyes partially closed
byte eyepartial[8]= {
B00000000,
		B00111100,
		B00100100,
		B00110100,
		B00110100,
		B00100100,
		B00111100,
		B00000000
	};	

// eyes partially open
byte eyepartialopen[8]= {
	B00000000,
	B00111100,
	B01000010,
	B01011010,
	B01011010,
	B01000010,
	B00111100,
	B00000000};

// eyes slightleft
byte slightleft[8]= {
B00111100,
B01000010,
B10011001,
B10011001,
B10000001,
B10000001,
B01000010,
B00111100};

// eyes left
byte left[8]= {
B00111100,
B01011010,
B10011001,
B10000001,
B10000001,
B10000001,
B01000010,
B00111100};

// eyes realleft
byte realleft[8]= {
B00111100,
B01011010,
B10000001,
B10000001,
B10000001,
B01000010,
B00111100,
B00000000};

// eyes slightright
byte slightright[8]= {
B00111100,
B01000010,
B10000001,
B10000001,
B10011001,
B10011001,
B01000010,
B00111100};

// eyes right
byte right[8]= {
B00111100,
B01000010,
B10000001,
B10000001,
B10000001,
B10011001,
B01011010,
B00111100};

// eyes realright
byte realright[8]= {
B00000000,
B01111110,
B10000001,
B10000001,
B10000001,
B10000001,
B01011010,
B00111100};


// ======= 中断和状态变量 =======
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int pulseCount = 0;
bool buttonPressed = false;

float smoothedFreq = 0.0;
float smoothedBaseFreq = 0.0;
float smoothedDelta = 0.0;
int baseFreq = 0;
bool baseFreqSet = false;

float lastSmoothedFreq = 0.0;  // 用于判断稳定性
const float stableEpsilon = 3;  // Hz，变化阈值，低于认为稳定
int stableCount = 0;
const int stableThreshold = 10;  // 稳定计数达到多少次，认为真的稳定（比如10次循环）

bool autoSetBase = true; // 是否启用自动基准设定

// ======= 定时器中断读取频率 =======
void IRAM_ATTR onTimer() {
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT, &count);
  portENTER_CRITICAL_ISR(&timerMux);
  pulseCount = count;
  portEXIT_CRITICAL_ISR(&timerMux);
  pcnt_counter_clear(PCNT_UNIT);
}

// ======= 按钮中断 =======
void IRAM_ATTR onButton() {
  buttonPressed = true;
}

// ======= 初始化 PCNT 差频输入 =======
void setupPCNT() {
  pcnt_config_t pcnt_config;
  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  pcnt_config.unit = PCNT_UNIT;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.pos_mode = PCNT_COUNT_INC;
  pcnt_config.neg_mode = PCNT_COUNT_DIS;
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim = 32767;
  pcnt_config.counter_l_lim = -32767;

  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}

// ======= 初始化定时器：200ms =======
void setupTimer() {
  timer = timerBegin(0, 80, true);      // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 50000, true); // 200 ms
  timerAlarmEnable(timer);
}

// ======= 初始化 PWM 输出 =======
void setupPWM() {
  ledcAttachPin(PWM_OUT_PIN, 0);
  ledcSetup(0, 1000, 8);  // 1kHz, 8-bit
  ledcWrite(0, 0);        // 初始为 0
}

// ======= 初始化按钮 =======
void setupButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_PIN, onButton, FALLING);
}

// ======= ESP-NOW 发送回调 =======
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  // 发送状态打印可以取消注释调试
  // Serial.print("📡 发送状态：");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "成功" : "失败");
}

// ======= 初始化 ESP-NOW =======
void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW 初始化失败");
    return;
  }

  esp_now_register_send_cb(onSent);

esp_now_peer_info_t peerInfo = {};
memcpy(peerInfo.peer_addr, peerAddress, 6);
peerInfo.channel = 1;  // 与接收端一致
peerInfo.encrypt = false;

  if (!esp_now_add_peer(&peerInfo)) {
    // Serial.println("✅ 添加 ESP-NOW 接收端成功");
  }
}

// ======= setup() =======
void setup() {
  SPI.begin();
  WiFi.mode(WIFI_STA);
  Serial.begin(115200);
  setupPCNT();
  setupTimer();
  setupPWM();
  setupButton();
  setupESPNow();

lc.shutdown(0,false); // Wake up Matrix displays
lc.shutdown(1,false);

lc.setIntensity(0,5); // Set intensity levels
lc.setIntensity(1,5);

lc.clearDisplay(0); // Clear Displays
lc.clearDisplay(1);
}
  void sLeftEyeOpen()
{
for (int i = 0; i < 8; i++) 
{
lc.setRow(0,i,LeftEyeOpen[i]);
}
}

void sLeftEyeBlink()
{
for (int i = 0; i < 8; i++)
{
lc.setRow(0,i,LeftEyeBlink[i]);
}
}
void sRightEyeOpen()
{
for (int i = 0; i < 8; i++)
{
lc.setRow(1,i,RightEyeOpen[i]);
}
}

void sRightEyeBlink()
{
for (int i = 0; i < 8; i++)
{
lc.setRow(1,i,RightEyeBlink[i]);
}
}
  // Serial.println("✅ ESP32-S3 Theremin 发送端已启动");


// ======= loop() =======
void loop() {
  delay(20); // 增加刷新率

  int currentFreq;
  portENTER_CRITICAL(&timerMux);
  currentFreq = pulseCount;
  portEXIT_CRITICAL(&timerMux);

  // 平滑当前频率
  float alphaFreq = 0.5;
  smoothedFreq = alphaFreq * currentFreq + (1 - alphaFreq) * smoothedFreq;

   // 判断稳定性：比较 smoothedFreq 和上一次的差值，且 smoothedDelta <= 4 时不增加计数
  float freqChange = fabs(smoothedFreq - lastSmoothedFreq);
  bool isStable = false;

  if (smoothedDelta >= 4) {
    // smoothedDelta >= 4 时，不增加 stableCount，重置计数
    stableCount = 0;
  } else if (freqChange < stableEpsilon) {
    stableCount++;
    if (stableCount >= stableThreshold) {
      isStable = true;
    }
  } else {
    stableCount = 0;
  }

  lastSmoothedFreq = smoothedFreq;
  
  // 按钮按下立即设定基准频率
  if (buttonPressed) {
    baseFreq = smoothedFreq;
    smoothedBaseFreq = smoothedFreq;
    baseFreqSet = true;
    buttonPressed = false;
    stableCount = 0; // 重置稳定计数
   // Serial.printf("🎯 设置基准频率为 %.1f Hz\n", smoothedFreq);
  }

  // 只有当基准已设且稳定计数达到阈值时，才缓慢更新基准频率（避免温度漂移导致偏差）
 // if (baseFreqSet && stableCount >= stableThreshold) {
  //  float alphaBase = 0.01; // 更新速度非常慢
  //  smoothedBaseFreq = alphaBase * smoothedFreq + (1 - alphaBase) * smoothedBaseFreq;
 // }

  // 加上“自动基准设定”功能，如果稳定且启用自动基准设定，则直接设定基准频率为当前值
  if (isStable && autoSetBase) {
    smoothedBaseFreq = smoothedFreq;
  //  Serial.printf("🔄 自动设定新基准为 %.1f Hz\n", smoothedFreq);
  }

  // 计算频率差并平滑
  float delta = smoothedBaseFreq - smoothedFreq;
  if (delta < 0) delta = 0;
  float alphaDelta = 0.6;
  smoothedDelta = alphaDelta * delta + (1 - alphaDelta) * smoothedDelta;

  // 映射到 PWM（0~255）
  int duty = map(smoothedDelta, 4, 20, 0, 255);
  duty = constrain(duty, 0, 255);
  ledcWrite(0, duty); // 本地输出
  
  looking = map(smoothedDelta, 4, 20, 8, 0);// map it to a number 0-7 and store it to looking 
  looking = constrain(looking, 0, 8);
if (looking == 8) {  // wait until IR is HIGH again


sLeftEyeOpen();
sRightEyeOpen();
delay(100);

int displayFrame=random(1, 10); // Randomize Eyes

switch (displayFrame) {

case 2: // Put #2 frame on both Display
sLeftEyeBlink();
sRightEyeBlink();
delay(10);
break;

// case 3: // Put #3 frame on both Display
// sLeftEyeLookLeft();
// sRightEyeLookLeft();
// delay(2000);
// break;

// case 4: // Put #4 frame on both Display
// sLeftEyeLookRight();
// sRightEyeLookRight();
// delay(2000);
// break;
}
}

else  if (looking == 0) {
  lc.setRow(0,0,realleft[0]);
  lc.setRow(0,1,realleft[1]);
  lc.setRow(0,2,realleft[2]);
  lc.setRow(0,3,realleft[3]);
  lc.setRow(0,4,realleft[4]);
  lc.setRow(0,5,realleft[5]);
  lc.setRow(0,6,realleft[6]);
  lc.setRow(0,7,realleft[7]);
  lc.setRow(1,0,realleft[0]);
  lc.setRow(1,1,realleft[1]);
  lc.setRow(1,2,realleft[2]);
  lc.setRow(1,3,realleft[3]);
  lc.setRow(1,4,realleft[4]);
  lc.setRow(1,5,realleft[5]);
  lc.setRow(1,6,realleft[6]);
  lc.setRow(1,7,realleft[7]);
  }
else if (looking == 1) {
  lc.setRow(0,0,left[0]);
  lc.setRow(0,1,left[1]);
  lc.setRow(0,2,left[2]);
  lc.setRow(0,3,left[3]);
  lc.setRow(0,4,left[4]);
  lc.setRow(0,5,left[5]);
  lc.setRow(0,6,left[6]);
  lc.setRow(0,7,left[7]);
  lc.setRow(1,0,left[0]);
  lc.setRow(1,1,left[1]);
  lc.setRow(1,2,left[2]);
  lc.setRow(1,3,left[3]);
  lc.setRow(1,4,left[4]);
  lc.setRow(1,5,left[5]);
  lc.setRow(1,6,left[6]);
  lc.setRow(1,7,left[7]);
  }

else if (looking == 2) {
  lc.setRow(0,0,slightleft[0]);
  lc.setRow(0,1,slightleft[1]);
  lc.setRow(0,2,slightleft[2]);
  lc.setRow(0,3,slightleft[3]);
  lc.setRow(0,4,slightleft[4]);
  lc.setRow(0,5,slightleft[5]);
  lc.setRow(0,6,slightleft[6]);
  lc.setRow(0,7,slightleft[7]);
  lc.setRow(1,0,slightleft[0]);
  lc.setRow(1,1,slightleft[1]);
  lc.setRow(1,2,slightleft[2]);
  lc.setRow(1,3,slightleft[3]);
  lc.setRow(1,4,slightleft[4]);
  lc.setRow(1,5,slightleft[5]);
  lc.setRow(1,6,slightleft[6]);
  lc.setRow(1,7,slightleft[7]);
  }

else if (looking == 3) {
  lc.setRow(0,0,eyeopen[0]);
  lc.setRow(0,1,eyeopen[1]);
  lc.setRow(0,2,eyeopen[2]);
  lc.setRow(0,3,eyeopen[3]);
  lc.setRow(0,4,eyeopen[4]);
  lc.setRow(0,5,eyeopen[5]);
  lc.setRow(0,6,eyeopen[6]);
  lc.setRow(0,7,eyeopen[7]);
  lc.setRow(1,0,eyeopen[0]);
  lc.setRow(1,1,eyeopen[1]);
  lc.setRow(1,2,eyeopen[2]);
  lc.setRow(1,3,eyeopen[3]);
  lc.setRow(1,4,eyeopen[4]);
  lc.setRow(1,5,eyeopen[5]);
  lc.setRow(1,6,eyeopen[6]);
  lc.setRow(1,7,eyeopen[7]);
}

else if (looking == 4) {
  lc.setRow(0,0,slightright[0]);
  lc.setRow(0,1,slightright[1]);
  lc.setRow(0,2,slightright[2]);
  lc.setRow(0,3,slightright[3]);
  lc.setRow(0,4,slightright[4]);
  lc.setRow(0,5,slightright[5]);
  lc.setRow(0,6,slightright[6]);
  lc.setRow(0,7,slightright[7]);
  lc.setRow(1,0,slightright[0]);
  lc.setRow(1,1,slightright[1]);
  lc.setRow(1,2,slightright[2]);
  lc.setRow(1,3,slightright[3]);
  lc.setRow(1,4,slightright[4]);
  lc.setRow(1,5,slightright[5]);
  lc.setRow(1,6,slightright[6]);
  lc.setRow(1,7,slightright[7]);
  }

else if (looking == 5) {
  lc.setRow(0,0,right[0]);
  lc.setRow(0,1,right[1]);
  lc.setRow(0,2,right[2]);
  lc.setRow(0,3,right[3]);
  lc.setRow(0,4,right[4]);
  lc.setRow(0,5,right[5]);
  lc.setRow(0,6,right[6]);
  lc.setRow(0,7,right[7]);
  lc.setRow(1,0,right[0]);
  lc.setRow(1,1,right[1]);
  lc.setRow(1,2,right[2]);
  lc.setRow(1,3,right[3]);
  lc.setRow(1,4,right[4]);
  lc.setRow(1,5,right[5]);
  lc.setRow(1,6,right[6]);
  lc.setRow(1,7,right[7]);
  }

else if (looking == 6) {
  lc.setRow(0,0,realright[0]);
  lc.setRow(0,1,realright[1]);
  lc.setRow(0,2,realright[2]);
  lc.setRow(0,3,realright[3]);
  lc.setRow(0,4,realright[4]);
  lc.setRow(0,5,realright[5]);
  lc.setRow(0,6,realright[6]);
  lc.setRow(0,7,realright[7]);
  lc.setRow(1,0,realright[0]);
  lc.setRow(1,1,realright[1]);
  lc.setRow(1,2,realright[2]);
  lc.setRow(1,3,realright[3]);
  lc.setRow(1,4,realright[4]);
  lc.setRow(1,5,realright[5]);
  lc.setRow(1,6,realright[6]);
  lc.setRow(1,7,realright[7]);
  }

  
 
    else if (looking == 7) {
   lc.setRow(0,0,eyeopen[0]);
  lc.setRow(0,1,eyeopen[1]);
  lc.setRow(0,2,eyeopen[2]);
  lc.setRow(0,3,eyeopen[3]);
  lc.setRow(0,4,eyeopen[4]);
  lc.setRow(0,5,eyeopen[5]);
  lc.setRow(0,6,eyeopen[6]);
  lc.setRow(0,7,eyeopen[7]);
  lc.setRow(1,0,eyeopen[0]);
  lc.setRow(1,1,eyeopen[1]);
  lc.setRow(1,2,eyeopen[2]);
  lc.setRow(1,3,eyeopen[3]);
  lc.setRow(1,4,eyeopen[4]);
  lc.setRow(1,5,eyeopen[5]);
  lc.setRow(1,6,eyeopen[6]);
  lc.setRow(1,7,eyeopen[7]);
    }
  // 发送给接收端
  pwm_packet_t packet;
  packet.pwm = duty;
  esp_now_send(peerAddress, (uint8_t *)&packet, sizeof(packet));

  // 串口打印监视
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.printf("Freq: %.1f Hz | Base: %.1f Hz | Δf: %.1f | PWM: %d | StableCnt: %d | looking: %d\n",
                  smoothedFreq, smoothedBaseFreq, smoothedDelta, duty, stableCount, looking );
    lastPrint = millis();
  }
}
