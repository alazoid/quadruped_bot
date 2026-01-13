#include <Arduino.h> // The core framework
#include <pgmspace.h> // 8x8 LED faces in Flash memory
#include <BluetoothSerial.h> // built-in ESP32 Bluetooth 
#include <SPI.h>  // for MAX7219 
#include <Wire.h> // for PCA9685 
#include <Adafruit_PWMServoDriver.h> // for PCA9685 
#include "esp_camera.h"
#include <WiFi.h>

// ====== CAMERA PINS (AI-THINKER) ======
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Forward Declaration for the Server Tab
void startCameraServer();

// ====== MAX7219 LED-MATRIX SETUP ======
#define CS_PIN 15           
const int LASER_PIN = 2;    
#define ISD1820_PLAY_PIN 4  
// ultrasonic front-sensor 
const int trigPin = 12; 
const int echoPin = 14; 
const int minSafeDistance = 25; 

BluetoothSerial SerialBT; 
bool isYelling = false; 

// 4 small faces stored in flash 
const uint8_t IMAGES[][8] PROGMEM = { 
{ 0,0,0,0b10000001,0b11011011,0b01011010,0b00100100,0 }, 
{ 0,0,0,0b11000011,0b01011010,0b00100100,0,0 }, 
{ 0,0,0b11000011,0b01011010,0b01100110,0b01000010,0b00111100,0 }, 
{ 0,0b10011001,0b10111101,0b01100110,0b01000010,0b01000010,0b00111100,0 } 
}; 

// --- Matrix Functions ---
void maxSend(uint8_t reg, uint8_t val) { 
digitalWrite(CS_PIN, LOW); 
SPI.transfer(reg); 
SPI.transfer(val); 
digitalWrite(CS_PIN, HIGH); 
} 

void matrixInit(uint8_t intensity = 8) { 
SPI.begin(); 
pinMode(CS_PIN, OUTPUT); 
// display test, no decode, row scan, awakening, brightness 
maxSend(0x0F, 0x00); maxSend(0x09, 0x00); 
maxSend(0x0B, 0x07); maxSend(0x0C, 0x01); 
maxSend(0x0A, intensity&0x0F);
for (uint8_t r=1; r<=8; r++) maxSend(r,0); 
} 

void matrixSetRow(uint8_t row, uint8_t bits) { maxSend(row+1, bits); } 

void drawShape(uint8_t idx) { 
for (uint8_t r = 0; r < 8; r++) { 
uint8_t b = pgm_read_byte(&IMAGES[idx][r]); 
matrixSetRow(r, b); 
} 
} 

void playYell() { 
digitalWrite(ISD1820_PLAY_PIN, HIGH); 
delay(100); 
digitalWrite(ISD1820_PLAY_PIN, LOW); 
// animate forward 
drawShape(1); delay(100); drawShape(2); delay(100); drawShape(3); delay(1000); 
// animate back 
drawShape(2); delay(100); drawShape(1); delay(100); drawShape(0); delay(1000); 
isYelling = false; 
} 

void bark() {
  isYelling = true;
  digitalWrite(ISD1820_PLAY_PIN, HIGH);
  drawShape(3); // Show angry face
  delay(150);   // Short pulse to trigger sound
  digitalWrite(ISD1820_PLAY_PIN, LOW);
  // We don't put a long delay here so the camera can keep scanning
}

// ====== PCA9685 SERVO BOARD SETUP ====== 
static const uint8_t CH_FL_KNEE = 0; 
static const uint8_t CH_FL_HIP = 1; 
static const uint8_t CH_FR_KNEE = 2; 
static const uint8_t CH_FR_HIP = 3; 
static const uint8_t CH_BR_KNEE = 4; 
static const uint8_t CH_BR_HIP = 5; 
static const uint8_t CH_BL_KNEE = 6; 
static const uint8_t CH_BL_HIP = 7;

// --- Servo Logic ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
const int SERVO_FREQ = 50; 
const int MIN_PULSE = 150; 
const int MAX_PULSE = 600; 

void setServoAngle(uint8_t chan, int angle) { 
angle = constrain(angle, 0, 180); 
uint16_t pulse = MIN_PULSE + (uint32_t)angle * (MAX_PULSE - MIN_PULSE) / 180; 
pwm.setPWM(chan, 0, pulse); 
} 

// calibration & state 
int da = -12, db = 10, dc = -18, dd = 12; 
int s11=90, s12=90, s21=90, s22=90, s31=90, s32=90, s41=90, s42=90; 
int spd = 3, high = 15; 
int a90,a120,a150,a180,b0,b30,b60,b90,c90,c120,c150,c180,d0,d30,d60,d90; 

// center all 8 servos 
void center_servos() {
setServoAngle(0, 90); setServoAngle(1, 90); setServoAngle(2, 90); setServoAngle(3, 90); 
setServoAngle(4, 90); setServoAngle(5, 90); setServoAngle(6, 90); setServoAngle(7, 90); 
s11 = s12 = s21 = s22 = s31 = s32 = s41 = s42 = 90; 
} 

// interpolate from current sXY → target pXY 
void srv(int p11,int p21,int p31,int p41, int p12,int p22,int p32,int p42, int sp1,int sp2,int sp3,int sp4) { 
p12 += high*3; p22 += high*3; p32 += high*3; p42 += high*3; 
int targetP[4] = { p11+da, p21+db, p31+dc, p41+dd }; 
int targetL[4] = { p12, p22, p32, p42 }; 
int curP[4] = { s11, s21, s31, s41 }; 
int curL[4] = { s12, s22, s32, s42 }; 
auto step = [&](int &cur, int targ, int stepSize){ if (abs(cur - targ) >= stepSize) cur += (cur < targ ? stepSize : -stepSize); else cur = targ; }; 
while (true)
// move each pivot 
{ 
step(curP[0], targetP[0], sp1); step(curP[1], targetP[1], sp2); step(curP[2], targetP[2], sp3); step(curP[3], targetP[3], sp4); 
// move each lift 
step(curL[0], targetL[0], sp1); step(curL[1], targetL[1], sp2); step(curL[2], targetL[2], sp3); step(curL[3], targetL[3], sp4); 
// write pivots (with calibration) 
setServoAngle(1, curP[0]); setServoAngle(7, curP[1]); setServoAngle(5, curP[2]); setServoAngle(3, curP[3]); 
// write lifts 
setServoAngle(0, curL[0]); setServoAngle(6, curL[1]); setServoAngle(4, curL[2]); setServoAngle(2, curL[3]); 
// update globals 
s11=curP[0]; s21=curP[1]; s31=curP[2]; s41=curP[3]; s12=curL[0]; s22=curL[1]; s32=curL[2]; s42=curL[3]; 
// break when all reached 
bool done = true; for (int i=0; i<4; i++) if (curP[i]!=targetP[i]||curL[i]!=targetL[i]) done=false; 
if (done) break; delay(spd); 
} 
} 

long getDistanceCm() { 
digitalWrite(trigPin, LOW); delayMicroseconds(2); digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW); 
long duration = pulseIn(echoPin, HIGH, 30000UL); 
if (duration == 0) return 999; 
return (duration / 2) / 29.1; 
} 

// gait functions (verbatim from mePed ref) 
void forward() { 
// recalc all pivot targets 
a90 = 90 + da; a120 = 120 + da; a150 = 150 + da; a180 = 180 + da; 
b0 = 0 + db; b30 = 30 + db; b60 = 60 + db; b90 = 90 + db; 
c90 = 90 + dc; c120 = 120 + dc; c150 = 150 + dc; c180 = 180 + dc; 
d0 = 0 + dd; d30 = 30 + dd; d60 = 60 + dd; d90 = 90 + dd; 
// helper macro: if we cross the threshold, stop immediately 
#define CHECK_STOP() do { if (getDistanceCm() < minSafeDistance) { center_servos(); return; } } while(0) 
CHECK_STOP(); srv(a180, b0, c120, d60, 42, 33, 33, 42, 1,3,1,1); 
CHECK_STOP(); srv(a90, b30, c90, d30, 6, 33, 33, 42, 3,1,1,1); 
CHECK_STOP(); srv(a90, b30, c90, d30, 42, 33, 33, 42, 3,1,1,1); 
CHECK_STOP(); srv(a120, b60, c180, d0, 42, 33, 6, 42, 1,1,3,1); 
CHECK_STOP(); srv(a120, b60, c180, d0, 42, 33, 33, 42, 1,1,3,1); 
CHECK_STOP(); srv(a150, b90, c150, d90, 42, 33, 33, 6, 1,1,1,3); 
CHECK_STOP(); srv(a150, b90, c150, d90, 42, 33, 33, 42, 1,1,1,3); 
CHECK_STOP(); srv(a180, b0, c120, d60, 42, 6, 33, 42, 1,3,1,1); 
#undef CHECK_STOP 
} 

void back() { srv(180,0, 120,60, 42,33,33,42, 3,1,1,1); srv(150,90,150,90, 42,18,33,42, 1,3,1,1); srv(150,90,150,90, 42,33,33,42, 1,3,1,1); srv(120,60,180,0, 42,33,33,6, 1,1,1,3); srv(120,60,180,0, 42,33,33,42, 1,1,1,3); srv(90,30, 90,30, 42,33,18,42, 1,1,3,1); srv(90,30, 90,30, 42,33,33,42, 1,1,3,1); srv(180,0, 120,60, 6,33,33,42, 3,1,1,1); } 
void turn_left() { srv(150,90,90, 30, 42,6, 33,42, 1,3,1,1); srv(150,90,90, 30, 42,33,33,42, 1,3,1,1); srv(120,60,180,0, 42,33,6, 42, 1,1,3,1); srv(120,60,180,0, 42,33,33,24, 1,1,3,1); srv(90,30, 150,90, 42,33,33,6, 1,1,1,3); srv(90,30, 150,90, 42,33,33,42, 1,1,1,3); srv(180,0, 120,60, 6,33,33,42, 3,1,1,1); srv(180,0, 120,60, 42,33,33,33, 3,1,1,1); } 
void turn_right() { srv(90, 30, 150,90, 6, 33,33,42, 3,1,1,1); srv(90, 30, 150,90, 42,33,33,42, 3,1,1,1); srv(120,60,180,0, 42,33,33,6, 1,1,1,3); srv(120,60,180,0, 42,33,33,42, 1,1,1,3); srv(150,90,90, 30, 42,33,6, 42, 1,1,3,1); srv(150,90,90, 30, 42,33,33,42, 1,1,3,1); srv(180,0, 120,60, 42,6, 33,42, 1,3,1,1); srv(180,0, 120,60, 42,33,33,42, 1,3,1,1); } 

void setup() {
Serial.begin(115200);
matrixInit(8); 

// --- Camera Init ---
camera_config_t config;
config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM; config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
config.xclk_freq_hz = 20000000; config.pixel_format = PIXFORMAT_JPEG;
if(psramFound()){ config.frame_size = FRAMESIZE_QVGA; config.jpeg_quality = 10; config.fb_count = 2; }
esp_camera_init(&config);

WiFi.begin(ssid, password);
while (WiFi.status() != WL_CONNECTED) delay(500);
startCameraServer();

SerialBT.begin("ESP32_CONTROLS");
pinMode(ISD1820_PLAY_PIN, OUTPUT); digitalWrite(ISD1820_PLAY_PIN, LOW);
pinMode(LASER_PIN, OUTPUT); digitalWrite(LASER_PIN, LOW);
pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
Wire.begin(); pwm.begin(); pwm.setPWMFreq(SERVO_FREQ);
center_servos();
}

void loop() {
// always show face 0 unless yelling 
if (!isYelling) drawShape(0); 
if (SerialBT.available()) {
char c = SerialBT.read();
switch (c) {
case 'B': if (!isYelling) { isYelling = true; playYell(); } break;
case 'Z': digitalWrite(LASER_PIN, HIGH); break;
case 'z': digitalWrite(LASER_PIN, LOW); break;
case 'U': if (getDistanceCm() >= minSafeDistance) forward();
// too close! immediately stop any partial step and flash a warning face
else { center_servos(); drawShape(3); delay(200); drawShape(0); } break;
case 'D': back(); break;
case 'L': turn_left(); break;
case 'R': turn_right(); break;
case '0': center_servos(); break;
}
}
}
