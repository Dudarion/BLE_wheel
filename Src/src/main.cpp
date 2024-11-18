#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>

void bno_init();
void get_imu(int16_t* res);
void BLE_init();
uint8_t get_charge();
void check_off_btn(uint8_t left, uint8_t right);

#define BNO_ADDR 0x28 
#define EN 16
#define BTN_L 15
#define BTN_R 2

BLEDis bledis;
BLEBas blebas; // battery service
BLEHidGamepad blegamepad;

hid_gamepad_report_t gp;

int16_t imu_arr[6]; // imu data (AccX, AccY, AccZ, GyroX, GyroY, GyroZ)
unsigned long last_batt = 0;

void setup() {
  pinMode(13, OUTPUT); // Blue LED
  pinMode(EN, OUTPUT); // EN
  pinMode(BTN_L, INPUT); // LEFT
  pinMode(BTN_R, INPUT_PULLUP); // RIGHT
  digitalWrite(13, HIGH);
  digitalWrite(EN, HIGH);

  Serial.begin(115200);
  Serial.println("Made by Dudarion ");

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); 

  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeout(10000);

  delay(400); 
  bno_init(); 
  delay(800); 

  BLE_init();

    // ADC
  analogReference(AR_INTERNAL);
  analogReadResolution(12);
  delay(10);
  blebas.write(get_charge());
}

void BLE_init(){
  Bluefruit.begin();
  Bluefruit.setTxPower(4); 
  Bluefruit.setName("Science rock");
  bledis.setManufacturer("Science rock");
  bledis.setModel("Science rock");
  bledis.begin();
  blegamepad.begin();

  blebas.begin();
  blebas.write(87);

  Bluefruit.Periph.setConnInterval(6, 12); 


  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);
  
  // Include BLE HID service
  Bluefruit.Advertising.addService(blegamepad);

  // There is enough room for 'Name' in the advertising packet
  Bluefruit.Advertising.addName();
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 160);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void bno_init(){
  Serial.println("IMU initialization..");
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x41); // axis remap
  Wire.write(0x21); 
  uint8_t err = Wire.endTransmission();
  if(err){
    while(1) {
      Serial.println("i2c err 1");
      digitalToggle(13);
      delay(30);
    }
  }

  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x42); // axis reverse
  Wire.write(0b1); 
  err = Wire.endTransmission();
  if(err){
    while(1) {
      Serial.println("i2c err 2");
      digitalToggle(13);
      delay(30);
    }
  }

  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x3D); // opr mode
  Wire.write(0b1000); // IMU
  err = Wire.endTransmission();
  if(err){
    while(1) {
      Serial.println("i2c err 3");
      digitalToggle(13);
      delay(30);
    }
  }
}

void get_imu(int16_t* res){
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x08); // ACC
  delay(1);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(BNO_ADDR, 6, true);
  for(uint8_t cc=0; cc<3; cc++){
    res[cc] = Wire.read() | (Wire.read() << 8);
  }

  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x14); // GYRO
  delay(1);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(BNO_ADDR, 6, true);
  for(uint8_t cc=3; cc<6; cc++){
    res[cc] = Wire.read() | (Wire.read() << 8);
  }
}

void get_euler(int16_t* res){
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x1A); // EUL
  delay(1);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(BNO_ADDR, 6, true);
  for(uint8_t cc=0; cc<3; cc++){
    res[cc] = Wire.read() | (Wire.read() << 8);
  }

  // GYRO Z instaed of yaw !!!!!!
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x18); // Gyro Z
  delay(1);
  Wire.endTransmission();
  delay(1);
  Wire.requestFrom(BNO_ADDR, 2, true);
  res[0] = Wire.read() | (Wire.read() << 8);
}

// get battery charge in %
uint8_t get_charge(){ // battery charge in %
  uint16_t adc = analogRead(5);
  uint16_t mvolts = (float)adc * 1.758; //2(resistors)*2400mv(ref vol)/4096(12 bit)

  if(mvolts < 3000){
    digitalWrite(EN, 0); // OFF////////////////////////////////////////////////////////////////////
  }

  if(mvolts < 3300) {
    return 0;
  }
 
  if(mvolts < 3600) {
    mvolts -= 3300;
    return mvolts/10;
  }
 
  if(mvolts < 4150){
    mvolts -= 3600;
    return 30 + (mvolts * 0.127F );  // thats mvolts /6.66666666
  }

  return 100;
}

void check_off_btn(uint8_t left, uint8_t right){
  static unsigned long off_started_time = 0;
  static uint8_t first = 1;

  if(left && right){

    if (first == 1) {
      off_started_time = millis();
      first = 0;
    }
    
    if((millis() - off_started_time) > 2000) {
      digitalWrite(EN, 0); // OFF
      while(1){
        digitalToggle(13);
        delay(100);
      }
    }

  }
  else{
    first = 1;
  }
}

void clamp_imu(){
  if(imu_arr[0] > 2032) imu_arr[0] = 2032; // gyro
  if(imu_arr[0] < -2032) imu_arr[0] = -2032;

  for(int i=1; i<3; i++){  // dont clamp gyro
    if(imu_arr[i] > 508) imu_arr[i] = 508;
    if(imu_arr[i] < -508) imu_arr[i] = -508;
  }
}

void loop() {
  uint8_t bt_left = (~digitalRead(BTN_L))&1;
  uint8_t bt_right = (~digitalRead(BTN_R))&1;


  check_off_btn(bt_left, bt_right);

  uint8_t batt = get_charge(); // read Battery charge level in %

  if ((millis() - last_batt) > 30000){
    last_batt = millis();
    blebas.write(batt); // BLE write
  }

  // get_imu(imu_arr); // 6*int16 - raw data
  get_euler(imu_arr); // 3*int16 - euler

  // for(uint8_t i=0; i<3; i++){
  //   Serial.print(imu_arr[i]);
  //   Serial.print(' ');
  // }
  // Serial.print("batt: ");
  // Serial.print(batt);
  // Serial.println();

  if(Bluefruit.connected()){
    digitalWrite(13, HIGH);
    clamp_imu();
    
    
    gp.x       = -(imu_arr[1]/4); 
    gp.y       = -(imu_arr[2]/4); 
    gp.z       = -(imu_arr[0]/16);
    gp.rz      = 127;
    gp.rx      = 0;
    gp.ry      = 0;
    gp.hat     = 0;
    gp.buttons = (bt_left << 8) | (bt_right << 9);
    blegamepad.report(&gp);
    // blegamepad.notify(&gp, 11);
  }
  else{
    digitalWrite(13, LOW);
  }
}

