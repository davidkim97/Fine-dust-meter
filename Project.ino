//미세먼지 측정 코드
#include <LiquidCrystal_I2C.h>           // LiquidCrystal_I2C의 라이브러리를 불러옵니다.(lcd창에 데이터를 띄우기 위한 설정 라이브러리)
int Vo = A0;
int V_LED = 2;

float Vo_value = 0;
float Voltage =0;
float dustDensity = 0;

void setup() {
Serial.begin(9600);

pinMode(V_LED, OUTPUT);
pinMODE(V0, INPUT);

delay(1000);
Serial.println();
Serial.println("***********************");
Serial.println("*****HEllO*************");
Serial.println("***********************");
Serial.println("******THANK YOU******");
Serial.println("***********************");
}
void loop(){
digitalWrite(V_LED, LOW);
delayMicroseconds(280);
Vo_value = analogRead(Vo);
delayMicroseconds(40);
digitalWrite(V_LED, HIGH);
delayMicroseconds(9680);

Voltage=V0_value / 1024*5.0;
dustDensity=(Voltage-0.3)/0.005;
Serial.println(dustDensity);
delsy(1000);
}

LiquidCrystal_I2C lcd(0x27, 16, 2);
int Vo = A0;
int V_LED = 2;

float Vo_value = 0;
float Voltage =0;
float dustDensity = 0;

void setup() {
//put your setup code here, to run once;
Serial.begin(9600);

pinMode(V_LED, OUTPUT);
pinMODE(V0, INPUT);
lcd.init(); //LCD_I2C 통신을 시작
lcd.backlight(); //LCD backlight를 ON
}
void loop(){
//put your main code here to run repeatedly;
digitalWrite(V_LED, LOW);
delayMicroseconds(280);
Vo_value = analogRead(Vo);
delayMicroseconds(40);
digitalWrite(V_LED, HIGH);
delayMicroseconds(9680);

Voltage=V0_value / 1024*5.0;
dustDensity=(Voltage-0.3)/0.005;
Serial.println(dustDensity);
}
///////////////////////////////////////
lcd.clear();//lcd 화면 지우기
lcd.home();//lcd 커서 위치 0, 1로 위치
if(dustDensity > 150){ //매우나쁨//
lcd.print("AIR : VERY BAD!!);
}else if(dustDensity > 80){//나쁨//
lcd.print("AIR : BAD!");
}else if(dustDensity > 30){//보통//
lcd.print("AIR : NORMAL");
}else{//좋음//
lcd.print("AIR : GOOD!");
}
lcd.setCursor(0, 1);//lcd 커서의 위치를 4, 0으로 설정
lcd.print("ug/m3:")//현재 lcd커서 위치로부터 “GOOD DAY" 내용 출력
lcd.print(dustDensity);

delay(1000);
}
//3색LED를 장착하여 단계별 LED 동작
LiquidCrystal_I2C lcd(0x27, 16, 2);      // lcd(LCD의 I2C 슬레이브 주소, lcd 1줄당 출력할 글자수, lcd 줄의 수)
int Vo = A0;
int V_LED = 2;

float Vo_value = 0;
float Voltage = 0;
float dustDensity = 0;

int LED_R = 9;
int LED_G = 10;
int LED_B = 11;
int FAN = A1;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(V_LED, OUTPUT);
  pinMode(Vo, INPUT);

  lcd.init();                // LCD_I2C 통신을 시작합니다.
  lcd.backlight();           // LCD backlight를 ON

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);

  pinMode(FAN, OUTPUT);
  digitalWrite(FAN, LOW);
}
void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(V_LED, LOW);
  delayMicroseconds(280);
  Vo_value = analogRead(Vo);
  delayMicroseconds(40);
  digitalWrite(V_LED, HIGH);
  delayMicroseconds(9680);
  Voltage = Vo_value / 1024 * 5.0;
  dustDensity = (Voltage - 0.3) / 0.005;
  Serial.println(dustDensity);
  lcd.clear();        //lcd 화면을 지웁니다.
  lcd.home();        //lcd 커서 위치를 0,1로 위치시킵니다.
  if( dustDensity > 150 ){    // 매우 나쁨 //
    lcd.print("AIR : VERY BAD!!");   
  }else if( dustDensity > 80){   // 나쁨 //
    lcd.print("AIR : BAD!      ");   
  }else if( dustDensity > 30){  // 보통 //
    lcd.print("AIR : NORMAL     ");   
  }else{                    // 좋음
    lcd.print("AIR : GOOD      ");   
  }
  lcd.setCursor(0, 1); // lcd 커서의 위치를 4,0으로 설정합니다.
  lcd.print("ug/m3:"); // 현재 lcd 커서 위치로부터 "Good Day" 내용을 출력합니다.
  lcd.print(dustDensity);
  if(dustDensity > 150){
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  }else if(dustDensity > 80){
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);    
  }else if(dustDensity > 30){
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);    
  }else{
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);    
  }
delay(1000);
}
//RELAY 및 FAN 작동
LiquidCrystal_I2C lcd(0x27, 16, 2);      // lcd(LCD의 I2C 슬레이브 주소, lcd 1줄당 출력할 글자수, lcd 줄의 수)
int Vo = A0;
int V_LED = 2;

float Vo_value = 0;
float Voltage = 0;
float dustDensity = 0;

int LED_R = 9;
int LED_G = 10;
int LED_B = 11;
int FAN = A1;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(V_LED, OUTPUT);
  pinMode(Vo, INPUT);

  lcd.init();                // LCD_I2C 통신을 시작합니다.
  lcd.backlight();           // LCD backlight를 ON

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);

  pinMode(FAN, OUTPUT);
  digitalWrite(FAN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(V_LED, LOW);
  delayMicroseconds(280);
  Vo_value = analogRead(Vo);
  delayMicroseconds(40);
  digitalWrite(V_LED, HIGH);
  delayMicroseconds(9680);
  Voltage = Vo_value / 1024 * 5.0;
  dustDensity = (Voltage - 0.3) / 0.005;
  Serial.println(dustDensity);
  lcd.clear();        //lcd 화면을 지웁니다.
  lcd.home();        //lcd 커서 위치를 0,1로 위치시킵니다.
  if( dustDensity > 150 ){    // 매우 나쁨 //
    lcd.print("AIR : VERY BAD!!");   
  }else if( dustDensity > 80){   // 나쁨 //
    lcd.print("AIR : BAD!      ");   
  }else if( dustDensity > 30){  // 보통 //
    lcd.print("AIR : NORMAL     ");   
  }else{                    // 좋음
    lcd.print("AIR : GOOD      ");   
  }
  lcd.setCursor(0, 1); // lcd 커서의 위치를 4,0으로 설정합니다.
  lcd.print("ug/m3:"); // 현재 lcd 커서 위치로부터 "Good Day" 내용을 출력합니다.
  lcd.print(dustDensity);
  if(dustDensity > 150){
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  }else if(dustDensity > 80){
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);    
  }else if(dustDensity > 30){
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);    
  }else{
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);    
  }
 if(dustDensity > 150){
    digitalWrite(FAN, HIGH);
  }else{
    digitalWrite(FAN, LOW);    
  } 
  delay(1000);
}
