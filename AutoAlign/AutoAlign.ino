#include <EEPROM.h>
// #include <avr/wdt.h> //使用看門狗計時器的含括檔
// #include <esp_task_wdt.h>
// #include <FLASHLED.h>
#include <curveFitting.h>
// #include <U8glib.h>
#include <U8g2lib.h>
#include <BluetoothSerial.h>

TaskHandle_t Task_1;
#define WDT_TIMEOUT 3

const byte X_STP_Pin = 15; //x軸 步進控制
const byte X_DIR_Pin = 2;  //X軸 步進馬達方向控制
const byte Y_STP_Pin = 0;  //y軸 步進控制
const byte Y_DIR_Pin = 4;  //y軸 步進馬達方向控/.//制
const byte Z_STP_Pin = 16; //z軸 步進控制
const byte Z_DIR_Pin = 17; //z軸 步進馬達方向控制

int ButtonSelected = 0;
// const byte Key_1_Pin = 41;
// const byte Key_2_Pin = 42;
// const byte Key_3_Pin = 43;
// const byte Key_4_Pin = 44;
// const byte Key_5_Pin = 45;
// const byte Key_6_Pin = 46;

U8G2_ST7920_128X64_F_SW_SPI lcd(U8G2_R0, 5, 18, 19, U8X8_PIN_NONE); //data 4 , en, rs
// U8G2_ST7920_128X64_1_SW_SPI lcd(U8G2_R0, 5, 18, 19, U8X8_PIN_NONE); //data 4 , en, rs
// U8X8_ST7920_128X64_SW_SPI lcd(U8G2_R0, 5, 18, 19, U8X8_PIN_NONE); //data 4 , en, rs

int LCD_Encoder_A_pin = 22;
int LCD_Encoder_B_pin = 23;
uint8_t LCD_Select_pin = 21;

bool LCD_Encoder_State = false;
bool LCD_Encoder_LastState = false;
int LCD_en_count = 0;
int current_selection = 0;

int LCD_Update_Mode = 0;
uint8_t LCD_PageNow = 1;

// #define KEY_NONE 0
// #define KEY_SELECT 3

// uint8_t uiKeySelect = 39;

// uint8_t uiKeyCode = KEY_NONE;

// uint8_t menu_current = 0;
// uint8_t menu_redraw_required = 0;
// uint8_t last_key_code = KEY_NONE;
// int curIndex = 0;
// uint8_t mainMenuSelected = 0;
// uint8_t PageNow = 0; //0 is main page
// uint8_t ItemsCount = 0;

// #define MENU_ITEMS 6
// char *menu_strings[MENU_ITEMS] =
//     {"Target IL",
//      "Status",
//      "Get Ref",
//      "Set X Steps",
//      "Set Y Steps",
//      "Set Z Steps"};

// #define MENU_TITLES 7
// String menu_title_strings[MENU_TITLES] =
//     {"Main",
//      "Target IL",
//      "Status",
//      "Get Ref",
//      "Set X Steps",
//      "Set Y Steps",
//      "Set Z Steps"};

// String MenuTitle = "";

// #define Page_1_ITEMS 2 //Target IL

// String page_1_strings[Page_1_ITEMS] =
//     {"-1.0", " <<"};

// #define Page_100_ITEMS 1 //None define
// String page_100_strings[Page_100_ITEMS] =
//     {" <<"};

// float targetIL = -1.0;

// bool isLCD = true;
// bool textAlignLR = false; // false is left, true is right
// bool isbtnTrigger = false;
// bool isValueJudgeMode = false;

const byte R_0 = 12;
// const byte C_0 = GND;

/* Keyboard Pin Setting */
const byte R_1 = 14;
const byte R_2 = 27;
const byte R_3 = 26;
const byte C_1 = 25;
const byte C_2 = 33;
const byte C_3 = 32;

// const byte LED_Align = 53;
const byte PD_Pin = 34;
int Tablet_PD_mode_Trigger_Pin = 13;

const byte LED_Align = 5;

int MotorDir_Pin = 0;
int MotorSTP_Pin = 0;
bool MotorCC = false;
bool MotorCC_X = false;
bool MotorCC_Y = false;
bool MotorCC_Z = false;

int delayBetweenStep = 600;
int delayBetweenStep_X = 8;
int delayBetweenStep_Y = 8;
int delayBetweenStep_Z = 8;
int MinMotorDelayTime = 320;
long MinMotroStep = 20;
int M_Level = 10;

int xyz = 0;

long X_Pos_Record = 0;
long Y_Pos_Record = 0;
long Z_Pos_Record = 0;
long X_Pos_Now = 0;
long Y_Pos_Now = 0;
long Z_Pos_Now = 0;
long Z_Pos_reLoad = 0;

int X_rotator_steps = 2;
int Y_rotator_steps = 2;
int Z_rotator_steps = 20;

int X_backlash = 0;
int Y_backlash = 0;
int Z_backlash = 0;

int X_ScanSTP = 12;
int Y_ScanSTP = 10;
int Z_ScanSTP = 200;

int X_ScanStable = 25;
int Y_ScanStable = 50;
int Z_ScanStable = 80;

double averagePDInput = 0;

double ref_Dac = 0; //PD reference
double ref_IL = 0;  //PD reference

double PDValue_Best = 0;
double AutoCuring_Best_IL = 0, PD_Now = 0, PD_Before = 0;

unsigned long time_curing_0, time_curing_1, time_curing_2, time_curing_3;
unsigned long timer_Get_IL_1 = 0, timer_Get_IL_2;

bool btn_isTrigger = false;
int Threshold;
int stableDelay;
bool key_ctrl = false;

double Motor_Unit_Idx = 0.01953125; /* (1/51.2) um/pulse */

// int Get_PD_Points = 15;
int Get_PD_Points = 500;
double Target_IL = 0; //0 dB
double StopValue = 0; //0 dB
int cmd_No = 0;

//int LED_Twinkling = 0;

bool isStop = false, isGetPower = true, isILStable = false;
bool sprial_JumpToBest = true;
byte GetPower_Mode = 1;

BluetoothSerial BT; //宣告藍芽物件，名稱為BT

bool isWatchDog_Flag = false;
bool isLCD = true;

void Task_1_sendData(void *pvParameters)
{
  while (true)
  {
    if (!digitalRead(LCD_Select_pin))
    {
      LCD_Encoder_Selected();
    }

    int idx = LCD_en_count / 2;
    // Serial.println(String(idx));
    updateUI(idx);
    // updateUI_1(1);
    lcd.clearBuffer();

    // if (isWatchDog_Flag)
    // {
    //   Serial.print("Task1 running on core ");
    //   Serial.println(xPortGetCoreID());
    //   Serial.println("WathcDog Online");
    //   isWatchDog_Flag = false;
    // }
    // else
    // {
    //   Serial.println("WathcDog Offline");
    // }

    delay(150);
    // lcd.clearDisplay();
    // delay(150);

    //Task1休息，delay(1)不可省略
    delay(1);
  }
}

void step(byte stepperPin, long steps, int delayTime)
{
  steps = abs(steps);

  for (long i = 0; i < steps; i++)
  {
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(delayTime);
  }

  //Position Record
  if (MotorCC)
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now += steps;
      MotorCC_X = true;
      break;
    case Y_STP_Pin:
      Y_Pos_Now += steps;
      MotorCC_Y = true;
      break;
    case Z_STP_Pin:
      Z_Pos_Now += steps;
      MotorCC_Z = true;
      break;
    }
  }
  else
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now -= steps;
      MotorCC_X = false;
      break;
    case Y_STP_Pin:
      Y_Pos_Now -= steps;
      MotorCC_Y = false;
      break;
    case Z_STP_Pin:
      Z_Pos_Now -= steps;
      MotorCC_Z = false;
      break;
    }
  }
}

void step(byte stepperPin, long steps, int delayTime, byte dirPin, bool dir)
{
  // steps = abs(steps);
  digitalWrite(dirPin, dir);

  for (long i = 0; i < steps; i++)
  {
    digitalWrite(stepperPin, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(stepperPin, LOW);
    delayMicroseconds(delayTime);
  }

  //Position Record
  if (MotorCC == true)
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now += steps;
      MotorCC_X = true;
      break;
    case Y_STP_Pin:
      Y_Pos_Now += steps;
      MotorCC_Y = true;
      break;
    case Z_STP_Pin:
      Z_Pos_Now += steps;
      MotorCC_Z = true;
      break;
    }
  }
  else
  {
    switch (stepperPin)
    {
    case X_STP_Pin:
      X_Pos_Now -= steps;
      MotorCC_X = false;
      break;
    case Y_STP_Pin:
      Y_Pos_Now -= steps;
      MotorCC_Y = false;
      break;
    case Z_STP_Pin:
      Z_Pos_Now -= steps;
      MotorCC_Z = false;
      break;
    }
  }
}

int KeyValueConverter()
{
  int keyNo = -1;
  bool isKeyPressed = false;
  int keyValueSum = 0;

  if (!digitalRead(R_1))
  {
    isKeyPressed = true;
    keyValueSum += 10;
  }
  else if (!digitalRead(R_2))
  {
    isKeyPressed = true;
    keyValueSum += 5;
  }
  else if (!digitalRead(R_3))
  {
    isKeyPressed = true;
    keyValueSum += 0;
  }

  if (isKeyPressed)
  {
    pinMode(C_1, INPUT_PULLUP);
    pinMode(C_2, INPUT_PULLUP);
    pinMode(C_3, INPUT_PULLUP);

    pinMode(R_1, OUTPUT);
    pinMode(R_2, OUTPUT);
    pinMode(R_3, OUTPUT);

    // digitalWrite(R_1, false);
    // digitalWrite(R_1, false);
    // digitalWrite(R_1, false);

    delay(2);

    if (!digitalRead(C_1))
    {
      keyValueSum += 1;
    }
    else if (!digitalRead(C_2))
      keyValueSum += 2;
    else if (!digitalRead(C_3))
      keyValueSum += 3;
    else
      keyValueSum = 0;

    pinMode(R_1, INPUT_PULLUP);
    pinMode(R_2, INPUT_PULLUP);
    pinMode(R_3, INPUT_PULLUP);

    pinMode(C_1, OUTPUT);
    pinMode(C_2, OUTPUT);
    pinMode(C_3, OUTPUT);

    // digitalWrite(C_1, false);
    // digitalWrite(C_2, false);
    // digitalWrite(C_3, false);

    delay(2);
  }

  if (keyValueSum != 0)
  {

    switch (keyValueSum)
    {
    case 1:
      keyNo = 101;
      break;
    case 2:
      keyNo = 102;
      break;
    case 3:
      keyNo = 103;
      break;
    case 6:
      keyNo = 104;
      break;
    case 7:
      keyNo = 105;
      break;
    case 8:
      keyNo = 106;
      break;
    case 11:
      keyNo = 7;
      break;
    case 12:
      keyNo = 8;
      break;
    case 13:
      keyNo = 9;
      break;
    default:
      keyNo = -1;
      break;
    }

    isKeyPressed = false;

    // Serial.println("key:" + String(keyNo));
  }

  // ButtonSelected = keyNo;
  return keyNo;
}

double a1 = 0.0374, a2 = -65.561;
double b1 = 0.0394, b2 = -67.778;

double ILConverter(double pdDac)
{
  double IL = 0;

  if (pdDac >= 1200) /* >20 dBm */
    IL = a1 * pdDac + a2;
  else
    IL = b1 * pdDac + b2;

  return IL;
}

//Calculate PD input value, Return Dac
double Cal_PD_Input_Dac(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;
  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += analogRead(34);
  }

  //Function: (PD Value) - (reference) + 300
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  // wdt_reset(); //喂狗操作，使看門狗定時器復位

  return averagePDInput;
}

//Calculate PD input value, Return IL
double Cal_PD_Input_IL(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;
  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += analogRead(34);
  }
  // ref_IL = 0;
  //Function: (PD Value) - (reference) + 300
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  // wdt_reset(); //喂狗操作，使看門狗定時器復位
  double IL = ILConverter(averagePDInput) - ref_IL;

  return IL;
}

//Calculate PD input value, Return Row Dac
double Cal_PD_Input_Row_IL(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;
  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += analogRead(34);
  }
  //Function: (PD Value)
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  // wdt_reset(); //喂狗操作，使看門狗定時器復位

  double IL = ILConverter(averagePDInput);

  return IL;
}

//Calculate PD input value, Return Row Dac
double Cal_PD_Input_Row_Dac(int averageCount)
{
  digitalWrite(X_DIR_Pin, false);
  digitalWrite(Y_DIR_Pin, false);
  digitalWrite(Z_DIR_Pin, false);
  delay(1);

  double averagePDInput = 0;
  double PDAvgInput = 0;
  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += analogRead(34);
  }
  //Function: (PD Value)
  averagePDInput = (PDAvgInput / averageCount);

  digitalWrite(X_DIR_Pin, MotorCC_X);
  digitalWrite(Y_DIR_Pin, MotorCC_Y);
  digitalWrite(Z_DIR_Pin, MotorCC_Z);
  delay(1);

  // wdt_reset(); //喂狗操作，使看門狗定時器復位

  return averagePDInput;
}

void CleanEEPROM(int startPosition, int datalength)
{
  for (size_t i = startPosition; i < (startPosition + datalength); i++)
  {
    EEPROM.write(i, ' ');
  }
  Serial.println("Clean EEPROM");
}

void WriteInfoEEPROM(String data, int start_position)
{
  for (int i = 0; i < data.length(); ++i)
  {
    EEPROM.write(i + start_position, data[i]);
  }
}

String ReadInfoEEPROM(int start_position, int data_length)
{
  String EEPROM_String;
  for (int i = 0; i < data_length; i++)
  {
    EEPROM_String += char(EEPROM.read(i + start_position));
    //    Serial.print("Reading String : ");
    //    Serial.println(char(EEPROM.read(i + start_position)));
  }
  EEPROM_String.trim();
  return EEPROM_String;
}

String WR_EEPROM(int start_position, String data)
{
  CleanEEPROM(start_position, 8); //Clean EEPROM(int startPosition, int datalength)

  WriteInfoEEPROM(String(data), start_position); //Write Data to EEPROM (data, start_position)
  EEPROM.commit();

  String s = ReadInfoEEPROM(start_position, 8);
  return s;
}

bool Contains(String text, String search)
{
  if (text.indexOf(search) == -1)
    return false;
  else
    return true;
}

void EmergencyStop()
{
  isStop = true;

  isLCD = true;
  LCD_Update_Mode = 0;
  LCD_PageNow = 100;

  Serial.println("EmergencyStop");
  isWatchDog_Flag = !isWatchDog_Flag;
  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
}

#define MENU_ITEMS 6
char *UI_Menu_Items[MENU_ITEMS] =
    {"1. Status",
     "2. Target IL",
     "3. Get Ref",
     "4. Set X Speed",
     "5. Set Y Speed",
     "6. Set Z Speed"};

uint8_t i, h, w, title_h, H;

//Full Page method
void updateUI(int pageIndex)
{
  if (isLCD)
  {
    Serial.println("LCD Update: " + String(pageIndex) + ", Mode: " + String(LCD_Update_Mode));

    if (LCD_Update_Mode == 0 && pageIndex != LCD_PageNow)
    {

      // lcd.clearBuffer();
      // lcd.clearDisplay();
      // lcd.clear();

      H = lcd.getHeight();
      h = lcd.getFontAscent() - lcd.getFontDescent() + 2;
      w = lcd.getWidth();
      title_h = h + 2;

      // lcd.setCursor(3, h);
      // lcd.print("Menu");
      int title_w = (w / 2) - (lcd.getStrWidth("Menu") / 2);
      lcd.drawStr(title_w, h - 1, "Menu");

      lcd.drawBox(0, h + 1, w, 1); //Seperate Line

      int start_i = 0; //start_i<=2
      if (pageIndex == 4)
        start_i = 1;
      else if (pageIndex == 5)
        start_i = 2;

      lcd.drawFrame(0, (pageIndex - start_i) * h + 1 + title_h, w, h + 1); //Select Box

      //Draw each item in UI_Menu_Items
      for (i = start_i; i < start_i + 4; i++)
      {
        lcd.drawStr(3, title_h + ((i + 1 - start_i) * h) - 1, UI_Menu_Items[i]);

        switch (i)
        {
        case 1:
          lcd.drawStr(lcd.getWidth() - lcd.getStrWidth("-00.0") - 2, title_h + ((i + 1 - start_i) * h) - 1, String(Target_IL).c_str());
          break;

        default:
          break;
        }
      }

      if (pageIndex < MENU_ITEMS - 1)
        lcd.drawTriangle(w / 2 - 2, H - 3, w / 2 + 3, H - 3, w / 2, H);

      LCD_PageNow = pageIndex;

      lcd.sendBuffer();
    }

    else if (LCD_Update_Mode == 1) /* Auto-Aligning */
    {
      lcd.clearBuffer();
      lcd.clearDisplay();

      h = lcd.getFontAscent() - lcd.getFontDescent() + 2;
      w = lcd.getWidth();

      int H = lcd.getHeight();

      int title_w = (w / 2) - (lcd.getStrWidth("Auto-Aligning") / 2);
      lcd.drawStr(title_w, H / 2, "Auto-Aligning");

      lcd.drawBox(0, H / 2 - h, w, 1); //Seperate Line
      lcd.drawBox(0, H / 2 + 5, w, 1); //Seperate Line

      lcd.sendBuffer();
    }

    else if (LCD_Update_Mode == 12)
    {
      // lcd.clearBuffer();
      lcd.clearDisplay();

      // H = lcd.getHeight();
      // h = lcd.getFontAscent() - lcd.getFontDescent() + 2;
      // w = lcd.getWidth();
      // title_h = h + 2;

      int title_w = (w / 2) - (lcd.getStrWidth("Menu") / 2);
      lcd.drawStr(title_w, h - 1, "Menu");

      lcd.drawBox(0, h + 1, w, 1); //Seperate Line

      pageIndex = 1;

      int start_i = 0; //start_i<=2
      if (pageIndex == 4)
        start_i = 1;
      else if (pageIndex == 5)
        start_i = 2;

      //Draw each item in UI_Menu_Items
      for (i = start_i; i < start_i + 4; i++)
      {
        lcd.drawStr(3, title_h + ((i + 1 - start_i) * h) - 1, UI_Menu_Items[i]);

        //Item Value
        switch (i)
        {
        case 1:
          lcd.drawStr(lcd.getWidth() - lcd.getStrWidth("-00.0") - 2, title_h + ((i + 1 - start_i) * h) - 1, String(Target_IL).c_str());
          lcd.drawFrame(lcd.getWidth() - lcd.getStrWidth("-00.0") - 4, (pageIndex - start_i) * h + 1 + title_h, lcd.getStrWidth("-00.0") + 4, h + 1); //Select Box
          break;

        default:
          break;
        }
      }

      if (pageIndex < MENU_ITEMS - 1)
        lcd.drawTriangle(w / 2 - 2, H - 3, w / 2 + 3, H - 3, w / 2, H);

      LCD_PageNow = LCD_Update_Mode;

      lcd.sendBuffer();
    }

    Serial.println("PageNow:" + String(LCD_PageNow));

    isLCD = false;
  }
}

void LCD_Encoder_Rise()
{
  isLCD = true;
  LCD_Encoder_State = digitalRead(LCD_Encoder_A_pin);

  if (LCD_Encoder_State != LCD_Encoder_LastState)
  {
    if (digitalRead(LCD_Encoder_B_pin) != LCD_Encoder_State)
    {
      LCD_en_count++;

      if (LCD_PageNow == 12)
      {
        Target_IL += 0.05;
      }
    }
    else
    {
      LCD_en_count--;

      if (LCD_PageNow == 12)
      {
        Target_IL -= 0.05;
      }
    }
  }
  LCD_Encoder_LastState = LCD_Encoder_State;

  if (LCD_en_count < 0)
    LCD_en_count = 0;
  else if (LCD_en_count > MENU_ITEMS * 2 - 2)
    LCD_en_count = MENU_ITEMS * 2 - 2;

  // Serial.println(String(LCD_en_count));
}

void LCD_Encoder_Selected()
{
  if (!btn_isTrigger)
  {
    btn_isTrigger = true;

    Serial.println("Encoder_Pressed_PageNow:" + String(LCD_PageNow));
    switch (LCD_PageNow)
    {
    case 1: /* Target IL */
      // Target_IL = -1.2;
      LCD_Update_Mode = 12;
      isLCD = true;
      Serial.println("LCD_Update_Mode:" + String(LCD_Update_Mode));
      // cmd_No = 18;  /* Set Target IL */
      break;
    case 2:
      cmd_No = 19;
      break;

    case 12: /* Target IL */
      // Target_IL = -1.2;
      LCD_en_count = 2;
      LCD_Update_Mode = 0;
      isLCD = true;
      cmd_No = 18; /* Set Target IL */
      break;

    default:
      break;
    }

    delay(500);

    btn_isTrigger = false;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(20); //設定序列埠接收資料時的最大等待時間

  BT.begin("AL02"); //BLE ID
  BT.setTimeout(20);
  // u8g2_font_5x7_tf
  lcd.begin();

  // Serial.println("h:" + String(h), + ",w:" + String(w) + ",titleH:" + String(title_h));
  lcd.setFont(u8g2_font_6x10_tf);
  lcd.clearDisplay();

  isLCD = true;
  LCD_Update_Mode = 0;

  pinMode(LED_Align, OUTPUT);
  pinMode(X_STP_Pin, OUTPUT);
  pinMode(X_DIR_Pin, OUTPUT);
  pinMode(Y_STP_Pin, OUTPUT);
  pinMode(Y_DIR_Pin, OUTPUT);
  pinMode(Z_STP_Pin, OUTPUT);
  pinMode(Z_DIR_Pin, OUTPUT);

  pinMode(R_0, INPUT_PULLUP);
  attachInterrupt(R_0, EmergencyStop, FALLING);
  // keyValueConverter()
  pinMode(R_1, INPUT_PULLUP); //keyValue:10
  pinMode(R_2, INPUT_PULLUP); // /keyValue:5
  pinMode(R_3, INPUT_PULLUP); // /keyValue:0
  // attachInterrupt(R_1, keyValueConverter, CHANGE);
  // attachInterrupt(R_2, keyValueConverter, CHANGE);
  // attachInterrupt(R_3, keyValueConverter, CHANGE);

  pinMode(C_1, OUTPUT); ///keyValue:1
  pinMode(C_2, OUTPUT); ///keyValue:2
  pinMode(C_3, OUTPUT); ///keyValue:3
  // attachInterrupt(C_1, keyValueConverter, CHANGE);
  // attachInterrupt(C_2, keyValueConverter, CHANGE);
  // attachInterrupt(C_3, keyValueConverter, CHANGE);

  pinMode(LCD_Encoder_A_pin, INPUT_PULLUP);                     // /keyValue:0
  pinMode(LCD_Encoder_B_pin, INPUT_PULLUP);                     // /keyValue:0
  pinMode(LCD_Select_pin, INPUT_PULLUP);                        //Encoder switch
  attachInterrupt(LCD_Encoder_A_pin, LCD_Encoder_Rise, CHANGE); //啟用中斷函式(中斷0，函式，模式)
  // attachInterrupt(LCD_Select_pin, LCD_Encoder_Selected, FALLING); //啟用中斷函式(中斷0，函式，模式)

  digitalWrite(C_1, false);
  digitalWrite(C_2, false);
  digitalWrite(C_3, false);

  // pinMode(PD_Pin, INPUT);                      //PD signal input
  pinMode(Tablet_PD_mode_Trigger_Pin, OUTPUT);    //Control Tablet Mode
  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

  digitalWrite(LED_Align, true);

  Serial.println("~~ Auto-Align System ~~");
  // BT.println("~~ Auto-Align System ~~");

  // Serial.println("Watch Dog Online");
  // esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  // esp_task_wdt_add(NULL);               //add current thread to WDT watch

  //宣告使用EEPROM 512 個位置
  EEPROM.begin(512);

  // CleanEEPROM(0, 511);

  //一次寫入
  // EEPROM.commit();

  String eepromString;

  eepromString = ReadInfoEEPROM(0, 8);                //Reading EEPROM(int start_position, int data_length)
  Serial.println("EEPROM - PD ref: " + eepromString); //(start_position, data_length)
  ref_Dac = eepromString.toDouble();
  ref_IL = ILConverter(ref_Dac);
  Serial.println("Ref IL: " + String(ref_IL));

  eepromString = ReadInfoEEPROM(72, 8);                  //Reading EEPROM(int start_position, int data_length)
  Serial.println("EEPROM - Target IL: " + eepromString); //(start_position, data_length)
  Target_IL = eepromString.toDouble();
  Serial.println("Target IL: " + String(Target_IL));

  double t1 = millis();
  for (int i = 0; i < 1; i++)
  {
    PD_Now = Cal_PD_Input_IL(100); //6 ms
  }
  double t2 = millis();
  Serial.println("Get PD Timespan : " + String(t2 - t1));

  //在核心0啟動任務1
  xTaskCreatePinnedToCore(
      Task_1_sendData, /* 任務實際對應的Function */
      "Task_1",        /* 任務名稱 */
      10000,           /* 堆疊空間 */
      NULL,            /* 無輸入值 */
      0,               /* 優先序0 */
      &Task_1,         /* 對應的任務變數位址 */
      0);              /*指定在核心0執行 */
}

bool isMsgShow = false;

void loop()
{
  // digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

  // delay(1000);

  // digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode

  // delay(1000);

  // Serial.println("Resetting WDT...");
  // esp_task_wdt_reset(); //喂狗操作，使看門狗定時器復位
  // delay(1500);

  // if (millis() - timer_Get_IL_1 > 1000)
  // {
  // }

  //Re-Initialize
  isStop = false;
  ButtonSelected = -1;
  // cmd_No = 0;

  //Keyboard Detect
  ButtonSelected = KeyValueConverter();

  // if(ButtonSelected == -1 )
  // {
  //   cmd_No = 100;
  // }
  // Serial.println("Btn:" +String(ButtonSelected) + ", CMD:" + String(cmd_No));

  String rsMsg = "";
  if (Serial.available())
    rsMsg = Serial.readString();

  String cmd = rsMsg;

  //Function Classification
  if (cmd != "" && ButtonSelected < 0)
  {
    cmd.trim();
    Serial.println("get cmd: " + String(cmd));

    if (cmd == "Xp1")
    {
      cmd_No = 102;
    }
    else if (cmd == "Xm1")
    {
      cmd_No = 105;
    }
    else if (cmd == "Yp1")
    {
      cmd_No = 106;
    }
    else if (cmd == "Ym1")
    {
      cmd_No = 104;
    }
    else if (cmd == "Zp1")
    {
      cmd_No = 103;
    }
    else if (cmd == "Zm1")
    {
      cmd_No = 101;
    }

    //Jog
    else if (Contains(cmd, "Jog_"))
    {
      cmd.remove(0, 4);

      byte dirPin, stpPin;
      bool dirt;

      if (Contains(cmd, "X"))
      {
        dirPin = X_DIR_Pin;
        stpPin = X_STP_Pin;
      }
      else if (Contains(cmd, "Y"))
      {
        dirPin = Y_DIR_Pin;
        stpPin = Y_STP_Pin;
      }
      else if (Contains(cmd, "Z"))
      {
        dirPin = Z_DIR_Pin;
        stpPin = Z_STP_Pin;
      }

      cmd.remove(0, 1);

      if (Contains(cmd, "m"))
      {
        dirt = false;
      }
      else if (Contains(cmd, "p"))
      {
        dirt = true;
      }

      cmd.remove(0, 2);

      Move_Motor(dirPin, stpPin, dirt, cmd.toDouble(), 80, 100); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
    }

    //Abs
    else if (Contains(cmd, "Abs_"))
    {
      cmd.remove(0, 4);

      byte dirPin, stpPin, xyz;
      bool dirt;

      if (Contains(cmd, "X"))
      {
        dirPin = X_DIR_Pin;
        stpPin = X_STP_Pin;
        xyz = 0;
      }
      else if (Contains(cmd, "Y"))
      {
        dirPin = Y_DIR_Pin;
        stpPin = Y_STP_Pin;
        xyz = 1;
      }
      else if (Contains(cmd, "Z"))
      {
        dirPin = Z_DIR_Pin;
        stpPin = Z_STP_Pin;
        xyz = 2;
      }

      cmd.remove(0, 2);

      Move_Motor_abs(xyz, cmd.toDouble());
    }

    //Abs All
    else if (Contains(cmd, "Abs_All_"))
    {
      Serial.println("Abs all");
      cmd.remove(0, 8);

      int travel_x = 0, travel_y = 0, travel_z = 0;

      travel_x = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz

      cmd.remove(0, cmd.indexOf('_') + 1);

      travel_y = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz

      cmd.remove(0, cmd.indexOf('_') + 1);

      travel_z = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz

      Move_Motor_abs_all(travel_x, travel_y, travel_z);
    }

    //(CScan) Scan Twoway Command
    else if (Contains(cmd, "CScan_"))
    {
      cmd.remove(0, 6);
      Serial.println(cmd);

      int XYZ;
      int count;
      int motorStep;
      int stableDelay;
      bool Direction;
      int StopPDValue;
      int Get_PD_Points;
      int Trips;

      XYZ = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //xyz
      cmd.remove(0, cmd.indexOf('_') + 1);

      count = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //count
      cmd.remove(0, cmd.indexOf('_') + 1);

      motorStep = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //step
      cmd.remove(0, cmd.indexOf('_') + 1);

      stableDelay = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stable delay
      cmd.remove(0, cmd.indexOf('_') + 1);

      Direction = cmd.substring(0, cmd.indexOf('_')) == "1";
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //direction
      cmd.remove(0, cmd.indexOf('_') + 1);

      StopPDValue = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stopValue
      cmd.remove(0, cmd.indexOf('_') + 1);

      Get_PD_Points = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //average points
      cmd.remove(0, cmd.indexOf('_') + 1);

      Trips = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd); //trips

      int delayBetweenStep = 50;
      String msg = "Trip ";

      CMDOutput("AS");
      Serial.println("Auto-Align Start");

      Scan_AllRange_TwoWay(XYZ, count, motorStep, stableDelay,
                           Direction, delayBetweenStep, StopPDValue, Get_PD_Points, Trips, msg);

      CMDOutput("%:");

      Serial.println("Auto Align End");
    }

    //(SScan) Spiral Scan Command
    else if (Contains(cmd, "SScan_"))
    {
      cmd.remove(0, 6);
      Serial.println(cmd);

      int matrix;
      int motorStep;
      int stb;
      int delay_btw_steps;
      int StopPDValue;
      int Z_Layers;
      int Z_Steps;

      matrix = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //matrix
      cmd.remove(0, cmd.indexOf('_') + 1);

      motorStep = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //step
      cmd.remove(0, cmd.indexOf('_') + 1);

      stb = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stable delay
      cmd.remove(0, cmd.indexOf('_') + 1);

      //          delay_btw_steps = cmd.substring(0, cmd.indexOf('_')) == "1";
      delay_btw_steps = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //delay_btw_steps
      cmd.remove(0, cmd.indexOf('_') + 1);

      StopPDValue = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stopValue
      cmd.remove(0, cmd.indexOf('_') + 1);
      //          Serial.println(cmd);  //stopValue

      Z_Layers = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //Z_Layers
      cmd.remove(0, cmd.indexOf('_') + 1);

      Z_Steps = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd); //Z_Steps

      digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
      delay(5);

      M_Level = matrix;

      CMDOutput("AS");
      Serial.println("Auto-Align Start");
      CMDOutput("^X");
      CMDOutput("R:" + String(M_Level * 2 + 1));
      CMDOutput("C:" + String(M_Level * 2 + 1));
      // Serial.println("Rows=" + String(M_Level * 2 + 1));
      // Serial.println("Columns=" + String(M_Level * 2 + 1));
      // Serial.println("^X");

      MinMotroStep = motorStep; //350
      stableDelay = stb;
      delayBetweenStep = delay_btw_steps;

      if (Z_Layers > 1)
        sprial_JumpToBest = false;

      for (int ZL = 0; ZL < Z_Layers; ZL++)
      {
        if (ZL > 0)
        {
          Move_Motor(Z_DIR_Pin, Z_STP_Pin, false, Z_Steps, 8, 150); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
        }

        AutoAlign_Spiral(M_Level, StopPDValue, stableDelay); //Input : (Sprial Level, Threshold, stable) Threshold:128
      }

      sprial_JumpToBest = true;

      CMDOutput("X^");
      // Serial.println("X^");

      digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
    }

    //Set BackLash Command
    else if (Contains(cmd, "_BL:"))
    {
      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 5);

        X_backlash = cmd.toInt();

        CleanEEPROM(24, 8); //Clean EEPROM(int startPosition, int datalength)

        WriteInfoEEPROM(String(cmd), 24); //(data, start_position)  // Write Data to EEPROM

        Serial.println("Set X BackLash: " + String(String(cmd)));

        // Reading Data from EEPROM
        Serial.println("X BackLash in eeprom: " + ReadInfoEEPROM(24, 8)); //(start_position, data_length)

        X_backlash = ReadInfoEEPROM(24, 8).toInt();
      }

      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 5);

        Y_backlash = cmd.toInt();

        CleanEEPROM(32, 8); //Clean EEPROM(int startPosition, int datalength)

        WriteInfoEEPROM(String(cmd), 32); //(data, start_position)  // Write Data to EEPROM

        Serial.println("Set Y BackLash: " + String(String(cmd)));

        // Reading Data from EEPROM
        Serial.println("Y BackLash in eeprom: " + ReadInfoEEPROM(32, 8)); //(start_position, data_length)

        Y_backlash = ReadInfoEEPROM(32, 8).toInt();
      }

      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 5);

        Z_backlash = cmd.toInt();

        CleanEEPROM(40, 8); //Clean EEPROM(int startPosition, int datalength)

        WriteInfoEEPROM(String(cmd), 40); //(data, start_position)  // Write Data to EEPROM

        Serial.println("Set Z BackLash: " + String(String(cmd)));

        // Reading Data from EEPROM
        Serial.println("Z BackLash in eeprom: " + ReadInfoEEPROM(40, 8)); //(start_position, data_length)

        Z_backlash = ReadInfoEEPROM(40, 8).toInt();
      }
    }

    //eStep Command
    else if (Contains(cmd, "_eStep:"))
    {
      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 8);
        X_rotator_steps = cmd.toInt();
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 8);
        Y_rotator_steps = cmd.toInt();
      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 8);
        Z_rotator_steps = cmd.toInt();
      }
    }

    //Set Scan Steps Command
    else if (Contains(cmd, "_ScanSTP:"))
    {
      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 10);

        X_ScanSTP = cmd.toInt();

        CleanEEPROM(48, 8);                                         //Clean EEPROM(int startPosition, int datalength)
        WriteInfoEEPROM(String(cmd), 48);                           //(data, start_position)  // Write Data to EEPROM
        Serial.println("Save in eeprom: " + ReadInfoEEPROM(48, 8)); //(start_position, data_length)

        Serial.println("Set X Scan Step: " + String(X_ScanSTP));
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 10);
        Y_ScanSTP = cmd.toInt();

        CleanEEPROM(56, 8);                                         //Clean EEPROM(int startPosition, int datalength)
        WriteInfoEEPROM(String(cmd), 56);                           //(data, start_position)  // Write Data to EEPROM
        Serial.println("Save in eeprom: " + ReadInfoEEPROM(56, 8)); //(start_position, data_length)

        Serial.println("Set Y Scan Step: " + String(String(cmd)));
      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 10);
        Z_ScanSTP = cmd.toInt();

        CleanEEPROM(64, 8);                                         //Clean EEPROM(int startPosition, int datalength)
        WriteInfoEEPROM(String(cmd), 64);                           //(data, start_position)  // Write Data to EEPROM
        Serial.println("Save in eeprom: " + ReadInfoEEPROM(64, 8)); //(start_position, data_length)

        Serial.println("Set Z Scan Step: " + String(String(cmd)));
      }
    }

    //Set Ref Command
    else if (Contains(cmd, "Set_Ref:"))
    {
      cmd.remove(0, 8);

      CleanEEPROM(0, 8);               //Clean EEPROM(int startPosition, int datalength)
      WriteInfoEEPROM(String(cmd), 0); //(data, start_position)  // Write Data to EEPROM
      EEPROM.commit();

      Serial.println("Update Ref Value : " + String(cmd));

      String eepromString = ReadInfoEEPROM(0, 8); //(int start_position, int data_length)

      Serial.println("PD ref: " + eepromString); //(start_position, data_length)  // Reading Data from EEPROM

      ref_Dac = eepromString.toDouble();
      ref_IL = ILConverter(ref_Dac);
    }

    //Set Manual Control Motor Speed
    else if (Contains(cmd, "Set_Motor_Speed_"))
    {
      cmd.remove(0, 16);

      if (Contains(cmd, "X"))
      {
        cmd.remove(0, 2);
        delayBetweenStep_X = cmd.toInt();
      }
      else if (Contains(cmd, "Y"))
      {
        cmd.remove(0, 2);
        delayBetweenStep_Y = cmd.toInt();
      }
      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 2);
        delayBetweenStep_Z = cmd.toInt();
      }

      Serial.println("Set Manual Control Motor Speed:" + cmd);
    }

    //Set PD average points
    else if (Contains(cmd, "Set_PD_average_Points:"))
    {
      cmd.remove(0, 22);
      Get_PD_Points = cmd.toInt();

      Serial.println("Set PD avg points:" + String(Get_PD_Points));
    }

    //Command No.
    else if (Contains(cmd, "cmd"))
    {
      cmd.remove(0, 3);
      cmd_No = cmd.toInt();
      delay(10);

      //          cmd_No = 4;  //Auto-Align
      //          cmd_No = 5;   //Fine scan
      //          cmd_No = 6;   //Auto curing
      //          cmd_No = 7;  //To re-load position
      //          cmd_No = 8;  //To Home
      //          cmd_No = 9;  //To Home
      //          cmd_No = 10;  //To Home
      //          cmd_No = 16;  //Set reLoad
      //          cmd_No = 17;  //Set home
      //          cmd_No = 18;  //Set Z target
      //          cmd_No = 19;  //Get ref
      //          cmd_No = 20;  //Spiral
      //          cmd_No = 21;  //Keep print IL to PC
      //          cmd_No = 22;  //Scan X
      //          cmd_No = 23;  //Scan Y
      //          cmd_No = 24;  //Scan Z
    }
  }
  else if (ButtonSelected >= 0)
  {
    //Keyboard No. to Cmd Set No.
    switch (ButtonSelected)
    {
    case 7:
      cmd_No = 1;
      break;

    case 8:
      cmd_No = 2;
      break;

    case 9:
      cmd_No = 3;
      break;

    default:
      cmd_No = ButtonSelected;
      break;
    }
    // cmd_No = ButtonSelected;
  }

  //Bluetooth : Receive Data
  if (cmd == "" && cmd_No == 0)
  {
    // Serial.println("BLE mode");
    if (BT.connected(50))
    {
      isMsgShow = true;

      if (BT.available())
      {
        String BTdata = BT.readString();
        BT.println(BTdata);
        Serial.println(BTdata);

        if (BTdata == "Up")
        {
          MotorSTP_Pin = X_STP_Pin;

          delayBetweenStep = 9;
          MinMotroStep = 1000;

          if (MotorDir_Pin != X_DIR_Pin || MotorCC_X == true || MotorCC == true)
          {
            MotorCC = false;
            MotorCC_X = false;
            MotorDir_Pin = X_DIR_Pin;
            digitalWrite(MotorDir_Pin, MotorCC); //步進馬達方向控制, false為負方向
            delay(10);
          }

          step(MotorSTP_Pin, MinMotroStep, delayBetweenStep);
        }
        else if (BTdata == "Down")
        {
          MotorSTP_Pin = X_STP_Pin;

          delayBetweenStep = 9;
          MinMotroStep = 1000;

          if (MotorDir_Pin != X_DIR_Pin || MotorCC_X == false || MotorCC == false)
          {
            MotorCC = true;
            MotorCC_X = true;
            MotorDir_Pin = X_DIR_Pin;
            digitalWrite(MotorDir_Pin, MotorCC); //步進馬達方向控制, false為負方向
            delay(10);
          }

          step(MotorSTP_Pin, MinMotroStep, delayBetweenStep);
        }
      }
    }
    else
    {
      if (isMsgShow)
      {
        Serial.println("Disconnected");
        isMsgShow = false;
        // ESP.restart();
      }
    }
  }

  // if(ButtonSelected <0 && cmd =="")
  // {
  //   cmd_No = 0;
  // }
  // Serial.println("Btn:" +String(ButtonSelected) + ", CMD:" + String(cmd_No));

  //Function Execution
  if (cmd_No != 0)
  {
    // Serial.println("Btn:" + String(ButtonSelected) + ", CMD:" + String(cmd_No));

    //Functions: Alignment
    if (cmd_No <= 100)
    {
      switch (cmd_No)
      {
        //Functions: Auto Align
      case 1: 
        if (true)
        {
          // StopValue = -1.2; //-1.25 dB

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          // digitalWrite(LED_Align, true);
          delay(3);

          AutoAlign();

          // digitalWrite(LED_Align, false);
          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
          Serial.println("Auto Align End");
          MotorCC = true;

          StopValue = 0; //0 dB

          isLCD = true;
          LCD_Update_Mode = 0;
          LCD_PageNow = 100;
        }
        cmd_No = 0;
        break;

        //Functions: Fine Scan
      case 2: 
        if (!btn_isTrigger)
        {
          StopValue = 0; //0 dB

          isLCD = true;
          LCD_Update_Mode = 1;
          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode

          CMDOutput("AS");
          Serial.println("Auto-Align Start");

          Scan_AllRange_TwoWay(2, 5, 200, 0, 0, 60, StopValue, 500, 2, "Z Scan, Trip "); //steps:150
          CMDOutput("%:");

          if (isStop) true;

          Scan_AllRange_TwoWay(1, 7, 20, 0, 0, 80, StopValue, 550, 2, "Y Scan, Trip "); //steps:350
          CMDOutput("%:");

          if (isStop) true;

          Scan_AllRange_TwoWay(0, 8, 22, 0, 0, 80, StopValue, 550, 2, "X Scan, Trip "); //steps:350
          CMDOutput("%:");

          if (isStop) true;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

          Serial.println("Auto Align End");

          isLCD = true;
          LCD_Update_Mode = 0;
          LCD_PageNow = 100;
        }
        cmd_No = 0;
        break;

      //Functions: Auto Curing
      case 3: /* Auto Curing */
        if (!btn_isTrigger)
        {
          btn_isTrigger = false;

          isILStable = false;

          double IL_stable_count = 0;
          double Acceptable_Delta_IL = 20; //0.8

          time_curing_0 = millis();
          time_curing_1 = time_curing_0;
          time_curing_2 = time_curing_1;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(5);

          AutoCuring_Best_IL = Cal_PD_Input_IL(300);

          StopValue = AutoCuring_Best_IL; //0 dB
          PD_Now = Cal_PD_Input_IL(300);

          Z_ScanSTP = 80; //180

          Serial.println("Auto-Curing");

          while (true)
          {
            Serial.println("Curing Time:" + String((millis() - time_curing_0) / 1000) + " s");
            Serial.println("Stop Value: " + String(StopValue) + ", Now: " + String(PD_Now));
            delay(700);

            if (isStop)
            {
              Serial.println("Emergency Stop");
              break;
            }
            else if (millis() - time_curing_0 > 540000)
            {
              Acceptable_Delta_IL = 0.8;
            }
            //IL Stable Time ,  70 secs,  curing time threshold , 12.5 mins
            else if (time_curing_2 - time_curing_1 > 70000 && millis() - time_curing_0 > 750000)
            {
              Serial.println("IL Stable - Stop Auto Curing");
              break;
            }
            //Total curing time , 14 mins
            else if (millis() - time_curing_0 > 840000)
            {
              Serial.println("Over Limit Curing Time - Stop Auto Curing");
              break;
            }

            if (isILStable && (millis() - time_curing_0) > 780000)
            {
              Serial.println("IL Stable in Scan - Stop Auto Curing");
              break;
            }

            PD_Now = Cal_PD_Input_IL(300);

            if (PD_Now >= (AutoCuring_Best_IL - (Acceptable_Delta_IL + 0.2)))
            {
              time_curing_2 = millis();
              continue;
            }
            else
            {
              CMDOutput("AS");
              Serial.println("Auto-Align Start");

              time_curing_3 = millis();
              Serial.println("Auto-Curing Time: " + String((time_curing_3 - time_curing_0) / 1000) + " s");

              if ((time_curing_3 - time_curing_0) > 540000 && (time_curing_3 - time_curing_0) < 600000)
              {
                Z_ScanSTP = 60; //60
                Serial.println("Update Z Scan Step: " + String(Z_ScanSTP));
              }
              else if ((time_curing_3 - time_curing_0) > 600000)
              {
                Z_ScanSTP = 45; //40
                Serial.println("Update Z Scan Step: " + String(Z_ScanSTP));
              }
              else if ((time_curing_3 - time_curing_0) > 720000)
              {
                Z_backlash = 45; //40
                Serial.println("Update Z Scan Step: " + String(Z_ScanSTP));
              }

              PD_Now = Cal_PD_Input_IL(300);

              if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL))
              {
                Fine_Scan(2, false);
              }

              if (isStop)
              {
                Serial.println("Emergency Stop");
                break;
              }

              PD_Now = Cal_PD_Input_IL(300);

              if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL))
              {
                Fine_Scan(1, false);
              }

              if (isStop)
              {
                Serial.println("Emergency Stop");
                break;
              }

              PD_Before = Cal_PD_Input_IL(300);

              if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL))
              {
                Scan_AllRange_TwoWay(2, 3, Z_ScanSTP, 0, 0, 50, StopValue, 300, 2, "Z Scan, Trip ");
                CMDOutput("%:");
              }

              if (isStop)
              {
                Serial.println("Emergency Stop");
                break;
              }

              PD_Now = Cal_PD_Input_IL(300);

              if (abs(PD_Before - PD_Now) < 1.2 && (time_curing_3 - time_curing_0) > 750000)
              {
                IL_stable_count++;

                if (IL_stable_count > 4)
                {
                  Serial.println("IL stable to break");
                  break;
                }
              }

              time_curing_1 = millis();
              time_curing_2 = time_curing_1;
            }
          }

          time_curing_3 = millis();
          Serial.println("Total Auto-Curing Time: " + String((time_curing_3 - time_curing_0) / 1000) + " s");

          StopValue = -1.65;                              //-1.9 dB
          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

          String eepromString = ReadInfoEEPROM(40, 8);                              //(int start_position, int data_length)  // Reading Data from EEPROM
          Serial.println("Reset Z backlash from EEPROM: " + ReadInfoEEPROM(40, 8)); //(start_position, data_length)
          Z_backlash = eepromString.toInt();

          cmd_No = 0;
        }
        break;

      case 18: /* Set Target IL */
        if (true)
        {
          StopValue = Target_IL;

          Serial.println("Update Target IL : " + WR_EEPROM(72, String(Target_IL)));
        }
        cmd_No = 0;
        break;

      case 19: /* Get Ref */
        if (true)
        {
          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(3);

          averagePDInput = 0;
          for (int i = 0; i < 500; i++)
            averagePDInput += analogRead(34);

          averagePDInput = (averagePDInput / 500);

          ref_Dac = averagePDInput;
          ref_IL = ILConverter(averagePDInput);

          CleanEEPROM(0, 8); //Clean EEPROM(int startPosition, int datalength)

          WriteInfoEEPROM(String(averagePDInput), 0); //Write Data to EEPROM (data, start_position)
          EEPROM.commit();

          Serial.println("Update Ref Value : " + String(averagePDInput));

          Serial.println("ref_Dac: " + ReadInfoEEPROM(0, 8) + ", ref_IL: " + String(ref_IL)); //Reading Data from EEPROM(start_position, data_length)

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
          delay(3);
        }

        cmd_No = 0;
        break;

      case 21:
        isGetPower = true;
        // GetPower_Mode = 1;

        Serial.println("Cmd: Get IL On");
        digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode

        cmd_No = 0;
        break;

      case 22:
        isGetPower = false;

        Serial.println("Cmd: Get Power Off");
        digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

        cmd_No = 0;
        break;

      case 23:
        GetPower_Mode = 1;
        Serial.println("Cmd: Get Power Mode: IL(dB)");
        cmd_No = 0;
        break;

      case 24:
        GetPower_Mode = 2;
        Serial.println("Cmd: Get Power Mode: Dac");
        cmd_No = 0;
        break;

      case 25:
        GetPower_Mode = 3;
        Serial.println("Cmd: Get Power Mode: Row IL(dBm)");
        cmd_No = 0;
        break;

      case 26:
        GetPower_Mode = 4;
        Serial.println("Cmd: Get Power Mode: Row Dac");
        cmd_No = 0;
        break;

      case 27:
        Serial.println(String(Cal_PD_Input_Row_Dac(Get_PD_Points)));
        cmd_No = 0;
        break;
      }
    }

    //Functions: Motion
    if (cmd_No > 100)
      switch (cmd_No)
      {
        // Function: Cont-------------------------------------------------------------------------
        //Z feed - cont
      case 101:
        while (true)
        {
          MotorCC = MotorCC_Z;
          Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, false, 400, delayBetweenStep_Z);
          MotorCC_Z = false;

          // DataOutput();

          if (cmd == "")
          {
            if (digitalRead(R_3))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;
      case 103:
        while (true)
        {
          MotorCC = MotorCC_Z;
          Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, true, 400, delayBetweenStep_Z);
          MotorCC_Z = true;

          // DataOutput();

          if (cmd == "")
          {
            if (digitalRead(R_3))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;

        //X feed - cont
      case 102:

        while (true)
        {
          MotorCC = MotorCC_X;
          Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, false, 400, delayBetweenStep_X);
          MotorCC_X = false;

          // DataOutput();

          if (cmd == "")
          {
            if (digitalRead(R_3))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;
        //X+ - cont
      case 105:

        while (true)
        {
          MotorCC = MotorCC_X;
          Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, true, 400, delayBetweenStep_X);
          MotorCC_X = true;

          // DataOutput();

          if (cmd == "")
          {
            if (digitalRead(R_2))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;

        //Y feed - cont
      case 106:

        while (true)
        {
          MotorCC = MotorCC_Y;
          Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, false, 400, delayBetweenStep_Y);
          MotorCC_Y = false;

          // DataOutput();

          if (cmd == "")
          {
            if (digitalRead(R_2))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }
        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;

      case 104:

        while (true)
        {
          MotorCC = MotorCC_Y;
          Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, true, 400, delayBetweenStep_Y);
          MotorCC_Y = true;

          // DataOutput();

          if (cmd == "")
          {
            if (digitalRead(R_2))
              break;
          }
          else
          {
            if (Serial.available())
            {
              String msg = Serial.readString();
              if (Contains(msg, "0"))
                break;
            }
          }
        }

        DataOutput();
        // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
        cmd_No = 0;
        break;

        // Function: Jog-------------------------------------------------------------------------

      //X+ feed - jog
      case 107:
        MotorCC = MotorCC_X;
        Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, false, 400, delayBetweenStep_X);
        MotorCC_X = true;

        break;

        //X- feed - jog
      case 108:
        MotorCC = MotorCC_X;
        Move_Motor_Cont(X_DIR_Pin, X_STP_Pin, true, 400, delayBetweenStep_X);
        MotorCC_X = false;

        break;

      //Y+ feed - jog
      case 109:
        MotorCC = MotorCC_Y;
        Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, false, 400, delayBetweenStep_Y);
        MotorCC_Y = true;
        break;

      //Y- feed - jog
      case 110:
        MotorCC = MotorCC_Y;
        Move_Motor_Cont(Y_DIR_Pin, Y_STP_Pin, true, 400, delayBetweenStep_Y);
        MotorCC_Y = false;
        break;

        //Z+ feed - jog
      case 111:
        MotorCC = MotorCC_Z;
        Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, true, 400, delayBetweenStep_Z);
        MotorCC_Z = true;
        break;

        //Z- feed - jog
      case 112:
        MotorCC = MotorCC_Z;
        Move_Motor_Cont(Z_DIR_Pin, Z_STP_Pin, false, 400, delayBetweenStep_Z);
        MotorCC_Z = false;
        break;

        //Go Home
      case 130:
        Move_Motor_abs_all(0, 0, 0);
        break;
      }
  }

  if (isGetPower)
  {

    // Serial.println("GetPower");
    if (millis() - timer_Get_IL_1 > 500)
    {
      // Serial.println("GetPower 2");

      timer_Get_IL_1 = millis();

      double value;

      switch (GetPower_Mode)
      {
      case 1:
        value = Cal_PD_Input_IL(Get_PD_Points);
        break;

      case 2:
        value = Cal_PD_Input_Dac(Get_PD_Points);
        break;

      case 3:
        value = Cal_PD_Input_Row_IL(Get_PD_Points);
        break;

      case 4:
        value = Cal_PD_Input_Row_Dac(Get_PD_Points);
        break;

      default:
        break;
      }

      Serial.println("PD Power:" + String(value)); //dB
    }

    // if (cmd == "" && cmd_No == 0)
    //   delay(100);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

void Move_Motor_abs(int xyz, long Target)
{
  String axis = "";
  long Pos_Now = 0;
  switch (xyz)
  {
  case 0:
    MotorDir_Pin = X_DIR_Pin;
    MotorSTP_Pin = X_STP_Pin;
    Pos_Now = X_Pos_Now;
    axis = "X";
    break;
  case 1:
    MotorDir_Pin = Y_DIR_Pin;
    MotorSTP_Pin = Y_STP_Pin;
    Pos_Now = Y_Pos_Now;
    axis = "Y";
    break;
  case 2:
    MotorDir_Pin = Z_DIR_Pin;
    MotorSTP_Pin = Z_STP_Pin;
    Pos_Now = Z_Pos_Now;
    axis = "Z";
    break;
  }

  if (Target - Pos_Now < 0)
    MotorCC = false;
  else if (Target - Pos_Now > 0)
    MotorCC = true;
  else
    return;

  delayBetweenStep = 20;
  MinMotroStep = abs(Target - Pos_Now);
  Serial.println(axis + " Go to position: " + String(Target) + ", origin position: " + String(Pos_Now));

  //byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay
  Move_Motor(MotorDir_Pin, MotorSTP_Pin, MotorCC, MinMotroStep, delayBetweenStep, 0);
}

void Move_Motor_abs_all(int x, int y, int z)
{
  Move_Motor_abs(0, x);
  Move_Motor_abs(1, y);
  Move_Motor_abs(2, z);
}

void Move_Motor(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay)
{
  MotorCC = dirt;
  digitalWrite(dir_pin, dirt);
  delay(2);
  if (moveSteps > 0)
  {
    step(stp_pin, moveSteps, delayStep);
    delay(stableDelay);
    DataOutput(false);
  }
}

void Move_Motor(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep, int stableDelay, bool isOutputPosition)
{
  MotorCC = dirt;
  digitalWrite(dir_pin, dirt);
  delay(5);
  if (moveSteps > 0)
  {
    step(stp_pin, moveSteps, delayStep);
    delay(stableDelay);

    if (isOutputPosition)
      DataOutput(false);
  }
}

void Move_Motor_Cont(byte dir_pin, byte stp_pin, bool dirt, long moveSteps, int delayStep)
{
  MotorSTP_Pin = dir_pin;

  if (MotorDir_Pin != dir_pin || MotorCC != dirt)
  {
    MotorCC = dirt;
    MotorDir_Pin = dir_pin;
    digitalWrite(MotorDir_Pin, MotorCC); //步進馬達方向控制, false為負方向
    delay(3);
  }

  step(stp_pin, moveSteps, delayStep);
}

//------------------------------------------------------------------------------------------------------------------------------------------

String Region, msg;
bool Fine_Scan(int axis, bool Trip2Stop)
{
  isLCD = true;
  LCD_Update_Mode = 1;

  Serial.println("");
  Serial.println("Fine Scan ");

  Serial.println("Stop Value: " + String(StopValue));

  digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
  delay(5);

  Threshold = -37.3;
  delayBetweenStep = 50;

  double pdBest = Cal_PD_Input_IL(Get_PD_Points);

  // Region = Region + "_Fine_Scan";
  String msg;

  bool K_OK = true;

  if (axis < 4)
  {
    switch (axis)
    {
    case 1:

      //        X_ScanSTP = 12;
      PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);
      msg = Region + ", X Scan, Rount " + String(1) + ", Trip ";

      Scan_AllRange_TwoWay(0, 8, 20, 0, 0, 80, StopValue, 300, 2, "X Scan, Trip "); //steps:350
      // K_OK = Scan_1D_TwoWay(0, 30, Threshold, X_ScanSTP, 0, MotorCC_X, delayBetweenStep, 2 * Get_PD_Points, StopValue, msg, PD_Now, Trip2Stop); //Delay:50

      CMDOutput("%:");

      // if (!K_OK)
      // {
      //   msg = Region + ", X Re-Scan, Rount " + String(1) + ", Trip ";
      //   Scan_1D_TwoWay(0, 30, Threshold, X_ScanSTP, 0, MotorCC_X, delayBetweenStep, 2 * Get_PD_Points, StopValue, msg, PD_Now, Trip2Stop); //Delay:50
      //   CMDOutput("%:");
      // }

      break;

    case 2:

      //        Y_ScanSTP = 10;
      PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);
      msg = Region + ", Y Scan, Rount " + String(1) + ", Trip ";

      Scan_AllRange_TwoWay(1, 8, 20, 0, 0, 80, StopValue, 300, 2, "Y Scan, Trip "); //steps:350
      // K_OK = Scan_1D_TwoWay(1, 30, Threshold, Y_ScanSTP, 0, MotorCC_Y, delayBetweenStep, 2 * Get_PD_Points, StopValue, msg, PD_Now, Trip2Stop); //Delay:50

      CMDOutput("%:");

      // if (!K_OK)
      // {
      //   msg = Region + ", Y Re-Scan, Rount " + String(1) + ", Trip ";
      //   Scan_1D_TwoWay(1, 30, Threshold, Y_ScanSTP, 0, MotorCC_Y, delayBetweenStep, 2 * Get_PD_Points, StopValue, msg, PD_Now, Trip2Stop); //Delay:50
      //   CMDOutput("%:");
      // }

      break;

    case 3:

      //        Z_ScanSTP = 100;
      PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);
      msg = Region + ", Z Scan, Rount " + String(1) + ", Trip ";

      Scan_AllRange_TwoWay(2, 6, 200, 0, 0, 60, StopValue, 300, 2, "Z Scan, Trip ");
      // K_OK = Scan_1D_TwoWay(2, 14, Threshold, Z_ScanSTP, 0, MotorCC_Z, delayBetweenStep, 2 * Get_PD_Points, StopValue, msg, PD_Now, Trip2Stop); //Delay:130

      CMDOutput("%:");
      break;
    }
  }
  //Case 4: all actions should be excuted
  else if (axis == 4)
  {
    double PD_Value[6];
    double Delta_PD[3];
    double delta_PD_X, delta_PD_Y, delta_PD_Z;

    //X judge
    PD_Value[0] = Cal_PD_Input_IL(2 * Get_PD_Points);
    Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC_X, X_ScanSTP * 2, 5, 50); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
    PD_Value[1] = Cal_PD_Input_IL(2 * Get_PD_Points);
    if (PD_Value[1] < PD_Value[0])
      Move_Motor(X_DIR_Pin, X_STP_Pin, !MotorCC_X, X_ScanSTP * 2, 5, 50); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
    Delta_PD[0] = abs(PD_Value[1] - PD_Value[0]);
    delta_PD_X = Delta_PD[0];

    //Y judge
    PD_Value[2] = Cal_PD_Input_IL(2 * Get_PD_Points);
    Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC_Y, Y_ScanSTP * 2, 5, 50); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
    PD_Value[3] = Cal_PD_Input_IL(2 * Get_PD_Points);
    if (PD_Value[3] < PD_Value[2])
      Move_Motor(Y_DIR_Pin, Y_STP_Pin, !MotorCC_Y, Y_ScanSTP * 2, 5, 50); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
    Delta_PD[1] = abs(PD_Value[3] - PD_Value[2]);
    delta_PD_Y = Delta_PD[1];

    //Z judge
    PD_Value[4] = Cal_PD_Input_IL(2 * Get_PD_Points);
    Move_Motor(Z_DIR_Pin, Z_STP_Pin, MotorCC_Z, Z_ScanSTP * 2, 5, 130); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
    PD_Value[5] = Cal_PD_Input_IL(2 * Get_PD_Points);
    //    if (PD_Value[5] < PD_Value[4]) {
    //      Move_Motor(Z_DIR_Pin, Z_STP_Pin, !MotorCC_Z, Z_ScanSTP * 8, 10, 130); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
    //      Move_Motor(Z_DIR_Pin, Z_STP_Pin, MotorCC_Z, Z_ScanSTP * 8, 10, 130); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
    //    }

    Delta_PD[2] = abs(PD_Value[5] - PD_Value[4]);
    delta_PD_Z = Delta_PD[2];

    Serial.println("PD_Value Array:");
    for (int i = 0; i < 6; i++)
    {
      Serial.println(String(PD_Value[i]));
    }

    PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);
    Serial.println("PD Now: " + String(PD_Now));

    Serial.println("Delta IL X,Y,Z: " + String(Delta_PD[0]) + ", " + String(Delta_PD[1]) + ", " + String(Delta_PD[2]));

    //    return;

    //Bubble rank
    for (int i = 0; i < 3; i++)
    {
      for (int j = 1; j < 3; ++j)
      {
        double tempValue;

        if (Delta_PD[j] < Delta_PD[j - 1])
        {
          tempValue = Delta_PD[j - 1];
          Delta_PD[j - 1] = Delta_PD[j];
          Delta_PD[j] = tempValue;
        }
      }
    }

    Serial.println("Delta IL X,Y,Z in order: " + String(Delta_PD[0]) + ", " + String(Delta_PD[1]) + ", " + String(Delta_PD[2]));

    for (int k = 2; k >= 0; k--)
    {
      if (Delta_PD[k] == delta_PD_X)
      {
        Serial.println("X Scan");
        //        X_ScanSTP = 12;
        PD_Now = Cal_PD_Input_IL(4 * Get_PD_Points);
        msg = Region + ", X Scan, Rount " + String(1) + ", Trip ";
        //(int XYZ, int count, int Threshold, int motorStep, int stableDelay,bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg, double preIL)
        Scan_1D_TwoWay(0, 30, Threshold, X_ScanSTP, 25, MotorCC_X, delayBetweenStep, 4 * Get_PD_Points, StopValue, msg, PD_Now, Trip2Stop); //Delay:50
        CMDOutput("%:");
      }

      else if (Delta_PD[k] == delta_PD_Y)
      {
        Serial.println("Y Scan");
        //        Y_ScanSTP = 10;
        PD_Now = Cal_PD_Input_IL(20);
        msg = Region + ", Y Scan, Rount " + String(1) + ", Trip ";
        //(int XYZ, int count, int Threshold, int motorStep, int stableDelay,bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg, double preIL)
        Scan_1D_TwoWay(1, 30, Threshold, Y_ScanSTP, 25, MotorCC_Y, delayBetweenStep, 4 * Get_PD_Points, StopValue, msg, PD_Now, Trip2Stop); //Delay:50
        CMDOutput("%:");
      }

      else if (Delta_PD[k] == delta_PD_Z)
      {
        Serial.println("Z Scan");
        //        Z_ScanSTP = 100;
        PD_Now = Cal_PD_Input_IL(20);
        msg = Region + ", Z Scan, Rount " + String(1) + ", Trip ";
        //(int XYZ, int count, int Threshold, int motorStep, int stableDelay,bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg, double preIL)
        //    Scan_1D_TwoWay(2, 14,  Threshold,  80, 180, MotorCC_Z, delayBetweenStep, 10, StopValue, msg, PD_Now);  //Delay:220

        Scan_1D_TwoWay(2, 14, Threshold, Z_ScanSTP, 80, MotorCC_Z, delayBetweenStep, 4 * Get_PD_Points, StopValue, msg, PD_Now, Trip2Stop); //Delay:130
        CMDOutput("%:");
      }
    }

    Serial.println("Delta IL X,Y,Z: " + String(delta_PD_X) + ", " + String(delta_PD_Y) + ", " + String(delta_PD_Z));
    //    Serial.println("Delta IL X,Y,Z: " + String(Delta_PD[0]) + ", " + String(Delta_PD[1]) + ", " + String(Delta_PD[2]));
  }
  else if (axis == 5)
  {
    // Region = Region + "_Fine_Scan (All Range)";
    msg = Region + "_Fine_Scan (All Range)" + ", Z Scan, Trip ";
    Scan_AllRange_TwoWay(2, 6, 200, 0, 0, 60, StopValue, 300, 2, "Z Scan, Trip ");
    CMDOutput("%:");

    if (isStop)
    {
      Serial.println("Emergency Stop");
      digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
      return true;
    }

    msg = Region + ", Y Scan, Trip ";
    Scan_AllRange_TwoWay(1, 8, 20, 0, 0, 80, StopValue, 300, 2, "Y Scan, Trip ");
    CMDOutput("%:");

    if (isStop)
    {
      Serial.println("Emergency Stop");
      digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
      return true;
    }

    msg = Region + ", X Scan, Trip ";
    Scan_AllRange_TwoWay(0, 8, 20, 0, 0, 80, StopValue, 300, 2, "X Scan, Trip ");
    CMDOutput("%:");
  }

  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
  delay(5);
  Serial.println("Fine Scan End");

  isLCD = true;
  LCD_Update_Mode = 0;
  LCD_PageNow = 100;
}

//------------------------------------------------------------------------------------------------------------------------------------------

void AutoAlign()
{
  isLCD = true;
  LCD_Update_Mode = 1;

  //  Reset_Z_to_Origin(); //Back to origin

  Move_Motor(Z_DIR_Pin, Z_STP_Pin, false, 15000, 12, 150); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)

  unsigned long time1 = 0, time2 = 0, time3 = 0, time4 = 0, time5 = 0, time6 = 0, time7 = 0;
  double PD_LV1, PD_LV2, PD_LV3, PD_LV4, PD_LV5, PD_LV6;
  double PD_Now = 0;
  time1 = millis();
  Serial.println(" ");
  CMDOutput("AS");
  Serial.println("Auto-Align Start");
  Serial.println("... Spiral ...");

  //Spiral - Rough - 1
  int matrix_level = 10;
  CMDOutput("^X");
  CMDOutput("R:" + String(M_Level * 2 + 1));
  CMDOutput("C:" + String(M_Level * 2 + 1));
  // Serial.println("Rows=" + String(matrix_level * 2 + 1));
  // Serial.println("Columns=" + String(matrix_level * 2 + 1));
  // Serial.println("^X");

  delayBetweenStep = 25;
  MinMotroStep = 2000; //1200
  Region = "Sprial(Rough)";
  AutoAlign_Spiral(matrix_level, -48, 0); //Input : (Sprial Level, Threshold, stable)  Threshold:123
  CMDOutput("X^");
  // Serial.println("X^");

  DataOutput();

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  int Threshold = -21.4; //default: 144
  int stableDelay = 0;   //default : 25
  // Get_PD_Points = 6;    //default: 15
  MinMotroStep = 150;

  msg = Region + ",Y Scan, Trip ";

  if (PD_Now > -9)
    MinMotroStep = 25;
  else if (PD_Now > -16 && PD_Now <= -9)
    MinMotroStep = 30;
  else if (PD_Now > -22 && PD_Now <= -16)
    MinMotroStep = 40;
  else
    MinMotroStep = 50;
  AutoAlign_Scan_DirectionJudge_V2(1, 20, Threshold, MinMotroStep, stableDelay, MotorCC_Y, delayBetweenStep, Get_PD_Points, -4.7, msg);

  CMDOutput("%:");

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  MinMotroStep = 250;

  msg = Region + ",X Scan, Trip ";

  if (PD_Now > -9)
    MinMotroStep = 30;
  else if (PD_Now > -16 && PD_Now <= -9)
    MinMotroStep = 40;
  else if (PD_Now > -22 && PD_Now <= -16)
    MinMotroStep = 60;
  else if (PD_Now > -29.6)
    MinMotroStep = 80;
  else
    MinMotroStep = 140;
  AutoAlign_Scan_DirectionJudge_V2(0, 17, Threshold, MinMotroStep, stableDelay, MotorCC_X, delayBetweenStep, Get_PD_Points, -4.7, msg);

  CMDOutput("%:");

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  PD_LV1 = Cal_PD_Input_IL(Get_PD_Points);
  time2 = millis();
  Serial.println("Sprial(Rough) TimeSpan : " + String((time2 - time1) / 1000) + " s");
  Serial.println(" ");

  if (isStop)
    return;

  Serial.println(String(PD_LV1));
  Serial.println(String(isStop));

  //X, Y Scan - For Spiral Detail
  if (PD_LV1 < -40) //default: 300
  {
    Region = "Sprial(Detail)";
    int matrix_level = 10;
    CMDOutput("^X");
    CMDOutput("R:" + String(M_Level * 2 + 1));
    CMDOutput("C:" + String(M_Level * 2 + 1));
    // Serial.println("Rows=" + String(matrix_level * 2 + 1));
    // Serial.println("Columns=" + String(matrix_level * 2 + 1));
    // Serial.println("^X");

    delayBetweenStep = 25;
    MinMotroStep = 1500;                    //700
    AutoAlign_Spiral(matrix_level, -37, 0); //Input : (Sprial Level, Threshold, stable)  Threshold:166

    CMDOutput("X^");
    // Serial.println("X^");

    //XY Scan after Detail Spiral Scan
    if (false)
    {
      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

      int Threshold = 204;  //default: 144
      int stableDelay = 50; //default : 25

      delayBetweenStep = 60;
      Get_PD_Points = 500;
      MinMotroStep = 150;

      msg = Region + ",Y Scan, Trip ";

      if (PD_Now > 260)
        MinMotroStep = 25;
      else if (PD_Now > 230 && PD_Now <= 260)
        MinMotroStep = 30;
      else if (PD_Now > 200 && PD_Now <= 230)
        MinMotroStep = 40;
      else
        MinMotroStep = 50;
      // AutoAlign_Scan_DirectionJudge_V2(1, 20, Threshold, MinMotroStep, stableDelay, MotorCC_Y, delayBetweenStep, Get_PD_Points, 279, msg);
      Scan_AllRange_TwoWay(1, 5, MinMotroStep, 40, 0, delayBetweenStep, -4.5, Get_PD_Points, 2, msg);

      CMDOutput("%:");

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

      MinMotroStep = 250;

      msg = Region + ",X Scan, Trip ";

      if (PD_Now > -9)
        MinMotroStep = 30;
      else if (PD_Now > -16 && PD_Now <= -9)
        MinMotroStep = 40;
      else if (PD_Now > -22.7 && PD_Now <= -16)
        MinMotroStep = 60;
      else if (PD_Now > -29.5)
        MinMotroStep = 80;
      else
        MinMotroStep = 140;
      // AutoAlign_Scan_DirectionJudge_V2(0, 17, Threshold, MinMotroStep, stableDelay, MotorCC_X, delayBetweenStep, Get_PD_Points, 279, msg);
      Scan_AllRange_TwoWay(0, 5, MinMotroStep, 40, 0, delayBetweenStep, -4.5, Get_PD_Points, 2, msg);

      CMDOutput("%:");
    }
  }

  PD_LV2 = Cal_PD_Input_IL(Get_PD_Points);
  time3 = millis();
  Serial.println("X, Y Scan (after Spiral) TimeSpan : " + String((time3 - time2) / 1000) + " s");
  Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));

  if (isStop)
    return;

  PD_Now = PD_LV2;
  if (PD_Now < -50)
  {
    Serial.println("High loss before rough-scan");
    return;
  }

  Serial.println(" ");
  Serial.println("... X, Y, Z Scan(Rough) ...");
  Region = "Scan(Rough)";
  double PD_Before = 0;
  double PD_After = 0;
  delayBetweenStep = 50; //Default 20
  Threshold = 124;
  stableDelay = 10; //Default 25
  int scanPoints = 8;
  int stopValue = -3; //Default : -2.9
  int delta_X, delta_Y, X_pos_before, Y_pos_before;
  double PDValue_After_Scan = -60;

  //Rough-Scan
  if (PD_Now < -4 && true)
  {
    Serial.println("-- Scan(Rough) --");
    for (int i = 0; i < 15; i++)
    {
      if (i > 0 && abs(PD_After - PD_Before) < 0.1) //1.2
      {
        Serial.println("Break Loop ... :" + String(PD_After) + ", " + String(PD_Before));
        return;
      }
      else
        Serial.println("X, Y, Z Scan(Rough) Round :" + String(i + 1) + ", After:" + String(PD_After) + ", Before:" + String(PD_Before));

      PD_Before = Cal_PD_Input_IL(Get_PD_Points);

      if (PD_Before > stopValue)
        break;

      stableDelay = 100;
      double PD_Z_before = 0;
      for (int r = 0; r < 5; r++)
      {
        if (isStop)
          return;

        PD_Z_before = Cal_PD_Input_IL(Get_PD_Points);
        Serial.println("PD Z before:" + String(PD_Z_before));

        int motorStep = 1000;
        if (PD_Z_before < -30)
        {
          motorStep = 10000;
          Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, motorStep, 20, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
        }
        else
        {
          double ratio_idx = 2.5;
          if (PD_Z_before > -29.5 && PD_Z_before <= -22.7)
            ratio_idx = 3.2; //default: 2.5
          else if (PD_Z_before > -22.7 && PD_Z_before <= -18.2)
            ratio_idx = 2.9; //default: 2
          else if (PD_Z_before > -18.2 && PD_Z_before <= -9)
            ratio_idx = 2.5; //default: 1.8
          else if (PD_Z_before > -9)
            ratio_idx = 1.8; //default: 0.8

          //default: 0.5
          motorStep = abs(ratio_idx * (588 + (55 * abs(PD_Z_before)))); //Default: 588-55*pd
          if (motorStep < 1000)
            motorStep = 1000; //default: 800

          Serial.println("ratio_idx : " + String(ratio_idx));

          Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, motorStep, 20, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
        }

        Serial.println("Z feed : " + String(motorStep));
        DataOutput();

        PD_Now = Cal_PD_Input_IL(Get_PD_Points);

        if (PD_Now > stopValue)
        {
          Serial.println("Over Stop Value");
        }

        if (PD_Now <= PD_Z_before || (PD_Z_before - PD_Now) > 30 || abs(PD_Z_before - PD_Now) <= 1.5)
        {
          Serial.println("Z feed break, Now: " + String(PD_Now) + ", Before: " + String(PD_Z_before) + ", Z Pos Now:" + String(Z_Pos_Now));
          Serial.println(" ");
          break;
        }
        else
        {
          Serial.println("Z feed , Now: " + String(PD_Now) + ", Before: " + String(PD_Z_before) + ", Z Pos Now:" + String(Z_Pos_Now));
        }
      }

      Threshold = 120;
      scanPoints = 11;
      // delayBetweenStep = 8;
      stableDelay = 25; //default: 25

      //default:220
      if (PD_Now < -54)
      {
        Serial.println("Spiral : in z-feed region");

        CMDOutput("^X");
        CMDOutput("R:" + String(M_Level * 2 + 1));
        CMDOutput("C:" + String(M_Level * 2 + 1));
        // Serial.println("Rows=" + String(23));
        // Serial.println("Columns=" + String(23));
        // Serial.println("^X");

        MinMotroStep = 300; //350
        //Input : (Sprial Level, Threshold, stable)
        if (!AutoAlign_Spiral(11, -36.4, 0))
        {
          CMDOutput("X^");
          // Serial.println("X^");
          Serial.println("Spiral: Target IL not found");
          Serial.println(" ");

          return;
        }
        CMDOutput("X^");
        // Serial.println("X^");

        Serial.println(" ");

        if (isStop)
          return;

        stableDelay = 0;

        Region = "Scan(Rough) (1)";

        msg = Region + ", Y Scan" + ", Trip ";

        AutoAlign_Scan_DirectionJudge_V2(1, 20, Threshold, 150, stableDelay, MotorCC_Y, delayBetweenStep, Get_PD_Points, 279, msg);
        // Scan_AllRange_TwoWay(1, 5, 200, 40, 0, 8, 279, 1, 1, msg);

        CMDOutput("%:");

        msg = Region + ", X Scan" + ", Trip ";

        AutoAlign_Scan_DirectionJudge_V2(0, 25, Threshold, 150, stableDelay, MotorCC_X, delayBetweenStep, Get_PD_Points, 279, msg);
        // Scan_AllRange_TwoWay(0, 5, 200, 40, 0, 8, 279, 1, 1, msg);

        CMDOutput("%:");

        PD_Now = Cal_PD_Input_IL(Get_PD_Points);
        if (PD_Now < -54)
          return;

        double b, a;
        int rd = 0;
        for (int s = 0; s < rd; s++)
        {
          b = Cal_PD_Input_IL(Get_PD_Points);

          if (isStop)
            return;

          Region = "Scan(Rough) (2)";

          //(int XYZ, int count, int Threshold, int motorStep, int stableDelay,
          //bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg)

          msg = Region + ", Y Scan" + ", Trip ";
          AutoAlign_Scan_DirectionJudge_V2(1, 15, Threshold, 80, stableDelay, MotorCC_Y, delayBetweenStep, Get_PD_Points, stopValue, msg);
          CMDOutput("%:");

          msg = Region + ", X Scan" + ", Trip ";
          AutoAlign_Scan_DirectionJudge_V2(0, 13, Threshold, 100, stableDelay, MotorCC_X, delayBetweenStep, Get_PD_Points, stopValue, msg);
          CMDOutput("%:");

          a = Cal_PD_Input_IL(Get_PD_Points);
          if (abs(a - b) < 5)
            break;
        }
      }

      PD_After = Cal_PD_Input_IL(Get_PD_Points);
      if (PD_After > stopValue)
        break;

      delayBetweenStep = 80;
      MinMotroStep = 350; //800
      stableDelay = 0;    //default:45

      if (isStop)
        return;

      Region = "Scan(Rough) (3)";

      msg = Region + ", Y Scan" + ", Trip ";
      if (PD_After > -9)
        MinMotroStep = 25; //default:25
      else if (PD_After > -16 && PD_After <= -9)
        MinMotroStep = 30; //default:30
      else if (PD_After > -22.7 && PD_After <= -16)
        MinMotroStep = 40; //default:40
      else
        MinMotroStep = 70; //default:70

      PD_After = AutoAlign_Scan_DirectionJudge_V2(1, 20, Threshold, MinMotroStep, stableDelay, MotorCC_Y, delayBetweenStep, Get_PD_Points, stopValue, msg);
      // Scan_AllRange_TwoWay(1, 6, MinMotroStep, 0, 0, delayBetweenStep, stopValue, Get_PD_Points, 2, msg);

      CMDOutput("%:");

      // PD_After = Cal_PD_Input_IL(Get_PD_Points);

      msg = Region + ", X Scan" + ", Trip ";
      if (PD_After > -9)
        MinMotroStep = 25; //default:25
      else if (PD_After > -16 && PD_After <= -9)
        MinMotroStep = 30; //default:30
      else if (PD_After > -22.7 && PD_After <= -16)
        MinMotroStep = 80; //default:80
      else if (PD_After > -30)
        MinMotroStep = 100; //default:100
      else
        MinMotroStep = 120; //default:120

      PD_After = AutoAlign_Scan_DirectionJudge_V2(0, 20, Threshold, MinMotroStep, stableDelay, MotorCC_X, delayBetweenStep, Get_PD_Points, stopValue, msg);
      // Scan_AllRange_TwoWay(0, 6, MinMotroStep, 0, 0, delayBetweenStep, stopValue, Get_PD_Points, 2, msg);

      CMDOutput("%:");

      PD_After = Cal_PD_Input_IL(Get_PD_Points);

      DataOutput();

      if (abs(PD_After - PDValue_After_Scan) < 0.1 && PD_After > -10) //default:16
      {
        Serial.println("Scan(Rough)(A) - Pass best Z position");
        Serial.println(String(PD_After) + ", " + String(PDValue_After_Scan));
        break;
      }

      if (PD_After < PDValue_After_Scan && PD_After > -10) //default:16
      {
        Serial.println("Scan(Rough)(B) - Pass best Z position : " + String(PD_After) + ", " + String(PDValue_After_Scan));
        break;
      }

      if (PD_After < -42) //default:16
      {
        Serial.println("Scan(Rough)(C) - Pass best Z position");
        Serial.println(String(PD_After) + ", " + String(PDValue_After_Scan));
        break;
      }

      PDValue_After_Scan = PD_After;

      if (i == 1)
      {
        X_pos_before = X_Pos_Now;
        Y_pos_before = Y_Pos_Now;
      }
      else if (i > 1)
      {
        delta_X = X_Pos_Now - X_pos_before;
        delta_Y = Y_Pos_Now - Y_pos_before;
        X_pos_before = X_Pos_Now;
        Y_pos_before = Y_Pos_Now;
        Serial.println("------- Delta X: " + String(delta_X) + ", Delta Y: " + String(delta_Y));
      }

      if (PD_After > stopValue)
      {
        Serial.println("Better than stopValue: " + String(PD_After) + ", stopvalue: " + String(stopValue));
        Serial.println(" ");
        break;
      }

      if (PD_After < -54)
      {
        Serial.println("Rough Scan : High loss");
        return;
      }
    }
  }

  PD_LV3 = Cal_PD_Input_IL(Get_PD_Points);
  time4 = millis();
  Serial.println("Scan(Rough) TimeSpan : " + String((time4 - time3) / 1000) + " s");

  if (isStop)
    return;

  //Scan(Detail 1)
  Serial.println(" ");
  Serial.println("... X, Y, Z Scan(Detail 1) ...");
  Region = "Scan(Detail 1)";

  if (false)
  {
    // Fine_Scan(4, true);

    Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
  }

  PD_LV4 = Cal_PD_Input_IL(Get_PD_Points);
  time5 = millis();
  Serial.println("Scan(Detail 1) TimeSpan : " + String((time5 - time4) / 1000) + " s");

  if (isStop)
    return;

  //Scan(Detail 2)
  Serial.println(" ");
  Serial.println("........ X, Y, Z Scan(Detail 2) ........");
  Region = "Scan(Detail 2)";

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  PDValue_Best = PD_Now;

  if (true && PD_Now > -15)
  {
    Scan_AllRange_TwoWay(2, 6, 200, 0, 0, 60, StopValue, 500, 2, Region + "_Z Scan, Trip ");
    CMDOutput("%:");

    // Fine_Scan(2, true);
    Scan_AllRange_TwoWay(1, 8, 20, 0, 0, 80, StopValue, 550, 2, Region + "_Y Scan, Trip ");
    CMDOutput("%:");

    // Fine_Scan(1, true);
    Scan_AllRange_TwoWay(0, 8, 20, 0, 0, 80, StopValue, 550, 2, Region + "_X Scan, Trip ");
    CMDOutput("%:");

    Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
  }

  //    return;

  PD_LV5 = Cal_PD_Input_IL(Get_PD_Points);
  PDValue_Best = -2.45;
  time6 = millis();

  if (isStop)
    return;

  //Detail-Scan-C
  Serial.println(" ");
  Serial.println("... X, Y, Z Scan(Detail 3) ...");
  Region = "Scan(Detail 3)";

  if (false && abs(PD_LV5 - PD_LV4) > 0.8)
  {
    // Scan_AllRange_TwoWay(2, 3, 150, 180, 0, 5, StopValue, 11, 2, Region + "_Z Scan, Trip ");
    Scan_AllRange_TwoWay(2, 3, 50, 0, 0, delayBetweenStep, StopValue, Get_PD_Points, 2, Region + "_Z Scan, Trip ");
    CMDOutput("%:");

    // Fine_Scan(2, true);
    Scan_AllRange_TwoWay(1, 5, 30, 0, 0, delayBetweenStep, StopValue, Get_PD_Points, 2, Region + "_Y Scan, Trip ");
    CMDOutput("%:");

    // Fine_Scan(1, true);
    Scan_AllRange_TwoWay(0, 4, 25, 0, 0, delayBetweenStep, StopValue, Get_PD_Points, 2, Region + "_X Scan, Trip ");
    CMDOutput("%:");

    //    Fine_Scan(5, false);

    //    Scan_AllRange_TwoWay(XYZ, count, motorStep, stableDelay,
    //                         Direction, delayBetweenStep, StopPDValue, Get_PD_Points, msg);

    Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
  }

  PD_LV6 = Cal_PD_Input_IL(Get_PD_Points);
  time7 = millis();

  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

  if (isStop)
    return;

  isLCD = true;
  LCD_Update_Mode = 0;
  LCD_PageNow = 100;

  //      Threshold = 198;
  Serial.println(" ");
  Serial.println("Sprial(Rough) TimeSpan : " + String((time2 - time1) / 1000) + " s, PD : " + String(PD_LV1));
  Serial.println("Sprial(Detail) TimeSpan : " + String((time3 - time2) / 1000) + " s, PD : " + String(PD_LV2));
  Serial.println("Scan(Rough) TimeSpan : " + String((time4 - time3) / 1000) + " s, PD : " + String(PD_LV3));
  Serial.println("Scan(Detail 1) TimeSpan : " + String((time5 - time4) / 1000) + " s, PD : " + String(PD_LV4));
  Serial.println("Scan(Detail 2) TimeSpan : " + String((time6 - time5) / 1000) + " s, PD : " + String(PD_LV5));
  Serial.println("Scan(Detail 3) TimeSpan : " + String((time7 - time6) / 1000) + " s, PD : " + String(PD_LV6));
  Serial.println("Auto Align TimeSpan : " + String((time7 - time1) / 1000) + " s");
  Serial.println("Position XYZ: " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) +
                 ", " + String(Z_Pos_Now));
}

//------------------------------------------------------------------------------------------------------------------------------------------

int matrix_edge;
int x = 0, y = 0;
double AutoAlign_Result[3] = {0, 0, 0};

bool AutoAlign_Spiral(int M, double StopValue, int stableDelay)
{
  Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
  // Serial.println("Set_Steps:" + String(MinMotroStep));
  CMDOutput("ST" + String(MinMotroStep));
  // CMDOutput("M:" + String(M));
  // Serial.println("M:" + String(M));
  Serial.println("StopValue:" + String(StopValue));
  Serial.println("stableDelay:" + String(stableDelay));

  double ts = 0;
  unsigned long timer_1 = 0, timer_2 = 0;
  timer_1 = millis();

  double PD_BestIL = -100, PD_Now = -100;
  int PD_BestIL_Position[2];
  int PD_Best_Pos_Abs[2];

  double SpiralStop_Threshold = StopValue; //Default : 198
  bool isFindThreshold = false;

  AutoAlign_Result[0] = 0;
  AutoAlign_Result[1] = 0;
  AutoAlign_Result[2] = 0;

  matrix_edge = 2 * M + 1;
  x = 0;
  y = 0;

  PD_BestIL = -100;
  PD_BestIL_Position[0] = x;
  PD_BestIL_Position[1] = y;

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  int m = 1;

  CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now); //[0,0]
  // Serial.println("$[" + String(x) + "," + String(y) + "]=" + PD_Now);
  PD_BestIL = PD_Now;
  PD_BestIL_Position[0] = x;
  PD_BestIL_Position[1] = y;
  PD_Best_Pos_Abs[0] = X_Pos_Now;
  PD_Best_Pos_Abs[1] = Y_Pos_Now;
  Serial.println("Inital:(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));

  if (PD_Now >= SpiralStop_Threshold)
  {
    Serial.println("Over Threshold: " + String(PD_Now) + ", Threshold: " + String(SpiralStop_Threshold));
    return true;
  }

  for (int n = 1; abs(n) < (M + 1); n++)
  {
    if (isStop)
      return true;

    CMDOutput("ML");
    Serial.println("Matrix Layers: " + String(n));

    if (n > m)
      m++;

    y--;
    MotorCC = false;
    Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay, true);

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    // Serial.println("$[" + String(x) + "," + String(y) + "," + String(Z_Pos_Now) + "]=" + PD_Now);  //[0,-1]
    CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now); //[0,-1]

    if (PD_Now > PD_BestIL)
    {
      PD_BestIL = PD_Now;
      PD_BestIL_Position[0] = x;
      PD_BestIL_Position[1] = y;
      PD_Best_Pos_Abs[0] = X_Pos_Now;
      PD_Best_Pos_Abs[1] = Y_Pos_Now;

      if (PD_Now >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_Now) + ", Threshold: " + String(SpiralStop_Threshold));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));
        // Serial.print("TS:");
        // Serial.println(ts, 2);

        return true;
      }
    }

    x--;

    if (isStop)
      return true;

    //To Left

    MotorCC = false;
    // Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC, X_backlash, delayBetweenStep, stableDelay, false);

    while (x >= (-n))
    {
      Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);
      // Serial.println("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          Serial.println(String(PD_BestIL_Position[0]) + ", " + String(PD_BestIL_Position[1]));
          return true;
        }
      }
      x--;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL) + ", Threshold: " + String(SpiralStop_Threshold));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));
        // Serial.print("TS:");
        // Serial.println(ts, 2);

        return true;
      }

      if (isStop)
        return true;
    }

    x++;
    y++;

    if (isStop)
      return true;

    //Up

    MotorCC = true;

    // Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, Y_backlash, delayBetweenStep, stableDelay, false);

    int nM = n;
    while (y <= (nM))
    {
      Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);
      // Serial.println("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          return true;
        }
      }
      y++;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));
        // Serial.print("TS:");
        // Serial.println(ts, 2);

        return true;
      }

      if (isStop)
        return true;
    }

    y--;
    x++;

    if (isStop)
      return true;

    //To Right

    MotorCC = true;
    // Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC, X_backlash, delayBetweenStep, stableDelay, false);

    while (x <= (n))
    {
      Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);
      // Serial.println("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          return true;
        }
      }
      x++;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));
        // Serial.print("TS:");
        // Serial.println(ts, 2);

        return true;
      }

      if (isStop)
        return true;
    }

    x--;
    y--;

    if (isStop)
      return true;

    //Down

    MotorCC = false;
    // Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, Y_backlash, delayBetweenStep, stableDelay);

    while (y >= (-n))
    {
      Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);
      // Serial.println("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

      if (PD_Now > PD_BestIL)
      {
        PD_BestIL = PD_Now;
        PD_BestIL_Position[0] = x;
        PD_BestIL_Position[1] = y;
        Serial.println("Best IL Update : (" + String(PD_BestIL_Position[0]) + "," + String(PD_BestIL_Position[1]) + ") = " + String(PD_BestIL));
        Serial.println("(" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + ") = " + String(PD_BestIL));
        PD_Best_Pos_Abs[0] = X_Pos_Now;
        PD_Best_Pos_Abs[1] = Y_Pos_Now;

        if (abs(PD_BestIL_Position[0]) >= 100 || abs(PD_BestIL_Position[1]) >= 100)
        {
          return true;
        }
      }
      y--;

      if (PD_BestIL >= SpiralStop_Threshold)
      {
        Serial.println("Over Threshold: " + String(PD_BestIL));

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));
        // Serial.print("TS:");
        // Serial.println(ts, 2);

        return true;
      }

      if (isStop)
        return true;
    }

    y++;
  }

  CMDOutput("ML");
  Serial.println("Matrix Layers: Max");

  if (isStop)
    return true;

  int delta_X = 0, delta_Y = 0;

  if (!sprial_JumpToBest)
  {
    PD_BestIL_Position[0] = 0;
    PD_BestIL_Position[1] = 0; //Jump to (0,0)
  }

  if (PD_BestIL_Position[0] <= 2 * M && PD_BestIL_Position[1] <= 2 * M)
  {
    Move_Motor_abs(0, PD_Best_Pos_Abs[0]);
    Move_Motor_abs(1, PD_Best_Pos_Abs[1]);

    delay(200);

    // delta_X = PD_BestIL_Position[0] - x;
    // delta_Y = PD_BestIL_Position[1] - y;

    // String ss = "Delta x, y = " + String(delta_X) + "," + String(delta_Y);
    // Serial.println(ss);

    // if (delta_X < 0)
    //   MotorCC = false;
    // else
    //   MotorCC = true;

    // digitalWrite(X_DIR_Pin, MotorCC);
    // delay(5);
    // step(X_STP_Pin, MinMotroStep * abs(delta_X), 5);
    // delay(stableDelay);

    // if (delta_Y < 0)
    //   MotorCC = false;
    // else
    //   MotorCC = true;

    // digitalWrite(Y_DIR_Pin, MotorCC);
    // delay(5);
    // step(Y_STP_Pin, MinMotroStep * abs(delta_Y), 5);
    // delay(stableDelay);

    double finalIL = Cal_PD_Input_IL(Get_PD_Points);
    Serial.println("Final IL : " + String(finalIL));

    Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));

    AutoAlign_Result[0] = PD_BestIL_Position[0];
    AutoAlign_Result[1] = PD_BestIL_Position[1];
    AutoAlign_Result[2] = finalIL;

    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));
    // Serial.print("TS:");
    // Serial.println(ts, 2);
  }
  else
  {
    Serial.println("Delta step out of range.");
  }
  //  Serial.println("X^");
  return false;
}

//------------------------------------------------------------------------------------------------------------------------------------------

bool Scan_1D_TwoWay(int XYZ, int count, int Threshold, int motorStep, int stableDelay,
                    bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg, double preIL, bool Tirp2Stop)
{

  int DIR_Pin = 0;
  int STP_Pin = 0;
  MotorCC = Direction; // direction first
  int backlash = 40;
  int reverse_Step;
  int pos;
  int trip = 1;
  long ini_position = 0;
  bool process_stop = true;
  unsigned long timer_1 = 0, timer_2 = 0;
  timer_1 = millis();

  switch (XYZ)
  {
  case 0:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    //      backlash = 50;
    backlash = X_backlash;
    delay(5);
    break;
  case 1:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    //      backlash = 80;  //60
    backlash = Y_backlash;
    delay(5);
    break;
  case 2:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    //      backlash = 100;  //100
    backlash = Z_backlash;
    delay(5);
    break;
  }
  Serial.println("Scan Step: " + String(motorStep));

  CMDOutput(">>" + msg + String(trip));
  // Serial.println(">>" + msg + String(trip)); //Trip 1

  double PD_Best = 0,
         PD_Trip1_Best = 0, PD_Now = 0;
  double PD_Value[2 * count + 1];
  long PD_Best_Pos = 0;

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points);

  if (PD_initial > PD_Best)
  {
    PD_Best = PD_initial;
    PD_Best_Pos = Get_Position(XYZ);
    ini_position = Get_Position(XYZ);
  }

  DataOutput(XYZ, PD_initial); //int xyz, double pdValue

  if (PD_initial >= StopPDValue)
    return true;

  digitalWrite(DIR_Pin, MotorCC);
  delay(10);

  if (XYZ != 2)
    step(STP_Pin, motorStep * 4, delayBetweenStep); //3
  else
    step(STP_Pin, motorStep * 4, delayBetweenStep); //3

  delay(stableDelay + 100); //Trip 1 --------------------------------------------------------------------------------------

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  if (PD_Now == PD_initial)
    PD_Now = Cal_PD_Input_IL(Get_PD_Points);                                       //Re check PD value now is correct
  DataOutput(XYZ, PD_Now);                                                         //int xyz, double pdValue
  Serial.println("Initial: " + String(PD_initial) + ", After: " + String(PD_Now)); //~~~~Jump~~~

  if (PD_Now >= StopPDValue)
    return true;

  //  if (abs(PD_initial - PD_Now) < 0.4 || abs(PD_initial - PD_Now ) < 0.4) return true;

  if (PD_Now > PD_Best)
  {
    PD_Best = PD_Now;
    PD_Best_Pos = Get_Position(XYZ);
    ini_position = Get_Position(XYZ);
  }

  for (int i = 0; i < count; i++)
  {
    PD_Value[i] = 0;
  }

  if (PD_Now >= PD_initial)
  {
    MotorCC = MotorCC;
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);
  }
  else
  {
    Serial.println("MotorCC Reverse");

    MotorCC = !MotorCC;
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);
    step(STP_Pin, motorStep * 2, delayBetweenStep);

    //    Move_Motor(DIR_Pin, STP_Pin, MotorCC, motorStep * 2, 5, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)

    if ((motorStep * 3) < backlash)
    {
      step(STP_Pin, (backlash - (motorStep * 3)), delayBetweenStep);
    }

    delay(50);
  }

  trip++;
  CMDOutput("~:" + msg + String(trip));
  // Serial.println("~" + msg + String(trip)); //Trip 2 --------------------------------------------------------------------------------------

  double IL_Best_Trip2 = PD_Now;
  long Pos_Best_Trip2 = ini_position;
  long Pos_Ini_Trip2 = Get_Position(XYZ);

  for (int i = 0; i < count; i++)
  {
    if (isStop)
    {
      Serial.println("Emergency Stop");
      return true;
    }

    if (i == 0)
    {
      PD_Value[i] = PD_Now;
      continue;
    }

    step(STP_Pin, motorStep, delayBetweenStep);

    delay(stableDelay);

    PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points + 30);

    DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

    if (PD_Value[i] >= StopPDValue)
      return true;

    if (i > 9)
    {
      bool IL_stable = true;
      for (int c = i; c > i - 12; c--)
      {
        if (abs(PD_Value[i] - PD_Value[c]) > 0.6)
          IL_stable = false;
      }

      if (IL_stable)
      {
        isILStable = true;
        Serial.println("IL Stable in Scan");
        return true;
      }
    }

    if (PD_Value[i] > preIL)
      preIL = PD_Value[i];

    if (PD_Value[i] > PD_Best)
    {
      PD_Best = PD_Value[i];
      PD_Best_Pos = Get_Position(XYZ);

      IL_Best_Trip2 = PD_Value[i];
      Pos_Best_Trip2 = PD_Best_Pos;
    }

    if (Tirp2Stop && i >= 3 && PD_Value[i] < PD_Value[i - 1] && abs(PD_Value[i - 1] - PD_Value[i - 2]) < 1 && abs(PD_Value[i - 2] - PD_Value[i - 3]) < 1)
    {
      Serial.println("Tirp2 Stop - Scan break (A)");
      break;
    }

    if (Tirp2Stop && i >= 2 && PD_Value[i] < PD_Value[i - 1])
    {
      Serial.println("Tirp2 Stop - Scan break (B)");
      break;
    }

    if (i >= 5 && PD_Value[i] < PD_Value[i - 1] && PD_Value[i - 1] <= PD_Value[i - 2] && PD_Value[i - 2] >= PD_Value[i - 3] && PD_Value[i - 3] >= PD_Value[i - 4] && PD_Value[i - 4] >= PD_Value[i - 5] && PD_Value[i] > PD_Now)
    {
      Serial.println("Scan break (A)");
      break;
    }

    if (i >= 5 && PD_Value[i] < PD_Value[i - 1] && PD_Value[i - 1] < PD_Value[i - 2] && PD_Value[i - 2] <= PD_Value[i - 3] && PD_Value[i - 3] >= PD_Value[i - 4] && PD_Value[i] > PD_Now)
    {
      Serial.println("Scan break (B)");
      break;
    }

    if (i >= 5 && PD_Value[i] < PD_Value[i - 1] && PD_Value[i - 1] < PD_Value[i - 2] && PD_Value[i - 2] < PD_Value[i - 3] && PD_Value[i - 3] <= PD_Value[i - 4] && PD_Value[i] > PD_Now)
    {
      Serial.println("Scan break (C)");
      break;
    }

    if (i >= 5 && PD_Value[i] < PD_Value[i - 1] && PD_Value[i - 1] < PD_Value[i - 2] && PD_Value[i - 2] < PD_Value[i - 3] && PD_Value[i - 3] <= PD_Value[i - 4] && PD_Value[i - 4] <= PD_Value[i - 5])
    {
      Serial.println("Scan break (D)");
      break;
    }

    if (isStop)
    {
      Serial.println("Emergency Stop");
      return true;
    }
  }

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  Serial.println(String(PD_Now) + "  ,  " + String(preIL));

  Serial.println("Pos_Best_Trip2: " + String(Pos_Best_Trip2));

  double IL_Best_Trip3 = 0;
  long Pos_Best_Trip3 = 0;

  if (PD_Now < (preIL - 0.3) && !Tirp2Stop)
  {

    trip++;
    CMDOutput("~:" + msg + String(trip));
    // Serial.println("~" + msg + String(trip)); //Trip 3 --------------------------------------------------------------------------------------

    for (int i = 0; i < count; i++)
    {
      PD_Value[i] = 0;
    }

    Serial.println("MotorCC Reverse");
    MotorCC = !MotorCC;
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);
    step(STP_Pin, backlash, delayBetweenStep);
    delay(50);

    //    Move_Motor(DIR_Pin, STP_Pin, !MotorCC, motorStep, 5, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)

    for (int i = 0; i < count; i++)
    {
      if (isStop)
      {
        Serial.println("Emergency Stop");
        return true;
      }

      step(STP_Pin, motorStep, delayBetweenStep);

      delay(stableDelay);

      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points + 30);

      DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

      if (PD_Value[i] >= StopPDValue)
        return true;

      if (PD_Value[i] > PD_Best)
      {
        PD_Best = PD_Value[i];
        PD_Best_Pos = Get_Position(XYZ);
      }

      if (PD_Value[i] > IL_Best_Trip3)
      {
        IL_Best_Trip3 = PD_Value[i];
        Pos_Best_Trip3 = Get_Position(XYZ);
      }

      if (i >= 5 && PD_Value[i] < PD_Value[i - 1] && PD_Value[i - 1] <= PD_Value[i - 2] && PD_Value[i - 2] >= PD_Value[i - 3] && PD_Value[i - 3] >= PD_Value[i - 4] && PD_Value[i - 4] >= PD_Value[i - 5])
      {
        Serial.println("Scan break (A)");
        process_stop = false;
        break;
      }

      if (i >= 4 && PD_Value[i] < PD_Value[i - 1] && PD_Value[i - 1] > PD_Value[i - 2] && PD_Value[i - 2] >= PD_Value[i - 3] && PD_Value[i - 3] >= PD_Value[i - 4] && PD_Value[i] >= IL_Best_Trip2 - 0.6)
      {
        Serial.println("Scan break (B)");
        process_stop = false;
        break;
      }

      if (i >= 5 && PD_Value[i] < PD_Value[i - 1] && PD_Value[i - 1] < PD_Value[i - 2] && PD_Value[i - 2] <= PD_Value[i - 3] && PD_Value[i - 3] <= PD_Value[i - 4] && PD_Value[i - 4] <= PD_Value[i - 5])
      {
        Serial.println("Scan break (C)");
        process_stop = false;
        break;
      }

      //      if (i >= 5
      //          && PD_Value[i] < PD_Value[i - 1]
      //          && abs(PD_Value[i - 1] - PD_Value[i - 2]) < 0.8
      //          && abs(PD_Value[i - 2] - PD_Value[i - 3]) < 0.8
      //          && abs(PD_Value[i - 3] - PD_Value[i - 4]) < 0.8
      //          && abs(PD_Value[i - 4] - PD_Value[i - 5]) < 0.8
      //         )
      //      {
      //        Serial.println("Scan break (D)");
      //        process_stop = false;
      //        break;
      //      }

      if (i >= 2 && PD_Value[i] <= PD_Value[i - 1] && abs(PD_Value[i] - PD_Value[i - 1]) <= 1.1 && abs(PD_Value[i - 1] - PD_Value[i - 2]) <= 1.1 && abs(PD_Value[i - 2] - PD_Value[i - 3]) <= 1.1 && IL_Best_Trip3 > IL_Best_Trip2)
      {
        Serial.println("Scan break (E)");
        process_stop = false;
        break;
      }

      if (i == count - 1)
        process_stop = false;

      if (isStop)
      {
        Serial.println("Emergency Stop");
        break;
      }
    }
  }

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  Serial.println("Trip2: " + String(IL_Best_Trip2) + "  , Trip3: " + String(IL_Best_Trip3) + ", Now: " + String(PD_Now));

  //if (PD_Now < (preIL - 0.3) && !process_stop)
  if ((IL_Best_Trip2 > PD_Now && IL_Best_Trip2 - PD_Now > 0.6) && !process_stop && !Tirp2Stop)
  {

    trip++;
    CMDOutput("~:" + msg + String(trip));
    // Serial.println("~" + msg + String(trip)); //Trip 4 --------------------------------------------------------------------------------------

    for (int i = 0; i < count; i++)
    {
      PD_Value[i] = 0;
    }

    //    step(STP_Pin, backlash * 1.5, delayBetweenStep);
    //    delay(stableDelay);

    Serial.println("MotorCC Reverse");
    MotorCC = !MotorCC;
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    //    step(STP_Pin, backlash * 1.5, delayBetweenStep);
    //    delay(stableDelay);

    long Pos3 = Get_Position(XYZ);
    Serial.println("Pos_Trip_2 : " + String(Pos_Best_Trip2) + ", " + "Pos_Trip_3 : " + String(Pos3));
    int deltaPos = abs(Pos_Best_Trip2 - Pos3);
    Serial.println("Detla Pos : " + String(deltaPos));

    if ((Pos_Best_Trip2 - Get_Position(XYZ) > 0) && !MotorCC)
    {
      Serial.println("Situation A:" + String(Pos_Best_Trip2) + ", " + String(Get_Position(XYZ)));

      //      Move_Motor_abs(XYZ, ini_position);
      //      Move_Motor_abs(XYZ, Pos_Best_Trip2);
      //      delay(stableDelay);
      //      PD_Now = Cal_PD_Input_IL(20);
      //      DataOutput(XYZ, PD_Now);  //int xyz, double pdValue
    }
    else if ((Pos_Best_Trip2 - Get_Position(XYZ) < 0) && MotorCC)
    {
      Serial.println("Situation B:" + String(Pos_Best_Trip2) + ", " + String(Get_Position(XYZ)));

      //      Move_Motor_abs(XYZ, ini_position);
      //      Move_Motor_abs(XYZ, Pos_Best_Trip2);
      //      delay(stableDelay);
      //      PD_Now = Cal_PD_Input_IL(20);
      //      DataOutput(XYZ, PD_Now);  //int xyz, double pdValue
    }
    else
    {
      while (true)
      {
        if (deltaPos > motorStep)
        {
          deltaPos = deltaPos - motorStep;
          step(STP_Pin, motorStep, delayBetweenStep);
          delay(stableDelay);
          PD_Now = Cal_PD_Input_IL(6);
          DataOutput(XYZ, PD_Now); //int xyz, double pdValue

          if (PD_Now >= StopPDValue)
            break;

          if (PD_Now >= PD_Best)
            break;
        }
        else
        {
          step(STP_Pin, deltaPos, delayBetweenStep);
          delay(stableDelay);
          PD_Now = Cal_PD_Input_IL(6);
          DataOutput(XYZ, PD_Now); //int xyz, double pdValue
          break;
        }
      }
    }
  }

  timer_2 = millis();
  double ts = (timer_2 - timer_1) * 0.001;
  CMDOutput("t:" + String(ts, 2));
  // Serial.print("TS:");
  // Serial.println(ts, 2);

  PD_Now = Cal_PD_Input_IL(6);
  if (PD_Now < PD_Best - 2.5)
    return false;

  return true;

  //  if (process_stop)
  //    return false;
  //  else return true;
}

//------------------------------------------------------------------------------------------------------------------------------------------

bool Scan_AllRange_TwoWay(int XYZ, int count, int motorStep, int stableDelay,
                          bool Direction, int delayBetweenStep, int StopPDValue, int Get_PD_Points, int Trips, String msg)
{
  int DIR_Pin = 0;
  int STP_Pin = 0;
  int backlash = 40;
  MotorCC = Direction; // initial direction
  int trip = 1;
  int dataCount = 3;
  int dataCount_ori;
  int indexofBestIL = 0;
  double PD_Value[4 * count + 1];
  long Step_Value[4 * count + 1];
  unsigned long timer_1 = 0, timer_2 = 0;
  // delayBetweenStep = stableDelay;
  timer_1 = millis();

  dataCount = 2 * count + 1;
  dataCount_ori = dataCount;

  switch (XYZ)
  {
  case 0:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = X_backlash;
    delay(5);
    break;
  case 1:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = Y_backlash;
    delay(5);
    break;
  case 2:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = Z_backlash;
    delay(5);
    break;
  }
  Serial.println("Scan Step: " + String(motorStep));
  CMDOutput(">>" + msg + String(trip));
  // Serial.println(">>" + msg + String(trip)); //Trip 1

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points);
  if (PD_initial >= StopPDValue)
    return true;

  digitalWrite(DIR_Pin, MotorCC);
  delay(1);
  step(STP_Pin, motorStep * count, delayBetweenStep); //Jump to trip 1 initial position----------------

  // Serial.println(String(STP_Pin) + "," + String(motorStep) + "," + String(count) + "," + String(delayBetweenStep));
  // return true;

  MotorCC = !MotorCC; //Reverse direction
  digitalWrite(DIR_Pin, MotorCC);
  delay(1);

  delay(stableDelay + 100); //Trip 1 --------------------------------------------------------------------------------------

  CMDOutput(">>" + msg + String(trip));
  // Serial.println(">>" + msg + String(trip)); //Trip 1

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  DataOutput();
  DataOutput(XYZ, PD_Now); //int xyz, double pdValue

  if (PD_Now >= StopPDValue)
    return true;

  for (int i = 0; i < dataCount; i++)
  {
    PD_Value[i] = 0;
    Step_Value[i] = 0;
  }

  double IL_Best_Trip1 = PD_Now;
  long Pos_Best_Trip1 = Get_Position(XYZ);
  long Pos_Ini_Trip1 = Get_Position(XYZ);

  int data_plus_time = 0;

  for (int i = 0; i < dataCount; i++)
  {
    if (isStop) return true;

    if (i == 0)
    {
      PD_Value[i] = PD_Now;
      Step_Value[i] = Get_Position(XYZ);
      continue;
    }

    step(STP_Pin, motorStep, delayBetweenStep);
    delay(stableDelay);

    PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);
    Step_Value[i] = Get_Position(XYZ);

    if (PD_Value[i] > IL_Best_Trip1)
    {
      indexofBestIL = i;
      IL_Best_Trip1 = PD_Value[i];
      Pos_Best_Trip1 = Get_Position(XYZ);
    }

    DataOutput();
    DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

    if (PD_Value[i] >= StopPDValue)
      return true;

    if (i == (dataCount - 1) && Pos_Best_Trip1 == Get_Position(XYZ))
    {
      Serial.println("Datacount+3");
      dataCount = dataCount + 3;
      data_plus_time = data_plus_time + 1;

      if (dataCount - dataCount_ori > 20 || data_plus_time > 3)
      {
        Serial.println("Data plus time: " + String(data_plus_time));
        timer_2 = millis();
        double ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));
        // Serial.print("TS:");
        // Serial.println(ts, 2);

        return true;
      }
    }

    else if (indexofBestIL != 0 && i == (dataCount - 1) && Pos_Best_Trip1 != Get_Position(XYZ))
    {
      Serial.println("i:" + String(i) + ", Pos_Best_Trip1:" + String(Pos_Best_Trip1));
      double x[3];
      double y[3];
      for (int k = -1; k < 2; k++)
      {
        x[k + 1] = Step_Value[indexofBestIL + k]; //idex * step = real steps
        y[k + 1] = PD_Value[indexofBestIL + k];   //fill this with your sensor data
        Serial.println("Point : " + String(x[k + 1]) + " , " + String(y[k + 1]));
      }
      Pos_Best_Trip1 = Curfit(x, y, 3);
      Serial.println("Best IL position in trip 1 is: " + String(Pos_Best_Trip1));
    }
  }

  trip++;

  double IL_Best_Trip2 = 0;
  long Pos_Best_Trip2 = 0;
  long Pos_Ini_Trip2 = 0;

  if (trip <= Trips)
  {
    CMDOutput("~:" + msg + String(trip));
    // Serial.println("~" + msg + String(trip)); //Trip 2 --------------------------------------------------------------------------------------

    IL_Best_Trip2 = PD_Now;
    Pos_Best_Trip2 = Get_Position(XYZ);
    Pos_Ini_Trip2 = Get_Position(XYZ);

    for (int i = 0; i < dataCount; i++)
    {
      PD_Value[i] = 0;
      Step_Value[i] = 0;
    }

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    for (int i = 0; i < dataCount; i++)
    {
      if (isStop)
      {
        Serial.println("Emergency Stop");
        return true;
      }

      if (i == 0)
      {
        PD_Value[i] = PD_Now;
        Step_Value[i] = Get_Position(XYZ);
        continue;
      }

      step(STP_Pin, motorStep, delayBetweenStep);
      delay(stableDelay);

      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);
      Step_Value[i] = Get_Position(XYZ);

      if (PD_Value[i] > IL_Best_Trip2)
      {
        indexofBestIL = i;
        IL_Best_Trip2 = PD_Value[i];
        Pos_Best_Trip2 = Get_Position(XYZ);
      }

      DataOutput();
      DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

      if (PD_Value[i] >= StopPDValue)
        return true;

      if (indexofBestIL != 0 && i == (dataCount - 1) && Pos_Best_Trip2 != Get_Position(XYZ))
      {
        double x[3];
        double y[3];
        for (int k = -1; k < 2; k++)
        {
          x[k + 1] = Step_Value[indexofBestIL + k]; //idex * step = real steps
          y[k + 1] = PD_Value[indexofBestIL + k];   //fill this with your sensor data
          Serial.println("Point : " + String(x[k + 1]) + " , " + String(y[k + 1]));
        }
        Pos_Best_Trip2 = Curfit(x, y, 3);
        Serial.println("Best IL position in trip 2 is: " + String(Pos_Best_Trip2));
      }
    }
  }
  else
    trip--;

  trip++;
  CMDOutput("~:" + msg + String(trip));
  // Serial.println("~" + msg + String(trip)); //Trip 3 --------------------------------------------------------------------------------------

  double PD_Best = 0;
  int deltaPos = 0;

  if (IL_Best_Trip2 > IL_Best_Trip1)
  {
    if (isStop)
    {
      return true;
    }

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    Serial.println("Best in Trip 2 : " + String(Pos_Best_Trip2));

    PD_Best = IL_Best_Trip2;

    Move_Motor_abs(XYZ, Pos_Ini_Trip2); //Jump to trip 2 start position

    delay(300);

    deltaPos = abs(Pos_Best_Trip2 - Get_Position(XYZ));

    if (deltaPos < backlash)
    {
      Serial.println("Jump Backlesh");
      step(STP_Pin, (backlash * 1 - deltaPos), delayBetweenStep);
      delay(stableDelay);

      deltaPos = abs(Pos_Best_Trip2 - Get_Position(XYZ));
    }

    delay(300);

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    DataOutput();
    DataOutput(XYZ, PD_Now); //int xyz, double pdValue

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);
  }
  else
  {
    Serial.println("Best in Trip 1 : " + String(Pos_Best_Trip1));

    PD_Best = IL_Best_Trip1;
    deltaPos = abs(Pos_Best_Trip1 - Get_Position(XYZ));

    if (deltaPos < backlash)
    {
      Serial.println("Jump Backlesh");
      //      step(STP_Pin, (backlash * 1 - deltaPos), delayBetweenStep);
      step(STP_Pin, motorStep, delayBetweenStep);
      delay(150);

      deltaPos = abs(Pos_Best_Trip1 - Get_Position(XYZ));
    }

    if (isStop)
    {
      return true;
    }

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(1);
  }

  Serial.println("Delta Pos : " + String(deltaPos));

  while (true)
  {
    if (isStop)
    {
      return true;
    }

    if (deltaPos >= motorStep)
    {
      deltaPos = deltaPos - motorStep;
      step(STP_Pin, motorStep, delayBetweenStep);
      delay(stableDelay);
      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      DataOutput();
      DataOutput(XYZ, PD_Now); //int xyz, double pdValue

      if (PD_Now >= StopPDValue)
      {
        Serial.println("StopPDValue");
        break;
      }
      if (PD_Now >= PD_Best)
      {
        Serial.println("PD_Best");
        break;
      }
    }
    else if (deltaPos > 0)
    {
      step(STP_Pin, deltaPos, delayBetweenStep);
      delay(stableDelay);
      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      DataOutput();
      DataOutput(XYZ, PD_Now); //int xyz, double pdValue

      break;
    }
    else
      break;
  }

  // int descent_count = 0;
  // double pdV = Cal_PD_Input_IL(Get_PD_Points);
  // if (PD_Now < PD_Best - 1.5)
  // {
  //   Serial.println("Incorrect Position, IL is: " + String(PD_Now));
  //   for (int s = 0; s < 8; s++)
  //   {
  //     step(STP_Pin, motorStep, delayBetweenStep);
  //     delay(stableDelay);

  //     pdV = Cal_PD_Input_IL(Get_PD_Points);
  //     if (abs(-PD_Now) <= 1 && s != 0)
  //     {
  //       Serial.println("Break A");
  //       break;
  //     }

  //     if (pdV < PD_Now - 2)
  //     {
  //       Serial.println("Break B");
  //       break;
  //     }

  //     if (pdV < PD_Now)
  //       descent_count++;

  //     PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  //     DataOutput(XYZ, PD_Now); //int xyz, double pdValue

  //     if (abs(PD_Best - PD_Now) < 1.2)
  //     {
  //       Serial.println("Break C");
  //       break;
  //     }

  //     if (descent_count >= 3)
  //     {
  //       Serial.println("Break D");
  //       break;
  //     }
  //   }
  // }

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  Serial.println("Best IL: " + String(PD_Best));
  Serial.println("Final IL: " + String(PD_Now));
  timer_2 = millis();
  double ts = (timer_2 - timer_1) * 0.001;
  CMDOutput("t:" + String(ts, 2));
  // Serial.print("TS:");
  // Serial.println(ts, 2);

  return true;
}

//------------------------------------------------------------------------------------------------------------------------------------------

void BackLash_Reverse(int XYZ, bool dir, int stableDelay)
{
  int backlash = 40;
  int DIR_Pin = 0;
  int STP_Pin = 0;
  double Modify_Ratio = 1.3;

  switch (XYZ)
  {
  case 0:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = 60;    //default: 95
    Modify_Ratio = 1; //default:1.6
    delay(stableDelay);
    break;
  case 1:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = 40;    //default: 85
    Modify_Ratio = 1; //default:1.3
    delay(stableDelay);
    break;
  case 2:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = 500; //default: 1100
    Modify_Ratio = 1.5;
    delay(stableDelay);
    break;
  }

  MotorCC = dir;
  digitalWrite(DIR_Pin, MotorCC);
  delay(5);

  step(STP_Pin, (backlash + 20), delayBetweenStep); //Backlash about 40 pulse
  delay(stableDelay * 2);

  //Reverse
  MotorCC = !MotorCC;
  digitalWrite(DIR_Pin, MotorCC);
  delay(10);
  step(STP_Pin, (backlash * Modify_Ratio + 20), delayBetweenStep); //Backlash about 40 pulse
  delay(stableDelay * 2);

  switch (XYZ)
  {
  case 0:
    MotorCC_X = MotorCC;
    break;
  case 1:
    MotorCC_Y = MotorCC;
    break;
  case 2:
    MotorCC_Z = MotorCC;
    break;
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

long Curfit(double x1[], double y1[], int dataCount)
{
  Serial.println("Curfit Test");
  char buf[4];
  int xpower = 0;
  int order = 2;
  snprintf(buf, 4, "Fitting curve of order %i to data of power %i...\n", order, xpower);
  Serial.println(buf);

  double x[dataCount]; //idex * step = real steps
  double y[dataCount]; //fill this with your sensor data

  double center_x = x1[0];
  Serial.println(String(center_x));
  for (int i = 0; i < 3; i++)
  {
    x[i] = x1[i] - center_x;
    y[i] = y1[i];
    // Serial.println("X:" + String(x[i]));
  }

  int step_distance = abs(x[1] - x[0]);

  double coeffs[order + 1];

  int ret = fitCurve(order, sizeof(y) / sizeof(double), x, y, sizeof(coeffs) / sizeof(double), coeffs);

  if (ret == 0)
  { //Returned value is 0 if no error
    uint8_t c = 'a';
    // Serial.println("Coefficients are");

    // for (int i = 0; i < sizeof(coeffs) / sizeof(double); i++)
    // {
    //   snprintf(buf, 100, "%c=", c++);
    // Serial.print(buf);
    // Serial.print(coeffs[i]);
    // Serial.print('\t');
    // }
    // Serial.println("");

    long result_x = (-1 * coeffs[1]) / (2 * coeffs[0]);
    // Serial.println("Curfit X is : " + String(result_x));

    if (step_distance > abs(x[1] - result_x))
      return result_x + center_x;
    else
    {
      result_x = x1[1];
      // Serial.println("Final X is : " + String(result_x));
      return result_x;
    }
  }
  else
  {
    return x1[1];
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------

double AutoAlign_Scan_DirectionJudge_V2(int XYZ, int count, int Threshold, int motorStep, int stableDelay,
                                        bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg)
{

  int DIR_Pin = 0;
  int STP_Pin = 0;
  MotorCC = Direction; // direction first
  int backlash = 40, trip = 1;
  bool isReverse = false;
  double ts = 0;
  unsigned long timer_1 = 0, timer_2 = 0;
  timer_1 = millis();

  switch (XYZ)
  {
  case 0:
    DIR_Pin = X_DIR_Pin;
    STP_Pin = X_STP_Pin;
    backlash = 40;
    delay(stableDelay);
    break;
  case 1:
    DIR_Pin = Y_DIR_Pin;
    STP_Pin = Y_STP_Pin;
    backlash = 80;
    delay(stableDelay);
    break;
  case 2:
    DIR_Pin = Z_DIR_Pin;
    STP_Pin = Z_STP_Pin;
    backlash = 40;
    delay(stableDelay);
    break;
  }

  CMDOutput(">>" + msg + String(trip));
  // Serial.println(">>" + msg + String(trip)); //Trip 1--------------------------------------------------------------------------

  // if (motorStep <= backlash)
  // {
  //   Serial.println("Backlash travel");
  //   BackLash_Reverse(XYZ, Direction, stableDelay);
  // }

  double PD_Best = -64,
         PD_Now = -64;
  double PD_Value[count];
  double PD_Rvrs_Value[count];
  int PD_Best_Pos = 0;

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points);
  DataOutput(XYZ, PD_initial); //int xyz, double pdValue

  if (PD_initial >= StopPDValue)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));
    // Serial.print("TS:");
    // Serial.println(ts, 2);

    PD_Best = PD_initial;

    return PD_Best;
  }

  Move_Motor(DIR_Pin, STP_Pin, MotorCC, motorStep * 4, delayBetweenStep, 0, true);
  // digitalWrite(DIR_Pin, MotorCC);
  // delay(5);
  // step(STP_Pin, motorStep * 2, delayBetweenStep);

  delay(100);
  // delay(stableDelay);

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  DataOutput(XYZ, PD_Now); //int xyz, double pdValue

  Serial.println("Initial: " + String(PD_initial) + ", After:" + String(PD_Now));

  if (PD_Now >= StopPDValue)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));
    // Serial.print("TS:");
    // Serial.println(ts, 2);

    PD_Best = PD_Now;

    return PD_Best;
  }

  PD_Best_Pos = 0;

  for (int i = 0; i < count; i++)
  {
    PD_Value[i] = 0;
  }

  if (abs(PD_Now - PD_initial) < 0.1)
  {
    Serial.println("Delta IL < 0.1");

    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));
    // Serial.print("TS:");
    // Serial.println(ts, 2);

    return true;
  }

  bool isFirstPointPassBest = false;

  if (PD_Now >= PD_initial)
  {
    PD_Best = PD_Now;
    digitalWrite(DIR_Pin, MotorCC);
    delay(10);
    Serial.println("MotorCC Forward");

    PD_Value[0] = Cal_PD_Input_IL(Get_PD_Points);
  }
  else
  {
    //    MotorCC = !MotorCC;
    PD_Best = PD_initial;
    BackLash_Reverse(XYZ, MotorCC, stableDelay);
    Serial.println("MotorCC Reverse");
    isReverse = true;
  }

  trip++;
  CMDOutput("~:" + msg + String(trip));
  // Serial.println("~" + msg + String(trip)); //Trip 2------------------------------------------------------------

  PD_Value[0] = Cal_PD_Input_IL(Get_PD_Points);

  if (!isFirstPointPassBest)
  {
    for (int i = 0; i < count; i++)
    {
      if (isStop)
        return true;

      step(STP_Pin, motorStep, delayBetweenStep);
      delay(stableDelay);

      PD_Value[i] = Cal_PD_Input_IL(Get_PD_Points);
      DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

      if (PD_Value[i] >= StopPDValue)
      {
        Serial.println("Over StopPDValue");

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));
        // Serial.print("TS:");
        // Serial.println(ts, 2);

        Serial.println("");

        PD_Best = PD_Value[i];
        return PD_Best;
      }

      if (!isReverse && i == 0 && PD_Value[0] < PD_Best)
      {
        Serial.println("Forward pass best");
        return PD_Best;
      }

      if (PD_Value[i] > PD_Best)
      {
        PD_Best = PD_Value[i];
        PD_Best_Pos = i;
      }

      if (i >= 1 && PD_Value[i] < PD_Value[i - 1])
      {
        if (abs(PD_Value[i - 1] - PD_Best) < 0.8)
        {
          Serial.println("Pass best IL");
          break;
        }
      }

      if (PD_Value[i] < -53)
      {
        Serial.println("Miss Target");
        break;
      }
    }
  }

  delay(stableDelay);

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  if (abs(PD_Now - PD_Best) < 0.4)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));
    // Serial.print("TS:");
    // Serial.println(ts, 2);

    return PD_Best;
  }

  return PD_Best;

  //Back to best position
  BackLash_Reverse(XYZ, MotorCC, stableDelay);
  Serial.println("Final Reverse");

  trip++;
  CMDOutput("~:" + msg + String(trip));
  // Serial.println("~" + msg + String(trip)); //Trip 3

  double pv = 0;
  double bv = 0;
  for (int k = 0; k < 35; k++)
  {
    if (isStop)
      return true;

    step(STP_Pin, motorStep / 4, delayBetweenStep);
    delay(stableDelay);

    pv = Cal_PD_Input_IL(Get_PD_Points);
    DataOutput(XYZ, pv); //int xyz, double pdValue

    Serial.println("Reverse PD: " + String(pv));

    if (pv >= StopPDValue)
    {
      Serial.println("Over StopPDValue");

      timer_2 = millis();
      ts = (timer_2 - timer_1) * 0.001;
      CMDOutput("t:" + String(ts, 2));
      // Serial.print("TS:");
      // Serial.println(ts, 2);

      PD_Best = pv;

      return PD_Best;
    }

    if (abs(pv - PD_Best) < 3 || pv > PD_Best || (pv < bv && pv > Threshold))
      break;
    else
      bv = pv;
  }

  timer_2 = millis();
  ts = (timer_2 - timer_1) * 0.001;
  Serial.print("TS:");
  Serial.println(ts, 2);
  Serial.println(" ");
  return PD_Best;
}

//------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------

void CMDOutput(String cmd)
{
  Serial.println("CMD::" + cmd);
}

void DataOutput()
{
  double IL = Cal_PD_Input_IL(1);
  Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now) + ", " + String(IL));
}

void DataOutput(bool isIL)
{
  if (isIL)
  {
    double IL = Cal_PD_Input_IL(1);
    Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now) + ", " + String(IL));
  }
  else
    Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
}

void DataOutput(int xyz, double pdValue)
{
  switch (xyz)
  {
  case 0:
    CMDOutput(">:" + String(X_Pos_Now) + "," + String(pdValue));
    break;

  case 1:
    CMDOutput(">:" + String(Y_Pos_Now) + "," + String(pdValue));
    break;

  case 2:
    CMDOutput(">:" + String(Z_Pos_Now) + "," + String(pdValue));
    break;
  }
}

long Get_Position(int xyz)
{
  switch (xyz)
  {
  case 0:
    return X_Pos_Now;
    break;

  case 1:
    return Y_Pos_Now;
    break;

  case 2:
    return Z_Pos_Now;
    break;
  }
}
