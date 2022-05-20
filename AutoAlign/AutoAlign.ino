#include <EEPROM.h>
// #include <avr/wdt.h> //使用看門狗計時器的含括檔
// #include <esp_task_wdt.h>
// #include <FLASHLED.h>
// #include <ArduinoSort.h>
#include <curveFitting.h>
// #include <U8glib.h>
#include <U8g2lib.h>
// #include <BluetoothSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>

#include <ESPAsyncWebServer.h>

TaskHandle_t Task_1;
#define WDT_TIMEOUT 3

// Set your access point network credentials
const char *ssid = "ESP32-Access-Point";
const char *password = "123456789";

// String server_ID = "";
// String server_Password = "22101782";

const char *serverNameData = "http://192.168.4.1/Data";

bool isWiFiConnected = false;

String ID = "003";
String Station_ID = "A00";

// Create AsyncWebServer object on port 80
// AsyncWebServer server(80);

const byte X_STP_Pin = 15; //x軸 步進控制
const byte X_DIR_Pin = 2;  //X軸 步進馬達方向控制
const byte Y_STP_Pin = 0;  //y軸 步進控制
const byte Y_DIR_Pin = 4;  //y軸 步進馬達方向控/.//制
const byte Z_STP_Pin = 16; //z軸 步進控制
const byte Z_DIR_Pin = 17; //z軸 步進馬達方向控制

int ButtonSelected = 0;

U8G2_ST7920_128X64_F_SW_SPI lcd(U8G2_R0, 5, 18, 19, U8X8_PIN_NONE); //data 4 , en, rs

int LCD_Encoder_A_pin = 22; //22
int LCD_Encoder_B_pin = 23; //23
uint8_t LCD_Select_pin = 21;

bool LCD_Encoder_State = false;
bool LCD_Encoder_LastState = false;
int LCD_en_count = 0, idx = 0;
int LCD_sub_count = 0, idx_sub = 0;
int current_selection = 0;

int LCD_Update_Mode = 0;
uint8_t LCD_PageNow = 1;

const byte R_0 = 12;

/* Keyboard Pin Setting */
const byte R_1 = 14;
const byte R_2 = 27;
const byte R_3 = 26;
const byte C_1 = 25;
const byte C_2 = 33;
const byte C_3 = 32;

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

// int X_rotator_steps = 2;
// int Y_rotator_steps = 2;
// int Z_rotator_steps = 20;

int X_backlash = 0;
int Y_backlash = 0;
int Z_backlash = 0;

int X_ScanSTP = 12;
int Y_ScanSTP = 10;
int Z_ScanSTP = 200;

int X_ScanStable = 25;
int Y_ScanStable = 50;
int Z_ScanStable = 80;

//Intention _ Region _ MotionType _ ParaType _ Axis _ Rank = Value
int AA_SpiralRough_Feed_Steps_Z_A = 25000;
int AA_SpiralRough_Spiral_Steps_XY_A = 2000;
int AA_SpiralFine_Spiral_Steps_XY_A = 1500;
int AA_SpiralFine_Scan_Steps_X_A = 25;
int AA_SpiralFine_Scan_Steps_X_B = 30;
int AA_SpiralFine_Scan_Steps_X_C = 40;
int AA_SpiralFine_Scan_Steps_X_D = 50;
int AA_SpiralFine_Scan_Steps_Y_A = 30;
int AA_SpiralFine_Scan_Steps_Y_B = 40;
int AA_SpiralFine_Scan_Steps_Y_C = 60;
int AA_SpiralFine_Scan_Steps_Y_D = 80;
int AA_SpiralFine_Scan_Steps_Y_E = 140;
int AA_ScanRough_Feed_Steps_Z_A = 10000;
int AA_ScanRough_Feed_Steps_Z_B = 1000;
double AA_ScanRough_Feed_Ratio_Z_A = 2.8;
double AA_ScanRough_Feed_Ratio_Z_B = 2.5;
double AA_ScanRough_Feed_Ratio_Z_C = 2.0;
double AA_ScanRough_Feed_Ratio_Z_D = 1.5;
int AA_ScanRough_Scan_Steps_Y_A = 25;
int AA_ScanRough_Scan_Steps_Y_B = 30;
int AA_ScanRough_Scan_Steps_Y_C = 40;
int AA_ScanRough_Scan_Steps_Y_D = 70;
int AA_ScanRough_Scan_Steps_X_A = 25;
int AA_ScanRough_Scan_Steps_X_B = 30;
int AA_ScanRough_Scan_Steps_X_C = 80;
int AA_ScanRough_Scan_Steps_X_D = 100;
int AA_ScanRough_Scan_Steps_X_E = 120;
int AA_ScanFine_Scan_Steps_Z_A = 200;
int AA_ScanFine_Scan_Steps_Y_A = 20;
int AA_ScanFine_Scan_Steps_X_A = 20;
int AA_ScanFinal_Scan_Steps_Z_A = 125;
int AA_ScanFinal_Scan_Steps_Y_A = 20;
int AA_ScanFinal_Scan_Steps_X_A = 20;

int AQ_Scan_Compensation_Steps_Z_A = 12;
int AQ_Scan_Steps_Z_A = 40;  //125, 30
int AQ_Scan_Steps_Z_B = 40;  //120, 30
int AQ_Scan_Steps_Z_C = 45;   //70, 35
int AQ_Scan_Steps_Z_D = 50;   //50, 50

int AA_ScanFinal_Scan_Delay_X_A = 100;
int AA_ScanFinal_Scan_Delay_Y_A = 60;

int AQ_Total_TimeSpan = 840;

uint16_t FS_Count_X = 7;
uint16_t FS_Steps_X = 25;
uint16_t FS_Stable_X = 0;
uint16_t FS_DelaySteps_X = 50;
uint16_t FS_Avg_X = 600;
uint16_t FS_Count_Y = 8;
uint16_t FS_Steps_Y = 20;
uint16_t FS_Stable_Y = 0;
uint16_t FS_DelaySteps_Y = 120;
uint16_t FS_Avg_Y = 600;
uint16_t FS_Count_Z = 7;
uint16_t FS_Steps_Z = 80;
uint16_t FS_Stable_Z = 0;
uint16_t FS_DelaySteps_Z = 80;
uint16_t FS_Avg_Z = 800;

uint16_t EP_PD_Ref = 0;
uint16_t EP_Board_ID = 8;
uint16_t EP_Station_ID = 16;
uint16_t EP_X_backlash = 24;
uint16_t EP_Y_backlash = 32;
uint16_t EP_Z_backlash = 40;
uint16_t EP_delayBetweenStep_X = 48;
uint16_t EP_delayBetweenStep_Y = 56;
uint16_t EP_delayBetweenStep_Z = 64;
uint16_t EP_Target_IL = 72;
uint16_t EP_AA_ScanFinal_Scan_Delay_X_A = 80;
uint16_t EP_Server_ID = 88;
uint16_t EP_Server_Password = 128;
uint16_t EP_AQ_Scan_Compensation_Steps_Z_A = 160;
uint16_t EP_AQ_Total_TimeSpan = 168;
uint16_t EP_AQ_Scan_Steps_Z_A = 176;
uint16_t EP_AQ_Scan_Steps_Z_B = 184;
uint16_t EP_AQ_Scan_Steps_Z_C = 192;
uint16_t EP_AQ_Scan_Steps_Z_D = 200;
uint16_t EP_FS_Count_X = 240;
uint16_t EP_FS_Steps_X = 248;
uint16_t EP_FS_Stable_X = 256;
uint16_t EP_FS_DelaySteps_X = 264;
uint16_t EP_FS_Avg_X = 272;
uint16_t EP_FS_Count_Y = 280;
uint16_t EP_FS_Steps_Y = 288;
uint16_t EP_FS_Stable_Y = 296;
uint16_t EP_FS_DelaySteps_Y = 304;
uint16_t EP_FS_Avg_Y = 312;
uint16_t EP_FS_Count_Z = 320;
uint16_t EP_FS_Steps_Z = 328;
uint16_t EP_FS_Stable_Z = 336;
uint16_t EP_FS_DelaySteps_Z = 344;
uint16_t EP_FS_Avg_Z = 352;


double averagePDInput = 0;

double ref_Dac = 0; //PD reference
double ref_IL = 0;  //PD reference

double PDValue_Best = 0;
double AutoCuring_Best_IL = 0, PD_Now = 0, PD_Before = 0;

unsigned long time_curing_0, time_curing_1, time_curing_2, time_curing_3;
unsigned long timer_Get_IL_1 = 0, timer_Get_IL_2;

bool btn_isTrigger = false;
int Threshold;
int stableDelay = 0;
bool key_ctrl = false;

double Motor_Unit_Idx = 0.01953125; /* (1/51.2) um/pulse */

int Get_PD_Points = 500;
double Target_IL = 0; //0 dB
double StopValue = 0; //0 dB
int cmd_No = 0;

bool isStop = false, isGetPower = true, isILStable = false;
bool sprial_JumpToBest = true;
int Q_State = 0;
unsigned long Q_Time = 0;
unsigned long LCD_Auto_Update_TimeCount = 0;
byte GetPower_Mode = 1;
bool is_Scan_V2_ReWork = false;

// BluetoothSerial BT; //宣告藍芽物件，名稱為BT

bool isWatchDog_Flag = false;
bool isLCD = true;
bool isLCD_Auto_Update = false;

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

    if (isLCD_Auto_Update)
      if (millis() - LCD_Auto_Update_TimeCount > 5000)
      {
        LCD_Auto_Update_TimeCount = millis();
        isLCD = true;
      }

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

    delay(2);
  }

  if (keyValueSum != 0)
  {

    switch (keyValueSum)
    {
    case 1:
      keyNo = 101; /* Z- */
      break;
    case 2:
      keyNo = 102; /* X+ */
      break;
    case 3:
      keyNo = 103; /* Z+ */
      break;
    case 6:
      keyNo = 104; /* Y+ */
      break;
    case 7:
      keyNo = 105; /* X- */
      break;
    case 8:
      keyNo = 106; /* Y- */
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
  }

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
    PDAvgInput += analogRead(PD_Pin);
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

  // int* pdArray = 0;
  // pdArray = new int[averageCount];

  double averagePDInput = 0;
  double PDAvgInput = 0;

  // for (int i = 0; i < averageCount; i++)
  // {
  //   pdArray[i] = analogRead(PD_Pin);
  // }

  // sortArray(pdArray, averageCount);

  // for (int i = averageCount/4; i < 3/4*averageCount; i++)
  // {
  //   PDAvgInput += pdArray[i];
  // }

  for (int i = 0; i < averageCount; i++)
  {
    PDAvgInput += analogRead(PD_Pin);
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
    PDAvgInput += analogRead(PD_Pin);
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
    PDAvgInput += analogRead(PD_Pin);
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
  String EEPROM_String = "";
  for (int i = 0; i < data_length; i++)
  {
    uint8_t a = EEPROM.read(i + start_position);
    if (a != 255)
      EEPROM_String += char(EEPROM.read(i + start_position));
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

String WR_EEPROM(int start_position, int data_length, String data)
{
  CleanEEPROM(start_position, data_length); //Clean EEPROM(int startPosition, int datalength)

  WriteInfoEEPROM(String(data), start_position); //Write Data to EEPROM (data, start_position)
  EEPROM.commit();

  String s = ReadInfoEEPROM(start_position, data_length);
  return s;
}

bool Contains(String text, String search)
{
  if (text.indexOf(search) == -1)
    return false;
  else
    return true;
}


#define ITEMS_COUNT 100
char *UI_Items[ITEMS_COUNT] =
    {" "};

#define MENU_ITEMS 6
char *UI_Menu_Items[MENU_ITEMS] =
    {"1. Status",
     "2. Target IL",
     "3. StableDelay",
     "4. Q Z-offset",
     "5. Speed",
     "6. Get Ref"};

#define Speed_Page_ITEMS 4
char *UI_Speed_Page_Items[MENU_ITEMS] =
    {"1. X Speed",
     "2. Y Speed",
     "3. Z Speed",
     "<<"};

uint8_t i, h, w, title_h, H;

int PageLevel = 0;
int PageItemsCount = 1;

int Top_Item_Index = 0;
int Bottom_Item_Index = 3;
bool ui_YesNo_Selection = false;

int mainpageIndex = 0;

int subpageIndex = 0;
int subpage_itemsCount = 1;
bool item_is_selected = false;
bool plus_minus = false;

//Full Page method
void updateUI(int pageIndex)
{
  if (isLCD)
  {
    if (pageIndex > MENU_ITEMS - 1)
      pageIndex = MENU_ITEMS - 1;

    lcd.clearBuffer();

    H = lcd.getHeight();
    h = lcd.getFontAscent() - lcd.getFontDescent() + 2;
    w = lcd.getWidth();
    title_h = h + 2;
    lcd.drawBox(0, h + 1, w, 1); //Seperate Line

    // Serial.println("LCD_Now:" + String(LCD_PageNow) + ",Index:" + String(pageIndex));

    if (PageLevel == 0) //Main Page (Menu)
    {
      if (true)
      {
        PageItemsCount = MENU_ITEMS;

        int title_w = (w / 2) - (lcd.getStrWidth("Menu") / 2);
        lcd.drawStr(title_w, h - 1, "Menu"); //Draw Title

        int deltaIndex = 0;
        if (pageIndex > Bottom_Item_Index)
        {
          deltaIndex = abs(pageIndex - Bottom_Item_Index);
          Top_Item_Index = Top_Item_Index + deltaIndex;
          Bottom_Item_Index = Top_Item_Index + 3;
        }
        else if (pageIndex < Top_Item_Index)
        {
          deltaIndex = abs(Top_Item_Index - pageIndex);
          Top_Item_Index = Top_Item_Index - deltaIndex;
          Bottom_Item_Index = Top_Item_Index + 3;
        }

        //Draw each item in UI_Menu_Items
        Draw_ALL_UI_Items(LCD_Update_Mode, pageIndex);

        if (pageIndex < PageItemsCount - 1)
          lcd.drawTriangle(w / 2 - 2, H - 3, w / 2 + 3, H - 3, w / 2, H); //Draw Triangle

        mainpageIndex = pageIndex;

        // lcd.sendBuffer();
      }
    }

    else if (PageLevel == 1)
    {
      if (mainpageIndex == 4)
      {
        Draw_ALL_UI_Items(LCD_Update_Mode, pageIndex);
      }
      else if (mainpageIndex == 5)  //Get Ref
      {
        int H = lcd.getHeight();

        int title_w = (w / 2) - (lcd.getStrWidth("Get Ref ?") / 2);
        lcd.drawStr(title_w, H / 2 - (h / 2), "Get Ref ?");

        lcd.drawBox(0, H / 2 - 1.8 * h, w, 1); //Seperate Line
        lcd.drawBox(0, H / 2 + 1.8 * h, w, 1); //Seperate Line

        int location_X_Yes = (w / 2) - 7 - lcd.getStrWidth("Yes");
        int location_X_No = (w / 2) + 7;
        int location_Y = H / 2 + 0.9 * h;

        lcd.drawStr(location_X_Yes, location_Y, "Yes");
        lcd.drawStr(location_X_No, location_Y, "No");

        // Serial.println("YES NO :" + String(ui_YesNo_Selection));
        // Serial.println("PageLevel:" + String(PageLevel) + ", mainpageIndex:" + String(mainpageIndex));

        //Draw Selection box
        if (ui_YesNo_Selection)
          lcd.drawFrame(location_X_Yes - 2, location_Y - h + 1, lcd.getStrWidth("Yes") + 4, h + 2);
        else
          lcd.drawFrame(location_X_No - 2, location_Y - h + 1, lcd.getStrWidth("No") + 4, h + 2);

        LCD_PageNow = LCD_Update_Mode;

        // lcd.sendBuffer();
      }
    }
    
    else if (PageLevel == 101 || PageLevel == 102) /* Auto-Aligning */
    {
      lcd.clearBuffer();

      h = lcd.getFontAscent() - lcd.getFontDescent() + 2;
      w = lcd.getWidth();

      int H = lcd.getHeight();

      int title_w = (w / 2) - (lcd.getStrWidth("Auto-Aligning") / 2);
      lcd.drawStr(title_w, H / 2, "Auto-Aligning");

      lcd.drawBox(0, H / 2 - h, w, 1); //Seperate Line
      lcd.drawBox(0, H / 2 + 5, w, 1); //Seperate Line

      lcd.sendBuffer();
    }

    else if (PageLevel == 103) /* Auto-Curing */
    {
      lcd.clearBuffer();

      h = lcd.getFontAscent() - lcd.getFontDescent() + 2;
      w = lcd.getWidth();

      int H = lcd.getHeight();

      int title_w = (w / 2) - (lcd.getStrWidth("Auto-Curing") / 2);
      lcd.drawStr(title_w, H / 2 - (h / 2), "Auto-Curing");

      lcd.drawBox(0, H / 2 - 1.8 * h, w, 1); //Seperate Line
      lcd.drawBox(0, H / 2 + 1.8 * h, w, 1); //Seperate Line

      String Q_Time_Show = String(Q_Time) + " s";
      int Q_Time_w = (w / 2) - (lcd.getStrWidth(Q_Time_Show.c_str()) / 2);
      lcd.drawStr(Q_Time_w, H / 2 + 0.9 * h, Q_Time_Show.c_str());

      lcd.sendBuffer();
    }

    lcd.sendBuffer();
    isLCD = false;

    return;

    if (LCD_Update_Mode == 0 && pageIndex != LCD_PageNow) /* Main Page*/
    {
      // lcd.begin();
      lcd.clearBuffer();
      // lcd.clearDisplay();

      H = lcd.getHeight();
      h = lcd.getFontAscent() - lcd.getFontDescent() + 2;
      w = lcd.getWidth();
      title_h = h + 2;

      int title_w = (w / 2) - (lcd.getStrWidth("Menu") / 2);
      lcd.drawStr(title_w, h - 1, "Menu");

      lcd.drawBox(0, h + 1, w, 1); //Seperate Line

      int deltaIndex = 0;
      if (pageIndex > Bottom_Item_Index)
      {
        deltaIndex = abs(pageIndex - Bottom_Item_Index);
        Top_Item_Index = Top_Item_Index + deltaIndex;
        Bottom_Item_Index = Top_Item_Index + 3;
      }
      else if (pageIndex < Top_Item_Index)
      {
        deltaIndex = abs(Top_Item_Index - pageIndex);
        Top_Item_Index = Top_Item_Index - deltaIndex;
        Bottom_Item_Index = Top_Item_Index + 3;
      }

      // if (LCD_Update_Mode < 100)
      //   lcd.drawFrame(0, (pageIndex - Top_Item_Index) * h + 1 + title_h, w, h + 1); //Un-Selected Box

      //Draw each item in UI_Menu_Items
      Draw_ALL_UI_Items(LCD_Update_Mode, pageIndex);

      if (pageIndex < MENU_ITEMS - 1)
        lcd.drawTriangle(w / 2 - 2, H - 3, w / 2 + 3, H - 3, w / 2, H);

      LCD_PageNow = pageIndex;

      lcd.sendBuffer();
    }

    else if (LCD_Update_Mode == 1) /* Auto-Aligning */
    {
      // lcd.clearBuffer();
      lcd.clearDisplay();
      // lcd.clearWriteError();

      h = lcd.getFontAscent() - lcd.getFontDescent() + 2;
      w = lcd.getWidth();

      int H = lcd.getHeight();

      int title_w = (w / 2) - (lcd.getStrWidth("Auto-Aligning") / 2);
      lcd.drawStr(title_w, H / 2, "Auto-Aligning");

      lcd.drawBox(0, H / 2 - h, w, 1); //Seperate Line
      lcd.drawBox(0, H / 2 + 5, w, 1); //Seperate Line

      lcd.sendBuffer();
    }

    else if (LCD_Update_Mode == 2) /* Auto-Curing */
    {
      lcd.clearBuffer();
      // lcd.clearDisplay();
      // lcd.clearWriteError();

      h = lcd.getFontAscent() - lcd.getFontDescent() + 2;
      w = lcd.getWidth();

      int H = lcd.getHeight();

      int title_w = (w / 2) - (lcd.getStrWidth("Auto-Curing") / 2);
      lcd.drawStr(title_w, H / 2 - (h / 2), "Auto-Curing");

      lcd.drawBox(0, H / 2 - 1.8 * h, w, 1); //Seperate Line
      lcd.drawBox(0, H / 2 + 1.8 * h, w, 1); //Seperate Line

      String Q_Time_Show = String(Q_Time) + " s";
      int Q_Time_w = (w / 2) - (lcd.getStrWidth(Q_Time_Show.c_str()) / 2);
      lcd.drawStr(Q_Time_w, H / 2 + 0.9 * h, Q_Time_Show.c_str());

      lcd.sendBuffer();
    }

    else if (LCD_Update_Mode == 12) /* Target IL */
    {
      lcd.clearBuffer();
      // lcd.clearDisplay();

      int title_w = (w / 2) - (lcd.getStrWidth("Menu") / 2);
      lcd.drawStr(title_w, h - 1, "Menu");

      lcd.drawBox(0, h + 1, w, 1); //Seperate Line

      //Draw each item in UI_Menu_Items
      Draw_ALL_UI_Items(LCD_Update_Mode, pageIndex);

      if (pageIndex < MENU_ITEMS - 1)
        lcd.drawTriangle(w / 2 - 2, H - 3, w / 2 + 3, H - 3, w / 2, H);

      LCD_PageNow = LCD_Update_Mode;

      lcd.sendBuffer();
    }

    else if (LCD_Update_Mode == 14) /* Q Z-offset */
    {
      lcd.clearBuffer();
      // lcd.clearDisplay();

      int title_w = (w / 2) - (lcd.getStrWidth("Menu") / 2);
      lcd.drawStr(title_w, h - 1, "Menu");

      lcd.drawBox(0, h + 1, w, 1); //Seperate Line

      //Draw each item in UI_Menu_Items
      Draw_ALL_UI_Items(LCD_Update_Mode, pageIndex);

      if (pageIndex < MENU_ITEMS - 1)
        lcd.drawTriangle(w / 2 - 2, H - 3, w / 2 + 3, H - 3, w / 2, H);

      LCD_PageNow = LCD_Update_Mode;

      lcd.sendBuffer();
    }

    else if (LCD_Update_Mode == 15) /* X Speed */
    {
      lcd.clearBuffer();
      // lcd.clearDisplay();

      int title_w = (w / 2) - (lcd.getStrWidth("Menu") / 2);
      lcd.drawStr(title_w, h - 1, "Menu");

      lcd.drawBox(0, h + 1, w, 1); //Seperate Line

      //Draw each item in UI_Menu_Items
      Draw_ALL_UI_Items(LCD_Update_Mode, pageIndex);

      if (pageIndex < MENU_ITEMS - 1)
        lcd.drawTriangle(w / 2 - 2, H - 3, w / 2 + 3, H - 3, w / 2, H);

      LCD_PageNow = LCD_Update_Mode;

      lcd.sendBuffer();
    }

    else if (LCD_Update_Mode == 99) /* Get Ref ? */
    {
      lcd.clearBuffer();

      h = lcd.getFontAscent() - lcd.getFontDescent() + 2;
      w = lcd.getWidth();

      int H = lcd.getHeight();

      int title_w = (w / 2) - (lcd.getStrWidth("Get Ref ?") / 2);
      lcd.drawStr(title_w, H / 2 - (h / 2), "Get Ref ?");

      lcd.drawBox(0, H / 2 - 1.8 * h, w, 1); //Seperate Line
      lcd.drawBox(0, H / 2 + 1.8 * h, w, 1); //Seperate Line

      int location_X_Yes = (w / 2) - 7 - lcd.getStrWidth("Yes");
      int location_X_No = (w / 2) + 7;
      int location_Y = H / 2 + 0.9 * h;

      lcd.drawStr(location_X_Yes, location_Y, "Yes");
      lcd.drawStr(location_X_No, location_Y, "No");

      //Draw Selection box
      if (ui_YesNo_Selection)
        lcd.drawFrame(location_X_Yes - 2, location_Y - h + 1, lcd.getStrWidth("Yes") + 4, h + 2);
      else
        lcd.drawFrame(location_X_No - 2, location_Y - h + 1, lcd.getStrWidth("No") + 4, h + 2);

      LCD_PageNow = LCD_Update_Mode;

      lcd.sendBuffer();
    }

    else if (LCD_Update_Mode == 100) /*  */
    {
      // lcd.initDisplay();

      // delay(100);

      // lcd.begin();
      // lcd.clearBuffer();
      lcd.clearDisplay();
      // lcd.clearWriteError();
      delay(200);

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

    isLCD = false;

    // LCD_en_count =
  }
}

void Draw_ALL_UI_Items(int LCD_Update_Mode, int pageIndex)
{
  //Draw each item in UI_Menu_Items
  if (PageLevel == 1 && mainpageIndex == 4) //Speed mode
  {
    for (i = 0; i < Speed_Page_ITEMS; i++)
    {
      lcd.drawStr(3, title_h + ((i + 1) * h) - 1, UI_Speed_Page_Items[i]);

      //Item Content
      switch (i)
      {
      case 0:
        lcd.drawStr(lcd.getWidth() - lcd.getStrWidth("0000") - 2, title_h + ((i + 1) * h) - 1, String(delayBetweenStep_X).c_str());
        break;

      case 1:
        lcd.drawStr(lcd.getWidth() - lcd.getStrWidth("0000") - 2, title_h + ((i + 1) * h) - 1, String(delayBetweenStep_Y).c_str());
        break;

      case 2:
        lcd.drawStr(lcd.getWidth() - lcd.getStrWidth("0000") - 2, title_h + ((i + 1) * h) - 1, String(delayBetweenStep_Z).c_str());
        break;

      default:
        break;
      }

      if (item_is_selected)
      {
        //Selection Box
        switch (subpageIndex)
        {
        case 0:
          lcd.drawFrame(lcd.getWidth() - lcd.getStrWidth("0000") - 4, (0) * h + 1 + title_h, lcd.getStrWidth("0000") + 4, h + 1);
          break;

        case 1:
          lcd.drawFrame(lcd.getWidth() - lcd.getStrWidth("0000") - 4, (1) * h + 1 + title_h, lcd.getStrWidth("0000") + 4, h + 1);
          break;

        case 2:
          lcd.drawFrame(lcd.getWidth() - lcd.getStrWidth("0000") - 4, (2) * h + 1 + title_h, lcd.getStrWidth("0000") + 4, h + 1);
          break;

        default:
          break;
        }
      }
      else
      {
        lcd.drawFrame(0, (subpageIndex)*h + 1 + title_h, w, h + 1); //Un-Selected Box
      }
    }
  }
  else //Main Page (Menu) Items
  {
    for (i = Top_Item_Index; i < Top_Item_Index + 4; i++)
    {
      if (PageLevel == 0)
      {
        for (size_t j = 0; j < PageItemsCount; j++)
        {
          UI_Items[j] = UI_Menu_Items[j];
        }
      }

      lcd.drawStr(3, title_h + ((i + 1 - Top_Item_Index) * h) - 1, UI_Items[i]);

      //Item Content
      switch (i)
      {
      case 1:
        lcd.drawStr(lcd.getWidth() - lcd.getStrWidth("-00.0") - 2, title_h + ((i + 1 - Top_Item_Index) * h) - 1, String(Target_IL).c_str());
        break;

      case 3:
        lcd.drawStr(lcd.getWidth() - lcd.getStrWidth("0000") - 2, title_h + ((i + 1 - Top_Item_Index) * h) - 1, String(AQ_Scan_Compensation_Steps_Z_A).c_str());
        break;

      case 5:
        lcd.drawStr(lcd.getWidth() - lcd.getStrWidth("0000.00") - 2, title_h + ((i + 1 - Top_Item_Index) * h) - 1, String(ref_Dac).c_str());
        break;

      default:
        break;
      }

      if (item_is_selected == true)
      {
        //Selection Box
        switch (LCD_Update_Mode)
        {
        case 12:
          lcd.drawFrame(lcd.getWidth() - lcd.getStrWidth("-00.0") - 4, (pageIndex - Top_Item_Index) * h + 1 + title_h, lcd.getStrWidth("-00.0") + 4, h + 1);
          break;

        case 14:
          lcd.drawFrame(lcd.getWidth() - lcd.getStrWidth("0000") - 4, (pageIndex - Top_Item_Index) * h + 1 + title_h, lcd.getStrWidth("0000") + 4, h + 1);
          break;

        default:
          break;
        }
      }
      else
      {
        lcd.drawFrame(0, (pageIndex - Top_Item_Index) * h + 1 + title_h, w, h + 1); //Un-Selected Box
      }
    }
  }
}

int pre_LCD_Page_index = 0;
void LCD_Encoder_Rise()
{
  LCD_Encoder_State = digitalRead(LCD_Encoder_A_pin);

  bool is_update_LCD_en_count = false;

  if (LCD_Encoder_State != LCD_Encoder_LastState)
  {
    if (digitalRead(LCD_Encoder_B_pin) != LCD_Encoder_State)
    {
      if (PageLevel == 0 && item_is_selected)
      {
        if (mainpageIndex == 1)
        {
          Target_IL += 0.05;
        }
        else if (mainpageIndex == 3)
        {
          AQ_Scan_Compensation_Steps_Z_A += 1;
        }
      }
      else if (PageLevel == 1)
      {
        if (mainpageIndex == 4)  //Speed Setting Page
        {
          if (!item_is_selected)
          {
            if (subpageIndex < (subpage_itemsCount - 1) * 2){
              LCD_sub_count +=1;
              subpageIndex += 1;
            }
          }
          else
          {
            if (subpageIndex == 0)
              delayBetweenStep_X += 1;
            else if (subpageIndex == 1)
              delayBetweenStep_Y += 1;
            else if (subpageIndex == 2)
              delayBetweenStep_Z += 1;
          }
        }
        else if (mainpageIndex == 5)
        {
          ui_YesNo_Selection = false;
        }
      }

      if (PageLevel == 0 && !item_is_selected)
      {
        is_update_LCD_en_count = true;
        LCD_en_count++;
      }
    }
    else
    {
      if (PageLevel == 0 && item_is_selected)
      {
        if (mainpageIndex == 1)
        {
          Target_IL -= 0.05;
        }
        else if (mainpageIndex == 3)
        {
          AQ_Scan_Compensation_Steps_Z_A -= 1;
        }
      }
      else if (PageLevel == 1)
      {
        if (mainpageIndex == 4)
        {
          if (!item_is_selected)
          {
            if (subpageIndex > 0){
              LCD_sub_count-=1;
              subpageIndex -= 1;
            }
          }
          else
          {
            if (subpageIndex == 0)
              delayBetweenStep_X -= 1;
            else if (subpageIndex == 1)
              delayBetweenStep_Y -= 1;
            else if (subpageIndex == 2)
              delayBetweenStep_Z -= 1;
          }
        }
        else if (mainpageIndex == 5)
        {
          ui_YesNo_Selection = true;
        }
      }

      if (PageLevel == 0 && !item_is_selected)
      {
        is_update_LCD_en_count = true;
        LCD_en_count--;
      }
    }
  }
  LCD_Encoder_LastState = LCD_Encoder_State;

  idx = LCD_en_count / 2;

  if (PageLevel == 1)
  {
    if (mainpageIndex == 4)
    {
      if (!item_is_selected)
        subpageIndex = LCD_sub_count / 2;

        if(subpageIndex > 3){
          subpageIndex = 3;
          LCD_sub_count = 6;
        }
    }
  }

  isLCD = true;
}

void LCD_Encoder_Selected()
{
  if (!btn_isTrigger)
  {
    btn_isTrigger = true;

    Serial.println("PageLevel:" + String(PageLevel) + ", mainpageIndex:" + String(mainpageIndex) + ", subpageIndex:" + String(subpageIndex));
    Serial.println("subpageIndex:" + String(subpageIndex) + ", item_is_selected:" + String(item_is_selected));

    if (PageLevel == 0)
    {
      Serial.println("mainpageIndex:" + String(mainpageIndex));

      if (!item_is_selected)
      {
        switch (mainpageIndex)
        {
        case 1: /* Into Target IL Mode*/
          LCD_Update_Mode = 12;
          item_is_selected = true;
          isLCD = true;
          pre_LCD_Page_index = mainpageIndex;
          break;
        case 3: /* Into Q Z-offset Mode*/
          LCD_Update_Mode = 14;
          item_is_selected = true;
          isLCD = true;
          pre_LCD_Page_index = mainpageIndex;
          break;

        case 4: /* Into Speed Mode*/
          LCD_Update_Mode = 101;
          isLCD = true;
          PageLevel = 1;
          subpage_itemsCount = Speed_Page_ITEMS;
          subpageIndex = 0;
          pre_LCD_Page_index = mainpageIndex;
          break;

        case 5: /* Into Get Ref Mode*/
          LCD_Update_Mode = 99;
          isLCD = true;
          PageLevel = 1;
          pre_LCD_Page_index = mainpageIndex;
          break;
        case 99:
          if (ui_YesNo_Selection)
            cmd_No = 19; //Get Ref

          LCD_Update_Mode = 0;
          isLCD = true;
          break;

        default:
          break;
        }
      }
      else
      {
        switch (mainpageIndex)
        {
        case 1: /* Set Target IL */
          LCD_en_count = 2;
          LCD_Update_Mode = 0;
          isLCD = true;
          MSGOutput(WR_EEPROM(72, String(Target_IL)));
          item_is_selected = false;
          Serial.println("item_is_selected:" + String(item_is_selected));
          break;

        case 3: /* Set Q Z-offset */
          LCD_en_count = 6;
          LCD_Update_Mode = 0;
          isLCD = true;
          Serial.println("Write EEPROM AQ_Scan_Compensation_Steps_Z_A: " + WR_EEPROM(160, String(AQ_Scan_Compensation_Steps_Z_A)));
          item_is_selected = false;
          break;

        default:
          break;
        }
      }
      updateUI(pre_LCD_Page_index);
    }
    else if (PageLevel == 1)
    {
      if (mainpageIndex == 4) // speed mode
      {
        if (subpageIndex != 3)
        {
          if (item_is_selected)
          {
            switch (subpageIndex)
            {
            case 0:
              Serial.println("Write EEPROM X Speed: " + WR_EEPROM(48, String(delayBetweenStep_X)));
              break;

            case 1:
              Serial.println("Write EEPROM Y Speed: " + WR_EEPROM(56, String(delayBetweenStep_Y)));
              break;

            case 2:
              Serial.println("Write EEPROM Z Speed: " + WR_EEPROM(64, String(delayBetweenStep_Z)));
              break;

            default:
              break;
            }
          }

          item_is_selected = !item_is_selected;
        }
        else
        {
          item_is_selected = false;
          PageLevel = 0;
        }
        isLCD = true;
      }

      else if (mainpageIndex == 5) //get ref mode
      {
        PageLevel = 0;

        if (ui_YesNo_Selection)
        {
          ui_YesNo_Selection = false;
          cmd_No = 19; //Get Ref
        }

        isLCD = true;
      }
    }

    btn_isTrigger = false;
    delay(300);
    return;

    Serial.println("Encoder_Pressed_PageNow:" + String(LCD_PageNow));
    switch (LCD_PageNow)
    {
    case 1: /* Into Target IL Mode*/
      LCD_Update_Mode = 12;
      item_is_selected = true;
      isLCD = true;
      pre_LCD_Page_index = LCD_PageNow;
      Serial.println("LCD_Update_Mode:" + String(LCD_Update_Mode));
      break;
    case 3: /* Into Q Z-offset Mode*/
      LCD_Update_Mode = 14;
      item_is_selected = true;
      isLCD = true;
      pre_LCD_Page_index = LCD_PageNow;
      Serial.println("LCD_Update_Mode:" + String(LCD_Update_Mode));
      break;

    case 4: /* Into Speed Mode*/
      PageLevel = 1;
      LCD_Update_Mode = 101;
      subpage_itemsCount = Speed_Page_ITEMS;
      isLCD = true;
      pre_LCD_Page_index = LCD_PageNow;
      break;

    case 5: /* Into Get Ref Mode*/
      LCD_Update_Mode = 99;
      isLCD = true;
      pre_LCD_Page_index = LCD_PageNow;
      // Serial.println("LCD_Update_Mode:" + String(LCD_Update_Mode));

      break;
    case 99:
      if (ui_YesNo_Selection)
        cmd_No = 19; //Get Ref

      LCD_Update_Mode = 0;
      isLCD = true;
      break;

    case 12: /* Set Target IL */
      LCD_en_count = 2;
      LCD_Update_Mode = 0;
      isLCD = true;
      MSGOutput(WR_EEPROM(72, String(Target_IL)));
      item_is_selected = false;
      Serial.println("item_is_selected:" + String(item_is_selected));
      break;

    case 14: /* Set Q Z-offset */
      LCD_en_count = 6;
      LCD_Update_Mode = 0;
      isLCD = true;
      Serial.println("Write EEPROM AQ_Scan_Compensation_Steps_Z_A: " + WR_EEPROM(160, String(AQ_Scan_Compensation_Steps_Z_A)));
      item_is_selected = false;
      break;

    case 101: /* Into Set X Speed Mode */
      if (subpageIndex == 0)
        LCD_Update_Mode = 25;
      else if (subpageIndex == 1)
        LCD_Update_Mode = 26;
      else if (subpageIndex == 2)
        LCD_Update_Mode = 27;
      isLCD = true;
      break;

    case 102: /* Set X Speed */
      LCD_en_count = 8;
      LCD_Update_Mode = 0;
      isLCD = true;
      Serial.println("Write EEPROM X_Speed: " + WR_EEPROM(160, String(delayBetweenStep_X)));
      break;

    case 103: /* Set Y Speed */
      LCD_en_count = 8;
      LCD_Update_Mode = 0;
      isLCD = true;
      Serial.println("Write EEPROM Y_Speed: " + WR_EEPROM(160, String(delayBetweenStep_Y)));
      break;

    case 104: /* Set Z Speed */
      LCD_en_count = 8;
      LCD_Update_Mode = 0;
      isLCD = true;
      Serial.println("Write EEPROM Z_Speed: " + WR_EEPROM(160, String(delayBetweenStep_Z)));
      break;

    case 100: /* Back to menu */
      // LCD_en_count = 8;
      PageLevel = 0;
      LCD_Update_Mode = 0;
      isLCD = true;
      Serial.println("Menu");
      break;

    default:
      break;
    }
    updateUI(pre_LCD_Page_index);

    delay(300);

    btn_isTrigger = false;
  }
}

void EmergencyStop()
{
  isStop = true;

  Serial.println("EmergencyStop");
  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

  isLCD = true;
  PageLevel = 0;
}
//------------------------------------------------------------------------------------------------------------------------------------------
String httpGETRequest(const char *serverName)
{
  WiFiClient client;
  HTTPClient http;

  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "--";

  if (httpResponseCode > 0)
  {
    // Serial.print("HTTP Response code: ");
    // Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}
//------------------------------------------------------------------------------------------------------------------------------------------
// String httpTestRequest(const char *serverName)
// {
//   WiFiClient client;
//   HTTPClient http;

//   // Your Domain name with URL path or IP address with path
//   http.begin(client, serverName);

//   // http.addHeader("Station", ID);
//   // http.addHeader("Data", "Hello world~ ");
//   // Send HTTP POST request
//   // int httpResponseCode = http.POST("Hello ~ ~");
//   int httpResponseCode = http.GET();

//   String payload = "--";

//   if (httpResponseCode > 0)
//   {
//     // Serial.print("HTTP Test Response code: ");
//     // Serial.println(httpResponseCode);
//     // payload = http.getString();
//     // Serial.println("Data: " + payload);
//   }
//   else
//   {
//     Serial.print("Error code: ");
//     Serial.println(httpResponseCode);
//   }
//   // Free resources
//   http.end();

//   return payload;
// }
//------------------------------------------------------------------------------------------------------------------------------------------

String httpTestRequest(const char *serverName, const char *msg)
{
  WiFiClient client;
  HTTPClient http;

  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);

  http.addHeader("Station", ID.c_str());
  http.addHeader("Data", msg);
  int httpResponseCode = http.GET();

  String payload = "--";

  if (httpResponseCode <= 0)
  {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  http.end();

  return payload;
}

//------------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(20); //設定序列埠接收資料時的最大等待時間

  //宣告使用EEPROM 512 個位置
  EEPROM.begin(512);
  
  String server_ID = ReadInfoEEPROM(88, 32);
  String server_Password = ReadInfoEEPROM(120, 32);

  // Serial.println("Server ID: " + server_ID);
  // Serial.println("Server Password: " + server_Password);

  if (Contains(server_ID, "??") || server_ID == "")
  {
    server_ID = "GFI-ESP32-Access-Point";
  }

  if (Contains(server_Password, "??"))
  {
    server_Password = "22101782";
  }

  Serial.println("Server ID: " + server_ID);
  Serial.println("Server Password: " + server_Password);

  WiFi.begin(server_ID.c_str(), server_Password.c_str());
  // WiFi.begin(ssid, password);
  Serial.println("Connecting");

  int wifiConnectTime = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(300);
    Serial.print(".");

    wifiConnectTime += 300;
    if (wifiConnectTime > 2400)
      break;
  }

  if (wifiConnectTime <= 2400)
  {
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address:");
    Serial.println(WiFi.localIP());
    isWiFiConnected = true;
  }
  else
  {
    Serial.println("Connected to WiFi network failed");
  }

  // u8g2_font_5x7_tf
  lcd.begin();

  lcd.setFont(u8g2_font_6x10_tf);
  lcd.clearDisplay();

#pragma region pinMode Setting

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

  pinMode(C_1, OUTPUT); ///keyValue:1
  pinMode(C_2, OUTPUT); ///keyValue:2
  pinMode(C_3, OUTPUT); ///keyValue:3

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

#pragma endregion

  Serial.println("~~ Auto-Align System ~~");
  // BT.println("~~ Auto-Align System ~~");

  // Serial.println("Watch Dog Online");
  // esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  // esp_task_wdt_add(NULL);               //add current thread to WDT watch

  

  String eepromString;

  for (int i = 0; i < 511; i = i + 8)
  {
    eepromString = ReadInfoEEPROM(i, 8); //Reading EEPROM(int start_position, int data_length)
    MSGOutput("EEPROM(" + String(i) + ") - " + eepromString);
  }

  eepromString = ReadInfoEEPROM(0, 8); //Reading EEPROM(int start_position, int data_length)
  ref_Dac = eepromString.toDouble();
  ref_IL = ILConverter(ref_Dac);
  MSGOutput("Ref IL: " + String(ref_IL));

  ID = ReadInfoEEPROM(8, 8);
  MSGOutput("Board ID: " + ReadInfoEEPROM(8, 8)); 

  Station_ID = ReadInfoEEPROM(16, 8);
  MSGOutput("Station ID: " + ReadInfoEEPROM(16, 8)); 

  eepromString = ReadInfoEEPROM(24, 8);
  X_backlash = eepromString.toInt();
  MSGOutput("X_backlash: " + String(X_backlash));

  eepromString = ReadInfoEEPROM(32, 8);
  Y_backlash = eepromString.toInt();
  MSGOutput("Y_backlash: " + String(Y_backlash));

  eepromString = ReadInfoEEPROM(40, 8);
  Z_backlash = eepromString.toInt();
  MSGOutput("Z_backlash: " + String(Z_backlash));

  eepromString = ReadInfoEEPROM(48, 8);
  delayBetweenStep_X = eepromString.toInt();
  MSGOutput("delayBetweenStep_X: " + String(delayBetweenStep_X));

  eepromString = ReadInfoEEPROM(56, 8);
  delayBetweenStep_Y = eepromString.toInt();
  MSGOutput("delayBetweenStep_Y: " + String(delayBetweenStep_Y));

  eepromString = ReadInfoEEPROM(64, 8);
  delayBetweenStep_Z = eepromString.toInt();
  MSGOutput("delayBetweenStep_Z: " + String(delayBetweenStep_Z));

  eepromString = ReadInfoEEPROM(72, 8);
  Target_IL = eepromString.toDouble();
  MSGOutput("Target IL: " + String(Target_IL));

  AQ_Scan_Compensation_Steps_Z_A = ReadInfoEEPROM(160, 8).toInt();
  MSGOutput("AQ_Scan_Compensation_Steps_Z_A: " + String(AQ_Scan_Compensation_Steps_Z_A));

  AQ_Total_TimeSpan = ReadInfoEEPROM(168, 8).toInt();
  MSGOutput("AQ_Total_TimeSpan: " + String(AQ_Total_TimeSpan));

  AA_ScanFinal_Scan_Delay_X_A = ReadInfoEEPROM(80, 8).toInt();
  MSGOutput("AA_ScanFinal_Scan_Delay_X_A: " + String(AA_ScanFinal_Scan_Delay_X_A));

  AQ_Scan_Steps_Z_A = ReadInfoEEPROM(176, 8).toInt();
  MSGOutput("AQ_Scan_Steps_Z_A: " + String(AQ_Scan_Steps_Z_A));

  AQ_Scan_Steps_Z_B = ReadInfoEEPROM(184, 8).toInt();
  MSGOutput("AQ_Scan_Steps_Z_B: " + String(AQ_Scan_Steps_Z_B));

  AQ_Scan_Steps_Z_C = ReadInfoEEPROM(192, 8).toInt();
  MSGOutput("AQ_Scan_Steps_Z_C: " + String(AQ_Scan_Steps_Z_C));

  AQ_Scan_Steps_Z_D = ReadInfoEEPROM(200, 8).toInt();
  MSGOutput("AQ_Scan_Steps_Z_D: " + String(AQ_Scan_Steps_Z_D));

  FS_Count_X = ReadInfoEEPROM(EP_FS_Count_X, 8).toInt();
  MSGOutput("FS_Count_X: " + String(FS_Count_X));

  FS_Steps_X = ReadInfoEEPROM(EP_FS_Steps_X, 8).toInt();
  MSGOutput("FS_Steps_X: " + String(FS_Steps_X));

  FS_Stable_X = ReadInfoEEPROM(EP_FS_Stable_X, 8).toInt();
  MSGOutput("FS_Stable_X: " + String(FS_Stable_X));

  FS_DelaySteps_X = ReadInfoEEPROM(EP_FS_DelaySteps_X, 8).toInt();
  MSGOutput("FS_DelaySteps_X: " + String(FS_DelaySteps_X));

  FS_Avg_X = ReadInfoEEPROM(EP_FS_Avg_X, 8).toInt();
  MSGOutput("FS_Avg_X: " + String(FS_Avg_X));

  FS_Count_Y = ReadInfoEEPROM(EP_FS_Count_Y, 8).toInt();
  MSGOutput("FS_Count_Y: " + String(FS_Count_Y));

  FS_Steps_Y = ReadInfoEEPROM(EP_FS_Steps_Y, 8).toInt();
  MSGOutput("FS_Steps_Y: " + String(FS_Steps_Y));

  FS_Stable_Y = ReadInfoEEPROM(EP_FS_Stable_Y, 8).toInt();
  MSGOutput("FS_Stable_Y: " + String(FS_Stable_Y));

  FS_DelaySteps_Y = ReadInfoEEPROM(EP_FS_DelaySteps_Y, 8).toInt();
  MSGOutput("FS_DelaySteps_Y: " + String(FS_DelaySteps_Y));

  FS_Avg_Y = ReadInfoEEPROM(EP_FS_Avg_Y, 8).toInt();
  MSGOutput("FS_Avg_Y: " + String(FS_Avg_Y));

  FS_Count_Z = ReadInfoEEPROM(EP_FS_Count_Z, 8).toInt();
  MSGOutput("FS_Count_Z: " + String(FS_Count_Z));

  FS_Steps_Z = ReadInfoEEPROM(EP_FS_Steps_Z, 8).toInt();
  MSGOutput("FS_Steps_Z: " + String(FS_Steps_Z));

  FS_Stable_Z = ReadInfoEEPROM(EP_FS_Stable_Z, 8).toInt();
  MSGOutput("FS_Stable_Z: " + String(FS_Stable_Z));

  FS_DelaySteps_Z = ReadInfoEEPROM(EP_FS_DelaySteps_Z, 8).toInt();
  MSGOutput("FS_DelaySteps_Z: " + String(FS_DelaySteps_Z));

  FS_Avg_Z = ReadInfoEEPROM(EP_FS_Avg_Z, 8).toInt();
  MSGOutput("FS_Avg_Z: " + String(FS_Avg_Z));



  isLCD = true;
  LCD_Update_Mode = 0;
  PageLevel = 0;
  PageItemsCount = MENU_ITEMS;
  updateUI(0);

  //在core 0啟動 mision 1
  // xTaskCreatePinnedToCore(
  //     Task_1_sendData, /* 任務實際對應的Function */
  //     "Task_1",        /* 任務名稱 */
  //     10000,           /* 堆疊空間 */
  //     NULL,            /* 無輸入值 */
  //     0,               /* 優先序0 */
  //     &Task_1,         /* 對應的任務變數位址 */
  //     0);              /*指定在核心0執行 */
}

bool isMsgShow = false;
unsigned long previousMillis = 0;
const long interval = 2000;
String Data;

String ServerIP = "http://192.168.4.1/";
const char *serverTestData = "http://192.168.4.1/?param1=10&param2=hi";

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {    
    // Serial.println("Resetting WDT...");
    // esp_task_wdt_reset(); //喂狗操作，使看門狗定時器復位

    //Re-Initialize
    isStop = false;
    ButtonSelected = -1;

    //Keyboard Detect
    ButtonSelected = KeyValueConverter();

    String rsMsg = "";
    if (Serial.available())
      rsMsg = Serial.readString();

    String cmd = rsMsg;

    cmd_No = Function_Classification(cmd, ButtonSelected);

    // BLE_Function(cmd);

    cmd_No = Function_Excecutation(cmd, cmd_No);

    if (isGetPower)
    {
      if (millis() - timer_Get_IL_1 > 500)
      {
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

        MSGOutput("PD_Power:" + String(value)); //dB
      }
    }

    //LCD UI Update
    if (true)
    {
      // LCD_Encoder_Rise();

      if (!digitalRead(LCD_Select_pin))
      {
        LCD_Encoder_Selected();
      }

      //Update UI
      if (true)
      {
        if (idx == pre_LCD_Page_index)
        {
          if (LCD_Update_Mode <= 9)
            isLCD = false;
          else if (LCD_Update_Mode > 9 && isLCD)
            isLCD = true;
          else
            isLCD = false;
        }

        if (!isLCD)
          return;

        if (idx > pre_LCD_Page_index)
        {
          idx = pre_LCD_Page_index + 1;
          pre_LCD_Page_index = idx;
        }
        else if (idx < pre_LCD_Page_index)
        {
          idx = pre_LCD_Page_index - 1;
          pre_LCD_Page_index = idx;
        }
        LCD_en_count = idx * 2;

        if (idx < 0)
        {
          LCD_en_count = 0;
          idx = 0;
        }
        else if (idx >= MENU_ITEMS - 1)
        {
          LCD_en_count = MENU_ITEMS * 2 - 2;
          idx = MENU_ITEMS - 1;
        }

        // if (idx < 0)
        //   LCD_en_count = MENU_ITEMS * 2 - 2;
        // else if (idx == MENU_ITEMS - 1)
        //   LCD_en_count = MENU_ITEMS * 2 - 2;
        // else if (idx > MENU_ITEMS - 1)
        //   LCD_en_count = 0;

        // Serial.println("idx:" + String(idx) + ",pre_LCD_Page_index:" + String(pre_LCD_Page_index) + ", LCD_en_count:" + String(LCD_en_count));
        updateUI(idx);
        isLCD = false;
      }
    }
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

  MSGOutput(axis + " Go to position: " + String(Target) + ", origin position: " + String(Pos_Now));

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

  MSGOutput("");
  MSGOutput("Fine Scan ");

  MSGOutput("Stop Value: " + String(StopValue));

  digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
  delay(5);

  Threshold = -37.3;
  delayBetweenStep = 100;

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
      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

      CMDOutput("AS");
      // K_OK = Scan_AllRange_TwoWay(0, 8, 22, 0, 0, 120, StopValue, 500, 2, "X Scan,Trip_");
      // K_OK = Scan_AllRange_TwoWay(0, 7, 25, stableDelay, 0, 50, StopValue, 600, 2, "X Scan,Trip_");
      K_OK = Scan_AllRange_TwoWay(0, FS_Count_X, FS_Steps_X, FS_Stable_X, 0, FS_DelaySteps_X, StopValue, FS_Avg_X, 2, "X Scan,Trip_");
      CMDOutput("%:");

      if (!K_OK)
      {
        CMDOutput("AS");
        // Scan_AllRange_TwoWay(0, 7, 25, stableDelay, 0, 50, StopValue, 600, 2, "X Re-Scan,Trip_");
        Scan_AllRange_TwoWay(0, FS_Count_X, FS_Steps_X, FS_Stable_X, 0, FS_DelaySteps_X, StopValue, FS_Avg_X, 2, "X Scan,Trip_");
        CMDOutput("%:");
      }

      break;

    case 2:

      //        Y_ScanSTP = 10;
      PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);

      CMDOutput("AS");
      // Scan_AllRange_TwoWay(1, 6, 35, AA_ScanFinal_Scan_Delay_Y_A, 0, 100, StopValue, 600, 2, "Y Scan, Trip_");
      // K_OK = Scan_AllRange_TwoWay(1, 8, 20, 0, 0, 120, StopValue, 600, 2, "Y Scan,Trip_"); 
      K_OK = Scan_AllRange_TwoWay(1, FS_Count_Y, FS_Steps_Y, FS_Stable_Y, 0, FS_DelaySteps_Y, StopValue, FS_Avg_Y, 2, "Y Scan,Trip_"); 
      CMDOutput("%:");

      if (!K_OK)
      {
        CMDOutput("AS");
        // Scan_AllRange_TwoWay(1, 8, 20, 0, 0, 120, StopValue, 600, 2, "Y Re-Scan,Trip_");
        Scan_AllRange_TwoWay(1, FS_Count_Y, FS_Steps_Y, FS_Stable_Y, 0, FS_DelaySteps_Y, StopValue, FS_Avg_Y, 2, "Y Scan,Trip_"); 
        CMDOutput("%:");
      }

      break;

    case 3:

      //        Z_ScanSTP = 100;
      PD_Now = Cal_PD_Input_IL(2 * Get_PD_Points);

      CMDOutput("AS");
      // Scan_AllRange_TwoWay(2, 6, 100, AA_ScanFinal_Scan_Delay_X_A, 0, 80, StopValue, 600, 2, "Z Scan, Trip_"); //--Z--
      // Scan_AllRange_TwoWay(2, 8, 125, 0, 0, 100, StopValue, 600, 2, "Z_Scan,Trip_");
      // K_OK = Scan_AllRange_TwoWay(2, 7, 80, 0, 0, 80, StopValue, 800, 2, "Z Scan,Trip_");
      K_OK = Scan_AllRange_TwoWay(2, FS_Count_Z, FS_Steps_Z, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, 2, "Z Scan,Trip_");
      CMDOutput("%:");

      if (!K_OK)
      {
        CMDOutput("AS");
        // Scan_AllRange_TwoWay(2, 7, 80, 0, 0, 80, StopValue, 800, 2, "Z Re-Scan,Trip_");
        Scan_AllRange_TwoWay(2, FS_Count_Z, FS_Steps_Z, FS_Stable_Z, 0, FS_DelaySteps_Z, StopValue, FS_Avg_Z, 2, "Z Scan,Trip_");
        CMDOutput("%:");
      }

      break;
    }

    if(Q_Time !=0)
      MSGOutput("Scan at QTime:" + String(Q_Time));
  }
  //Case 4: all actions should be excuted
  else if (axis == 4)
  {
    // Region = Region + "_Fine_Scan (All Range)";
    CMDOutput("AS");
    msg = Region + "_Fine_Scan (All Range)" + ", Z Scan, Trip_";
    Scan_AllRange_TwoWay(2, 8, 125, 0, 0, 100, StopValue, 500, 2, "Z Scan, Trip_");
    CMDOutput("%:");

    if (isStop)
    {
      return true;
    }

    CMDOutput("AS");
    msg = Region + ", Y Scan, Trip_";
    Scan_AllRange_TwoWay(1, 7, 20, 0, 0, 120, StopValue, 500, 2, "Y Scan, Trip_");
    CMDOutput("%:");

    if (isStop)
    {
      return true;
    }

    CMDOutput("AS");
    msg = Region + ", X Scan, Trip_";
    Scan_AllRange_TwoWay(0, 8, 22, 0, 0, 120, StopValue, 500, 2, "X Scan, Trip_");
    CMDOutput("%:");
  }

  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
  delay(5);
  MSGOutput("Fine Scan End");

  isLCD = true;
  LCD_Update_Mode = 0;
  LCD_PageNow = 100;
}

//------------------------------------------------------------------------------------------------------------------------------------------

void AutoAlign()
{
  isLCD = true;
  PageLevel = 101;
  updateUI(PageLevel);

  StopValue = Target_IL;
  //  Reset_Z_to_Origin(); //Back to origin

  Move_Motor(Z_DIR_Pin, Z_STP_Pin, false, AA_SpiralRough_Feed_Steps_Z_A, 12, 150); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)

  unsigned long time1 = 0, time2 = 0, time3 = 0, time4 = 0, time5 = 0, time6 = 0, time7 = 0;
  double PD_LV1, PD_LV2, PD_LV3, PD_LV4, PD_LV5, PD_LV6;
  double PD_Now = 0;
  time1 = millis();
  MSGOutput(" ");
  CMDOutput("AA"); //Auto Align

  MSGOutput(" ");
  CMDOutput("AS"); //Align Start
  MSGOutput("... Spiral ...");

  //Spiral - Rough - 1
  int matrix_level = 10;
  CMDOutput("^X");
  CMDOutput("R:" + String(M_Level * 2 + 1));
  CMDOutput("C:" + String(M_Level * 2 + 1));

  delayBetweenStep = 15;                           //default:25
  MinMotroStep = AA_SpiralRough_Spiral_Steps_XY_A; //2000
  Region = "Sprial(Rough)";
  AutoAlign_Spiral(matrix_level, -42, 0); //Input : (Sprial Level, Threshold, stable)  Threshold:123
  CMDOutput("X^");

  DataOutput();

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);

  int Threshold = -21.4; //default: 144
  stableDelay = 0;       //default : 25

  msg = Region + ",Y Scan, Trip_";

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

  // MinMotroStep = 250;

  msg = Region + ",X Scan, Trip_";

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
  MSGOutput("Sprial(Rough) TimeSpan : " + String((time2 - time1) / 1000) + " s");
  MSGOutput(" ");

  if (isStop)
    return;

  MSGOutput(String(PD_LV1));
  MSGOutput(String(isStop));

  //Spiral Fine Scan
  if (PD_LV1 < -40) //default: 300
  {
    Region = "Sprial(Fine)";
    int matrix_level = 10;
    CMDOutput("^X");
    CMDOutput("R:" + String(M_Level * 2 + 1));
    CMDOutput("C:" + String(M_Level * 2 + 1));

    delayBetweenStep = 25;
    MinMotroStep = AA_SpiralFine_Spiral_Steps_XY_A; //1500
    AutoAlign_Spiral(matrix_level, -37, 0);         //Input : (Sprial Level, Threshold, stable)  Threshold:166

    CMDOutput("X^");
  }

  PD_LV2 = Cal_PD_Input_IL(Get_PD_Points);
  time3 = millis();

  if (isStop)
    return;

  PD_Now = PD_LV2;
  if (PD_Now < -50)
  {
    MSGOutput("High loss after spiral scan");
    return;
  }

  MSGOutput(" ");
  MSGOutput("... X, Y, Z Scan(Rough) ...");
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
    MSGOutput("Scan(Rough)");
    for (int i = 0; i < 15; i++)
    {
      //Scan(Rough) : Feed Z Loop
      if (true)
      {
        if (i > 0 && abs(PD_After - PD_Before) < 0.1) //1.2
        {
          MSGOutput("Break_Loop... :" + String(PD_After) + "," + String(PD_Before));
          return;
        }
        else
          MSGOutput("XYZ_Scan(Rough) Round :" + String(i + 1) + ",After:" + String(PD_After) + ",Before:" + String(PD_Before));

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
          MSGOutput("PD_Z_before:" + String(PD_Z_before));

          int motorStep = AA_ScanRough_Feed_Steps_Z_A; //default: 10000
          if (PD_Z_before < -30)
          {
            motorStep = AA_ScanRough_Feed_Steps_Z_A;
            Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, motorStep, 20, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
          }
          else
          {
            double ratio_idx = AA_ScanRough_Feed_Ratio_Z_A;
            if (PD_Z_before > -29.5 && PD_Z_before <= -22.7)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_A; //default: 3.2
            else if (PD_Z_before > -22.7 && PD_Z_before <= -18.2)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_B; //default: 2.9
            else if (PD_Z_before > -18.2 && PD_Z_before <= -9)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_C; //default: 2.5
            else if (PD_Z_before > -9)
              ratio_idx = AA_ScanRough_Feed_Ratio_Z_D; //default: 1.8

            //default: 0.5
            motorStep = abs(ratio_idx * (588 + (55 * abs(PD_Z_before)))); //Default: 588-55*pd
            if (motorStep < AA_ScanRough_Feed_Steps_Z_B)
              motorStep = AA_ScanRough_Feed_Steps_Z_B; //default: 1000

            MSGOutput("ratio_idx:" + String(ratio_idx));

            Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, motorStep, 50, stableDelay); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
          }

          MSGOutput("Z_feed:" + String(motorStep));
          DataOutput();

          PD_Now = Cal_PD_Input_IL(Get_PD_Points);

          if (PD_Now > stopValue)
          {
            MSGOutput("Over_Stop_Value");
          }

          if (PD_Now <= PD_Z_before || (PD_Z_before - PD_Now) > 30 || abs(PD_Z_before - PD_Now) <= 1.5)
          {
            MSGOutput("Z_feed_break,Now:" + String(PD_Now) + ",Before:" + String(PD_Z_before) + ",Z_Pos_Now:" + String(Z_Pos_Now));
            MSGOutput(" ");
            break;
          }
          else
          {
            MSGOutput("Z_feed,Now:" + String(PD_Now) + ",Before:" + String(PD_Z_before) + ",Z_Pos_Now:" + String(Z_Pos_Now));
          }
        }
      }

      Threshold = 120;
      scanPoints = 11;
      // delayBetweenStep = 8;
      stableDelay = 25; //default: 25

      //Scan(Rough) : Spiral, if IL<-54
      if (PD_Now < -54)
      {
        MSGOutput("Spiral:z_feed_region");

        CMDOutput("^X");
        CMDOutput("R:" + String(M_Level * 2 + 1));
        CMDOutput("C:" + String(M_Level * 2 + 1));

        MinMotroStep = 300; //350
        if (!AutoAlign_Spiral(11, -36.4, 0))
        {
          CMDOutput("X^");
          MSGOutput("Spiral:Target_IL_not_found");
          MSGOutput(" ");

          return;
        }
        CMDOutput("X^");

        MSGOutput(" ");

        if (isStop)
          return;

        stableDelay = 0;

        Region = "Scan(Rough)(1)";

        msg = Region + ",Y_Scan" + ",Trip_";

        AutoAlign_Scan_DirectionJudge_V2(1, 20, Threshold, 150, stableDelay, MotorCC_Y, delayBetweenStep, Get_PD_Points, 279, msg);

        CMDOutput("%:");

        msg = Region + ",X_Scan" + ",Trip_";

        AutoAlign_Scan_DirectionJudge_V2(0, 25, Threshold, 150, stableDelay, MotorCC_X, delayBetweenStep, Get_PD_Points, 279, msg);

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

          Region = "Scan(Rough)(2)";

          //(int XYZ, int count, int Threshold, int motorStep, int stableDelay,
          //bool Direction, int delayBetweenStep, int Get_PD_Points, int StopPDValue, String msg)

          msg = Region + ",Y_Scan" + ",Trip_";
          AutoAlign_Scan_DirectionJudge_V2(1, 15, Threshold, 80, stableDelay, MotorCC_Y, delayBetweenStep, Get_PD_Points, stopValue, msg);
          CMDOutput("%:");

          msg = Region + ",X_Scan" + ",Trip_";
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

      //Scan(Rough) : XY Scan
      if (true)
      {
        delayBetweenStep = 80;
        MinMotroStep = 350;                        //800
        stableDelay = AA_ScanFinal_Scan_Delay_X_A; //default:0

        if (isStop)
          return;

        Region = "Scan(Rough)(3)";

        msg = Region + ",Y_Scan" + ",Trip_";
        if (PD_After > -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_A; //default:25
        else if (PD_After > -16 && PD_After <= -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_B; //default:30
        else if (PD_After > -22.7 && PD_After <= -16)
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_C; //default:40
        else
          MinMotroStep = AA_ScanRough_Scan_Steps_Y_D; //default:70

        CMDOutput("AS");
        MSGOutput("Gap:" + MinMotroStep);
        PD_After = AutoAlign_Scan_DirectionJudge_V2(1, 20, Threshold, MinMotroStep, stableDelay, MotorCC_Y, delayBetweenStep, Get_PD_Points, Target_IL, msg);
        CMDOutput("%:");

        // if (is_Scan_V2_ReWork)
        // {
        //   CMDOutput("AS");
        //   MSGOutput("Gap:" + MinMotroStep);
        //   PD_After = AutoAlign_Scan_DirectionJudge_V2(1, 20, Threshold, MinMotroStep, stableDelay, MotorCC_Y, delayBetweenStep, Get_PD_Points, Target_IL, msg);
        //   CMDOutput("%:");
        // }

        msg = Region + ",X_Scan" + ",Trip_";
        if (PD_After > -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_A; //default:25
        else if (PD_After > -16 && PD_After <= -9)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_B; //default:30
        else if (PD_After > -22.7 && PD_After <= -16)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_C; //default:80
        else if (PD_After > -30 && PD_After <= -22.7)
          MinMotroStep = AA_ScanRough_Scan_Steps_X_D; //default:100
        else
          MinMotroStep = AA_ScanRough_Scan_Steps_X_E; //default:120

        CMDOutput("AS");
        MSGOutput("Gap:" + MinMotroStep);
        PD_After = AutoAlign_Scan_DirectionJudge_V2(0, 20, Threshold, MinMotroStep, stableDelay, MotorCC_X, delayBetweenStep, Get_PD_Points, Target_IL, msg);
        CMDOutput("%:");

        // if (is_Scan_V2_ReWork)
        // {
        //   CMDOutput("AS");
        //   MSGOutput("Gap:" + MinMotroStep);
        //   PD_After = AutoAlign_Scan_DirectionJudge_V2(0, 20, Threshold, MinMotroStep, stableDelay, MotorCC_X, delayBetweenStep, Get_PD_Points, Target_IL, msg);
        //   CMDOutput("%:");
        // }
      }

      PD_After = Cal_PD_Input_IL(Get_PD_Points);

      DataOutput();

      //Scan(Rough) : stop condition
      if (true)
      {
        if (abs(PD_After - PDValue_After_Scan) < 0.1 && PD_After > -10) //default:16
        {
          MSGOutput("Scan(Rough)(A)-Pass_best_Z_position");
          MSGOutput(String(PD_After) + "," + String(PDValue_After_Scan));
          break;
        }

        if (PD_After < PDValue_After_Scan && PD_After > -10) //default:16
        {
          MSGOutput("Scan(Rough)(B)-Pass_best_Z_position_:_" + String(PD_After) + "," + String(PDValue_After_Scan));
          break;
        }

        if (PD_After < -42) //default:16
        {
          MSGOutput("Scan(Rough)(C)-Pass_best_Z_position");
          Serial.println(String(PD_After) + "," + String(PDValue_After_Scan));
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
          Serial.println("------- Delta_X:" + String(delta_X) + ",Delta_Y:" + String(delta_Y));
        }

        if (PD_After > stopValue)
        {
          Serial.println("Better_than_stopValue:" + String(PD_After) + ",stopvalue:" + String(stopValue));
          break;
        }

        if (PD_After < -54)
        {
          Serial.println("Rough_Scan:High_loss");
          return;
        }
      }
    }
  }

  PD_LV3 = Cal_PD_Input_IL(Get_PD_Points);
  time4 = millis();

  if (isStop)
    return;

  //Scan(Fine)
  Serial.println(" ");
  Serial.println("........ XYZ_Scan(Fine) ........");
  Region = "Scan(Fine)";

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  PDValue_Best = PD_Now;

  if (true && PD_Now > -25)
  {
    CMDOutput("AS");
    // Scan_AllRange_TwoWay(2, 6, AA_ScanFine_Scan_Steps_Z_A, AA_ScanFinal_Scan_Delay_X_A, 0, 60, StopValue, 500, 2, Region + "_Z Scan, Trip_");
    Scan_AllRange_TwoWay(2, 6, AA_ScanFine_Scan_Steps_Z_A, AA_ScanFinal_Scan_Delay_X_A, 0, 60, Target_IL, 500, 2, Region + "_Z Scan, Trip_");
    CMDOutput("%:");

    CMDOutput("AS");
    // Scan_AllRange_TwoWay(1, 8, AA_ScanFine_Scan_Steps_Y_A, AA_ScanFinal_Scan_Delay_X_A, 0, 80, StopValue, 550, 2, Region + "_Y Scan, Trip_");
    Scan_AllRange_TwoWay(1, 8, AA_ScanFine_Scan_Steps_Y_A, AA_ScanFinal_Scan_Delay_X_A, 0, 60, Target_IL, 500, 2, Region + "_Y Scan, Trip_");
    CMDOutput("%:");

    CMDOutput("AS");
    // Scan_AllRange_TwoWay(0, 8, AA_ScanFine_Scan_Steps_X_A, AA_ScanFinal_Scan_Delay_X_A, 0, 80, StopValue, 550, 2, Region + "_X Scan, Trip_");
    Scan_AllRange_TwoWay(0, 8, AA_ScanFine_Scan_Steps_X_A, AA_ScanFinal_Scan_Delay_X_A, 0, 60, Target_IL, 500, 2, Region + "_X Scan, Trip_");
    CMDOutput("%:");

    // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
    DataOutput(false);
  }

  //    return;

  PD_LV4 = Cal_PD_Input_IL(Get_PD_Points);
  time5 = millis();

  PDValue_Best = -2.45;

  if (isStop)
    return;

  //Scan(Final)
  Serial.println(" ");
  Serial.println("... XYZ_Scan(Final) ...");
  Region = "Scan(Final)";

  if (false && abs(PD_LV4 - PD_LV3) > 0.1)
  {
    CMDOutput("AS");
    Scan_AllRange_TwoWay(2, 8, AA_ScanFinal_Scan_Steps_Z_A, AA_ScanFinal_Scan_Delay_X_A, 0, 100, StopValue, 600, 2, "Z Scan, Trip_"); //steps:150
    CMDOutput("%:");

    // Fine_Scan(2, true);
    CMDOutput("AS");
    Scan_AllRange_TwoWay(1, 7, AA_ScanFinal_Scan_Steps_Y_A, AA_ScanFinal_Scan_Delay_X_A, 0, 120, StopValue, 600, 2, "Y Scan, Trip_"); //steps:350
    CMDOutput("%:");

    // Fine_Scan(1, true);
    CMDOutput("AS");
    Scan_AllRange_TwoWay(0, 8, AA_ScanFinal_Scan_Steps_X_A, AA_ScanFinal_Scan_Delay_X_A, 0, 120, StopValue, 600, 2, "X Scan, Trip_"); //steps:350
    CMDOutput("%:");

    // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
    DataOutput(false);
  }

  PD_LV5 = Cal_PD_Input_IL(Get_PD_Points);
  time6 = millis();

  digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

  if (isStop)
    return;

  isLCD = true;
  PageLevel = 0;
  updateUI(PageLevel);

  Serial.println(" ");
  Serial.println("Sprial(Rough)_TimeSpan:" + String((time2 - time1) / 1000) + "s,PD:" + String(PD_LV1));
  Serial.println("Sprial(Fine)_TimeSpan:" + String((time3 - time2) / 1000) + "s,PD:" + String(PD_LV2));
  Serial.println("Scan(Rough)_TimeSpan:" + String((time4 - time3) / 1000) + "s,PD:" + String(PD_LV3));
  Serial.println("Scan(Fine)_TimeSpan:" + String((time5 - time4) / 1000) + "s,PD:" + String(PD_LV4));
  Serial.println("Scan(Final)_TimeSpan:" + String((time6 - time5) / 1000) + "s,PD:" + String(PD_LV5));
  Serial.println("Auto_Align_TimeSpan:" + String((time6 - time1) / 1000) + "s");
  DataOutput(true);
}

//------------------------------------------------------------------------------------------------------------------------------------------

int matrix_edge;
int x = 0, y = 0;
double AutoAlign_Result[3] = {0, 0, 0};

bool AutoAlign_Spiral(int M, double StopValue, int stableDelay)
{
  // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
  DataOutput(false);
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

        return true;
      }
    }

    x--;

    if (isStop)
      return true;

    //To Left

    MotorCC = false;

    while (x >= (-n))
    {
      Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);

      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

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

    int nM = n;
    while (y <= (nM))
    {
      Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

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

    while (x <= (n))
    {
      Move_Motor(X_DIR_Pin, X_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

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

    while (y >= (-n))
    {
      Move_Motor(Y_DIR_Pin, Y_STP_Pin, MotorCC, MinMotroStep, delayBetweenStep, stableDelay);

      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      CMDOutput("$[" + String(x) + "," + String(y) + "]=" + PD_Now);

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

    double finalIL = Cal_PD_Input_IL(Get_PD_Points);
    Serial.println("Final IL : " + String(finalIL));

    Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));

    AutoAlign_Result[0] = PD_BestIL_Position[0];
    AutoAlign_Result[1] = PD_BestIL_Position[1];
    AutoAlign_Result[2] = finalIL;

    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));
  }
  else
  {
    Serial.println("Delta step out of range.");
  }
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
  // Serial.println(">>" + msg + String(trip)); //Trip_1

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

  delay(stableDelay + 100); //Trip_1 --------------------------------------------------------------------------------------

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
  // Serial.println("~" + msg + String(trip)); //Trip_2 --------------------------------------------------------------------------------------

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
    // Serial.println("~" + msg + String(trip)); //Trip_3 --------------------------------------------------------------------------------------

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
    // Serial.println("~" + msg + String(trip)); //Trip_4 --------------------------------------------------------------------------------------

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
double maxIL_in_FineScan = 0;
double minIL_in_FineScan = -100;

bool Scan_AllRange_TwoWay(int XYZ, int count, int motorStep, int stableDelay,
                          bool Direction, int delayBetweenStep, double StopPDValue, int Get_PD_Points, int Trips, String msg)
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

  MSGOutput("Scan Step: " + String(motorStep));
  MSGOutput("Backlash: " + String(backlash));
  CMDOutput(">>" + msg + String(trip));

  MSGOutput("StopValue:" + String(StopPDValue));

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points);
  MSGOutput("Initial PD: " + String(PD_initial));

  maxIL_in_FineScan = PD_initial;
  minIL_in_FineScan = PD_initial;

  if (PD_initial >= StopPDValue)
    return true;

  digitalWrite(DIR_Pin, MotorCC);
  delay(1);
  step(STP_Pin, motorStep * count, delayBetweenStep); //Jump to Trip_1 initial position----------------

  // Serial.println(String(STP_Pin) + "," + String(motorStep) + "," + String(count) + "," + String(delayBetweenStep));
  // return true;

  MotorCC = !MotorCC; //Reverse direction
  digitalWrite(DIR_Pin, MotorCC);
  delay(5);

  delay(stableDelay + 100); //--------------------------------Trip_1 -----------------------------------------------

  // CMDOutput(">>" + msg + String(trip));
  // Serial.println(">>" + msg + String(trip)); //Trip_1

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
    if (isStop)
      return true;

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

    //Update Min, Max IL in Scan Process
    if(PD_Value[i]>maxIL_in_FineScan)
        maxIL_in_FineScan=PD_Value[i];
    if(PD_Value[i]<minIL_in_FineScan)
        minIL_in_FineScan=PD_Value[i];

    DataOutput();
    DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

    if (PD_Value[i] >= StopPDValue)
    {
      MSGOutput("Better than StopValue");
      return true;
    }

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

        return true;
      }
    }

    else if (indexofBestIL != 0 && i == (dataCount - 1) && Pos_Best_Trip1 != Get_Position(XYZ))
    {
      MSGOutput("i:" + String(i) + ", Pos_Best_Trip1:" + String(Pos_Best_Trip1));
      double x[3];
      double y[3];
      for (int k = -1; k < 2; k++)
      {
        x[k + 1] = Step_Value[indexofBestIL + k]; //idex * step = real steps
        y[k + 1] = PD_Value[indexofBestIL + k];   //fill this with your sensor data
        Serial.println("Point : " + String(x[k + 1]) + " , " + String(y[k + 1]));
      }
      Pos_Best_Trip1 = Curfit(x, y, 3);
      MSGOutput("Best IL position in Trip_1 is: " + String(Pos_Best_Trip1));
    }
  }

  trip++;

  double IL_Best_Trip2 = 0;
  long Pos_Best_Trip2 = 0;
  long Pos_Ini_Trip2 = 0;

  //------------------------------------Trip_2 ------------------------------------------------------------
  MSGOutput(" --- Trip 2 --- " );
  MSGOutput("trip: " + String(trip) );
  MSGOutput("Trips: " + String(Trips) );
  
  if (trip == Trips)
  {
    CMDOutput("~:" + msg + String(trip));

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

      //Update Min, Max IL in Scan Process
      if(PD_Value[i]>maxIL_in_FineScan)
          maxIL_in_FineScan=PD_Value[i];
      if(PD_Value[i]<minIL_in_FineScan)
          minIL_in_FineScan=PD_Value[i];

      DataOutput();
      DataOutput(XYZ, PD_Value[i]); //int xyz, double pdValue

      if (PD_Value[i] >= StopPDValue)
      {
        MSGOutput("Better than StopValue");
        return true;
      }

      if (indexofBestIL != 0 && i == (dataCount - 1) && Pos_Best_Trip2 != Get_Position(XYZ))
      {
        double x[3];
        double y[3];
        for (int k = -1; k < 2; k++)
        {
          x[k + 1] = Step_Value[indexofBestIL + k]; //idex * step = real steps
          y[k + 1] = PD_Value[indexofBestIL + k];   //fill this with your sensor data
          MSGOutput("Point : " + String(x[k + 1]) + " , " + String(y[k + 1]));
        }
        Pos_Best_Trip2 = Curfit(x, y, 3);
        MSGOutput("Best IL position in Trip_2 is: " + String(Pos_Best_Trip2));
      }
    }
  }
  else
    trip--;

  trip++;
  CMDOutput("~:" + msg + String(trip));

  //------------------------------------Trip_3 -------------------------------------------------------

  MSGOutput(" --- Trip 3 --- " );

  double PD_Best = 0;
  int deltaPos = 0;

  if (IL_Best_Trip2 > IL_Best_Trip1 && (IL_Best_Trip2 - IL_Best_Trip1)> 0.25 && Trips == 2)
  {
    if (isStop)
    {
      return true;
    }

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    MSGOutput("Best in Trip_2 : " + String(Pos_Best_Trip2)); //------------Best in Trip_2----------------

    if (XYZ == 2)
      Pos_Best_Trip2 = Pos_Best_Trip2 - AQ_Scan_Compensation_Steps_Z_A;

    MSGOutput("Best in Trip_2 (Compensation) : " + String(Pos_Best_Trip2));

    PD_Best = IL_Best_Trip2;

    Move_Motor_abs(XYZ, Pos_Ini_Trip2); //Jump to Trip_2 start position

    delay(100); //100

    deltaPos = abs(Pos_Best_Trip2 - Get_Position(XYZ));

    if (deltaPos < backlash)
    {
      MSGOutput("Jump Backlesh 2");
      step(STP_Pin, (backlash - deltaPos), delayBetweenStep);
      delay(stableDelay + 400);

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

  else if (Trips == 1)
  {
    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    MSGOutput("Jump to Trip Initial Pos : " + String(Pos_Best_Trip2));
    Move_Motor_abs(XYZ, Pos_Ini_Trip1); //Jump to Trip_1 start position

    delay(300); //100

    PD_Now = Cal_PD_Input_IL(Get_PD_Points);
    DataOutput();
    DataOutput(XYZ, PD_Now); //int xyz, double pdValue

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(5);

    deltaPos = abs(Pos_Best_Trip1 - Get_Position(XYZ));
    MSGOutput("deltaPos : " + String(deltaPos));
  }

  else //------------Best in Trip_1----------------
  {
    MSGOutput("Best in Trip_1 : " + String(Pos_Best_Trip1));

    if (XYZ == 2)
      Pos_Best_Trip1 = Pos_Best_Trip1 - AQ_Scan_Compensation_Steps_Z_A;

    MSGOutput("Best in Trip_1 (Compensation) : " + String(Pos_Best_Trip1));

    PD_Best = IL_Best_Trip1;
    deltaPos = abs(Pos_Best_Trip1 - Get_Position(XYZ));

    if (deltaPos < motorStep * 2)
    {
      MSGOutput("Jump Backlesh 1");
     
      step(STP_Pin, (backlash), delayBetweenStep+20);
      delay(stableDelay + 200);

      deltaPos = abs(Pos_Best_Trip2 - Get_Position(XYZ)); //Two curves are totally different, then back to best pos in trip 2

//      return false;
    }

    if (isStop)
    {
      return true;
    }

    MotorCC = !MotorCC; //Reverse direction
    digitalWrite(DIR_Pin, MotorCC);
    delay(2);
  }


  MSGOutput("Delta Pos : " + String(deltaPos));

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
        MSGOutput("StopPDValue");
        break;
      }
      if (PD_Now >= PD_Best)
      {
        MSGOutput("PD_Best");
        break;
      }
    }
    else if (deltaPos > 0 && deltaPos < motorStep)
    {
      step(STP_Pin, deltaPos, delayBetweenStep);
      delay(stableDelay);
      PD_Now = Cal_PD_Input_IL(Get_PD_Points);
      DataOutput();
      DataOutput(XYZ, PD_Now); //int xyz, double pdValue

      break;
    }
    else if (deltaPos == 0)
    {
      break;
    }
    else
      break;
  }

  PD_Now = Cal_PD_Input_IL(2*Get_PD_Points);
  MSGOutput("Best IL: " + String(PD_Best));
  MSGOutput("Final IL: " + String(PD_Now));

  timer_2 = millis();
  double ts = (timer_2 - timer_1) * 0.001;
  CMDOutput("t:" + String(ts, 2));

  if (PD_Now < PD_Best - 1.2)
    return false;
  else
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

  CMDOutput(">>" + msg + String(trip)); //Trip_1------------------------------------------------------------

  double PD_Best = -64,
         PD_Trip2_Best = -64,
         PD_Now = -64;

  double PD_Value[count * 2];
  double PD_Rvrs_Value[count];
  int PD_Best_Pos = 0;
  int PD_Best_Pos_Trip2 = 0;

  double PD_initial = Cal_PD_Input_IL(Get_PD_Points);
  DataOutput(XYZ, PD_initial); //int xyz, double pdValue

  if (PD_initial >= StopPDValue)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    PD_Best = PD_initial;

    return PD_Best;
  }

  Move_Motor(DIR_Pin, STP_Pin, MotorCC, motorStep * 4, delayBetweenStep, 0, true);

  delay(stableDelay);

  PD_Now = Cal_PD_Input_IL(Get_PD_Points);
  DataOutput(XYZ, PD_Now); //int xyz, double pdValue

  Serial.println("Initial: " + String(PD_initial) + ", After:" + String(PD_Now));

  if (PD_Now >= StopPDValue)
  {
    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    PD_Best = PD_Now;

    return PD_Best;
  }

  PD_Best_Pos = 0;

  for (int i = 0; i < count; i++)
  {
    PD_Value[i] = 0;
  }

  if (abs(PD_Now - PD_initial) < 0.05)
  {
    Serial.println("Delta IL < 0.05");

    timer_2 = millis();
    ts = (timer_2 - timer_1) * 0.001;
    CMDOutput("t:" + String(ts, 2));

    return true;
  }

  bool isFirstPointPassBest = false;

  if (PD_Now >= PD_initial)
  {
    PD_Best = PD_Now;
    digitalWrite(DIR_Pin, MotorCC);
    delay(10);
    Serial.println("MotorCC_Forward");

    PD_Value[0] = Cal_PD_Input_IL(Get_PD_Points);
  }
  else
  {
    PD_Best = PD_initial;
    BackLash_Reverse(XYZ, MotorCC, stableDelay);
    Serial.println("MotorCC_Reverse");
    isReverse = true;
  }

  trip++;
  CMDOutput("~:" + msg + String(trip)); //Trip_2------------------------------------------------------------

  PD_Value[0] = Cal_PD_Input_IL(Get_PD_Points);

  int Plus_Times = 0;

  // int Trip2_Start_Position = 0;
  long Pos_Ini_Trip2 = Get_Position(XYZ);

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

      if (i == 0)
      {
        PD_Trip2_Best = PD_Value[i];
        PD_Best_Pos_Trip2 = Get_Position(XYZ);
      }

      if (PD_Value[i] >= StopPDValue) //Condition 1
      {
        Serial.println("Over StopPDValue");

        timer_2 = millis();
        ts = (timer_2 - timer_1) * 0.001;
        CMDOutput("t:" + String(ts, 2));

        Serial.println("");

        PD_Best = PD_Value[i];
        return PD_Best;
      }

      if (!isReverse && i == 0 && PD_Value[0] < PD_Best) //Condition 2
      {
        Serial.println("Forward_pass_best");
        return PD_Best;
      }

      if (PD_Value[i] > PD_Best)
      {
        PD_Best = PD_Value[i];
        PD_Best_Pos = i;
      }

      if (PD_Value[i] > PD_Trip2_Best)
      {
        PD_Trip2_Best = PD_Value[i];
        PD_Best_Pos_Trip2 = Get_Position(XYZ);
      }

      if (i >= 1 && PD_Value[i] < PD_Value[i - 1]) //Condition 3
      {
        if (abs(PD_Value[i - 1] - PD_Best) < 0.8)
        {
          Serial.println("Pass best IL");
          break;
        }
      }

      if (PD_Value[i] < -53) //Condition 4
      {
        Serial.println("Miss Target");
        break;
      }

      if (i == count - 1 && PD_Value[i] != PD_Best) //Back to best position after all steps run out
      {
        trip++;
        CMDOutput("~:" + msg + String(trip)); //Trip_3------------------------------------------------------------

        Move_Motor_abs(XYZ, Pos_Ini_Trip2); //Jump to Trip_2 start position
        delay(300);

        DataOutput(XYZ, Cal_PD_Input_IL(Get_PD_Points)); //int xyz, double pdValue

        Move_Motor_abs(XYZ, PD_Best_Pos_Trip2); //Jump to Trip_2 start position
        delay(300);

        DataOutput(XYZ, Cal_PD_Input_IL(Get_PD_Points)); //int xyz, double pdValue
        MSGOutput("Back to best position");
      }
      else if (i == count - 1 && PD_Value[i] == PD_Best) //未通過最高點
      {
        if (Plus_Times < 3)
        {
          MSGOutput("Plus three points");
          count = count + 3;
          Plus_Times++;
        }
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

    return PD_Best;
  }

  return PD_Best;

  //Back to best position
  // BackLash_Reverse(XYZ, MotorCC, stableDelay);
  // Serial.println("Final Reverse");

  // trip++;
  // CMDOutput("~:" + msg + String(trip));

  // double pv = 0;
  // double bv = 0;
  // for (int k = 0; k < 35; k++)
  // {
  //   if (isStop)
  //     return true;

  //   step(STP_Pin, motorStep / 4, delayBetweenStep);
  //   delay(stableDelay);

  //   pv = Cal_PD_Input_IL(Get_PD_Points);
  //   DataOutput(XYZ, pv); //int xyz, double pdValue

  //   Serial.println("Reverse PD: " + String(pv));

  //   if (pv >= StopPDValue)
  //   {
  //     Serial.println("Over StopPDValue");

  //     timer_2 = millis();
  //     ts = (timer_2 - timer_1) * 0.001;
  //     CMDOutput("t:" + String(ts, 2));

  //     PD_Best = pv;

  //     return PD_Best;
  //   }

  //   if (abs(pv - PD_Best) < 3 || pv > PD_Best || (pv < bv && pv > Threshold))
  //     break;
  //   else
  //     bv = pv;
  // }

  timer_2 = millis();
  ts = (timer_2 - timer_1) * 0.001;
  Serial.print("TS:");
  Serial.println(ts, 2);
  Serial.println(" ");
  return PD_Best;
}

//------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------------------

int Function_Classification(String cmd, int ButtonSelected)
{
  if (cmd != "" && ButtonSelected < 0)
  {
    cmd.trim();
    MSGOutput("get_cmd:" + String(cmd));

    String cmdUpper = cmd;
    cmdUpper.toUpperCase();

//Keyboard - Motor Control
#pragma region - Keyboard - Motor Control
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
      cmd_No = 104;
    }
    else if (cmd == "Ym1")
    {
      cmd_No = 106;
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

      Move_Motor(dirPin, stpPin, dirt, cmd.toDouble(), delayBetweenStep_Y, 0); //(dir_pin, stp_pin, direction, steps, delaybetweensteps, stabledelay)
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
    else if (Contains(cmd, "AbsAll_"))
    {
      cmd.remove(0, 7);

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

#pragma endregion

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
      int delayBetweenStep;
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

      delayBetweenStep = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //delaySteps
      cmd.remove(0, cmd.indexOf('_') + 1);

      StopPDValue = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //stopValue
      cmd.remove(0, cmd.indexOf('_') + 1);

      Get_PD_Points = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd.substring(0, cmd.indexOf('_'))); //average points
      cmd.remove(0, cmd.indexOf('_') + 1);

      Trips = cmd.substring(0, cmd.indexOf('_')).toInt();
      Serial.println(cmd); //trips

      // int delayBetweenStep = 50;
      String msg = "Manual_Fine_Scan_Trip_";

      bool isOK = true;

      CMDOutput("AS");

      isOK = Scan_AllRange_TwoWay(XYZ, count, motorStep, stableDelay,
                                  Direction, delayBetweenStep, StopPDValue, Get_PD_Points, Trips, msg);

      CMDOutput("%:");

      if (!isOK)
      {
        CMDOutput("AS");
        Scan_AllRange_TwoWay(XYZ, count, motorStep, stableDelay,
                             Direction, delayBetweenStep, StopPDValue, Get_PD_Points, Trips, msg);
        CMDOutput("%:");
      }

      MSGOutput("Auto_Align_End");
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

    //Set auto-align / auto-curing Parameter
    else if (Contains(cmd, "Set::"))
    {
      cmd.remove(0, 5);

      String ParaName = cmd.substring(0, cmd.indexOf('='));
      cmd.remove(0, cmd.indexOf('=') + 1);

      Serial.println("ParaName:" + ParaName + ", Value:" + String(cmd.toDouble()));

      if (ParaName == "AA_SpiralRough_Feed_Steps_Z_A")
        AA_SpiralRough_Feed_Steps_Z_A = cmd.toInt();
      else if (ParaName == "AA_SpiralRough_Spiral_Steps_XY_A")
        AA_SpiralRough_Spiral_Steps_XY_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Spiral_Steps_XY_A")
        AA_SpiralFine_Spiral_Steps_XY_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_A")
        AA_SpiralFine_Scan_Steps_X_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_B")
        AA_SpiralFine_Scan_Steps_X_B = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_C")
        AA_SpiralFine_Scan_Steps_X_C = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_X_D")
        AA_SpiralFine_Scan_Steps_X_D = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_A")
        AA_SpiralFine_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_B")
        AA_SpiralFine_Scan_Steps_Y_B = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_C")
        AA_SpiralFine_Scan_Steps_Y_C = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_D")
        AA_SpiralFine_Scan_Steps_Y_D = cmd.toInt();
      else if (ParaName == "AA_SpiralFine_Scan_Steps_Y_E")
        AA_SpiralFine_Scan_Steps_Y_E = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Feed_Steps_Z_A")
        AA_ScanRough_Feed_Steps_Z_A = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Feed_Steps_Z_B")
        AA_ScanRough_Feed_Steps_Z_B = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_A")
        AA_ScanRough_Feed_Ratio_Z_A = cmd.toDouble();
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_B")
        AA_ScanRough_Feed_Ratio_Z_B = cmd.toDouble();
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_C")
        AA_ScanRough_Feed_Ratio_Z_C = cmd.toDouble();
      else if (ParaName == "AA_ScanRough_Feed_Ratio_Z_D")
        AA_ScanRough_Feed_Ratio_Z_D = cmd.toDouble();

      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_A")
        AA_ScanRough_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_B")
        AA_ScanRough_Scan_Steps_Y_B = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_C")
        AA_ScanRough_Scan_Steps_Y_C = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_Y_D")
        AA_ScanRough_Scan_Steps_Y_D = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_A")
        AA_ScanRough_Scan_Steps_X_A = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_B")
        AA_ScanRough_Scan_Steps_X_B = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_C")
        AA_ScanRough_Scan_Steps_X_C = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_D")
        AA_ScanRough_Scan_Steps_X_D = cmd.toInt();
      else if (ParaName == "AA_ScanRough_Scan_Steps_X_E")
        AA_ScanRough_Scan_Steps_X_E = cmd.toInt();
      else if (ParaName == "AA_ScanFine_Scan_Steps_Z_A")
        AA_ScanFine_Scan_Steps_Z_A = cmd.toInt();
      else if (ParaName == "AA_ScanFine_Scan_Steps_Y_A")
        AA_ScanFine_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_ScanFine_Scan_Steps_X_A")
        AA_ScanFine_Scan_Steps_X_A = cmd.toInt();
      else if (ParaName == "AA_ScanFinal_Scan_Steps_Z_A")
        AA_ScanFinal_Scan_Steps_Z_A = cmd.toInt();
      else if (ParaName == "AA_ScanFinal_Scan_Steps_Y_A")
        AA_ScanFinal_Scan_Steps_Y_A = cmd.toInt();
      else if (ParaName == "AA_ScanFinal_Scan_Steps_X_A")
        AA_ScanFinal_Scan_Steps_X_A = cmd.toInt();

      else if (ParaName == "AQ_Scan_Compensation_Steps_Z_A")
      {
        AQ_Scan_Compensation_Steps_Z_A = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Compensation_Steps_Z_A: " + WR_EEPROM(160, cmd));
      }
      else if (ParaName == "AQ_Total_TimeSpan")
      {
        AQ_Total_TimeSpan = cmd.toInt();
        Serial.println("Write EEPROM AQ_Total_TimeSpan: " + WR_EEPROM(168, cmd));
      }

      else if (ParaName == "AA_ScanFinal_Scan_Delay_X_A")
      {
        AA_ScanFinal_Scan_Delay_X_A = cmd.toInt();
        Serial.println("Write EEPROM AA_ScanFinal_Scan_Delay_X_A: " + WR_EEPROM(80, cmd));
      }

      else if (ParaName == "AQ_Scan_Steps_Z_A")
      {
        AQ_Scan_Steps_Z_A = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_A: " + WR_EEPROM(176, cmd));
      }
      else if (ParaName == "AQ_Scan_Steps_Z_B")
      {
        AQ_Scan_Steps_Z_B = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_B: " + WR_EEPROM(184, cmd));
      }
      else if (ParaName == "AQ_Scan_Steps_Z_C")
      {
        AQ_Scan_Steps_Z_C = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_C: " + WR_EEPROM(192, cmd));
      }
      else if (ParaName == "AQ_Scan_Steps_Z_D")
      {
        AQ_Scan_Steps_Z_D = cmd.toInt();
        Serial.println("Write EEPROM AQ_Scan_Steps_Z_D: " + WR_EEPROM(200, cmd));
      }

      else if (ParaName == "FS_Count_X")
      {
        FS_Count_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Count_X: " + WR_EEPROM(EP_FS_Count_X, cmd));
      }
      else if (ParaName == "FS_Steps_X")
      {
        FS_Steps_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Steps_X: " + WR_EEPROM(EP_FS_Steps_X, cmd));
      }
      else if (ParaName == "FS_Stable_X")
      {
        FS_Stable_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Stable_X: " + WR_EEPROM(EP_FS_Stable_X, cmd));
      }
      else if (ParaName == "FS_DelaySteps_X")
      {
        FS_DelaySteps_X = cmd.toInt();
        Serial.println("Write EEPROM FS_DelaySteps_X: " + WR_EEPROM(EP_FS_DelaySteps_X, cmd));
      }
      else if (ParaName == "FS_Avg_X")
      {
        FS_Avg_X = cmd.toInt();
        Serial.println("Write EEPROM FS_Avg_X: " + WR_EEPROM(EP_FS_Avg_X, cmd));
      }

      else if (ParaName == "FS_Count_Y")
      {
        FS_Count_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Count_Y: " + WR_EEPROM(EP_FS_Count_Y, cmd));
      }
      else if (ParaName == "FS_Steps_Y")
      {
        FS_Steps_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Steps_Y: " + WR_EEPROM(EP_FS_Steps_Y, cmd));
      }
      else if (ParaName == "FS_Stable_Y")
      {
        FS_Stable_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Stable_Y: " + WR_EEPROM(EP_FS_Stable_Y, cmd));
      }
      else if (ParaName == "FS_DelaySteps_Y")
      {
        FS_DelaySteps_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_DelaySteps_Y: " + WR_EEPROM(EP_FS_DelaySteps_Y, cmd));
      }
      else if (ParaName == "FS_Avg_Y")
      {
        FS_Avg_Y = cmd.toInt();
        Serial.println("Write EEPROM FS_Avg_Y: " + WR_EEPROM(EP_FS_Avg_Y, cmd));
      }

      else if (ParaName == "FS_Count_Z")
      {
        FS_Count_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Count_Z: " + WR_EEPROM(EP_FS_Count_Z, cmd));
      }
      else if (ParaName == "FS_Steps_Z")
      {
        FS_Steps_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Steps_Z: " + WR_EEPROM(EP_FS_Steps_Z, cmd));
      }
      else if (ParaName == "FS_Stable_Z")
      {
        FS_Stable_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Stable_Z: " + WR_EEPROM(EP_FS_Stable_Z, cmd));
      }
      else if (ParaName == "FS_DelaySteps_Z")
      {
        FS_DelaySteps_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_DelaySteps_Z: " + WR_EEPROM(EP_FS_DelaySteps_Z, cmd));
      }
      else if (ParaName == "FS_Avg_Z")
      {
        FS_Avg_Z = cmd.toInt();
        Serial.println("Write EEPROM FS_Avg_Z: " + WR_EEPROM(EP_FS_Avg_Z, cmd));
      }
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

        EEPROM.commit();

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

        EEPROM.commit();

        Serial.println("Set Y BackLash: " + String(String(cmd)));

        Serial.println("Y BackLash in eeprom: " + ReadInfoEEPROM(32, 8)); //(start_position, data_length)

        Y_backlash = ReadInfoEEPROM(32, 8).toInt();
      }

      else if (Contains(cmd, "Z"))
      {
        cmd.remove(0, 5);

        Z_backlash = cmd.toInt();

        CleanEEPROM(40, 8); //Clean EEPROM(int startPosition, int datalength)

        WriteInfoEEPROM(String(cmd), 40); //(data, start_position)  // Write Data to EEPROM

        EEPROM.commit();

        Serial.println("Set Z BackLash: " + String(String(cmd)));

        Serial.println("Z BackLash in eeprom: " + ReadInfoEEPROM(40, 8)); //(start_position, data_length)

        Z_backlash = ReadInfoEEPROM(40, 8).toInt();
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

    //Get Ref Command
    else if (cmd == "REF?")
    {
      String eepromString = ReadInfoEEPROM(0, 8); //(int start_position, int data_length)

      Serial.println("Get_Ref:" + eepromString); //(start_position, data_length)  // Reading Data from EEPROM

      ref_Dac = eepromString.toDouble();
      ref_IL = ILConverter(ref_Dac);

      Serial.println("Dac:" + eepromString);  //(start_position, data_length)  // Reading Data from EEPROM
      Serial.println("IL:" + String(ref_IL)); //(start_position, data_length)  // Reading Data from EEPROM
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

    //Set Target IL
    else if (Contains(cmd, "Set_Target_IL:"))
    {
      cmd.remove(0, 14);
      WR_EEPROM(72, cmd);
      Target_IL = ReadInfoEEPROM(72, 8).toDouble();
      MSGOutput("Set_Target_IL:" + String(Target_IL));
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

    //Set Board ID
    else if (Contains(cmd, "ID#"))
    {
      cmd.remove(0, 3);
      Serial.println("Set Board ID: " + WR_EEPROM(8, cmd));
    }

    //Get Board ID
    else if (cmd == "ID?")
    {
      Serial.println(ReadInfoEEPROM(8, 8));
    }

    //Set Station ID
    else if (Contains(cmd, "ID_Station#"))
    {
      cmd.remove(0, 11);
      Serial.println("Set Station ID: " + WR_EEPROM(16, cmd));
    }

    //Get Station ID
    else if (cmd == "ID_Station?")
    {
      Serial.println(ReadInfoEEPROM(16, 8));
    }

    //Set Server ID
    else if (Contains(cmd, "ID_Server#"))
    {
      cmd.remove(0, 10);
      Serial.println("Set Server ID: " + WR_EEPROM(88, 32, cmd));
    }

    //Get Server ID
    else if (cmd == "ID_Server?")
    {
      Serial.println(ReadInfoEEPROM(88, 32));
    }

    //Set Server Password
    else if (Contains(cmd, "PW_Server#"))
    {
      cmd.remove(0, 10);
      Serial.println("Set Server Password: " + WR_EEPROM(120, 32, cmd));
    }

    //Get Server Password
    else if (cmd == "PW_Server?")
    {
      Serial.println(ReadInfoEEPROM(120, 32));
    }

    //Clena EEPROM : Start position (default length = 8)
    else if (Contains(cmd, "Clean_EEPROM:"))
    {
      cmd.remove(0, 13);
      CleanEEPROM(cmd.toInt(), 8);
      WR_EEPROM(cmd.toInt(), "");
      EEPROM.commit();
      Serial.println("Clean_EEPROM:" + cmd);
      // Serial.println(ReadInfoEEPROM(cmd.toInt(), 8));
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

    //Action : Reply
    if (true)
    {
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
  }

  return cmd_No;
}

//------------------------------------------------------------------------------------------------------------------------------------------

int Function_Excecutation(String cmd, int cmd_No)
{
  //Function Execution
  // String cmd = "";

  if (cmd_No != 0)
  {
    // Serial.println("Btn:" + String(ButtonSelected) + ", CMD:" + String(cmd_No));

    //Functions: Alignment
    if (cmd_No <= 100)
    {
      switch (cmd_No)
      {
        //Functions: Auto Align
      case 1: /* Auto Align */
        if (true)
        {
          AQ_Scan_Compensation_Steps_Z_A = 0;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(3);

          AutoAlign();

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
          Serial.println("Auto Align End");
          MotorCC = true;

          StopValue = 0; //0 dB

          AQ_Scan_Compensation_Steps_Z_A = ReadInfoEEPROM(160, 8).toInt();
          Serial.println("AQ_Scan_Compensation_Steps_Z_A: " + String(AQ_Scan_Compensation_Steps_Z_A));

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);
          // updateUI(0);
        }
        cmd_No = 0;
        break;

        //Functions: Fine Scan
      case 2: /* Fine Scan */
        if (!btn_isTrigger)
        {
          StopValue = 0; //0 dB

          bool K_OK = true;

          isLCD = true;
          PageLevel = 102;
          updateUI(PageLevel);
          AQ_Scan_Compensation_Steps_Z_A = 0;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode

          //------------------------------------------------------------------------------------------------------------------------------Q Scan Z

          // CMDOutput("AS");

          // Scan_AllRange_TwoWay(2, 6, 100, AA_ScanFinal_Scan_Delay_X_A, 0, 80, StopValue, 600, 2, "Z Scan, Trip_"); //--Z--
          Fine_Scan(3, false); 

          CMDOutput("%:");

          if (isStop)
            true;

          //------------------------------------------------------------------------------------------------------------------------------Q Scan Y

          // CMDOutput("AS");
          // // K_OK = Scan_AllRange_TwoWay(1, 7, 20, AA_ScanFinal_Scan_Delay_X_A, 0, 120, StopValue, 500, 2, "Y Scan, Trip_"); //Fast
          // K_OK = Scan_AllRange_TwoWay(1, 6, 35, AA_ScanFinal_Scan_Delay_Y_A, 0, 100, StopValue, 600, 2, "Y Scan, Trip_"); //Slow
          // CMDOutput("%:");

          // if (!K_OK)
          // {
          //   CMDOutput("AS");
          //   // K_OK = Scan_AllRange_TwoWay(1, 7, 20, AA_ScanFinal_Scan_Delay_X_A, 0, 120, StopValue, 500, 2, "Y Scan, Trip_");//Fast
          //   K_OK = Scan_AllRange_TwoWay(1, 6, 35, AA_ScanFinal_Scan_Delay_Y_A, 0, 100, StopValue, 600, 2, "Y Scan, Trip_"); //Slow
          //   CMDOutput("%:");
          // }

           Fine_Scan(2, false); 
          CMDOutput("%:");

          if (isStop)
            true;

          //------------------------------------------------------------------------------------------------------------------------------Q Scan X

          CMDOutput("AS");

          // Scan_AllRange_TwoWay(0, 8, 22, AA_ScanFinal_Scan_Delay_X_A, 0, 120, StopValue, 500, 2, "X Scan, Trip_"); //steps:350
          Fine_Scan(1, false); 
          CMDOutput("%:");

          if (isStop)
            true;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

          MSGOutput("Auto Align End");

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);
          AQ_Scan_Compensation_Steps_Z_A = ReadInfoEEPROM(160, 8).toInt();
        }
        cmd_No = 0;
        break;

      //Functions: Auto Curing
      case 3: /* Auto Curing */
        if (!btn_isTrigger)
        {
          btn_isTrigger = false;

          isILStable = false;

          ButtonSelected = -1;

          double IL_stable_count = 0;
          double Acceptable_Delta_IL = 12; //0.8
          Q_Time = 0;

          time_curing_0 = millis();
          time_curing_1 = time_curing_0;
          time_curing_2 = time_curing_1;

          digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
          delay(5);

          AutoCuring_Best_IL = Cal_PD_Input_IL(Get_PD_Points * 2);

          StopValue = AutoCuring_Best_IL;

          Z_ScanSTP = AQ_Scan_Steps_Z_A; //125 (AQ_Scan_Steps_Z_A)
          MSGOutput("Auto-Curing");
          MSGOutput("StopValue : " + String(StopValue));
          CMDOutput("AQ");                             // Auto_Curing Start
          CMDOutput("QT" + String(AQ_Total_TimeSpan)); // Auto_Curing Start

          while (true)
          {
            PD_Now = Cal_PD_Input_IL(Get_PD_Points);
            Q_Time = ((millis() - time_curing_0) / 1000);
            MSGOutput("Curing Time:" + String(Q_Time) + " s");
            // MSGOutput("Threshold: " + String(AutoCuring_Best_IL - Acceptable_Delta_IL) + ", Now: " + String(PD_Now));
            MSGOutput("PD_Power:" + String(PD_Now)); //dB

            digitalWrite(Tablet_PD_mode_Trigger_Pin, false); //false is PD mode, true is Servo mode
            delay(5);

            if (Serial.available())
              cmd = Serial.readString();

            // MSGOutput("ButtonSelected: " + String(ButtonSelected));

            // MSGOutput("cmd in Q loop: " + String(cmd));

            cmd_No = Function_Classification(cmd, ButtonSelected);

            cmd = ""; //Reset command from serial port

            isLCD = true;
            PageLevel = 103;
            updateUI(103);
            delay(1000); //default: 700

            if (isStop)
              break;

            //Q State
            if (true)
            {
              if (Q_Time <= 540)
              {
                Q_State = 1;
              }
              else if (Q_Time > 540 && Q_Time <= 600)
              {
                Q_State = 2;
              }
              else if (Q_Time > 600 && Q_Time <= 700)
              {
                Q_State = 3;
              }
              else if (Q_Time > 700)
              {
                Q_State = 4;
              }
            }

            //Q Stop Conditions
            if (true)
            {
              //IL Stable Time ,  70 secs,  curing time threshold , 12.5 mins
              if (time_curing_2 - time_curing_1 > 70000 && Q_Time >= 800) // 800
              {
                MSGOutput("IL Stable - Stop Auto Curing");
                isStop = true;
                break;
              }
              //Total curing time , 14 mins, 840s
              else if (Q_Time >= AQ_Total_TimeSpan - 1)
              {
                MSGOutput("Over Limit Curing Time - Stop Auto Curing");
                isStop = true;
                break;
              }

              if (isILStable && (Q_Time) >= 800) //800
              {
                MSGOutput("IL Stable in Scan - Stop Auto Curing");
                break;
              }
            }

            if (isStop)
              break;

            //Q scan conditions
            if (true)
            {
              if (Q_State == 2)
              {
                if (Z_ScanSTP > AQ_Scan_Steps_Z_B)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_B; //125 - > 35 (AQ_Scan_Steps_Z_B)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }
              else if (Q_State == 3)
              {
                if (Z_ScanSTP > AQ_Scan_Steps_Z_C)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_C; //70 (AQ_Scan_Steps_Z_C)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }
              else if (Q_State == 4)
              {
                if (Z_ScanSTP > AQ_Scan_Steps_Z_D)
                {
                  Z_ScanSTP = AQ_Scan_Steps_Z_D; //50 (AQ_Scan_Steps_Z_D)
                  MSGOutput("Update Z Scan Step: " + String(Z_ScanSTP));
                }
              }

              if (Q_Time > 540)
              {
                if( Acceptable_Delta_IL != 0.2 ){
                  Acceptable_Delta_IL = 0.2; // Target IL changed 0.25
                  MSGOutput("Update Scan Condition: " + String(Acceptable_Delta_IL));     
                }
              }
            }

            PD_Now = Cal_PD_Input_IL(Get_PD_Points * 4);  //Increase IL stability

            if (PD_Now >= (AutoCuring_Best_IL - (Acceptable_Delta_IL)))
            {
              time_curing_2 = millis();
              continue;
            }
            else
            {
              time_curing_3 = millis();
              Q_Time = (time_curing_3 - time_curing_0) / 1000;
              MSGOutput("Auto-Curing Time: " + String(Q_Time) + " s");

              //Q Scan
              if (true)
              {
                PD_Now = Cal_PD_Input_IL(Get_PD_Points * 4);  //Increase IL stability

                if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL))
                {
                  Fine_Scan(1, false); //Q Scan X

                  MSGOutput("X PD_Now:" + String(PD_Now) + ", IL:" + String(Cal_PD_Input_IL(Get_PD_Points)));

                  if (PD_Now - Cal_PD_Input_IL(Get_PD_Points) > 1)
                    Fine_Scan(1, false); //Q Scan X

                  if (Q_State >= 4 && (maxIL_in_FineScan - minIL_in_FineScan)<0.5){
                    MSGOutput("Delta IL less than 0.5 , break curing loop");
                    MSGOutput("X maxIL_in_FineScan:" + String(maxIL_in_FineScan) 
                    + ", minIL_in_FineScan:" + String(minIL_in_FineScan));
                    break;
                  }
                }

                time_curing_3 = millis();
                Q_Time = (time_curing_3 - time_curing_0) / 1000;
                MSGOutput("Auto-Curing Time: " + String(Q_Time) + " s");

                if (isStop)
                  break;

                PD_Now = Cal_PD_Input_IL(Get_PD_Points * 4);  //Increase IL stability
                MSGOutput("Q_State: " + String(Q_State));

                if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL) || Q_State == 1)
                {
                  Fine_Scan(2, false); //--------------------------------------------------------Q Scan Y

                  MSGOutput("Y PD_Now:" + String(PD_Now) + ", IL:" + String(Cal_PD_Input_IL(Get_PD_Points)));

                  if (PD_Now - Cal_PD_Input_IL(Get_PD_Points) > 1)
                    Fine_Scan(2, false); //------------------------------------------------------Q Scan Y

                  if (Q_State >= 4 && (maxIL_in_FineScan - minIL_in_FineScan)<0.5){
                    MSGOutput("Delta IL less than 0.5 , break curing loop");
                    MSGOutput("Y maxIL_in_FineScan:" + String(maxIL_in_FineScan) 
                    + ", minIL_in_FineScan:" + String(minIL_in_FineScan));
                    break;
                  }
                }

                time_curing_3 = millis();
                Q_Time = (time_curing_3 - time_curing_0) / 1000;
                MSGOutput("Auto-Curing Time: " + String(Q_Time) + " s");

                if (isStop)
                  break;

                PD_Before = Cal_PD_Input_IL(Get_PD_Points);

                bool K_OK = true;

                if (PD_Now < (AutoCuring_Best_IL - Acceptable_Delta_IL))
                {
                  //-----------------------------------------------------------Q Scan Z
                  CMDOutput("AS");
                  K_OK = Scan_AllRange_TwoWay(2, 8, Z_ScanSTP, 70, 0, 120, StopValue, 600, 2, "Z Scan, Trip_");
                  if (!K_OK)
                  {
                    Scan_AllRange_TwoWay(2, 8, Z_ScanSTP, 70, 0, 120, StopValue, 600, 2, "Z Re-Scan, Trip_");
                  }
                  
                  CMDOutput("%:");
                }

                if (isStop)
                  break;
              }

              PD_Now = Cal_PD_Input_IL(Get_PD_Points);
              MSGOutput("Q_State: " + String(Q_State));

              if (abs(PD_Before - PD_Now) < 0.3 && (time_curing_3 - time_curing_0) > 750000)
              {
                IL_stable_count++;

                if (IL_stable_count > 4)
                {
                  MSGOutput("IL stable to break");
                  break;
                }
              }

              time_curing_1 = millis();
              time_curing_2 = time_curing_1;
            }
          }

          time_curing_3 = millis();
          MSGOutput("Total Auto-Curing Time: " + String((time_curing_3 - time_curing_0) / 1000) + " s");

          StopValue = Target_IL;
          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode

          String eepromString = ReadInfoEEPROM(40, 8);                         //Reading z backlash from EEPROM
          MSGOutput("Reset Z backlash from EEPROM: " + ReadInfoEEPROM(40, 8)); //(start_position, data_length)
          Z_backlash = eepromString.toInt();

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);
          MSGOutput("LCD Re-Start");

          MSGOutput("Auto Q End");

          Q_Time = 0;
        }
        cmd_No = 0;
        break;

      case 5: /* Fine Scan X */
        if (!btn_isTrigger){
          isLCD = true;
          PageLevel = 102;
          updateUI(PageLevel);

          Fine_Scan(1, false); 

          MSGOutput("Auto Align End");

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);
        }
        cmd_No = 0;
        break;

      case 6: /* Fine Scan Y */
        if (!btn_isTrigger){
          isLCD = true;
          PageLevel = 102;
          updateUI(PageLevel);

          Fine_Scan(2, false); 

          MSGOutput("Auto Align End");

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);
        }
        cmd_No = 0;
        break;

      case 7: /* Fine Scan Z */
        if (!btn_isTrigger){
          isLCD = true;
          PageLevel = 102;
          updateUI(PageLevel);

          Fine_Scan(3, false); 

          MSGOutput("Auto Align End");

          isLCD = true;
          PageLevel = 0;
          updateUI(PageLevel);
        }
        cmd_No = 0;
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

          Serial.println("Ref_Dac: " + ReadInfoEEPROM(0, 8) + ", Ref_IL: " + String(ref_IL)); //Reading Data from EEPROM(start_position, data_length)

          MSGOutput("EEPROM(" + String(0) + ") - " + ReadInfoEEPROM(0, 8)); // For update HMI ref value

          digitalWrite(Tablet_PD_mode_Trigger_Pin, true); //false is PD mode, true is Servo mode
          delay(3);

          isLCD = true;
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

      case 29: /* Get XYZ Position */
        DataOutput(false);
        cmd_No = 0;
        break;

      case 31:
        isLCD = true;
        LCD_Update_Mode = 100;
        Serial.println("LCD Re-Start");
        cmd_No = 0;
        break;

      case 51: /* Get ID */
        Serial.println(ReadInfoEEPROM(8, 8));
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

        //Y- feed - cont
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

      //Y+ feed - cont
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

  return cmd_No;
}

//------------------------------------------------------------------------------------------------------------------------------------------

void BLE_Function(String cmd)
{
  //Bluetooth : Receive Data
  // if (cmd == "" && cmd_No == 0)
  // {
  //   // Serial.println("BLE mode");
  //   if (BT.connected(30))
  //   {
  //     isMsgShow = true;

  //     if (BT.available())
  //     {
  //       String BTdata = BT.readString();

  //       BT.println(BTdata);
  //       Serial.println(BTdata);

  //       if (BTdata == "Z+")
  //       {
  //         Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, 500, 8, 150, true);
  //       }
  //       else if (BTdata == "Z-")
  //       {
  //         Move_Motor(Z_DIR_Pin, Z_STP_Pin, true, 500, 8, 150, true);
  //       }
  //     }
  //   }
  // }

  // if (ButtonSelected < 0 && cmd == "")
  // {
  //   cmd_No = 0;
  // }
}

//------------------------------------------------------------------------------------------------------------------------------------------

// bool isWiFiConnected = false;
void CMDOutput(String cmd)
{
  String msg = "CMD::" + cmd;
  Serial.println(msg);

  if (isWiFiConnected)
  {
    httpTestRequest(ServerIP.c_str(), msg.c_str());
  }

  // Check WiFi connection status
  // if (WiFi.status() == WL_CONNECTED)
  // {
  //   // Data = ServerIP + "?" + ID + "=" + msg;
  //   // httpTestRequest(Data.c_str()); //Send message to server
  //   httpTestRequest(ServerIP.c_str(), msg.c_str());

  //   isWiFiConnected = true;
  // }
  // else
  // {
  //   if (isWiFiConnected)
  //   {
  //     Serial.println("WiFi Disconnected");
  //     isWiFiConnected = false;
  //   }
  // }
}

void DataOutput()
{
  double IL = Cal_PD_Input_IL(1);
  Serial.println("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now) + "," + String(IL));
  // MSGOutput("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now) + "," + String(IL));
}

void DataOutput(bool isIL)
{
  if (isIL)
  {
    double IL = Cal_PD_Input_IL(1);
    // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now) + ", " + String(IL));
    MSGOutput("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now) + "," + String(IL));
  }
  else
    // Serial.println("Position : " + String(X_Pos_Now) + ", " + String(Y_Pos_Now) + ", " + String(Z_Pos_Now));
    MSGOutput("Position:" + String(X_Pos_Now) + "," + String(Y_Pos_Now) + "," + String(Z_Pos_Now));
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

void MSGOutput(String msg)
{
  Serial.println(msg);

  // Check WiFi connection status
  if (isWiFiConnected)
  {
    httpTestRequest(ServerIP.c_str(), msg.c_str());
  }
  // if (WiFi.status() == WL_CONNECTED)
  // {
  //   // Data = ServerIP + "?" + ID + "=" + msg;
  //   // httpTestRequest(Data.c_str()); //Send message to server

  //   httpTestRequest(ServerIP.c_str(), msg.c_str());

  //   isWiFiConnected = true;
  // }
  // else
  // {
  //   if (isWiFiConnected)
  //   {
  //     Serial.println("WiFi Disconnected");
  //     isWiFiConnected = false;
  //   }
  // }
}
