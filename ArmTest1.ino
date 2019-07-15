#include <ax12.h>
#include <BioloidController.h>
#include <avr/pgmspace.h>

BioloidController bioloid = BioloidController(1000000);

const PROGMEM uint16_t Center[] = {6, 2048, 2048, 2048, 2048, 512, 512};
const PROGMEM uint16_t Home[] = {6, 2033, 1698, 1448, 2336, 512, 512};

#define SERVOCOUNT 6 // Количество моторов
#define SERVO_SPD  5 // Скорость моторов

static volatile int id = 1;
static volatile int pos = 0;
static volatile boolean IDCheck = 1;
static volatile boolean RunCheck = 0;

void setup()
{
  // Инициализация COM порта
  pinMode(0, OUTPUT);
  Serial.begin(115200);
  delay(500);
  Serial.println("Serial Communication Established");
  Serial.println();
  CheckVoltage();
  ScanServo();
  MoveCenter();
  ServoTest1();
  ServoTest2();
  ServoTest3();
  ServoTest4();
  ServoTest5();
  ServoTest6();
  MoveHome();
  MenuOptions();
  RunCheck = 1;
}

void loop()
{
  int inByte = Serial.read();
  switch (inByte) {
    case '1':
      ScanServo();
      break;
    case '2':
      CheckVoltage();
      break;
    case '3':
      LEDTest();
      break;
    case '4':
      ServoTest1();
      break;
    case '5':
      ServoTest2();
      break;
    case '6':
      ServoTest3();
      break;
    case '7':
      ServoTest4();
      break;
    case '8':
      ServoTest5();
      break;
    case '9':
      ServoTest6();
      break;
    case '0':
      RelaxServos();
      break;
    case '-':
      MoveCenter();
      break;
    case '=':
      MoveHome();
      break;
  }
}

// Меню
void MenuOptions()
{
  Serial.println();
  Serial.println("Please enter option 1-5 to run individual tests again.");
  Serial.println("1) Servo Scanning Test");
  Serial.println("2) Check System Voltage");
  Serial.println("3) Perform LED Test");
  Serial.println("4) Perform servo #1 test");
  Serial.println("5) Perform servo #2 test");
  Serial.println("6) Perform servo #3 test");
  Serial.println("7) Perform servo #4 test");
  Serial.println("8) Perform servo #5 test");
  Serial.println("9) Perform servo #6 test");
  Serial.println("0) Relax Servos");
  Serial.println("-) Move Servos to Center");
  Serial.println("=) Move Servos to Home");
  Serial.println();
}


// Сканирование сервомоторов
void ScanServo()
{
  id = 1;
  Serial.println("Starting Servo Scanning Test...");
  while (id <= SERVOCOUNT)
  {
    pos =  ax12GetRegister(id, 36, 2);
    Serial.print("Servo ID: ");
    Serial.println(id);
    Serial.print("Servo Position: ");
    Serial.println(pos);
    if (pos <= 0)
    {
      Serial.println("ERROR! Servo ID: ");
      Serial.print(id);
      Serial.println(" not found. Please check connection and verify correct ID is set!");
      Serial.println();
      IDCheck = 0;
    }
    id = id + 1;
    delay(250);
  }
  if (IDCheck == 0)
  {
    Serial.println("ERROR! Servo ID(s) are missing from Scan. Please check connection and verify correct ID is set!");
    Serial.println();
  }
  else
  {
    Serial.println("All servo IDs present");
    Serial.println();
  }
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}

// Измерение напряжения питания
void CheckVoltage()
{
  float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
  Serial.println("System Voltage: ");
  Serial.print(voltage);
  Serial.println (" V");
  if (voltage < 10.0)
  {
    Serial.println("Voltage levels below 10v, please charge battery!");
    Serial.println();
    while (1);
  }
  if (voltage > 10.0)
  {
    Serial.println("Voltage levels nominal");
    Serial.println();
  }
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}

// Установка центральных позиций моторов
void MoveCenter()
{
  delay(100);                 // Рекомендованная пауза
  bioloid.loadPose(Center);   // Загрузка позиций из EEPROM
  bioloid.readPose();         // Чтение текущих позиций в буфер
  Serial.println("Moving servos to centered position...");
  Serial.println();
  delay(1000);
  bioloid.interpolateSetup(1000);
  // Двигаем моторы, пока не достигнут нужной позиции
  while (bioloid.interpolating > 0)
  {
    bioloid.interpolateStep();
    delay(SERVO_SPD);
  }
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}

// Возврат моторов в начальное положение
void MoveHome()
{
  delay(100);             // Рекомендованная пауза
  bioloid.loadPose(Home); // Загрузка позиций из EEPROM
  bioloid.readPose();     // Чтение текущих позиций в буфер
  Serial.println("Moving servos to Home position...");
  Serial.println();
  delay(1000);
  bioloid.interpolateSetup(1000);
  // Двигаем моторы, пока не достигнут нужной позиции
  while (bioloid.interpolating > 0)
  {
    bioloid.interpolateStep();
    delay(SERVO_SPD);
  }
  if (RunCheck == 1) {
    MenuOptions();
  }
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}

// Тестирование мотора #1
void ServoTest1()
{
  Serial.println("Servo #1 test");
  Serial.println();
  delay(500);
  pos = 2048;
  // Base Servo Test
  Serial.println("Moving Servo ID: 1");
  Serial.println();
  while (pos >= 1500)
  {
    SetPosition(1, pos);
    pos = pos - 1;
    delay(SERVO_SPD);
  }
  while (pos <= 2048)
  {
    SetPosition(1, pos);
    pos = pos + 1;
    delay(SERVO_SPD);
  }
  delay(500);
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}

// Тестирование мотора #2
void ServoTest2()
{
  Serial.println("Servo #2 test");
  Serial.println();
  delay(500);
  pos = 2048;
  // Shoulder Servo Test
  Serial.println("Moving Servo ID: 2 ");
  Serial.println();
  while (pos >= 1500)
  {
    SetPosition(2, pos);
    pos = pos - 1;
    delay(SERVO_SPD);
  }
  while (pos <= 2048)
  {
    SetPosition(2, pos);
    pos = pos + 1;
    delay(SERVO_SPD);
  }
  delay(500);
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}

// Тестирование мотора #3
void ServoTest3()
{
  Serial.println("Servo #3 test");
  Serial.println();
  delay(500);
  pos = 2048;
  // Elbow Servo Test
  Serial.println("Moving Servo ID: 3 ");
  Serial.println();
  while (pos <= 2400)
  {
    SetPosition(3, pos);
    pos = pos + 1;
    delay(SERVO_SPD);
  }
  while (pos >= 2048)
  {
    SetPosition(3, pos);
    pos = pos - 1;
    delay(SERVO_SPD);
  }
  delay(500);
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}

// Тестирование мотора #4
void ServoTest4()
{
  Serial.println("Servo #4 test");
  Serial.println();
  delay(500);
  pos = 2048;
  // Wrist Servo Test
  Serial.println("Moving Servo ID: 4");
  Serial.println();
  while (pos <= 2500)
  {
    SetPosition(4, pos);
    pos = pos + 1;
    delay(SERVO_SPD);
  }
  while (pos >= 2048)
  {
    SetPosition(4, pos);
    pos = pos - 1;
    delay(SERVO_SPD);
  }
  delay(500);
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}

// Тестирование мотора #5
void ServoTest5()
{
  Serial.println("Servo #5 test");
  Serial.println();
  delay(500);
  pos = 512;
  // Wrist Rotate Servo Test
  Serial.println("Moving Servo ID: 5");
  Serial.println();
  while (pos >= 312)
  {
    SetPosition(5, pos);
    pos = pos - 1;
    delay(SERVO_SPD);
  }
  while (pos <= 512)
  {
    SetPosition(5, pos);
    pos = pos + 1;
    delay(SERVO_SPD);
  }
  delay(500);
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}

// Тестирование мотора #6
void ServoTest6()
{
  Serial.println("Servo #6 test");
  Serial.println();
  delay(500);
  pos = 512;
  // Gripper Servo Test
  Serial.println("Moving Servo ID: 6");
  Serial.println();
  while (pos >= 312)
  {
    SetPosition(6, pos);
    pos = pos - 1;
    delay(SERVO_SPD);
  }
  while (pos <= 512)
  {
    SetPosition(6, pos);
    pos = pos + 1;
    delay(SERVO_SPD);
  }
  delay(500);
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}

// Отпускание моторов
void RelaxServos()
{
  id = 1;
  Serial.println("Relaxing Servos...");
  Serial.println();
  while (id <= SERVOCOUNT)
  {
    Relax(id);
    id = id + 1;
    delay(50);
  }
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}

// Тест светодиодов на моторах
void LEDTest()
{
  id = 1;
  Serial.println("Running LED Test...");
  Serial.println();
  while (id <= SERVOCOUNT)
  {
    ax12SetRegister(id, 25, 1);
    Serial.print("LED ON - Servo ID: ");
    Serial.println(id);
    delay(3000);
    ax12SetRegister(id, 25, 0);
    Serial.print("LED OFF - Servo ID: ");
    Serial.println(id);
    delay(3000);
    id = id + 1;
  }
  if (RunCheck == 1)
  {
    MenuOptions();
  }
}
