#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- Pins ---
const int btn1 = 2; 
const int btn2 = 7; 
const int pinR = 3; 
const int pinG = 5; 
const int pinB = 6;
const int pinLDR = A2;

// --- State Machine ---
enum AppState { WELCOME, MAIN_MENU, CALIB_SUB, CALIB_RUNNING, DETECTING, RAW_DATA, RESET_CONFIRM };
AppState currentState = WELCOME;

// --- Variables ---
int menuIdx = 0;
int calibIdx = 0;   // 0: Black, 1: White
int rawPageIdx = 0; 
unsigned long stateTimer = 0;
char detectedColorName[12];

// --- Screensaver ---
unsigned long lastActivity = 0;
const unsigned long SLEEP_TIME = 20000; // 20 seconds
bool isSleeping = false;

// Button Timing
unsigned long btnTimer[2] = {0, 0};
bool btnActive[2] = {false, false};
const int LONG_PRESS_MS = 500; // 500 ms long press duration

// Data Structures
struct CalibData { int r, g, b; };
CalibData blackRef, whiteRef;
struct SensorValues { int r, g, b; int rawR, rawG, rawB; };
SensorValues currentScan;

// KNN Database (PROGMEM)
struct ColorEntry { char name[12]; byte r, g, b; };
const ColorEntry colorDB[] PROGMEM = {
  {"Red", 255, 0, 0}, {"Green", 0, 255, 0}, {"Blue", 0, 0, 255},
  {"Yellow", 255, 255, 0}, {"Cyan", 0, 255, 255}, {"Magenta", 255, 0, 255},
  {"White", 255, 255, 255}, {"Black", 1, 1, 1}, {"Gray", 128, 128, 128},
  {"Orange", 255, 165, 0}, {"Brown", 139, 69, 19}, {"Pink", 255, 192, 203},
  {"Purple", 128, 0, 128}, {"Lime", 50, 205, 50}, {"Navy", 0, 0, 128},
  {"Teal", 0, 128, 128}, {"Gold", 255, 215, 0}, {"Maroon", 128, 0, 0},
  {"Silver", 192, 192, 192}, {"Violet", 238, 130, 238}, {"Olive", 128, 128, 0},
  {"SkyBlue", 135, 206, 235}, {"Crimson", 220, 20, 60}, {"Indigo", 75, 0, 130},
  {"Tan", 210, 180, 140}, {"Aqua", 0, 255, 255}, {"DarkRed", 139, 0, 0},
  {"DarkGreen", 0, 100, 0}, {"DarkBlue", 0, 0, 139}, {"Beige", 245, 245, 220},
  {"Khaki", 240, 230, 140}, {"Coral", 255, 127, 80}
};
const int DB_SIZE = 32;

void setup() {
  pinMode(btn1, INPUT_PULLUP); 
  pinMode(btn2, INPUT_PULLUP);
  pinMode(pinR, OUTPUT); 
  pinMode(pinG, OUTPUT); 
  pinMode(pinB, OUTPUT);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  
  // EEPROM Load: Black at 0, White at sizeof(CalibData)
  EEPROM.get(0, blackRef);
  EEPROM.get(sizeof(CalibData), whiteRef);
}

void loop() {
  handleButtons();
  // Screensaver Trigger
  if (!isSleeping && (millis() - lastActivity > SLEEP_TIME)) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    isSleeping = true;
  }

  if (!isSleeping) drawUI();
}


// 1. INPUT HANDLING
void handleButtons() {
  checkButton(btn1, 0);
  checkButton(btn2, 1);
}

void checkButton(int pin, int idx) {
  bool reading = !digitalRead(pin);
  if (reading == HIGH && !btnActive[idx]) {
    btnActive[idx] = true;
    btnTimer[idx] = millis();
  } else if (reading == LOW && btnActive[idx]) {
    unsigned long duration = millis() - btnTimer[idx];
    if (duration > LONG_PRESS_MS) executeAction(idx + 1, true);
    else executeAction(idx + 1, false);
    btnActive[idx] = false;
  }
}

void executeAction(int btn, bool isLong) {
  lastActivity = millis(); // Reset timer on any button interaction
  
  if (isSleeping) {
    display.ssd1306_command(SSD1306_DISPLAYON);
    isSleeping = false;
    return; // Wake up only, don't trigger menu action yet
  }
  switch (currentState) {
    case WELCOME:
      if (btn == 1 && isLong) currentState = MAIN_MENU;
      break;

    case MAIN_MENU:
      if (!isLong) {
        if (btn == 1) menuIdx = (menuIdx <= 0) ? 3 : menuIdx - 1; // Toggle Up
        if (btn == 2) menuIdx = (menuIdx >= 3) ? 0 : menuIdx + 1; // Toggle Down
      } else {
        if (btn == 1) {
          if (menuIdx == 0) currentState = CALIB_SUB;
          if (menuIdx == 1) runDetection();
          if (menuIdx == 2) currentState = RAW_DATA;
          if (menuIdx == 3) currentState = RESET_CONFIRM;
        }
        if (btn == 2) currentState = WELCOME;
      }
      break;

    case CALIB_SUB:
      if (!isLong) calibIdx = !calibIdx;
      else if (btn == 1) startCalibration();
      else if (btn == 2) currentState = MAIN_MENU;
      break;

    case DETECTING:
      if (btn == 2 && isLong) currentState = MAIN_MENU;
      break;

    case RAW_DATA:
      if (btn == 1) rawPageIdx = !rawPageIdx;
      if (btn == 2 && isLong) currentState = MAIN_MENU;
      break;

    case RESET_CONFIRM:
      if (btn == 1 && isLong) performReset();
      if (btn == 2 && isLong) currentState = MAIN_MENU;
      break;
  }
}


// 2. CORE LOGIC (KNN & ACQUISITION)
void scanHardware() {
  int pins[] = {pinR, pinG, pinB};
  int results[3]; // To store averaged R, G, B

  for (int i = 0; i < 3; i++) {
    digitalWrite(pins[i], HIGH);
    
    // 1. Let the LDR fully stabilize to the new light bouncing off the surface
    delay(150); 

    long sum = 0;
    // 2. Take rapid readings while the light is steady to filter electrical noise
    for (int j = 0; j < 5; j++) {
      sum += analogRead(pinLDR);
      delay(3); // Tiny 2ms delay between ADC reads
    }
    results[i] = sum / 5; // Clean, stable average for this color

    digitalWrite(pins[i], LOW);

    // Give the LDR time to forget this color before the next one turns on.
    delay(250); 
  }

  currentScan.rawR = results[0];
  currentScan.rawG = results[1];
  currentScan.rawB = results[2];
  
  // Scaling: Black=1, White=255
  currentScan.r = constrain(map(results[0], blackRef.r, whiteRef.r, 1, 255), 1, 255);
  currentScan.g = constrain(map(results[1], blackRef.g, whiteRef.g, 1, 255), 1, 255);
  currentScan.b = constrain(map(results[2], blackRef.b, whiteRef.b, 1, 255), 1, 255);
}

void runDetection() {
  showLoading("Scanning...");
  scanHardware();
  
  // KNN Euclidean Distance
  long minDim = 2000000; // Increased to ensure it's higher than max possible distance
  
  for (int i = 0; i < DB_SIZE; i++) {
    ColorEntry entry;
    // Copy the entire struct from PROGMEM into our temporary 'entry' variable
    memcpy_P(&entry, &colorDB[i], sizeof(ColorEntry));
    
    long dR = (long)currentScan.r - entry.r;
    long dG = (long)currentScan.g - entry.g;
    long dB = (long)currentScan.b - entry.b;
    long dist = (dR * dR) + (dG * dG) + (dB * dB);
    
    if (dist < minDim) {
      minDim = dist;
      strcpy(detectedColorName, entry.name);
    }
  }
  currentState = DETECTING;
}

void startCalibration() {
  showLoading("Calibrating...");
  scanHardware();
  if (calibIdx == 0) { // Black
    blackRef = {currentScan.rawR, currentScan.rawG, currentScan.rawB};
    EEPROM.put(0, blackRef);
  } else { // White
    whiteRef = {currentScan.rawR, currentScan.rawG, currentScan.rawB};
    EEPROM.put(sizeof(CalibData), whiteRef);
  }
  display.clearDisplay();
  display.setCursor(0,20); display.print("Calibration Complete!");
  display.display();
  delay(2000);
  currentState = CALIB_SUB;
}

void performReset() {
  showLoading("Resetting...");
  blackRef = {0,0,0}; whiteRef = {1023,1023,1023};
  EEPROM.put(0, blackRef);
  EEPROM.put(sizeof(CalibData), whiteRef);
  display.clearDisplay();
  display.setCursor(0,20); display.print("Reset Complete!");
  display.display();
  delay(2000);
  currentState = MAIN_MENU;
}


// 3. UI RENDERING
void showLoading(const char* msg) {
  for (int i = 0; i <= 100; i += 20) {
    display.clearDisplay();
    
    display.setTextSize(1); 
    display.setTextColor(WHITE);
    
    // Use the F() macro for the labels inside the function to save RAM
    display.setCursor(10, 15); 
    display.print(msg);
    
    display.drawRect(10, 30, 100, 10, WHITE);
    display.fillRect(10, 30, i, 10, WHITE);
    
    display.display();
    delay(50); 
  }
}


void drawUI() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  if (currentState == WELCOME) {
    display.setTextSize(2); display.setCursor(0,0); display.print("Team  ^w^\nMunchkins");
    display.setTextSize(1); display.setCursor(0,40); display.print("LDR Color Detector");
    display.drawLine(30, 55, 95, 55, WHITE);
    display.setCursor(0, 56); display.print("     "); display.setTextColor(BLACK, WHITE); display.print("  PROCEED  ");
    
  } 
  else if (currentState == MAIN_MENU) {
    display.setCursor(0, 0);
    display.setTextColor(WHITE);
    display.println(" --- SELECT MODE ---");
    display.drawLine(0, 9, 128, 9, WHITE); 

    const char* menuLabels[] = {"Calibration", "Color Detect", "Raw Data", "Reset System"};

    for (int i = 0; i < 4; i++) {
      display.setCursor(5, 15 + (i * 12)); 
      if (i == menuIdx) {
        display.setTextColor(BLACK, WHITE); 
      } else {
        display.setTextColor(WHITE, BLACK);
      }
      display.print(menuLabels[i]);
    }
  }
  else if (currentState == CALIB_SUB) {
    display.setCursor(0,0); display.print("Choose & Long Press:");
    display.setCursor(0,20); 
    if (calibIdx == 0) display.setTextColor(BLACK, WHITE); else display.setTextColor(WHITE);
    display.print(" i. Black ");
    display.setCursor(0,35); 
    display.setTextColor(WHITE);
    if (calibIdx == 1) display.setTextColor(BLACK, WHITE); else display.setTextColor(WHITE);
    display.print(" ii. White ");
    display.setCursor(0, 55); display.setTextColor(WHITE);
    display.print("B2 Long: Go Back");
  }
  else if (currentState == DETECTING) {
    display.setCursor(0,0); display.print("Detected Color:");
    display.setTextSize(2); display.setCursor(0,15); display.print(detectedColorName);
    display.setTextSize(1); display.setCursor(0,40);
    
    // Removed String concatenation
    display.print("R:"); display.print(currentScan.r);
    display.print(" G:"); display.print(currentScan.g);
    display.print(" B:"); display.print(currentScan.b);
    display.setCursor(0,55); display.print("B2 Long to Exit");
  }
  else if (currentState == RAW_DATA) {
    if (rawPageIdx == 0) { // Calibration Page
      display.setCursor(0,0); display.print("RAW CALIB (R|G|B)");
      display.setCursor(0,15); 
      display.print("BLK: "); display.print(blackRef.r); display.print("|"); display.print(blackRef.g); display.print("|"); display.print(blackRef.b);
      display.setCursor(0,30); 
      display.print("WHT: "); display.print(whiteRef.r); display.print("|"); display.print(whiteRef.g); display.print("|"); display.print(whiteRef.b);
    } else { // Detect Page
      display.setCursor(0,0); display.print("RAW DETECT (R|G|B)");
      display.setCursor(0,15); 
      display.print("SCL: "); display.print(currentScan.r); display.print("|"); display.print(currentScan.g); display.print("|"); display.print(currentScan.b);
      display.setCursor(0,30); 
      display.print("RAW: "); display.print(currentScan.rawR); display.print("|"); display.print(currentScan.rawG); display.print("|"); display.print(currentScan.rawB);
    }
    display.setCursor(0,50); display.print("B1:Page B2:Exit");
  }
  else if (currentState == RESET_CONFIRM) {
    display.setCursor(0,20); display.print("Are you sure?");
    display.setCursor(0,40); display.print("B1 Long: Confirm");
    display.setCursor(0,55); display.print("B2 Long: Go Back");
  }

  display.display();
}
