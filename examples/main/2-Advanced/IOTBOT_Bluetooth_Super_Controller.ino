/*
 * IOTBOT Super Bluetooth Controller Example
 * 
 * This advanced example turns your IOTBOT into a Bluetooth Master or Slave device.
 * It demonstrates a complete "Smart Device" interaction scenario.
 * 
 * Features:
 * 1. Role Selection: Choose to be a "Controller" (Master) or "Robot" (Slave) at startup.
 * 2. Scanning: The Controller scans for available Bluetooth devices and lists them on the LCD.
 * 3. Pairing & Security: 
 *    - Controller requests connection.
 *    - Robot asks user for confirmation (Button Press).
 *    - Robot requires a PIN code.
 *    - Controller user enters PIN using Joystick (Up/Down for numbers, Right for next digit).
 * 4. Data Menu: Once paired, the Controller can select sensor data (Temperature, Distance, etc.) to send to the Robot.
 * 
 * Instructions:
 * 1. Upload this code to TWO IOTBOTs (ESP32).
 * 2. On Device A, select "Robot (Slave)". It will wait for connection.
 * 3. On Device B, select "Controller". It will scan.
 * 4. Select Device A from the list on Device B.
 * 5. Follow the on-screen instructions for pairing and PIN entry.
 * 
 * Controls:
 * - Joystick Y: Scroll Menu / Change PIN Number
 * - Joystick X: Next PIN Digit
 * - Button 1: Select / Confirm
 */

#define USE_BLUETOOTH
#define USE_DHT // For sensor data example
#include <IOTBOT.h>

IOTBOT iotbot;
BluetoothSerial* bt;

// Application States
enum AppState {
  ROLE_SELECT,
  SLAVE_WAIT,
  SLAVE_AUTH_CONFIRM,
  SLAVE_CONNECTED,
  MASTER_SCAN,
  MASTER_LIST,
  MASTER_CONNECTING,
  MASTER_AUTH_PIN,
  MASTER_MENU,
  MASTER_SENDING
};

AppState currentState = ROLE_SELECT;

// Variables
String slaveName = "IOTBOT_SLAVE";
String masterName = "IOTBOT_MASTER";
BTScanResults* scanResults;
int selectedDeviceIndex = 0;
int deviceCount = 0;
bool isMaster = false;

// Security
String secretPIN = "1234"; // The PIN required by the Slave
String enteredPIN = "0000";
int pinCursor = 0;

// Helper for non-blocking delays
unsigned long lastTime = 0;

void setup() {
  iotbot.serialStart(115200);
  iotbot.begin();
  iotbot.playIntro();
  
  bt = iotbot.getBluetoothObject();
  
  // Initial Screen
  iotbot.lcdClear();
  iotbot.lcdWriteMid("Select Role:", "1. Controller", "2. Robot (Slave)", "(Use Buttons)");
}

void loop() {
  switch (currentState) {
    case ROLE_SELECT:
      handleRoleSelect();
      break;
      
    // --- SLAVE LOGIC ---
    case SLAVE_WAIT:
      handleSlaveWait();
      break;
    case SLAVE_AUTH_CONFIRM:
      handleSlaveAuthConfirm();
      break;
    case SLAVE_CONNECTED:
      handleSlaveConnected();
      break;
      
    // --- MASTER LOGIC ---
    case MASTER_SCAN:
      handleMasterScan();
      break;
    case MASTER_LIST:
      handleMasterList();
      break;
    case MASTER_CONNECTING:
      // Handled in transition
      break;
    case MASTER_AUTH_PIN:
      handleMasterAuthPin();
      break;
    case MASTER_MENU:
      handleMasterMenu();
      break;
  }
  
  delay(50); // Small delay for stability
}

// --- STATE HANDLERS ---

void handleRoleSelect() {
  if (iotbot.button1Read()) { // Button 1 -> Controller (Master)
    isMaster = true;
    iotbot.bluetoothStart(masterName);
    currentState = MASTER_SCAN;
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Role: Controller", "Starting...", "", "");
    delay(1000);
  } 
  else if (iotbot.button2Read()) { // Button 2 -> Robot (Slave)
    isMaster = false;
    iotbot.bluetoothStart(slaveName);
    currentState = SLAVE_WAIT;
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Role: Robot", "Waiting for", "Connection...", "");
    delay(1000);
  }
}

// --- SLAVE FUNCTIONS ---

void handleSlaveWait() {
  // Check if connected
  if (bt->hasClient()) {
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Device Connected!", "Waiting Auth...", "", "");
    currentState = SLAVE_AUTH_CONFIRM;
  }
}

void handleSlaveAuthConfirm() {
  // Simple protocol: Wait for "REQ_AUTH" from Master
  if (bt->available()) {
    String msg = bt->readStringUntil('\n');
    msg.trim();
    
    if (msg == "REQ_AUTH") {
      iotbot.lcdClear();
      iotbot.lcdWriteMid("Connection Req!", "Allow?", "B1: YES  B2: NO", "");
      
      while(true) {
        if (iotbot.button1Read()) {
          bt->println("AUTH_OK"); // Tell master to proceed to PIN
          iotbot.lcdClear();
          iotbot.lcdWriteMid("Waiting for", "PIN Code...", "", "");
          
          // Wait for PIN
          while(!bt->available()) delay(10);
          String pin = bt->readStringUntil('\n');
          pin.trim();
          
          if (pin == secretPIN) {
            bt->println("PIN_OK");
            iotbot.lcdClear();
            iotbot.lcdWriteMid("Pairing Success!", "System Ready", "", "");
            currentState = SLAVE_CONNECTED;
            iotbot.buzzerPlayTone(1000, 500);
          } else {
            bt->println("PIN_FAIL");
            iotbot.lcdClear();
            iotbot.lcdWriteMid("Wrong PIN!", "Disconnected", "", "");
            bt->disconnect();
            delay(2000);
            currentState = SLAVE_WAIT;
          }
          break;
        }
        if (iotbot.button2Read()) {
          bt->println("AUTH_DENY");
          bt->disconnect();
          currentState = SLAVE_WAIT;
          break;
        }
        delay(50);
      }
    }
  }
}

void handleSlaveConnected() {
  if (bt->available()) {
    String msg = bt->readStringUntil('\n');
    msg.trim();
    
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Received Data:", msg.c_str(), "", "");
    iotbot.buzzerPlayTone(2000, 100);
    
    // Echo back
    bt->println("ACK: " + msg);
  }
  
  if (!bt->hasClient()) {
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Lost Connection", "Resetting...", "", "");
    delay(2000);
    currentState = SLAVE_WAIT;
  }
}

// --- MASTER FUNCTIONS ---

void handleMasterScan() {
  iotbot.lcdClear();
  iotbot.lcdWriteMid("Scanning...", "Please Wait", "(5 Seconds)", "");
  
  // Start discovery (synchronous for simplicity in this example)
  scanResults = bt->discover(5000);
  
  if (scanResults->getCount() > 0) {
    deviceCount = scanResults->getCount();
    selectedDeviceIndex = 0;
    currentState = MASTER_LIST;
  } else {
    iotbot.lcdClear();
    iotbot.lcdWriteMid("No Devices Found", "Retrying...", "", "");
    delay(2000);
    // Retry scan
  }
}

void handleMasterList() {
  // Display current selection
  BTAdvertisedDevice* device = scanResults->getDevice(selectedDeviceIndex);
  
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "Select Device:");
  iotbot.lcdWriteCR(0, 1, "> " + String(device->getName().c_str()));
  iotbot.lcdWriteCR(0, 2, "Addr: " + String(device->getAddress().toString().c_str()));
  iotbot.lcdWriteCR(0, 3, "B1:Connect Y:Scroll");
  
  // Navigation
  int yVal = iotbot.joystickYRead();
  if (yVal > 3000) { // Down
    selectedDeviceIndex++;
    if (selectedDeviceIndex >= deviceCount) selectedDeviceIndex = 0;
    delay(200);
  } else if (yVal < 1000) { // Up
    selectedDeviceIndex--;
    if (selectedDeviceIndex < 0) selectedDeviceIndex = deviceCount - 1;
    delay(200);
  }
  
  // Select
  if (iotbot.button1Read()) {
    currentState = MASTER_CONNECTING;
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Connecting to:", device->getName().c_str(), "...", "");
    
    if (bt->connect(device->getAddress())) {
      iotbot.lcdClear();
      iotbot.lcdWriteMid("Connected!", "Requesting Auth...", "", "");
      bt->println("REQ_AUTH"); // Start Protocol
      
      // Wait for response
      long start = millis();
      while(!bt->available() && millis() - start < 5000); // 5s timeout
      
      if (bt->available()) {
        String resp = bt->readStringUntil('\n');
        resp.trim();
        if (resp == "AUTH_OK") {
          currentState = MASTER_AUTH_PIN;
          enteredPIN = "0000";
          pinCursor = 0;
        } else {
          iotbot.lcdClear();
          iotbot.lcdWriteMid("Auth Denied!", "By Remote", "", "");
          bt->disconnect();
          delay(2000);
          currentState = MASTER_SCAN;
        }
      } else {
        iotbot.lcdClear();
        iotbot.lcdWriteMid("No Response", "Timeout", "", "");
        bt->disconnect();
        delay(2000);
        currentState = MASTER_SCAN;
      }
    } else {
      iotbot.lcdClear();
      iotbot.lcdWriteMid("Connection", "Failed!", "Try Again", "");
      delay(2000);
      currentState = MASTER_SCAN;
    }
  }
}

void handleMasterAuthPin() {
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "Enter PIN Code:");
  
  // Display PIN with cursor
  String displayPin = "";
  for(int i=0; i<4; i++) {
    if(i == pinCursor) displayPin += "[" + String(enteredPIN[i]) + "]";
    else displayPin += " " + String(enteredPIN[i]) + " ";
  }
  iotbot.lcdWriteCR(2, 1, displayPin);
  iotbot.lcdWriteCR(0, 3, "Y:Change X:Next B1:OK");
  
  // Controls
  int yVal = iotbot.joystickYRead();
  int xVal = iotbot.joystickXRead();
  
  // Change Number
  if (yVal > 3000) { // Up
    char c = enteredPIN[pinCursor];
    if(c < '9') c++; else c = '0';
    enteredPIN[pinCursor] = c;
    delay(200);
  } else if (yVal < 1000) { // Down
    char c = enteredPIN[pinCursor];
    if(c > '0') c--; else c = '9';
    enteredPIN[pinCursor] = c;
    delay(200);
  }
  
  // Move Cursor
  if (xVal > 3000) { // Right
    pinCursor++;
    if(pinCursor > 3) pinCursor = 0;
    delay(200);
  }
  
  // Submit
  if (iotbot.button1Read()) {
    iotbot.lcdClear();
    iotbot.lcdWriteMid("Sending PIN...", "", "", "");
    bt->println(enteredPIN);
    
    while(!bt->available());
    String resp = bt->readStringUntil('\n');
    resp.trim();
    
    if (resp == "PIN_OK") {
      iotbot.lcdClear();
      iotbot.lcdWriteMid("Access Granted!", "", "", "");
      delay(1000);
      currentState = MASTER_MENU;
    } else {
      iotbot.lcdClear();
      iotbot.lcdWriteMid("Access Denied!", "Wrong PIN", "", "");
      bt->disconnect();
      delay(2000);
      currentState = MASTER_SCAN;
    }
  }
}

void handleMasterMenu() {
  iotbot.lcdClear();
  iotbot.lcdWriteCR(0, 0, "Main Menu:");
  iotbot.lcdWriteCR(0, 1, "1. Send Hello");
  iotbot.lcdWriteCR(0, 2, "2. Send Temp");
  iotbot.lcdWriteCR(0, 3, "3. Send Distance");
  
  // Simple selection via buttons for demo
  if (iotbot.button1Read()) {
    bt->println("Hello from Master!");
    showSentMessage("Hello");
  }
  else if (iotbot.button2Read()) {
    // Read Temp
    int temp = iotbot.moduleDhtTempReadC(14); // Example pin
    bt->println("Temp: " + String(temp) + "C");
    showSentMessage("Temp: " + String(temp));
  }
  else if (iotbot.joystickButtonRead()) {
    // Read Distance
    int dist = iotbot.moduleUltrasonicDistanceRead();
    bt->println("Dist: " + String(dist) + "cm");
    showSentMessage("Dist: " + String(dist));
  }
  
  if (!bt->connected()) {
    currentState = MASTER_SCAN;
  }
}

void showSentMessage(String msg) {
  iotbot.lcdClear();
  iotbot.lcdWriteMid("Sent:", msg.c_str(), "", "");
  delay(1000);
}
