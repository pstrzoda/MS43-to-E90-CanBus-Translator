#include <FlexCAN_T4.h>

// Constants for Engine and Overheat Thresholds
#define MAX_ENGINE_TORQUE 360
#define OVERHEAT_THRESHOLD 108



  // Overheat warning threshold (in °C)
#define SAFE_TEMP_THRESHOLD 90  // Safe temperature threshold to turn off warning (in °C)
#define INTERNAL_LED_PIN 13
#define FAN_CONTROL_PIN 19         // Define the pin for fan control
#define FAN_ON_TEMP_THRESHOLD 105   // Temperature to turn the fan on (in °C)
#define FAN_OFF_TEMP_THRESHOLD 97  // Temperature to turn the fan off (in °C)
#define CAN2_TIMEOUT_PERIOD 10000  // Timeout period in milliseconds (10 seconds)

#define BATTERY_LIGHT_INPUT 17
#define OIL_PRESS_LOW_INPUT 16
#define KL15_INPUT 15
#define STATUS_LED 13
bool ledState = false;  // Track the state of the LED
unsigned long previousMillis = 0;  // Store the last time the LED was updated
const long interval = 1000;  // Interval at which to blink (milliseconds)



// Loop counters for static frames
int index0AA = 0;
int index0A8 = 0;
int index0A9 = 0;

// Macros for Byte Manipulation
#define lo8(x) (uint8_t)((x)&0xFF)
#define hi8(x) (uint8_t)(((x) >> 8) & 0xFF)
// Define the Error State structure
typedef struct {
  uint8_t errorCode;           // Error code
  bool showError;              // Whether the error is currently active
  unsigned long lastSentTime;  // Last time the message was sent
} ErrorState;

// Define the array to hold the error states
#define NUM_ERRORS 16
ErrorState errorStates[NUM_ERRORS];

// Fan control
bool isFanOn = false;  // Track the state of the fan

// Global Variables for Sensors and Statuses
int aRPM, bRPM, RPM_ENG = 0;
int s_waterTemp = 0;
int s_oilTemp = 0;
uint8_t s_idleRegulatorState = 0;  // Status Switch Level RPM
int s_intakeTemp = 0;
int s_exhaustTemp = 0;
int s_speed = 0;
bool s_checkEngine = false;
double s_batteryVolts = 14;
bool s_emlLight = false;
bool s_waterOverheat = false;
bool s_lowOilPressureFromCan = false;
bool s_lowOilLevel = false;
bool s_batteryLight = false;
bool s_isEngineRunning = false;
uint8_t s_isEngineRunningCan = 0;
bool s_clutchDepressed = false;
bool s_brakeDepressed = false;
bool s_terminal15detected = false;  // Ignition Key Voltage State
double s_engineTorque = 0.0;
int s_ambientPressure = 0;
double s_throttlePosition = 0.0;

// Static data
const uint8_t staticData0AA[][8] = {
  { 0xE0, 0x29, 0x02, 0x00, 0xFC, 0x0A, 0x80, 0x83 },
  { 0xE1, 0x2A, 0x02, 0x00, 0xFC, 0x0A, 0x80, 0x83 },
  { 0xDA, 0x2B, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
  { 0xF3, 0x4C, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
  { 0xF8, 0x4D, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
  { 0xF9, 0x4E, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
  { 0xEB, 0x40, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
  { 0xE4, 0x41, 0x02, 0x00, 0xE8, 0x0A, 0x80, 0x83 },
  { 0xE5, 0x42, 0x02, 0x00, 0xE8, 0x0A, 0x80, 0x83 },
  { 0x03, 0x63, 0x02, 0x00, 0xE4, 0x0A, 0x80, 0x83 },
  { 0x0C, 0x64, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
  { 0xFC, 0x55, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
  { 0xFD, 0x56, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
  { 0xFA, 0x57, 0x02, 0x00, 0xE8, 0x0A, 0x80, 0x83 },
  { 0xFF, 0x58, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
  { 0x00, 0x59, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
  { 0x0A, 0x5A, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
  { 0xF6, 0x4B, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
  { 0xF7, 0x4C, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
  { 0xF0, 0x4D, 0x02, 0x00, 0xE8, 0x0A, 0x80, 0x83 },
  { 0xF5, 0x4E, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
  { 0xF7, 0x50, 0x02, 0x00, 0xEC, 0x0A, 0x80, 0x83 },
  { 0x00, 0x51, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
  { 0xF1, 0x42, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
  { 0xEE, 0x43, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
  { 0xEF, 0x44, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
  { 0xF0, 0x45, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
  { 0xF5, 0x46, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
  { 0xF6, 0x47, 0x02, 0x00, 0xF4, 0x0A, 0x80, 0x83 },
  { 0xF3, 0x48, 0x02, 0x00, 0xF0, 0x0A, 0x80, 0x83 },
  { 0xEC, 0x49, 0x02, 0x00, 0xE8, 0x0A, 0x80, 0x83 },
  // Add more static frames if needed
};

const uint8_t staticData0A8[][8] = {
  { 0xD5, 0xEE, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xC7, 0xE0, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xC8, 0xE1, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xC9, 0xE2, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xCA, 0xE3, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xCB, 0xE4, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xCC, 0xE5, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xDD, 0xE6, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xDE, 0xE7, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xCF, 0xE8, 0x02, 0xE2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xB0, 0xD9, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xB1, 0xDA, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xB2, 0xDB, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xB3, 0xDC, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xB4, 0xDD, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xB5, 0xDE, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xB6, 0xDF, 0x02, 0xD2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xE7, 0xF0, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xE8, 0xF1, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xE9, 0xF2, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xEA, 0xF3, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xEB, 0xF4, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xEC, 0xF5, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xED, 0xF6, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xEE, 0xF7, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xEF, 0xF8, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xF0, 0xF9, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xF1, 0xFA, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xF2, 0xFB, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xF3, 0xFC, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  { 0xF4, 0xFD, 0x02, 0xF2, 0x02, 0xF0, 0x03, 0x63 },
  // Add more static frames if needed
};

const uint8_t staticData0A9[][8] = {
  { 0x03, 0x1D, 0x7C, 0x8F, 0x2D, 0x7C, 0x6F, 0x17 },
  { 0x24, 0x1E, 0x7C, 0x8F, 0x2D, 0x7C, 0x8F, 0x17 },
  { 0x06, 0x10, 0x7C, 0x7F, 0x2D, 0x7C, 0x8F, 0x17 },
  { 0x67, 0x11, 0x7C, 0x7F, 0x2D, 0x7C, 0xEF, 0x17 },
  { 0x58, 0x12, 0x7C, 0x9F, 0x2D, 0x7C, 0xBF, 0x17 },
  { 0x69, 0x13, 0x7C, 0x9F, 0x2D, 0x7C, 0xCF, 0x17 },
  { 0x5A, 0x14, 0x7C, 0x8F, 0x2D, 0x7C, 0xCF, 0x17 },
  { 0x0B, 0x15, 0x7C, 0x8F, 0x2D, 0x7C, 0x7F, 0x17 },
  { 0x0C, 0x16, 0x7C, 0x8F, 0x2D, 0x7C, 0x7F, 0x17 },
  { 0x0D, 0x17, 0x7C, 0x8F, 0x2D, 0x7C, 0x7F, 0x17 },
  { 0x1C, 0x18, 0x7B, 0x9F, 0x2D, 0x7B, 0x7F, 0x17 },
  { 0x1D, 0x19, 0x7B, 0x9F, 0x2D, 0x7B, 0x7F, 0x17 },
  { 0x0E, 0x1A, 0x7B, 0x8F, 0x2D, 0x7B, 0x7F, 0x17 },
  { 0xEE, 0x1B, 0x7B, 0x8F, 0x2D, 0x7B, 0x5F, 0x17 },
  { 0xEF, 0x1C, 0x7B, 0x8F, 0x2D, 0x7B, 0x5F, 0x17 },
  { 0xF0, 0x1D, 0x7B, 0x8F, 0x2D, 0x7B, 0x5F, 0x17 },
  { 0xF1, 0x1E, 0x7B, 0x8F, 0x2D, 0x7B, 0x5F, 0x17 },
  { 0xD3, 0x10, 0x7B, 0x8F, 0x2D, 0x7B, 0x4F, 0x17 },
  { 0xB4, 0x11, 0x7B, 0x8F, 0x2D, 0x7B, 0x2F, 0x17 },
  { 0xB5, 0x12, 0x7B, 0x8F, 0x2D, 0x7B, 0x2F, 0x17 },
  { 0xB8, 0x13, 0x7C, 0x8F, 0x2D, 0x7C, 0x2F, 0x17 },
  { 0xA9, 0x14, 0x7C, 0x8F, 0x2D, 0x7C, 0x1F, 0x17 },
  { 0xDA, 0x15, 0x7C, 0xAF, 0x2D, 0x7C, 0x2F, 0x17 },
  { 0x9B, 0x16, 0x7C, 0xAF, 0x2D, 0x7C, 0xEF, 0x16 },
  { 0x8C, 0x17, 0x7C, 0x9F, 0x2D, 0x7C, 0xEF, 0x16 },
  { 0x8D, 0x18, 0x7C, 0x9F, 0x2D, 0x7C, 0xEF, 0x16 },
  { 0xAE, 0x19, 0x7C, 0xBF, 0x2D, 0x7C, 0xEF, 0x16 },
  { 0x60, 0x1A, 0x7C, 0xBF, 0x2D, 0x7C, 0x9F, 0x17 },
  { 0x71, 0x1B, 0x7C, 0xBF, 0x2D, 0x7C, 0xAF, 0x17 },
  { 0x72, 0x1C, 0x7C, 0xBF, 0x2D, 0x7C, 0xAF, 0x17 },
  { 0x53, 0x1D, 0x7C, 0x9F, 0x2D, 0x7C, 0xAF, 0x17 },
  // Add more static frames if needed
};


// Alive Counters
uint8_t ALIV_TORQ_1_DME = 8;  // Alive counter for Torque 1 DME (0x0A8)
uint8_t ALIV_TORQ_2_DME = 7;  // Alive counter for Torque 2 DME (0x0A9)
uint8_t ALIV_TORQ_3_DME = 7;  // Alive counter for Torque 3 DME (0x0AA)
int ALIV_COUNT_DME = 7;       // Alive counter for Engine Data (0x1D0)

// Message Timers
uint32_t MessageTimer0A8 = 20;         // Timer for sending 0x0A8 messages
uint32_t MessageTimer0A9 = 20;         // Timer for sending 0x0A9 messages
uint32_t MessageTimer0AA = 20;         // Timer for sending 0x0AA messages
uint32_t MessageTimer1D0 = 20;         // Timer for sending 0x1D0 messages
uint32_t MessageTimer3B4 = 28;         // Timer for sending 0x3B4 messages
uint32_t DMEMessageTimer153 = 0;          // Timer for sending 0x153 messages
uint32_t MessageTimerStatusDme = 0;    //Timer for sending Status to DME
unsigned long lastDmeMessageTime = 0;  // Track the last time a message was received from DME


// Signal Variables
uint8_t RPM_ENG_ERR = 0;           // Engine RPM Error (0x0AA)
uint8_t ST_IDLG_ENG = 0;           // Status Idling Engine (0x0AA)
uint8_t RQAM_FU = 51;              // Requested Amount of Fuel (0x0AA)
uint8_t ST_TORQ_AVL = 0;           // Status Torque Available (0x0A8)
uint8_t ST_DMEA_SWO = 0;           // Status DMEA Shut-off Warning (0x0A8)
uint8_t RCPT_ACC_SWO_SYS_DME = 0;  // Receipt Accelerator Shut-off System DME (0x0A8)
uint8_t ST_RCPT_ENG_ACC = 0;       // Status Receipt Engine Accelerator (0x0A8)
uint8_t ST_RCPT_ENG_ARS = 0;       // Status Receipt Engine Anti-Roll System (0x0A8)
uint8_t ST_RCPT_ENG_DSC = 0;       // Status Receipt Engine Dynamic Stability Control (0x0A8)
uint8_t ST_RCPT_ENG_EGS = 0;       // Status Receipt Engine Electronic Gearbox System (0x0A8)
uint8_t ST_OBD_CTFN_GRB = 2;       // Status OBD Certification GRB (0x0A8)
uint8_t ST_CT_BRPD_DME = 0;        // Status Catalytic Converter Bypass DME (0x0A8)
uint8_t ST_INFS = 0;               // Status Information Switch (0x0A9)
int16_t TORQ_AVL_MIN = -95;        // Torque Available Minimum (0x0A9)
int16_t TORQ_AVL_MAX = 360;        // Torque Available Maximum (0x0A9)
int16_t TORQ_AVL_SPAR_NEG = -115;  // Torque Available Spare Negative (0x0A9)
int16_t TORQ_AVL_SPAR_POS = 24;    // Torque Available Spare Positive (0x0A9)
uint8_t ST_SW_WAUP = 1;            // Warm-up Status (0x1D0)
uint8_t AIP_ENG = 183;             // Absolute Air Pressure (0x1D0)
uint16_t IJV_FU = 15054;           // Fuel Injection Amount (0x1D0)
uint8_t CTL_SLCK = 1;              // Shift Lock Control (0x1D0)
uint16_t RPM_IDLE_TARGET = 140;    // RPM Idle Target (0x1D0)


// CAN Objects
FlexCAN_T4<CAN1, RX_SIZE_128, TX_SIZE_8> can1;
FlexCAN_T4<CAN2, RX_SIZE_128, TX_SIZE_8> can2;
CAN_message_t dmeMsg;
CAN_message_t carMsg;

// Function Prototypes
void dmeCanRead();
void carCanSend();
void ManageErrorMessages();
void SetErrorState(uint8_t errorCode, bool showError);
uint8_t calculateChecksum(uint8_t* data, uint8_t len);
void SendMessage0AA();
void SendMessage0A8();
void SendMessage0A9();
void SendMessage1D0();
void SendMessage592(uint8_t errorCode, bool showError);
void SendMessage3B4();
void controlFan();

// Checksum Calculation
uint8_t calculateChecksum(uint8_t* data, uint8_t len) {
  // Step 1: Sum bytes 1 through 7
  uint16_t sum = 0;
  for (uint8_t i = 1; i <= 7; i++) {  // Sum bytes from index 1 to 7
    sum += data[i];
  }

  // Step 2: Subtract 595 and take modulo 255
  uint16_t calc = ((sum - 595) % 256) - 1;

  // Step 3: Apply the second formula (condition check)
  if (calc == 2 || calc == 11 || calc == 9) {
    calc = calc + 1;
  }

  // Return the calculated checksum as an 8-bit unsigned integer
  return (uint8_t)calc;
}

// Control fan based on temperature or communication failure
void controlFan() {
  unsigned long currentTime = millis();
  if (currentTime - lastDmeMessageTime > CAN2_TIMEOUT_PERIOD && digitalRead(KL15_INPUT) == HIGH) {
    // If the DME message timeout period is exceeded, turn on the fan as a fail-safe
    digitalWrite(FAN_CONTROL_PIN, HIGH);
    isFanOn = true;
  } else {
    // Control the fan based on the water temperature
    if (s_waterTemp >= FAN_ON_TEMP_THRESHOLD && !isFanOn) {
      // Turn the fan on
      digitalWrite(FAN_CONTROL_PIN, HIGH);
      isFanOn = true;
    } else if (s_waterTemp <= FAN_OFF_TEMP_THRESHOLD && isFanOn) {
      // Turn the fan off
      digitalWrite(FAN_CONTROL_PIN, LOW);
      isFanOn = false;
    }
  }
}

// Set Error State
void SetErrorState(uint8_t errorCode, bool showError) {
  for (int i = 0; i < NUM_ERRORS; i++) {
    if (errorStates[i].errorCode == errorCode) {
      errorStates[i].showError = showError;
      errorStates[i].lastSentTime = millis();
      return;
    }
  }

  for (int i = 0; i < NUM_ERRORS; i++) {
    if (errorStates[i].errorCode == 0) {
      errorStates[i].errorCode = errorCode;
      errorStates[i].showError = showError;
      errorStates[i].lastSentTime = millis();
      return;
    }
  }
}

// Manage Error Messages
void ManageErrorMessages() {
  unsigned long curTime = millis();

  // Check engine temperature and set error state if necessary
  if (s_waterTemp > OVERHEAT_THRESHOLD) {
    SetErrorState(0x27, true);  // Red Engine Overheating - not overheating
  } else if (s_waterTemp < SAFE_TEMP_THRESHOLD) {
    SetErrorState(0x27, false);  // Turn off overheating warning
  }
  // SetErrorState(0x99, RPM_ENG > 1500);  // Low Oil Pressure

  // Check other statuses
  SetErrorState(0x1F, s_checkEngine);            // Check Engine
  SetErrorState(0x1E, s_emlLight);               // EML Light
  SetErrorState(0xDA, s_lowOilLevel);            // Low Oil Level
  SetErrorState(0xDB, s_lowOilPressureFromCan);  // Low Oil Pressure
  SetErrorState(0xD4, digitalRead(OIL_PRESS_LOW_INPUT) == LOW && s_isEngineRunning);  // Low Oil Pressure
  SetErrorState(0xD5, digitalRead(BATTERY_LIGHT_INPUT) == HIGH && s_isEngineRunning);           // Alternator Failure

  // Check for communication failure with DME
  if (curTime - lastDmeMessageTime > CAN2_TIMEOUT_PERIOD && digitalRead(KL15_INPUT) == LOW) {
    SetErrorState(0x99, true);  // 0x99 is used for Red Gun Icon indicating communication failure
  } else {
    SetErrorState(0x99, false);  // Clear the error when communication is restored
  }

  // Manage errors
  for (int i = 0; i < NUM_ERRORS; i++) {
    if (errorStates[i].errorCode != 0) {
      if (errorStates[i].showError) {
        if (curTime - errorStates[i].lastSentTime >= 8000) {
          SendMessage592(errorStates[i].errorCode, true);
          errorStates[i].lastSentTime = curTime;
        }
      } else {
        if (curTime - errorStates[i].lastSentTime >= 8000) {
          SendMessage592(errorStates[i].errorCode, false);
          errorStates[i].lastSentTime = curTime;
          errorStates[i].errorCode = 0;
        }
      }
    }
  }
}

// Send CAN Messages

void SendMessage0AADyn() {
  carMsg.len = 8;
  carMsg.id = 0x0AA;

  // Calculate RPM_ENG
  // uint16_t rpm_eng = RPM_ENG / 0.25;
  uint16_t rpm_eng = 3000 / 0.25;
  carMsg.buf[4] = rpm_eng & 0xFF;         // Lower byte of RPM_ENG
  carMsg.buf[5] = (rpm_eng >> 8) & 0xFF;  // Upper byte of RPM_ENG

  // Static Data
  carMsg.buf[1] = (ALIV_TORQ_3_DME & 0x0F) | 0x40;  // ALIV_TORQ_3_DME + part of TORQ_DVCH (assuming 0x40)
  carMsg.buf[2] = 0x02;                             // TORQ_DVCH continuation (0x002 = 2)
  carMsg.buf[3] = 0x00;                             // ANG_ACPD (0%)

  carMsg.buf[6] = 0x80;  // RPM_ENG_ERR (0) + ST_IDLG_ENG (0) + ST_CLCTR_V (8)
  carMsg.buf[7] = 0x33;  // RQAM_FU (51 l/h)

  // Calculate Checksum (excluding first byte)
  carMsg.buf[0] = calculateChecksum(carMsg.buf, 8);

  // Send the message
  can1.write(carMsg);

  // Increment and wrap around the Alive counter (4 bits)
  ALIV_TORQ_3_DME = (ALIV_TORQ_3_DME + 1) & 0x0F;

  MessageTimer0AA = millis();
}

void SendMessage0AA() {
  carMsg.len = 8;
  carMsg.id = 0x0AA;


  // Copy the current static data frame into the carMsg buffer
  memcpy(carMsg.buf, staticData0AA[index0AA], 8);

  // Calculate RPM_ENG
  uint16_t rpm_eng = RPM_ENG / 0.25;
  // uint16_t rpm_eng = 3000 / 0.25;
  carMsg.buf[4] = rpm_eng & 0xFF;         // Lower byte of RPM_ENG
  carMsg.buf[5] = (rpm_eng >> 8) & 0xFF;  // Upper byte of RPM_ENG

  carMsg.buf[0] = calculateChecksum(carMsg.buf, 8);

  // Send the message
  can1.write(carMsg);
  // Increment the index and wrap around if necessary
  index0AA = (index0AA + 1) % (sizeof(staticData0AA) / 8);

  // Update the timer
  MessageTimer0AA = millis();
}




void SendMessage0A8() {
  carMsg.len = 8;
  carMsg.id = 0x0A8;


  // Copy the current static data frame into the carMsg buffer
  memcpy(carMsg.buf, staticData0A8[index0A8], 8);

  // Send the message
  can1.write(carMsg);

  // Increment the index and wrap around if necessary
  index0A8 = (index0A8 + 1) % (sizeof(staticData0A8) / 8);

  // Update the timer
  MessageTimer0A8 = millis();
}


void SendMessage0A9() {
  carMsg.len = 8;
  carMsg.id = 0x0A9;

  // Copy the current static data frame into the carMsg buffer
  memcpy(carMsg.buf, staticData0A9[index0A9], 8);

  // Send the message
  can1.write(carMsg);

  // Increment the index and wrap around if necessary
  index0A9 = (index0A9 + 1) % (sizeof(staticData0A9) / 8);

  // Update the timer
  MessageTimer0A9 = millis();
}

// void SendMessage1D0() {
//     carMsg.len = 8;
//     carMsg.id = 0x1D0;
//     memset(carMsg.buf, 0x00, 8); // Clear the buffer

//     // TEMP_ENG (Coolant temperature, 8 bits, offset -48)
//     carMsg.buf[0] = s_waterTemp + 48;

//     // TEMP_EOI (Oil temperature, 8 bits, offset -48)
//     carMsg.buf[1] = s_oilTemp + 48;

//     // Counter_464 (4 bits)
//     carMsg.buf[2] = (ALIV_COUNT_DME & 0x0F);
//     ALIV_COUNT_DME = (ALIV_COUNT_DME + 1) & 0x0F; // Increment and wrap around at 15

//     // ST_ENG_RUN (2 bits)
//     carMsg.buf[2] |= (s_isEngineRunningCan & 0x03) << 4;

//     // ST_SW_WAUP (2 bits)
//     carMsg.buf[2] |= (ST_SW_WAUP & 0x03) << 6;

//     // AIP_ENG (Absolute air pressure, 8 bits, factor 2, offset 598)
//     uint8_t air_pressure_scaled = (AIP_ENG - 598) / 2;
//     carMsg.buf[3] = air_pressure_scaled;

//     // IJV_FU (Fuel Injection amount, 16 bits)
//     carMsg.buf[4] = lo8(IJV_FU);
//     carMsg.buf[5] = hi8(IJV_FU);

//     // CTR_SLCK (Shift lock control, 2 bits)
//     carMsg.buf[6] = (CTL_SLCK & 0x03);

//     // RPM_IDLG_TAR (RPM idle target, 8 bits, factor 5)
//     carMsg.buf[7] = RPM_IDLE_TARGET / 5;

//     // CHK_SUM_ENG_DME (Checksum Engine DME)
//     carMsg.buf[0] = calculateChecksum(carMsg.buf, 8);

//     // Send the message
//     can1.write(carMsg);
//     MessageTimer1D0 = millis(); // Update the timer
// }
void SendMessage1D0() {
  carMsg.len = 8;
  carMsg.id = 0x1D0;

  // Adjust the water temperature correctly with the offset -48
  carMsg.buf[0] = s_waterTemp + 48;  // To convert physical temp to CAN value, add 48

  carMsg.buf[1] = s_oilTemp + 48;  // Assuming the same offset applies to oil temperature
  carMsg.buf[2] = ALIV_COUNT_DME;  // Alive counter

  carMsg.buf[3] = 0xBF;  // Static value for byte 3
  carMsg.buf[4] = 0x43;  // Static value for byte 4
  carMsg.buf[5] = 0xC1;  // Static value for byte 5

  carMsg.buf[6] = s_clutchDepressed ? 0xCD : 0xCC;  // ClutchPedal value with offset -204
  carMsg.buf[7] = 0x8C;                             // Static or calculated value based on requirements

  can1.write(carMsg);
  ALIV_COUNT_DME++;
  MessageTimer1D0 = millis();
}


void SendMessage592(uint8_t errorCode, bool showError) {
  carMsg.len = 8;
  carMsg.id = 0x592;

  carMsg.buf[0] = 0x40;
  carMsg.buf[1] = errorCode;
  carMsg.buf[2] = 0x00;
  carMsg.buf[3] = showError ? 0x31 : 0x30;
  carMsg.buf[4] = 0xFF;
  carMsg.buf[5] = 0xFF;
  carMsg.buf[6] = 0xFF;
  carMsg.buf[7] = 0xFF;

  can1.write(carMsg);
}

void SendMessage3B4() {
  carMsg.len = 8;
  carMsg.id = 0x3B4;
  memset(carMsg.buf, 0x00, 8);

  // Calculate the battery voltage
  uint16_t u_bt_scaled = (s_batteryVolts * 68);  // Reverse the division for proper scaling
  u_bt_scaled += 0xF000;                         // Add F000 as base offset

  carMsg.buf[0] = u_bt_scaled & 0xFF;         // Lower byte
  carMsg.buf[1] = (u_bt_scaled >> 8) & 0xFF;  // Higher byte

  // Engine running status
  carMsg.buf[2] = s_isEngineRunningCan ? 0x00 : 0x09;  // 0x00 for running, 0x09 for ignition on (engine off)

  carMsg.buf[3] = 0xFC;  // Static value as per the provided dump
  carMsg.buf[4] = 0xFF;  // Static values
  carMsg.buf[5] = 0xFF;
  carMsg.buf[6] = 0xFF;
  carMsg.buf[7] = 0xFF;

  can1.write(carMsg);
  MessageTimer3B4 = millis();
}

void SendStatusToDme() {
  uint16_t tempRpm = 12345;
  uint16_t tempMin = 3;

  carMsg.len = 8;
  carMsg.id = 0x613;
  carMsg.buf[0] = lo8(tempRpm);
  carMsg.buf[1] = hi8(tempRpm);

  carMsg.buf[2] = 0x39;
  carMsg.buf[3] = lo8(tempMin);
  carMsg.buf[4] = hi8(tempMin);
  carMsg.buf[5] = 0x00;
  carMsg.buf[6] = 0x00;
  carMsg.buf[7] = 0x00;
  // Serial.println("Send status");
  can2.write(carMsg);
  MessageTimerStatusDme = millis();
}

void SendDMEMessage153() {
    static uint8_t ascAliveCounter = 0x00;  // Rolling Alive counter (0x00 - 0x0F)
    
    carMsg.len = 8;
    carMsg.id = 0x153;

    // 1️⃣ Encode Speed
    int s_speed = 44; // Speed in km/h
    uint16_t speedRaw = s_speed * 16;  // Multiply speed by 16
    uint8_t speedLSB = speedRaw & 0x1F;   // LSB (only bits 3-7 used)
    uint8_t speedMSB = (speedRaw >> 5) & 0xFF;  // MSB (shift 5 bits)

    // 2️⃣ Encode Status Bitfields
    carMsg.buf[0] = 0x00;  // Default: No ASC/MSR intervention

    // Byte 1 (Speed LSB at bits 3-7)
    carMsg.buf[1] = (speedLSB << 3) & 0xF8;  // Shift left to align with bit 3

    // Byte 2 (Speed MSB)
    carMsg.buf[2] = speedMSB;

    // 3️⃣ Torque Intervention (ASC / MSR)
    carMsg.buf[3] = 0x00;  // MD_IND_ASC (0.0% default)
    carMsg.buf[4] = 0x00;  // MD_IND_MSR (0.0% default)
    carMsg.buf[5] = 0x00;  // Unused (always 0x00)
    carMsg.buf[6] = 0x00;  // MD_IND_ASC_LM (0.0% default)

    // 4️⃣ Alive Counter (rolls 0x00 - 0x0F)
    carMsg.buf[7] = ascAliveCounter;
    ascAliveCounter = (ascAliveCounter + 1) & 0x0F;  // Wrap around after 0x0F

    // Send CAN message
    can1.write(carMsg);
}

void setup(void) {
  for (int i = 0; i < NUM_ERRORS; i++) {
    errorStates[i].errorCode = 0;
    errorStates[i].showError = false;
    errorStates[i].lastSentTime = 0;
  }

  //Configuring outputs
  pinMode(FAN_CONTROL_PIN, OUTPUT);   // Configuring pin for /INT input
  pinMode(INTERNAL_LED_PIN, OUTPUT);  // Configuring pin for /INT input

  //configuring inputs
  pinMode(BATTERY_LIGHT_INPUT, INPUT_PULLUP);   // Configuring pin for /INT input
  pinMode(OIL_PRESS_LOW_INPUT, INPUT_PULLUP);  // Configuring pin for /INT input
  pinMode(KL15_INPUT, INPUT_PULLUP);  // Configuring pin for /INT input

  Serial.begin(9600);
  Serial.println("Starting MS43 Translator");
  can1.begin();
  can1.setBaudRate(500000);
  can2.begin();
  can2.setBaudRate(500000);
}

void blinkStatusLED() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    digitalWrite(STATUS_LED, ledState ? HIGH : LOW);
  }
}

void dmeCanRead() {
  if (can2.read(dmeMsg)) {
    lastDmeMessageTime = millis();  // Update the last received time
    Serial.println("GET:" + String(s_waterTemp));

    if (dmeMsg.id == 0x316) {
      aRPM = dmeMsg.buf[3] * 256;
      bRPM = aRPM + dmeMsg.buf[2];
      RPM_ENG = bRPM * 0.15625;
      s_engineTorque = dmeMsg.buf[4] * 0.39;
      s_isEngineRunning = RPM_ENG > 500;
    } else if (dmeMsg.id == 0x329) {
      s_waterTemp = (dmeMsg.buf[1] * 0.75) - 48;
      s_ambientPressure = (dmeMsg.buf[2] * 2) + 598;
      uint8_t byte3 = dmeMsg.buf[3];
      s_isEngineRunningCan = bitRead(byte3, 3);
      s_clutchDepressed = bitRead(byte3, 0);
      s_idleRegulatorState = bitRead(byte3, 1);
      s_throttlePosition = dmeMsg.buf[5] * 0.390625;
      uint8_t byte6 = dmeMsg.buf[6];
      s_brakeDepressed = bitRead(byte6, 0);
    } else if (dmeMsg.id == 0x720) {
      s_intakeTemp = dmeMsg.buf[1] - 48;
      s_exhaustTemp = dmeMsg.buf[2] * 4;
      s_oilTemp = dmeMsg.buf[3] - 48;
      s_batteryVolts = dmeMsg.buf[4] * 0.1;
      s_speed = (dmeMsg.buf[5] * 256) + dmeMsg.buf[6];
    } else if (dmeMsg.id == 0x545) {
      uint8_t byte0 = dmeMsg.buf[0];
      s_checkEngine = bitRead(byte0, 1);
      s_emlLight = bitRead(byte0, 4);
      uint8_t byte3 = dmeMsg.buf[3];
      s_lowOilLevel = bitRead(byte3, 1);
      s_waterOverheat = bitRead(byte3, 3);
      uint8_t byte5 = dmeMsg.buf[5];
      s_batteryLight = bitRead(byte5, 0);
      uint8_t byte7 = dmeMsg.buf[7];
      s_lowOilPressureFromCan = bitRead(byte7, 7);
    }
    // digitalWrite(INTERNAL_LED_PIN, LOW);
  }

  int32_t curTime = millis();
  if (curTime - MessageTimerStatusDme >= 200)
    SendStatusToDme();

  // if (curTime - DMEMessageTimer153 >= 100) {  // Send ASC1 speed every 100ms
  //     SendDMEMessage153();
  //     DMEMessageTimer153 = curTime;
  // }
}

void carCanSend() {
  uint32_t curTime = millis();
  if (curTime - MessageTimer0A8 >= 10) SendMessage0A8();
  if (curTime - MessageTimer0A9 >= 10) SendMessage0A9();
  if (curTime - MessageTimer0AA >= 10) SendMessage0AA();
  // if (curTime - MessageTimer1D0 >= 200) SendMessage1D0();
  if (curTime - MessageTimer3B4 >= 4000) SendMessage3B4();

  ManageErrorMessages();
}

void loop() {
  uint8_t kl15_input_s = digitalRead(KL15_INPUT) == LOW;
  uint8_t oil_press_input = digitalRead(OIL_PRESS_LOW_INPUT) == LOW;
  uint8_t alternator_input = digitalRead(BATTERY_LIGHT_INPUT) == LOW;

  // Serial.println("KL15: " + String(kl15_input_s) + " OIL: " + String(oil_press_input) + " ALT: " + String(alternator_input));
  Serial.println("Temp: " + String(s_waterTemp) + " RPM: " + String(RPM_ENG) + " Torque: " + String(s_engineTorque) + " Engine Running: " + String(s_isEngineRunning));
  dmeCanRead();
  carCanSend();
  controlFan();
  blinkStatusLED();
  ManageErrorMessages();
}