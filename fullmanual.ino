#include "BluetoothSerial.h"
#include <math.h>
#include <Wire.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

BluetoothSerial SerialBT;

const uint8_t BT_RX_MAX_LENGTH = 64;
const uint8_t BT_RX_BYTES_PER_POLL = 128;
char bluetoothRxBuffer[BT_RX_MAX_LENGTH + 1] = {0};
uint8_t bluetoothRxLength = 0;
bool bluetoothRxDiscarding = false;
const char *bluetoothRxDiscardReason = "";
const uint16_t BT_TX_LINE_CAPACITY = 192;
struct BluetoothTxMessage { char text[BT_TX_LINE_CAPACITY]; };
QueueHandle_t bluetoothAckQueue = nullptr;
QueueHandle_t bluetoothStatusQueue = nullptr;
TaskHandle_t bluetoothTxTaskHandle = nullptr;

// ==================== HARDWARE CONFIGURATION ====================
// TB6612FNG; STBY is wired directly to 3.3 V.
const int PWMA = 25;
const int AIN1 = 26;
const int AIN2 = 27;
const int PWMB = 4;
const int BIN1 = 16;
const int BIN2 = 17;

// S1 far left through S5 far right. Confirmed logic: LOW=black, HIGH=white.
const int IR_PINS[5] = {36, 39, 34, 35, 32};
// GPIO 18 rotating servo actuator. Clockwise direction and home angle must be
// calibrated on the assembled mechanism before automatic actuation.
const int ACTUATOR_SERVO_PIN = 18;
const int SERVO_PWM_FREQUENCY = 50;
const int SERVO_PWM_RESOLUTION = 16;
const int SERVO_MIN_PULSE_US = 1000;
const int SERVO_MAX_PULSE_US = 2000;

// TCS34725 remains reserved for future calibrated color work. The original
// manual sketch did not initialize this device; no color decisions are made.
const int COLOR_SDA_PIN = 21;
const int COLOR_SCL_PIN = 22;
const uint8_t COLOR_I2C_ADDRESS = 0x29;
// OLED assumption: SSD1306, 128x64, I2C address 0x3C. Change after checking
// the actual module; many 128x32 SSD1306 boards use 0x3C as well.
const uint8_t OLED_I2C_ADDRESS = 0x3C;
const uint8_t OLED_WIDTH = 128;
const uint8_t OLED_HEIGHT = 64;
const unsigned long SENSOR_POLL_INTERVAL_MS = 100;
const unsigned long OLED_REFRESH_INTERVAL_MS = 500;
// Set these from measured card data before enabling classification.
bool COLOR_CALIBRATION_CONFIGURED = false;
uint16_t COLOR_MIN_CLEAR = 0;
uint16_t COLOR_DOMINANCE_MARGIN = 0;
uint16_t COLOR_RED_MIN_RATIO_PERMILLE = 0;
uint16_t COLOR_BLUE_MIN_RATIO_PERMILLE = 0;
uint16_t COLOR_GREEN_MIN_RATIO_PERMILLE = 0;
const int COLOR_CONFIRM_SCANS = 5;
const int COLOR_EXIT_SCANS = 3;
const unsigned long COLOR_SCAN_INTERVAL_MS = 1000;
const unsigned long COLOR_SCAN_TIMEOUT_MS = 45000;
// Competition brief mappings; decoded readings still require calibration.
// Color 1 selects ball number, Color 2 station number, Color 3 return base.
const int COLOR_RED_ID = 1;
const int COLOR_BLUE_ID = 2;
const int COLOR_GREEN_ID = 3;
const int COLOR3_RED_BASE = 1;
const int COLOR3_BLUE_BASE = 2;
// Map specification: 19 mm line width, dimensions from line center, 10 mm gaps.
const int TRACK_LINE_WIDTH_MM = 19;
const int TRACK_DASH_GAP_MM = 10;

// ==================== MANUAL / BLUETOOTH SETTINGS ====================
int motorSpeed = 120;
String currentCommand = "S";
// Boot is always manual. Mode changes only on an explicit Bluetooth MODE command.
String operatingMode = "MANUAL";
String robotState = "IDLE";
unsigned long lastCommandTime = 0;
unsigned long lastStatusTime = 0;
const unsigned long COMMAND_TIMEOUT = 700;
const unsigned long STATUS_INTERVAL = 500;
uint8_t statusReportGroup = 0;

// ==================== AUTONOMOUS TUNING ====================
const unsigned long AUTO_START_DELAY_MS = 3000;
const unsigned long AUTO_SAMPLE_INTERVAL_MS = 5;
const int AUTO_BASE_SPEED = 125;
const int AUTO_MIN_MOTOR_SPEED = 55;
const int AUTO_MAX_MOTOR_SPEED = 200;
const float AUTO_STEERING_STRENGTH = 38.0f;
const int AUTO_MAX_CORRECTION = 90;
const int AUTO_CURVE_SPEED = 105;
const int AUTO_SHARP_TURN_SPEED = 110;   // outer wheel
const int AUTO_SHARP_INNER_SPEED = 28;   // inner wheel; no reverse steering
const int AUTO_RECOVERY_SPEED = 82;
const unsigned long LINE_LOST_GRACE_MS = 60;
const unsigned long AUTO_RECOVERY_TIMEOUT_MS = 1500;
const float SHARP_TURN_ERROR_THRESHOLD = 1.45f;
const int AUTO_INITIAL_SEARCH_DIRECTION = -1; // -1 left, +1 right

// ==================== JUNCTION CALIBRATION ====================
// Bit 0=S1 (far left) ... bit 4=S5 (far right); a set bit means BLACK.
// This provisional classifier requires centre + both side groups and at least
// three black sensors. All-black (00000 raw) is excluded by default because a
// broad black area is not proof of a junction. Tune after recording real track
// patterns. Set JUNCTION_ALLOW_ALL_BLACK=true only if testing confirms it.
const int JUNCTION_MIN_BLACK_COUNT = 3;
const bool JUNCTION_REQUIRE_CENTER = true;
const bool JUNCTION_REQUIRE_BOTH_SIDE_GROUPS = true;
const bool JUNCTION_ALLOW_ALL_BLACK = false;
// Optional exact pattern constraints; bit 0=S1 ... bit 4=S5, black=1 here.
// Example required center+outer sensors: 0b10101. Zero disables each mask.
const uint8_t JUNCTION_REQUIRED_BLACK_MASK = 0;
const uint8_t JUNCTION_FORBIDDEN_BLACK_MASK = 0;
const int JUNCTION_CONFIRM_SAMPLES = 4;
const int JUNCTION_EXIT_MAX_BLACK_COUNT = 2;
const bool JUNCTION_EXIT_REQUIRE_CENTER_BLACK = true;
const int JUNCTION_EXIT_CONFIRM_SAMPLES = 8;
const unsigned long JUNCTION_ACTION_TIMEOUT_MS = 3000;
const unsigned long WIDE_BLACK_STOP_MS = 250;
const int ALL_BLACK_CONFIRM_SAMPLES = 4;
const int ALL_BLACK_EXIT_CONFIRM_SAMPLES = 8;
const unsigned long ALL_BLACK_COOLDOWN_MS = 600;
const int FINAL_WHITE_CONFIRM_SAMPLES = 20; // 100 ms at the 5 ms sensor sampling interval; tune on track
const unsigned long FINAL_SEQUENCE_TIMEOUT_MS = 30000;
const int MAIN_FIRST_RIGHT_JUNCTION = 1; // map-derived; confirm on track
const int MAIN_SECOND_RIGHT_JUNCTION = 5; // top corner + 3 station branches + bottom corner
const unsigned long SERVO_SETTLE_MS = 700;
const int FINAL_BASE1_STATION_EXIT_TURN_CODE = 2; // from the station branch T, right enters the northbound route
const int FINAL_BASE2_STATION_EXIT_TURN_CODE = 1; // from the station branch T, left enters the southbound route
const int FINAL_BASE1_TURN_CODE = 1; // at the upper-right corner, turn left toward Base 1
const int FINAL_BASE2_TURN_CODE = 2; // at the lower-right corner, turn right toward Base 2

// ==================== TURNING / REACQUISITION CALIBRATION ====================
const int TURN_SPEED = 92;
const int TURN_ALIGN_SPEED = 82;
const int TURN_REACQUIRE_SEARCH_SPEED = 68;
const unsigned long TURN_INITIAL_MOTION_MS = 120; // initial movement only
const unsigned long TURN_TIMEOUT_MS = 4500;
const unsigned long UTURN_TIMEOUT_MS = 6500;
const unsigned long TURN_REACQUISITION_TIMEOUT_MS = 3000;
const int TURN_ALIGN_STABLE_SAMPLES = 3;
const int UTURN_DIRECTION = -1; // configurable rotation direction

// ==================== IR SENSOR PROCESSING ====================
struct IRFrame {
  bool black[5];
  int blackCount;
  uint8_t blackMask;
  bool lineDetected;
  bool wideBlack;
  float linePosition;
};

IRFrame irFrame = {};
uint8_t sensorHistory[5] = {0, 0, 0, 0, 0};
float lastKnownLinePosition = 0.0f;
int lastKnownLineDirection = AUTO_INITIAL_SEARCH_DIRECTION;
float smoothedLinePosition = 0.0f;
bool smoothingInitialized = false;

// ==================== NAVIGATION / MISSION MODEL ====================
enum NavigationAction : uint8_t {
  GO_STRAIGHT,
  TURN_LEFT_ACTION,
  TURN_RIGHT_ACTION,
  TURN_180_ACTION,
  STOP_ACTION,
  COLLECT_BALL_ACTION // future placeholder; never actuates hardware
};

enum MissionPhase : uint8_t {
  PHASE_START,
  PHASE_MAIN_TRACK,
  PHASE_BALL_SELECTION,
  PHASE_BALL_BRANCH,
  PHASE_BALL_COLLECTION_PENDING,
  PHASE_RETURN_NAVIGATION,
  PHASE_STATION_SELECTION,
  PHASE_STATION_BRANCH,
  PHASE_BALL_DELIVERY_PENDING,
  PHASE_FINAL_ROUTE,

  PHASE_FINISHED
};

enum TrackEventType : uint8_t {
  EVENT_NORMAL_JUNCTION,
  EVENT_ALL_BLACK,
  EVENT_WIDE_BLACK,
  EVENT_T_JUNCTION,
  EVENT_LINE_LOST,
  EVENT_UNKNOWN_TRACK
};
enum FinalSequenceState : uint8_t {
  FINISH_SEQ_WAIT_FIRST_BLACK,
  FINISH_SEQ_WAIT_WHITE,
  FINISH_SEQ_WAIT_SECOND_BLACK,
  FINISH_SEQ_COMPLETE
};
enum ServoMissionAction : uint8_t { SERVO_ACTION_NONE, SERVO_ACTION_BALL, SERVO_ACTION_STATION };

struct TrackEvent {
  TrackEventType type;
  String filteredPattern;
  uint8_t blackMask;
  uint8_t blackCount;
  unsigned long timestamp;
  unsigned long totalJunctionCount;
  unsigned long missionJunctionCount;
  MissionPhase phase;
};

struct MissionRouteStep {
  MissionPhase phase;
  TrackEventType eventType;
  uint16_t eventOrdinal;
  NavigationAction action;
  MissionPhase nextPhase;
  const char *segmentName;
  const char *targetName;
};

enum MissionState : uint8_t {
  MISSION_IDLE,
  AUTONOMOUS_START_DELAY,
  PHASE_1_MAIN_TRACK,
  COLOR_SCAN_PENDING,
  PHASE_2_BALL_BRANCH,
  BALL_COLLECTION_PENDING,
  PHASE_4_RETURN,
  PHASE_5_STATION_BRANCH,
  BALL_DELIVERY_PENDING,
  PHASE_7_FINAL_ROUTE,
  COLOR_SCAN_INCOMPLETE,
  SERVO_SETTLE,
  FINAL_WAIT_FIRST_BLACK,
  FINAL_WAIT_WHITE,
  FINAL_WAIT_SECOND_BLACK,
  LINE_FOLLOW,
  JUNCTION_CONFIRMATION,
  JUNCTION_ACTION,
  TURN_LEFT_STATE,
  TURN_RIGHT_STATE,
  TURN_180_STATE,
  LINE_REACQUISITION,
  LINE_RECOVERY,
  WAITING_FOR_MISSION_COMMAND,
  MISSION_PAUSED,
  MISSION_COMPLETE,
  ERROR_STOP
};

MissionState missionState = MISSION_IDLE;
MissionPhase currentPhase = PHASE_START;
MissionPhase pendingNextPhase = PHASE_MAIN_TRACK;
String currentSegment = "START";
String pendingNextSegment = "MAIN_TRACK";
String targetBall = "UNCONFIGURED";
String targetStation = "UNCONFIGURED";
String targetBase = "UNCONFIGURED";
bool finalRouteConfigured = false;
String color1 = "NOT_CALIBRATED";
String color2 = "NOT_CALIBRATED";
String color3 = "NOT_CALIBRATED";
String colorStatus = "POSTPONED";
String ballStatus = "NOT_STARTED";
uint16_t colorRawR = 0, colorRawG = 0, colorRawB = 0, colorRawClear = 0;
bool colorSensorReady = false;
bool colorDataValid = false;
bool oledReady = false;
unsigned long lastColorPollAt = 0;
unsigned long lastOledRefreshAt = 0;
uint8_t oledBuffer[OLED_WIDTH * OLED_HEIGHT / 8] = {0};
bool actuatorPwmReady = false;
bool actuatorHomeConfigured = false;
int actuatorClockwiseSign = 0; // 0=not calibrated, +1 or -1 after a safe direction check
int actuatorCurrentAngle = 0;
String actuatorStatus = "NOT_CONFIGURED";
NavigationAction lastJunctionAction = STOP_ACTION;
String lastJunctionActionName = "NONE";
String errorStatus = "NONE";

void queueBluetoothMessage(const String &line, bool priority = false) {
  if (!SerialBT.connected()) return;
  QueueHandle_t queue = priority ? bluetoothAckQueue : bluetoothStatusQueue;
  if (queue == nullptr) return;
  BluetoothTxMessage message = {};
  line.toCharArray(message.text, sizeof(message.text));
  if (xQueueSend(queue, &message, 0) != pdTRUE && priority) errorStatus = "BT_ACK_QUEUE_FULL";
}

void bluetoothTxWorker(void *parameter) {
  (void)parameter;
  BluetoothTxMessage message;
  for (;;) {
    BaseType_t received = xQueueReceive(bluetoothAckQueue, &message, 0);
    if (received != pdTRUE) received = xQueueReceive(bluetoothStatusQueue, &message, 0);
    if (received == pdTRUE) {
      if (SerialBT.connected()) SerialBT.println(message.text);
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

bool parseUnsignedRange(String value, int minimum, int maximum, int &parsed) {
  value.trim();
  if (value.length() == 0) return false;
  int result = 0;
  for (size_t i = 0; i < value.length(); i++) {
    const char digit = value[i];
    if (digit < '0' || digit > '9') return false;
    result = result * 10 + (digit - '0');
    if (result > maximum) return false;
  }
  if (result < minimum) return false;
  parsed = result;
  return true;
}

const uint8_t NAV_SEQUENCE_CAPACITY = 32;
NavigationAction navigationSequence[NAV_SEQUENCE_CAPACITY];
uint8_t navigationSequenceLength = 0;
uint8_t navigationSequenceIndex = 0;
unsigned long totalJunctionCount = 0;
unsigned long missionJunctionCount = 0;
unsigned long ballJunctionCount = 0;
unsigned long stationJunctionCount = 0;
unsigned long finalJunctionCount = 0;
unsigned long allBlackEventCount = 0;
unsigned long currentPhaseEventCount = 0;
uint16_t finalCornerJunctionOrdinal = 0;
bool finalStationExitTurnExecuted = false;
bool finalCornerTurnExecuted = false;
FinalSequenceState finalSequenceState = FINISH_SEQ_WAIT_FIRST_BLACK;
ServoMissionAction pendingServoAction = SERVO_ACTION_NONE;
bool allBlackArmed = true;
int allBlackCandidateSamples = 0;
int allBlackExitSamples = 0;
unsigned long lastAllBlackEventAt = 0;
bool junctionRearmPending = false;
int allWhiteCandidateSamples = 0;
int finishMarkerCount = 0;
unsigned long finalSequenceStartedAt = 0;
unsigned long servoSettleStartedAt = 0;
int pendingServoDelta = 0;
bool ballActuationComplete = false;
bool stationActuationComplete = false;
bool colorScanningEnabled = false;
bool colorMarkerArmed = true;
uint32_t colorAccumR = 0, colorAccumG = 0, colorAccumB = 0, colorAccumClear = 0;
uint16_t colorAccumCount = 0;
unsigned long colorScanStartedAt = 0, lastColorScanAt = 0;
int colorCandidateId = 0, colorCandidateStreak = 0, colorExitStreak = 0;
int colorScanStoredCount = 0;
int targetStationEncounter = 0;
uint16_t phaseJunctionOrdinal = 0;
unsigned long lastManualIRSampleAt = 0;
const uint8_t ROUTE_TABLE_CAPACITY = 32;
MissionRouteStep missionRoute[ROUTE_TABLE_CAPACITY];
uint8_t missionRouteLength = 0;
uint16_t phaseEventCounters[6] = {0, 0, 0, 0, 0, 0};

bool junctionArmed = true;
int junctionCandidateSamples = 0;
int junctionExitSamples = 0;
unsigned long junctionActionStartedAt = 0;
unsigned long autoStartStartedAt = 0;
unsigned long lastAutoSampleAt = 0;
unsigned long lineLostStartedAt = 0;
unsigned long wideBlackStartedAt = 0;
unsigned long recoveryStartedAt = 0;
unsigned long turnStartedAt = 0;
unsigned long reacquisitionStartedAt = 0;
unsigned long activeTurnTimeoutMs = TURN_TIMEOUT_MS;
bool turnSawJunctionExit = false;
int turnAlignedSamples = 0;
int activeTurnDirection = -1;

// ==================== I2C COLOR SENSOR + OLED ====================
void i2cWrite(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write((uint8_t)(0x80 | reg));
  Wire.write(value);
  Wire.endTransmission();
}

bool i2cRead(uint8_t address, uint8_t command, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(address);
  Wire.write(command);
  if (Wire.endTransmission(false) != 0) return false;
  const uint8_t received = Wire.requestFrom(address, count);
  if (received != count) return false;
  for (uint8_t i = 0; i < count; i++) data[i] = Wire.read();
  return true;
}

bool oledCommand(uint8_t command) {
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  Wire.write((uint8_t)0x00);
  Wire.write(command);
  return Wire.endTransmission() == 0;
}

bool oledInit() {
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) return false;
  const uint8_t initCommands[] = {0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,
    0x8D,0x14,0x20,0x00,0xA1,0xC8,0xDA,0x12,0x81,0x7F,0xD9,0xF1,0xDB,0x40,
    0xA4,0xA6,0xAF};
  for (uint8_t c : initCommands) if (!oledCommand(c)) return false;
  memset(oledBuffer, 0, sizeof(oledBuffer));
  return true;
}

uint8_t glyphRow(char c, uint8_t row) {
  // Compact 3x5 uppercase font, sufficient for the OLED status screen.
  static const char chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789:-./_ ";
  static const uint8_t glyphs[][5] = {
    {2,5,7,5,5},{6,5,6,5,6},{3,4,4,4,3},{6,5,5,5,6},{7,4,6,4,7},
    {7,4,6,4,4},{3,4,5,5,3},{5,5,7,5,5},{7,2,2,2,7},{1,1,1,5,2},
    {5,5,6,5,5},{4,4,4,4,7},{5,7,7,5,5},{5,7,7,7,5},{2,5,5,5,2},
    {6,5,6,4,4},{2,5,5,3,1},{6,5,6,5,5},{3,4,2,1,6},{7,2,2,2,2},
    {5,5,5,5,7},{5,5,5,5,2},{5,5,7,7,5},{5,5,2,5,5},{5,5,2,2,2},
    {7,1,2,4,7},
    {2,5,5,5,2},{2,6,2,2,7},{6,1,2,4,7},{6,1,2,1,6},{5,5,7,1,1},
    {7,4,6,1,6},{3,4,6,5,2},{7,1,2,2,2},{2,5,2,5,2},{2,5,3,1,6},
    {2,2,2,0,2},{0,0,0,0,7},{0,0,0,0,0},{0,2,0,2,0},
    {0,0,7,0,0},{0,0,0,0,0}
  };
  if (row >= 5) return 0;
  if (c >= 'a' && c <= 'z') c -= ('a' - 'A');
  for (uint8_t i = 0; i < sizeof(chars) - 1; i++) if (chars[i] == c) return glyphs[i][row];
  return 0;
}

void oledPixel(int x, int y, bool on) {
  if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
  const uint16_t index = x + (y / 8) * OLED_WIDTH;
  const uint8_t mask = (uint8_t)(1U << (y & 7));
  if (on) oledBuffer[index] |= mask;
  else oledBuffer[index] &= (uint8_t)~mask;
}

void oledText(int x, int y, String text) {
  for (uint16_t i = 0; i < text.length() && x + i * 4 < OLED_WIDTH; i++) {
    for (uint8_t row = 0; row < 5; row++) {
      const uint8_t bits = glyphRow(text[i], row);
      for (uint8_t col = 0; col < 3; col++) oledPixel(x + i * 4 + col, y + row, (bits & (1U << (2-col))) != 0);
    }
  }
}

void oledFlush() {
  if (!oledReady) return;
  for (uint8_t page = 0; page < 8; page++) {
    oledCommand((uint8_t)(0xB0 | page)); oledCommand(0x00); oledCommand(0x10);
    for (uint8_t start = 0; start < OLED_WIDTH; start += 24) {
      Wire.beginTransmission(OLED_I2C_ADDRESS);
      Wire.write((uint8_t)0x40);
      for (uint8_t x = start; x < start + 24; x++) Wire.write(oledBuffer[page * OLED_WIDTH + x]);
      Wire.endTransmission();
    }
  }
}

void refreshOled() {
  if (!oledReady || millis() - lastOledRefreshAt < OLED_REFRESH_INTERVAL_MS) return;
  lastOledRefreshAt = millis();
  memset(oledBuffer, 0, sizeof(oledBuffer));
  oledText(0, 1, "MODE:" + operatingMode);
  oledText(0, 11, "STATE:" + robotState);
  oledText(0, 21, "PHASE:" + phaseName(currentPhase));
  oledText(0, 31, "RGB:" + String(colorRawR) + "," + String(colorRawG) + "," + String(colorRawB));
  oledText(0, 41, "IR:" + readIRPattern());
  oledText(0, 51, actuatorHomeConfigured ? "ACT:" + String(actuatorCurrentAngle) : "ACT:SET-HOME");
  oledFlush();
}

void initColorSensor() {
  uint8_t id = 0;
  if (!i2cRead(COLOR_I2C_ADDRESS, 0x92, &id, 1) || (id != 0x44 && id != 0x4D)) {
    colorSensorReady = false;
    colorStatus = "SENSOR_NOT_FOUND";
    return;
  }
  i2cWrite(COLOR_I2C_ADDRESS, 0x00, 0x01); // power on
  delay(3);
  i2cWrite(COLOR_I2C_ADDRESS, 0x01, 0xEB); // 50.4 ms integration
  i2cWrite(COLOR_I2C_ADDRESS, 0x0F, 0x01); // 4x gain, adjustable
  i2cWrite(COLOR_I2C_ADDRESS, 0x00, 0x03); // power + RGBC ADC
  colorSensorReady = true;
  colorStatus = "RGB_RAW_UNCALIBRATED";
}

void pollColorSensor() {
  if (!colorSensorReady || millis() - lastColorPollAt < SENSOR_POLL_INTERVAL_MS) return;
  lastColorPollAt = millis();
  uint8_t status = 0;
  if (!i2cRead(COLOR_I2C_ADDRESS, 0x93, &status, 1) || !(status & 0x01)) return;
  uint8_t bytes[8];
  if (!i2cRead(COLOR_I2C_ADDRESS, 0xB4, bytes, 8)) return; // auto-increment from CDATA
  colorRawClear = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
  colorRawR = (uint16_t)bytes[2] | ((uint16_t)bytes[3] << 8);
  colorRawG = (uint16_t)bytes[4] | ((uint16_t)bytes[5] << 8);
  colorRawB = (uint16_t)bytes[6] | ((uint16_t)bytes[7] << 8);
  colorDataValid = true;
  if (colorScanningEnabled) {
    colorAccumR += colorRawR; colorAccumG += colorRawG;
    colorAccumB += colorRawB; colorAccumClear += colorRawClear;
    if (colorAccumCount < 1000) colorAccumCount++;
  }
}

int classifyColorScan(uint16_t r, uint16_t g, uint16_t b, uint16_t clear) {
  if (!COLOR_CALIBRATION_CONFIGURED || clear < COLOR_MIN_CLEAR) return 0;
  const uint32_t sum = (uint32_t)r + g + b;
  if (sum == 0) return 0;
  const uint16_t rn = (uint16_t)((uint32_t)r * 1000 / sum);
  const uint16_t gn = (uint16_t)((uint32_t)g * 1000 / sum);
  const uint16_t bn = (uint16_t)((uint32_t)b * 1000 / sum);
  if (rn >= COLOR_RED_MIN_RATIO_PERMILLE && rn > gn + COLOR_DOMINANCE_MARGIN && rn > bn + COLOR_DOMINANCE_MARGIN) return COLOR_RED_ID;
  if (bn >= COLOR_BLUE_MIN_RATIO_PERMILLE && bn > rn + COLOR_DOMINANCE_MARGIN && bn > gn + COLOR_DOMINANCE_MARGIN) return COLOR_BLUE_ID;
  if (gn >= COLOR_GREEN_MIN_RATIO_PERMILLE && gn > rn + COLOR_DOMINANCE_MARGIN && gn > bn + COLOR_DOMINANCE_MARGIN) return COLOR_GREEN_ID;
  return 0; // NO_COLOR / ambiguous
}

void acceptColorScan(int id) {
  if (!colorMarkerArmed) {
    if (id == 0) {
      if (++colorExitStreak >= COLOR_EXIT_SCANS) {
        colorMarkerArmed = true;
        colorExitStreak = 0;
        colorCandidateId = 0;
        colorCandidateStreak = 0;
      }
    } else colorExitStreak = 0;
    return;
  }
  if (id == 0) { colorCandidateId = 0; colorCandidateStreak = 0; return; }
  if (id == colorCandidateId) colorCandidateStreak++;
  else { colorCandidateId = id; colorCandidateStreak = 1; }
  if (colorCandidateStreak < COLOR_CONFIRM_SCANS) return;

  const String value = String(id);
  if (colorScanStoredCount == 0) color1 = value;
  else if (colorScanStoredCount == 1) color2 = value;
  else if (colorScanStoredCount == 2) color3 = value;
  if (colorScanStoredCount < 3) colorScanStoredCount++;
  colorStatus = "CAPTURED_" + String(colorScanStoredCount);
  colorMarkerArmed = false;
  colorExitStreak = 0;
  colorCandidateStreak = 0;
  if (colorScanStoredCount == 3) {
    colorStatus = "THREE_COLORS_CAPTURED";
    colorScanningEnabled = false;
  }
}

void updateColorScan(unsigned long now) {
  if (!colorScanningEnabled || currentPhase != PHASE_MAIN_TRACK) return;
  if (now - colorScanStartedAt >= COLOR_SCAN_TIMEOUT_MS) {
    colorStatus = "COLOR_SCAN_TIMEOUT";
    failMission("COLOR_SCAN_INCOMPLETE");
    return;
  }
  if (!colorSensorReady) return;
  if (now - lastColorScanAt < COLOR_SCAN_INTERVAL_MS) return;
  lastColorScanAt = now;
  if (colorAccumCount == 0) return;
  const uint16_t r = colorAccumR / colorAccumCount;
  const uint16_t g = colorAccumG / colorAccumCount;
  const uint16_t b = colorAccumB / colorAccumCount;
  const uint16_t c = colorAccumClear / colorAccumCount;
  colorAccumR = colorAccumG = colorAccumB = colorAccumClear = 0;
  colorAccumCount = 0;
  acceptColorScan(classifyColorScan(r, g, b, c));
}

bool actuatorWriteAngle(int angle) {
  if (angle < 0 || angle > 180) { actuatorStatus = "ANGLE_OUT_OF_RANGE"; return false; }
  if (!actuatorPwmReady) {
    actuatorPwmReady = ledcAttach(ACTUATOR_SERVO_PIN, SERVO_PWM_FREQUENCY, SERVO_PWM_RESOLUTION);
    if (!actuatorPwmReady) { actuatorStatus = "PWM_ATTACH_FAILED"; return false; }
  }
  const int pulseUs = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  const uint32_t duty = ((uint64_t)pulseUs * 65535ULL) / 20000ULL;
  ledcWrite(ACTUATOR_SERVO_PIN, duty);
  actuatorCurrentAngle = angle;
  actuatorHomeConfigured = true;
  actuatorStatus = "TARGET_SET_NO_FEEDBACK";
  return true;
}

bool rotateActuatorClockwise(int degrees) {
  if (!actuatorHomeConfigured) { actuatorStatus = "SET_HOME_FIRST"; return false; }
  if (actuatorClockwiseSign == 0) { actuatorStatus = "SET_CW_DIRECTION_FIRST"; return false; }
  if (degrees <= 0 || degrees > 180) { actuatorStatus = "BAD_CW_STEP"; return false; }
  const int targetAngle = actuatorCurrentAngle + actuatorClockwiseSign * degrees;
  if (targetAngle < 0 || targetAngle > 180) { actuatorStatus = "ANGLE_LIMIT"; return false; }
  return actuatorWriteAngle(targetAngle);
}

// ==================== COMMON MOTOR CONTROL ====================
int boundedPwm(int value) { return constrain(value, 0, 255); }

// Positive wheel values preserve the established manual forward directions.
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed > 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else if (leftSpeed < 0) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); }

  if (rightSpeed > 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else if (rightSpeed < 0) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); }

  analogWrite(PWMA, boundedPwm(abs(leftSpeed)));
  analogWrite(PWMB, boundedPwm(abs(rightSpeed)));
}

void stopMotorOutputs() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

void stopMotors() { stopMotorOutputs(); robotState = "IDLE"; }
void moveForward() { setMotorSpeeds(motorSpeed, motorSpeed); robotState = "FORWARD"; }
void moveBackward() { setMotorSpeeds(-motorSpeed, -motorSpeed); robotState = "BACKWARD"; }
void turnLeft() { setMotorSpeeds(-motorSpeed, motorSpeed); robotState = "LEFT"; }
void turnRight() { setMotorSpeeds(motorSpeed, -motorSpeed); robotState = "RIGHT"; }

// ==================== IR SENSOR PROCESSING MODULE ====================
void resetIRFilter() {
  for (int i = 0; i < 5; i++) {
    sensorHistory[i] = 0;
    irFrame.black[i] = false;
  }
  irFrame.blackCount = 0;
  irFrame.blackMask = 0;
  irFrame.lineDetected = false;
  irFrame.wideBlack = false;
  irFrame.linePosition = 0.0f;
}

void sampleIRFrame() {
  static const int weights[5] = {-2, -1, 0, 1, 2};
  irFrame.blackCount = 0;
  irFrame.blackMask = 0;
  int weightedTotal = 0;
  int activeCount = 0;

  for (int i = 0; i < 5; i++) {
    const bool rawBlack = (digitalRead(IR_PINS[i]) == LOW); // confirmed 0=black
    sensorHistory[i] = (uint8_t)(((sensorHistory[i] << 1) | (rawBlack ? 1 : 0)) & 0x07);
    const int votes = ((sensorHistory[i] >> 0) & 1) +
                      ((sensorHistory[i] >> 1) & 1) +
                      ((sensorHistory[i] >> 2) & 1);
    irFrame.black[i] = (votes >= 2);
    if (irFrame.black[i]) {
      irFrame.blackCount++;
      irFrame.blackMask |= (uint8_t)(1U << i);
      weightedTotal += weights[i];
      activeCount++;
    }
  }

  irFrame.lineDetected = (activeCount > 0);
  irFrame.linePosition = irFrame.lineDetected ?
      (float)weightedTotal / (float)activeCount : lastKnownLinePosition;
  irFrame.wideBlack = (irFrame.blackCount >= 4);
}

bool getLinePosition(float &position) {
  if (!irFrame.lineDetected) return false;
  position = irFrame.linePosition;
  return true;
}

bool detectWideBlack() { return irFrame.wideBlack; }
bool detectAllBlack() { return (irFrame.blackMask == 0x1F); }

bool stableNormalLinePattern() {
  const bool central = !JUNCTION_EXIT_REQUIRE_CENTER_BLACK || irFrame.black[1] || irFrame.black[2] || irFrame.black[3];
  return irFrame.blackCount >= 1 && irFrame.blackCount <= 2 && central &&
         !detectAllBlack() && irFrame.blackMask != 0;
}

void updateAllBlackRearm() {
  if (detectAllBlack()) { allBlackExitSamples = 0; return; }
  if (stableNormalLinePattern()) {
    if (allBlackExitSamples < ALL_BLACK_EXIT_CONFIRM_SAMPLES) allBlackExitSamples++;
    if (allBlackExitSamples >= ALL_BLACK_EXIT_CONFIRM_SAMPLES) allBlackArmed = true;
  } else allBlackExitSamples = 0;
}

bool confirmAllBlack() {
  if (!detectAllBlack()) {
    allBlackCandidateSamples = 0;
    updateAllBlackRearm();
    return false;
  }
  allBlackExitSamples = 0;
  if (!allBlackArmed || millis() - lastAllBlackEventAt < ALL_BLACK_COOLDOWN_MS) return false;
  if (allBlackCandidateSamples < ALL_BLACK_CONFIRM_SAMPLES) allBlackCandidateSamples++;
  if (allBlackCandidateSamples < ALL_BLACK_CONFIRM_SAMPLES) return false;
  allBlackArmed = false;
  allBlackCandidateSamples = 0;
  lastAllBlackEventAt = millis();
  return true;
}

bool isAllBlackExited() { return allBlackArmed; }

// Junction candidates are deliberately configurable and provisional. The
// all-black pattern is excluded by default and therefore cannot alone trigger.
bool detectJunction() {
  const bool center = irFrame.black[2];
  const bool leftSide = irFrame.black[0] || irFrame.black[1];
  const bool rightSide = irFrame.black[3] || irFrame.black[4];
  const bool allBlack = (irFrame.blackCount == 5);

  if (irFrame.blackCount < JUNCTION_MIN_BLACK_COUNT) return false;
  if (JUNCTION_REQUIRE_CENTER && !center) return false;
  if (JUNCTION_REQUIRE_BOTH_SIDE_GROUPS && !(leftSide && rightSide)) return false;
  if ((irFrame.blackMask & JUNCTION_REQUIRED_BLACK_MASK) != JUNCTION_REQUIRED_BLACK_MASK) return false;
  if ((irFrame.blackMask & JUNCTION_FORBIDDEN_BLACK_MASK) != 0) return false;
  if (!JUNCTION_ALLOW_ALL_BLACK && allBlack) return false;
  return true;
}

bool confirmJunction() {
  if (!detectJunction()) {
    junctionCandidateSamples = 0;
    return false;
  }
  if (junctionCandidateSamples < JUNCTION_CONFIRM_SAMPLES) junctionCandidateSamples++;
  return junctionCandidateSamples >= JUNCTION_CONFIRM_SAMPLES;
}

bool isJunctionExited() {
  if (stableNormalLinePattern() && irFrame.blackCount <= JUNCTION_EXIT_MAX_BLACK_COUNT) {
    if (junctionExitSamples < JUNCTION_EXIT_CONFIRM_SAMPLES) junctionExitSamples++;
  } else {
    junctionExitSamples = 0;
  }
  return junctionExitSamples >= JUNCTION_EXIT_CONFIRM_SAMPLES;
}

void resetJunctionDetection() {
  junctionArmed = true;
  junctionCandidateSamples = 0;
  junctionExitSamples = 0;
  junctionRearmPending = false;
}

String readIRSensors() {
  // Preserve the existing raw comma-separated protocol and S1..S5 order.
  String result = "";
  for (int i = 0; i < 5; i++) {
    result += String(digitalRead(IR_PINS[i]));
    if (i < 4) result += ",";
  }
  return result;
}

String readIRPattern() {
  // Human-readable filtered pattern; 0=black, 1=white.
  String pattern = "";
  for (int i = 0; i < 5; i++) pattern += irFrame.black[i] ? "0" : "1";
  return pattern;
}

// ==================== LINE FOLLOWING / RECOVERY ====================
int clampAutoSpeed(int speed) {
  return constrain(speed, AUTO_MIN_MOTOR_SPEED, AUTO_MAX_MOTOR_SPEED);
}

void followPosition(float position, int baseSpeed) {
  if (!smoothingInitialized) {
    smoothedLinePosition = position;
    smoothingInitialized = true;
  } else {
    smoothedLinePosition = 0.55f * position + 0.45f * smoothedLinePosition;
  }
  int correction = (int)(smoothedLinePosition * AUTO_STEERING_STRENGTH);
  correction = constrain(correction, -AUTO_MAX_CORRECTION, AUTO_MAX_CORRECTION);

  // Positive position=line right: left wheel faster, right wheel slower.
  setMotorSpeeds(clampAutoSpeed(baseSpeed + correction),
                 clampAutoSpeed(baseSpeed - correction));
}

void setMissionState(MissionState state, const char *name) {
  missionState = state;
  robotState = name;
}

void failMission(const char *reason) {
  stopMotorOutputs();
  errorStatus = reason;
  setMissionState(ERROR_STOP, "ERROR_STOP");
}

String actionName(NavigationAction action) {
  switch (action) {
    case GO_STRAIGHT: return "GO_STRAIGHT";
    case TURN_LEFT_ACTION: return "TURN_LEFT";
    case TURN_RIGHT_ACTION: return "TURN_RIGHT";
    case TURN_180_ACTION: return "TURN_180";
    case STOP_ACTION: return "STOP";
    case COLLECT_BALL_ACTION: return "COLLECT_BALL_PENDING";
  }
  return "INVALID";
}

String missionStateName() {
  switch (missionState) {
    case MISSION_IDLE: return "IDLE";
    case AUTONOMOUS_START_DELAY: return "AUTONOMOUS_START_DELAY";
    case PHASE_1_MAIN_TRACK: return "PHASE_1_MAIN_TRACK";
    case COLOR_SCAN_PENDING: return "COLOR_SCAN_PENDING";
    case PHASE_2_BALL_BRANCH: return "PHASE_2_BALL_BRANCH";
    case BALL_COLLECTION_PENDING: return "BALL_COLLECTION_PENDING";
    case PHASE_4_RETURN: return "PHASE_4_RETURN";
    case PHASE_5_STATION_BRANCH: return "PHASE_5_STATION_BRANCH";
    case BALL_DELIVERY_PENDING: return "BALL_DELIVERY_PENDING";
    case PHASE_7_FINAL_ROUTE: return "PHASE_7_FINAL_ROUTE";
    case COLOR_SCAN_INCOMPLETE: return "COLOR_SCAN_INCOMPLETE";
    case SERVO_SETTLE: return "SERVO_SETTLE";
    case FINAL_WAIT_FIRST_BLACK: return "FINAL_WAIT_FIRST_BLACK";
    case FINAL_WAIT_WHITE: return "FINAL_WAIT_WHITE";
    case FINAL_WAIT_SECOND_BLACK: return "FINAL_WAIT_SECOND_BLACK";
    case LINE_FOLLOW: return "LINE_FOLLOW";
    case JUNCTION_CONFIRMATION: return "JUNCTION_CONFIRMATION";
    case JUNCTION_ACTION: return "JUNCTION_ACTION";
    case TURN_LEFT_STATE: return "TURN_LEFT";
    case TURN_RIGHT_STATE: return "TURN_RIGHT";
    case TURN_180_STATE: return "TURN_180";
    case LINE_REACQUISITION: return "LINE_REACQUISITION";
    case LINE_RECOVERY: return "LINE_RECOVERY";
    case WAITING_FOR_MISSION_COMMAND: return "WAITING_FOR_MISSION_COMMAND";
    case MISSION_PAUSED: return "MISSION_PAUSED";
    case MISSION_COMPLETE: return "MISSION_COMPLETE";
    case ERROR_STOP: return "ERROR_STOP";
  }
  return "INVALID";
}

String phaseName(MissionPhase phase) {
  switch (phase) {
    case PHASE_START: return "START";
    case PHASE_MAIN_TRACK: return "MAIN_TRACK";
    case PHASE_BALL_SELECTION: return "BALL_SELECTION";
    case PHASE_BALL_BRANCH: return "BALL_BRANCH";
    case PHASE_BALL_COLLECTION_PENDING: return "BALL_COLLECTION_PENDING";
    case PHASE_RETURN_NAVIGATION: return "RETURN_NAVIGATION";
    case PHASE_STATION_SELECTION: return "STATION_SELECTION";
    case PHASE_STATION_BRANCH: return "STATION_BRANCH";
    case PHASE_BALL_DELIVERY_PENDING: return "BALL_DELIVERY_PENDING";
    case PHASE_FINAL_ROUTE: return "FINAL_ROUTE";
    case PHASE_FINISHED: return "FINISHED";
  }
  return "INVALID";
}

String eventTypeName(TrackEventType type) {
  switch (type) {
    case EVENT_NORMAL_JUNCTION: return "NORMAL_JUNCTION";
    case EVENT_ALL_BLACK: return "ALL_BLACK";
    case EVENT_WIDE_BLACK: return "WIDE_BLACK";
    case EVENT_T_JUNCTION: return "T_JUNCTION";
    case EVENT_LINE_LOST: return "LINE_LOST";
    case EVENT_UNKNOWN_TRACK: return "UNKNOWN_TRACK_EVENT";
  }
  return "UNKNOWN_TRACK_EVENT";
}

void enterMissionPhase(MissionPhase phase, const char *segment) {
  currentPhase = phase;
  currentSegment = segment;
  phaseJunctionOrdinal = 0;
  if (phase == PHASE_BALL_SELECTION) ballJunctionCount = 0;
  if (phase == PHASE_STATION_SELECTION) stationJunctionCount = 0;
  if (phase == PHASE_FINAL_ROUTE) {
    finalJunctionCount = 0;
    finalSequenceState = FINISH_SEQ_WAIT_FIRST_BLACK;
    finishMarkerCount = 0;
    allWhiteCandidateSamples = 0;
    allBlackArmed = true;
    allBlackCandidateSamples = 0;
    finalStationExitTurnExecuted = false;
    finalCornerTurnExecuted = false;
    finalSequenceStartedAt = millis();
    const int station = color2.toInt();
    if (station < 1 || station > 3) { failMission("INVALID_COLOR2_STATION_ID"); return; }
    if (color3 == String(COLOR_RED_ID)) {
      targetBase = "BASE_1";
      finalCornerJunctionOrdinal = (uint16_t)(station + 1);
    } else if (color3 == String(COLOR_BLUE_ID)) {
      targetBase = "BASE_2";
      finalCornerJunctionOrdinal = (uint16_t)(5 - station);
    } else { failMission("INVALID_COLOR3_BASE_ID"); return; }
  }
}

void startMarkerServoAction(ServoMissionAction action) {
  if (!actuatorHomeConfigured || actuatorClockwiseSign == 0) {
    failMission("SERVO_NOT_CALIBRATED");
    return;
  }
  const int delta = (action == SERVO_ACTION_BALL ? 45 : -45) * actuatorClockwiseSign;
  const int target = actuatorCurrentAngle + delta;
  if (!actuatorWriteAngle(target)) { failMission("SERVO_ANGLE_LIMIT"); return; }
  pendingNextPhase = (action == SERVO_ACTION_BALL) ? PHASE_RETURN_NAVIGATION : PHASE_FINAL_ROUTE;
  pendingNextSegment = (action == SERVO_ACTION_BALL) ? "RETURN_NAVIGATION" : "FINAL_ROUTE";
  pendingServoAction = action;
  pendingServoDelta = delta;
  servoSettleStartedAt = millis();
  stopMotorOutputs();
  ballStatus = (action == SERVO_ACTION_BALL) ? "BALL_MARKER_ACTUATED" : "STATION_MARKER_ACTUATED";
  setMissionState(SERVO_SETTLE, "SERVO_SETTLE");
}

void reportTrackEvent(TrackEventType type) {
  if (type == EVENT_ALL_BLACK) allBlackEventCount++;
  if (type == EVENT_NORMAL_JUNCTION || type == EVENT_T_JUNCTION) {
    totalJunctionCount++;
    missionJunctionCount++;
  }
  if (SerialBT.connected()) {
    queueBluetoothMessage("EVENT:TRACK:" + eventTypeName(type) + ":" + readIRPattern() + ":" +
      String(irFrame.blackCount) + ":" + String(irFrame.blackMask) + ":" + String(millis()) + ":" +
      String(totalJunctionCount) + ":" + String(missionJunctionCount) + ":" + phaseName(currentPhase));
  }
  if (type != EVENT_ALL_BLACK) return;

  // All-black markers are phase events, never normal junctions.
  if (currentPhase == PHASE_BALL_BRANCH && !ballActuationComplete) {
    startMarkerServoAction(SERVO_ACTION_BALL);
  } else if (currentPhase == PHASE_STATION_BRANCH && !stationActuationComplete) {
    startMarkerServoAction(SERVO_ACTION_STATION);
  } else if (currentPhase == PHASE_FINAL_ROUTE) {
    if (finalSequenceState == FINISH_SEQ_WAIT_FIRST_BLACK) {
      finishMarkerCount = 1;
      finalSequenceState = FINISH_SEQ_WAIT_WHITE;
      setMissionState(FINAL_WAIT_WHITE, "FINAL_WAIT_WHITE");
    } else if (finalSequenceState == FINISH_SEQ_WAIT_SECOND_BLACK) {
      finishMarkerCount = 2;
      finalSequenceState = FINISH_SEQ_COMPLETE;
      currentPhase = PHASE_FINISHED;
      stopMotorOutputs();
      setMissionState(MISSION_COMPLETE, "MISSION_COMPLETE");
      queueBluetoothMessage("EVENT:MISSION_COMPLETE", true);
    }
  }
  // In other phases the marker is observed and locked, but causes no motion.
}

void updateFinalSequence(unsigned long now) {
  if (currentPhase != PHASE_FINAL_ROUTE) return;
  if (now - finalSequenceStartedAt >= FINAL_SEQUENCE_TIMEOUT_MS) {
    failMission("FINAL_SEQUENCE_TIMEOUT");
    return;
  }
  if (finalSequenceState != FINISH_SEQ_WAIT_WHITE) return;
  if (irFrame.blackMask == 0) {
    if (allWhiteCandidateSamples < FINAL_WHITE_CONFIRM_SAMPLES) allWhiteCandidateSamples++;
    if (allWhiteCandidateSamples >= FINAL_WHITE_CONFIRM_SAMPLES) {
      finalSequenceState = FINISH_SEQ_WAIT_SECOND_BLACK;
      allBlackArmed = true;
      lastAllBlackEventAt = now - ALL_BLACK_COOLDOWN_MS; // confirmed white separator authorizes the next marker
      allBlackCandidateSamples = 0;
      allBlackExitSamples = 0;
      setMissionState(FINAL_WAIT_SECOND_BLACK, "FINAL_WAIT_SECOND_BLACK");
    }
  } else allWhiteCandidateSamples = 0;
}

String finalSequenceName() {
  switch (finalSequenceState) {
    case FINISH_SEQ_WAIT_FIRST_BLACK: return "FINAL_WAIT_FIRST_BLACK";
    case FINISH_SEQ_WAIT_WHITE: return "FINAL_WAIT_WHITE";
    case FINISH_SEQ_WAIT_SECOND_BLACK: return "FINAL_WAIT_SECOND_BLACK";
    case FINISH_SEQ_COMPLETE: return "MISSION_COMPLETE";
  }
  return "INVALID";
}

String junctionTypeName() {
  if (irFrame.blackCount == 5) return "WIDE_BLACK_AMBIGUOUS";
  if (detectJunction()) return "POSSIBLE_T_OR_CROSS";
  if (detectWideBlack()) return "WIDE_BLACK_AMBIGUOUS";
  return "NONE";
}

bool parseAction(String value, NavigationAction &action) {
  value.trim(); value.toUpperCase();
  if (value == "STRAIGHT" || value == "GO_STRAIGHT") action = GO_STRAIGHT;
  else if (value == "LEFT" || value == "TURN_LEFT") action = TURN_LEFT_ACTION;
  else if (value == "RIGHT" || value == "TURN_RIGHT") action = TURN_RIGHT_ACTION;
  else if (value == "U" || value == "UTURN" || value == "TURN_180") action = TURN_180_ACTION;
  else if (value == "STOP") action = STOP_ACTION;
  else return false;
  return true;
}

// API for a mission controller or future map module. Each entry is the action
// to execute at the next confirmed junction; line following is implicit between.
bool setNavigationSequence(const NavigationAction *sequence, uint8_t length) {
  if (length > NAV_SEQUENCE_CAPACITY || (length > 0 && sequence == nullptr)) return false;
  for (uint8_t i = 0; i < length; i++) navigationSequence[i] = sequence[i];
  navigationSequenceLength = length;
  navigationSequenceIndex = 0;
  return true;
}

bool setMissionRoute(const MissionRouteStep *steps, uint8_t length) {
  if (length > ROUTE_TABLE_CAPACITY || (length && steps == nullptr)) return false;
  for (uint8_t i = 0; i < length; i++) missionRoute[i] = steps[i];
  missionRouteLength = length;
  return true;
}

void resetMissionCounters(bool resetTotal) {
  missionJunctionCount = 0;
  ballJunctionCount = 0;
  stationJunctionCount = 0;
  finalJunctionCount = 0;
  phaseJunctionOrdinal = 0;
  navigationSequenceIndex = 0;
  if (resetTotal) {
    totalJunctionCount = 0;
    allBlackEventCount = 0;
  }
  lastJunctionActionName = "NONE";
  lastJunctionAction = STOP_ACTION;
}

void beginTurn(NavigationAction action) {
  activeTurnDirection = (action == TURN_LEFT_ACTION) ? -1 :
                        (action == TURN_RIGHT_ACTION) ? 1 : UTURN_DIRECTION;
  turnStartedAt = millis();
  activeTurnTimeoutMs = (action == TURN_180_ACTION) ? UTURN_TIMEOUT_MS : TURN_TIMEOUT_MS;
  reacquisitionStartedAt = 0;
  turnSawJunctionExit = false;
  turnAlignedSamples = 0;
  lineLostStartedAt = 0;
  if (action == TURN_LEFT_ACTION) setMissionState(TURN_LEFT_STATE, "TURN_LEFT");
  else if (action == TURN_RIGHT_ACTION) setMissionState(TURN_RIGHT_STATE, "TURN_RIGHT");
  else setMissionState(TURN_180_STATE, "TURN_180");
}

void beginJunctionAction(NavigationAction action) {
  lastJunctionAction = action;
  lastJunctionActionName = actionName(action);
  junctionActionStartedAt = millis();
  switch (action) {
    case GO_STRAIGHT:
      junctionExitSamples = 0;
      setMissionState(JUNCTION_ACTION, "JUNCTION_GO_STRAIGHT");
      break;
    case TURN_LEFT_ACTION:
    case TURN_RIGHT_ACTION:
    case TURN_180_ACTION:
      beginTurn(action);
      break;
    case STOP_ACTION:
      stopMotorOutputs();
      setMissionState(MISSION_PAUSED, "MISSION_PAUSED");
      break;
  }
}

void handleConfirmedJunction() {
  junctionArmed = false;
  junctionRearmPending = false;
  junctionCandidateSamples = 0;
  junctionExitSamples = 0;
  const bool isFirstNormalJunction = (totalJunctionCount == 0);
  colorScanningEnabled = false;
  colorStatus = (colorScanStoredCount == 3) ? "SCAN_COMPLETE" : "SCAN_STOPPED_AT_JUNCTION";
  const TrackEventType type = irFrame.blackCount >= 4 ? EVENT_T_JUNCTION : EVENT_NORMAL_JUNCTION;
  reportTrackEvent(type);
  if (isFirstNormalJunction && colorScanStoredCount < 3) {
    failMission("COLOR_SCAN_INCOMPLETE");
    robotState = "COLOR_SCAN_INCOMPLETE";
    return;
  }

  phaseJunctionOrdinal++;
  NavigationAction action = GO_STRAIGHT;
  pendingNextPhase = currentPhase;
  pendingNextSegment = currentSegment;

  if (currentPhase == PHASE_MAIN_TRACK) {
    if (phaseJunctionOrdinal == MAIN_FIRST_RIGHT_JUNCTION) action = TURN_RIGHT_ACTION;
    else if (phaseJunctionOrdinal == MAIN_SECOND_RIGHT_JUNCTION) {
      action = TURN_RIGHT_ACTION;
      pendingNextPhase = PHASE_BALL_SELECTION;
      pendingNextSegment = "BALL_SELECTION";
    }
  } else if (currentPhase == PHASE_BALL_SELECTION) {
    ballJunctionCount++;
    const int selectedBall = color1.toInt();
    targetBall = "BALL_" + String(selectedBall);
    if (selectedBall < 1 || selectedBall > 3) { failMission("INVALID_COLOR1_BALL_ID"); return; }
    if (ballJunctionCount == (unsigned long)selectedBall) {
      action = TURN_LEFT_ACTION;
      pendingNextPhase = PHASE_BALL_BRANCH;
      pendingNextSegment = targetBall;
    }
  } else if (currentPhase == PHASE_RETURN_NAVIGATION) {
    if (phaseJunctionOrdinal == 1) {
      action = TURN_RIGHT_ACTION; // return from the ball spur to the lower main track
    } else if (phaseJunctionOrdinal == 2) {
      action = TURN_LEFT_ACTION; // lower-right corner: enter the station track heading upward
      pendingNextPhase = PHASE_STATION_SELECTION;
      pendingNextSegment = "STATION_SELECTION";
    } else { failMission("UNEXPECTED_RETURN_JUNCTION"); return; }
  } else if (currentPhase == PHASE_STATION_SELECTION) {
    stationJunctionCount++;
    const int colorStation = color2.toInt();
    if (colorStation < 1 || colorStation > 3) { failMission("INVALID_COLOR2_STATION_ID"); return; }
    targetStationEncounter = 4 - colorStation;
    targetStation = "STATION_" + String(colorStation);
    if (stationJunctionCount == (unsigned long)targetStationEncounter) {
      action = TURN_RIGHT_ACTION;
      pendingNextPhase = PHASE_STATION_BRANCH;
      pendingNextSegment = targetStation;
    }
  } else if (currentPhase == PHASE_FINAL_ROUTE) {
    finalJunctionCount++;
    const int base = color3.toInt();
    if (base != COLOR_RED_ID && base != COLOR_BLUE_ID) {
      failMission("INVALID_COLOR3_BASE_ID"); return;
    }
    if (!finalStationExitTurnExecuted && finalJunctionCount == 1) {
      if (base == COLOR_RED_ID) action = FINAL_BASE1_STATION_EXIT_TURN_CODE == 1 ? TURN_LEFT_ACTION : TURN_RIGHT_ACTION;
      else action = FINAL_BASE2_STATION_EXIT_TURN_CODE == 1 ? TURN_LEFT_ACTION : TURN_RIGHT_ACTION;
      finalStationExitTurnExecuted = true;
    } else if (!finalCornerTurnExecuted && finalJunctionCount == finalCornerJunctionOrdinal) {
      if (base == COLOR_RED_ID) action = FINAL_BASE1_TURN_CODE == 1 ? TURN_LEFT_ACTION : TURN_RIGHT_ACTION;
      else action = FINAL_BASE2_TURN_CODE == 1 ? TURN_LEFT_ACTION : TURN_RIGHT_ACTION;
      finalCornerTurnExecuted = true;
    }
  } else {
    failMission("UNEXPECTED_JUNCTION_PHASE");
    return;
  }
  beginJunctionAction(action);
}

void completeJunctionAction() {
  junctionCandidateSamples = 0;
  junctionRearmPending = true;
  if (junctionExitSamples >= JUNCTION_EXIT_CONFIRM_SAMPLES) {
    junctionArmed = true;
    junctionRearmPending = false;
  }
  lineLostStartedAt = 0;
  recoveryStartedAt = 0;
  smoothingInitialized = false;
  if (pendingNextPhase != currentPhase || pendingNextSegment != currentSegment) {
    enterMissionPhase(pendingNextPhase, pendingNextSegment.c_str());
    if (missionState == ERROR_STOP) return;
  }
  setMissionState(LINE_FOLLOW, "LINE_FOLLOW");
}

void updateLineFollowing(unsigned long now) {
  float position = 0.0f;
  const bool lineDetected = getLinePosition(position);

  if (junctionRearmPending && isJunctionExited()) {
    junctionArmed = true;
    junctionRearmPending = false;
    junctionCandidateSamples = 0;
    junctionExitSamples = 0;
  }
  updateFinalSequence(now);

  if (confirmAllBlack()) {
    reportTrackEvent(EVENT_ALL_BLACK);
    if (missionState == SERVO_SETTLE || missionState == MISSION_COMPLETE || missionState == ERROR_STOP) return;
    if (missionState == FINAL_WAIT_WHITE || missionState == FINAL_WAIT_SECOND_BLACK) {
      followPosition(0.0f, AUTO_CURVE_SPEED);
      return;
    }
  }

  if (junctionArmed && detectJunction()) {
    setMissionState(JUNCTION_CONFIRMATION, "JUNCTION_CONFIRMATION");
    followPosition(lastKnownLinePosition, AUTO_CURVE_SPEED);
    if (confirmJunction()) handleConfirmedJunction();
    return;
  }
  if (missionState == JUNCTION_CONFIRMATION) {
    junctionCandidateSamples = 0;
    setMissionState(LINE_FOLLOW, "LINE_FOLLOW");
  }

  if (detectWideBlack() && !detectAllBlack()) {
    // Ambiguous wide patch: do not invent a route or count it as a junction.
    if (wideBlackStartedAt == 0) wideBlackStartedAt = now;
    if (now - wideBlackStartedAt >= WIDE_BLACK_STOP_MS) {
      stopMotorOutputs();
      errorStatus = "WIDE_BLACK_AMBIGUOUS";
      setMissionState(MISSION_PAUSED, "WIDE_BLACK_STOP");
      return;
    }
    followPosition(lastKnownLinePosition, AUTO_CURVE_SPEED);
    robotState = "WIDE_BLACK_SLOW";
    return;
  }
  wideBlackStartedAt = 0;

  if (!lineDetected) {
    if (lineLostStartedAt == 0) lineLostStartedAt = now;
    if (now - lineLostStartedAt < LINE_LOST_GRACE_MS) {
      setMissionState(LINE_FOLLOW, "LINE_GAP");
      followPosition(lastKnownLinePosition, AUTO_CURVE_SPEED);
      return;
    }
    if (missionState != LINE_RECOVERY) {
      recoveryStartedAt = now;
      setMissionState(LINE_RECOVERY, "LINE_RECOVERY");
    }
    if (now - recoveryStartedAt >= AUTO_RECOVERY_TIMEOUT_MS) {
      failMission("LINE_RECOVERY_TIMEOUT");
      return;
    }
    const int direction = lastKnownLineDirection;
    if (direction < 0) setMotorSpeeds(-AUTO_RECOVERY_SPEED, AUTO_RECOVERY_SPEED);
    else setMotorSpeeds(AUTO_RECOVERY_SPEED, -AUTO_RECOVERY_SPEED);
    return;
  }

  lineLostStartedAt = 0;
  recoveryStartedAt = 0;
  lastKnownLinePosition = position;
  if (position < -0.15f) lastKnownLineDirection = -1;
  else if (position > 0.15f) lastKnownLineDirection = 1;

  if (fabs(position) >= SHARP_TURN_ERROR_THRESHOLD) {
    if (position < 0) setMotorSpeeds(AUTO_SHARP_INNER_SPEED, AUTO_SHARP_TURN_SPEED);
    else setMotorSpeeds(AUTO_SHARP_TURN_SPEED, AUTO_SHARP_INNER_SPEED);
    robotState = "SHARP_LINE_DEVIATION";
    return;
  }

  setMissionState(LINE_FOLLOW, "LINE_FOLLOW");
  followPosition(position, fabs(position) >= 0.75f ? AUTO_CURVE_SPEED : AUTO_BASE_SPEED);
  if (currentPhase == PHASE_FINAL_ROUTE) {
    if (finalSequenceState == FINISH_SEQ_WAIT_FIRST_BLACK) setMissionState(FINAL_WAIT_FIRST_BLACK, "FINAL_WAIT_FIRST_BLACK");
    else if (finalSequenceState == FINISH_SEQ_WAIT_WHITE) setMissionState(FINAL_WAIT_WHITE, "FINAL_WAIT_WHITE");
    else if (finalSequenceState == FINISH_SEQ_WAIT_SECOND_BLACK) setMissionState(FINAL_WAIT_SECOND_BLACK, "FINAL_WAIT_SECOND_BLACK");
  }
}

void updateJunctionAction(unsigned long now) {
  if (now - junctionActionStartedAt >= JUNCTION_ACTION_TIMEOUT_MS) {
    failMission("JUNCTION_EXIT_TIMEOUT");
    return;
  }
  if (isJunctionExited()) {
    completeJunctionAction();
    followPosition(irFrame.linePosition, AUTO_CURVE_SPEED);
    return;
  }
  // Exit the junction along the current route without a distance-based decision.
  const int correction = constrain((int)(lastKnownLinePosition * AUTO_STEERING_STRENGTH),
                                   -AUTO_MAX_CORRECTION / 2, AUTO_MAX_CORRECTION / 2);
  setMotorSpeeds(clampAutoSpeed(AUTO_CURVE_SPEED + correction),
                 clampAutoSpeed(AUTO_CURVE_SPEED - correction));
}

void updateTurn(unsigned long now) {
  if (now - turnStartedAt >= activeTurnTimeoutMs) {
    failMission("TURN_TIMEOUT");
    return;
  }

  const bool exited = isJunctionExited();
  if (exited && now - turnStartedAt >= TURN_INITIAL_MOTION_MS) {
    turnSawJunctionExit = true;
  }

  if (missionState == LINE_REACQUISITION) {
    if (now - reacquisitionStartedAt >= TURN_REACQUISITION_TIMEOUT_MS) {
      failMission("LINE_REACQUISITION_TIMEOUT");
      return;
    }
    float position = 0.0f;
    if (!getLinePosition(position)) {
      if (activeTurnDirection < 0) setMotorSpeeds(-TURN_REACQUIRE_SEARCH_SPEED, TURN_REACQUIRE_SEARCH_SPEED);
      else setMotorSpeeds(TURN_REACQUIRE_SEARCH_SPEED, -TURN_REACQUIRE_SEARCH_SPEED);
      turnAlignedSamples = 0;
      return;
    }

    const bool centralLine = irFrame.black[1] || irFrame.black[2] || irFrame.black[3];
    followPosition(position, TURN_ALIGN_SPEED);
    if (centralLine && fabs(position) <= 0.65f) turnAlignedSamples++;
    else turnAlignedSamples = 0;
    if (turnAlignedSamples >= TURN_ALIGN_STABLE_SAMPLES) {
      lastKnownLinePosition = position;
      completeJunctionAction();
    }
    return;
  }

  if (turnSawJunctionExit) {
    float position = 0.0f;
    if (getLinePosition(position) && (irFrame.black[1] || irFrame.black[2] || irFrame.black[3])) {
      reacquisitionStartedAt = now;
      turnAlignedSamples = 0;
      setMissionState(LINE_REACQUISITION, "LINE_REACQUISITION");
      followPosition(position, TURN_ALIGN_SPEED);
      return;
    }
  }

  if (activeTurnDirection < 0) setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
  else setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
}

void updateAutonomous() {
  if (operatingMode != "AUTO") return;
  const unsigned long now = millis();

  if (missionState == AUTONOMOUS_START_DELAY) {
    stopMotorOutputs();
    if (now - autoStartStartedAt >= AUTO_START_DELAY_MS) {
      resetIRFilter();
      resetJunctionDetection();
      enterMissionPhase(PHASE_MAIN_TRACK, "MAIN_TRACK");
      setMissionState(PHASE_1_MAIN_TRACK, "PHASE_1_MAIN_TRACK");
      lastKnownLinePosition = 0.0f;
      lastKnownLineDirection = AUTO_INITIAL_SEARCH_DIRECTION;
      smoothingInitialized = false;
      setMissionState(LINE_FOLLOW, "LINE_FOLLOW");
      colorScanningEnabled = true;
      colorScanStartedAt = now;
      lastColorScanAt = now;
      colorStatus = colorSensorReady ? "SCANNING_BEFORE_FIRST_JUNCTION" : "SENSOR_NOT_FOUND";
      lastAutoSampleAt = now - AUTO_SAMPLE_INTERVAL_MS;
    }
    return;
  }

  if (missionState == MISSION_IDLE || missionState == MISSION_PAUSED ||
      missionState == MISSION_COMPLETE || missionState == ERROR_STOP ||
      missionState == WAITING_FOR_MISSION_COMMAND) {
    stopMotorOutputs();
    return;
  }
  if (now - lastAutoSampleAt < AUTO_SAMPLE_INTERVAL_MS) return;
  lastAutoSampleAt = now;
  sampleIRFrame();

  if (missionState == SERVO_SETTLE) {
    stopMotorOutputs();
    if (now - servoSettleStartedAt >= SERVO_SETTLE_MS) {
      if (pendingServoAction == SERVO_ACTION_BALL) ballActuationComplete = true;
      if (pendingServoAction == SERVO_ACTION_STATION) stationActuationComplete = true;
      pendingServoAction = SERVO_ACTION_NONE;
      beginTurn(TURN_180_ACTION);
    }
    return;
  }

  if (missionState == JUNCTION_ACTION) { updateJunctionAction(now); return; }
  if (missionState == TURN_LEFT_STATE || missionState == TURN_RIGHT_STATE ||
      missionState == TURN_180_STATE || missionState == LINE_REACQUISITION) {
    updateTurn(now);
    return;
  }
  updateLineFollowing(now);
}

// ==================== BLUETOOTH / MISSION COMMANDS ====================
void processCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  // Explicit mode commands have highest priority and are acknowledged after action.
  if (command == "MODE:M") {
    operatingMode = "MANUAL";
    currentCommand = "S";
    missionState = MISSION_IDLE;
    colorScanningEnabled = false;
    pendingServoAction = SERVO_ACTION_NONE;
    junctionRearmPending = false;
    junctionCandidateSamples = 0;
    junctionExitSamples = 0;
    junctionArmed = true;
    lineLostStartedAt = recoveryStartedAt = 0;
    wideBlackStartedAt = 0;
    turnStartedAt = reacquisitionStartedAt = 0;
    stopMotors();
    queueBluetoothMessage("ACK:MODE:M", true);
    return;
  }
  if (command == "MODE:A") {
    if (operatingMode == "AUTO") {
      queueBluetoothMessage("ACK:MODE:A", true);
      return; // A repeated command cannot restart/reset the active mission.
    }
    stopMotorOutputs();
    operatingMode = "AUTO";
    currentCommand = "S";
    errorStatus = "NONE";
    resetMissionCounters(true);
    resetJunctionDetection();
    targetBall = targetStation = targetBase = "UNCONFIGURED";
    targetStationEncounter = 0;
    finishMarkerCount = 0;
    ballStatus = "NOT_STARTED";
    wideBlackStartedAt = 0;
    allBlackArmed = true;
    allBlackCandidateSamples = allBlackExitSamples = 0;
    ballActuationComplete = stationActuationComplete = false;
    color1 = color2 = color3 = "NOT_CALIBRATED";
    colorScanStoredCount = 0;
    colorCandidateId = colorCandidateStreak = colorExitStreak = 0;
    colorMarkerArmed = true;
    colorScanningEnabled = false;
    colorAccumR = colorAccumG = colorAccumB = colorAccumClear = 0;
    colorAccumCount = 0;
    enterMissionPhase(PHASE_START, "START");
    autoStartStartedAt = millis();
    setMissionState(AUTONOMOUS_START_DELAY, "AUTONOMOUS_START_DELAY");
    queueBluetoothMessage("ACK:MODE:A", true);
    return;
  }

  if (command.startsWith("CMD:")) {
    String action = command.substring(4);
    action.trim();
    if (action.length() != 1 ||
        (action != "F" && action != "B" && action != "L" && action != "R" && action != "S")) {
      errorStatus = "BAD_CMD";
      queueBluetoothMessage("NACK:CMD:BAD_VALUE", true);
      return;
    }
    if (operatingMode != "MANUAL") {
      currentCommand = "S";
      stopMotorOutputs();
      setMissionState(MISSION_PAUSED, action == "S" ? "MISSION_PAUSED" : "MANUAL_COMMAND_REJECTED");
      if (action == "S") queueBluetoothMessage("ACK:CMD:S", true);
      else queueBluetoothMessage("NACK:CMD:USE_MODE_M", true);
      return;
    }

    lastCommandTime = millis();
    currentCommand = action;
    if (action == "F") moveForward();
    else if (action == "B") moveBackward();
    else if (action == "L") turnLeft();
    else if (action == "R") turnRight();
    else stopMotors();
    queueBluetoothMessage("ACK:CMD:" + action, true);
    return;
  }

  if (command.startsWith("SPD:")) {
    int requestedSpeed = 0;
    if (!parseUnsignedRange(command.substring(4), 0, 255, requestedSpeed)) {
      errorStatus = "BAD_SPEED";
      queueBluetoothMessage("NACK:SPD:EXPECTED_0_TO_255", true);
      return;
    }
    motorSpeed = requestedSpeed;
    if (operatingMode == "MANUAL") {
      if (currentCommand == "F") moveForward();
      else if (currentCommand == "B") moveBackward();
      else if (currentCommand == "L") turnLeft();
      else if (currentCommand == "R") turnRight();
      else stopMotors();
    }
    queueBluetoothMessage("ACK:SPD:" + String(motorSpeed), true);
    return;
  }

  // Legacy CLAW:* tokens remain for compatibility; they control the single rotary actuator.
  if (command.startsWith("CLAW:HOME:")) {
    int angle = 0;
    if (!parseUnsignedRange(command.substring(10), 0, 180, angle)) {
      actuatorStatus = "ANGLE_OUT_OF_RANGE";
      queueBluetoothMessage("NACK:CLAW:HOME", true);
    } else {
      actuatorCurrentAngle = angle;
      actuatorHomeConfigured = true;
      actuatorStatus = "HOME_SET_NO_MOTION";
      queueBluetoothMessage("ACK:CLAW:HOME:" + String(angle), true);
    }
    return;
  }
  if (command == "CLAW:DIR:+") { actuatorClockwiseSign = 1; actuatorStatus = "CW_HIGHER_ANGLE"; queueBluetoothMessage("ACK:CLAW:DIR:+", true); return; }
  if (command == "CLAW:DIR:-") { actuatorClockwiseSign = -1; actuatorStatus = "CW_LOWER_ANGLE"; queueBluetoothMessage("ACK:CLAW:DIR:-", true); return; }
  if (command.startsWith("CLAW:CW:")) {
    int degrees = 0;
    if (!parseUnsignedRange(command.substring(8), 1, 180, degrees) || !rotateActuatorClockwise(degrees))
      queueBluetoothMessage("NACK:CLAW:CW:" + actuatorStatus, true);
    else queueBluetoothMessage("ACK:CLAW:CW:" + String(degrees), true);
    return;
  }
  if (command.startsWith("CLAW:ANGLE:")) {
    int angle = 0;
    if (!parseUnsignedRange(command.substring(11), 0, 180, angle) || !actuatorWriteAngle(angle))
      queueBluetoothMessage("NACK:CLAW:ANGLE:" + actuatorStatus, true);
    else queueBluetoothMessage("ACK:CLAW:ANGLE:" + String(angle), true);
    return;
  }

  // Existing navigation controls remain available.
  if (command.startsWith("NAV:ADD:")) {
    NavigationAction action;
    if (!parseAction(command.substring(8), action)) { errorStatus = "BAD_NAV_ACTION"; queueBluetoothMessage("NACK:NAV:ADD", true); return; }
    if (navigationSequenceLength >= NAV_SEQUENCE_CAPACITY) { errorStatus = "NAV_SEQUENCE_FULL"; queueBluetoothMessage("NACK:NAV:FULL", true); return; }
    navigationSequence[navigationSequenceLength++] = action;
    errorStatus = "NONE";
    queueBluetoothMessage("ACK:NAV:ADD", true);
    return;
  }
  if (command == "NAV:CLEAR") {
    navigationSequenceLength = 0;
    navigationSequenceIndex = 0;
    errorStatus = "NONE";
    queueBluetoothMessage("ACK:NAV:CLEAR", true);
    return;
  }
  if (command == "NAV:RESET") {
    resetMissionCounters(false);
    errorStatus = "NONE";
    queueBluetoothMessage("ACK:NAV:RESET", true);
    return;
  }
  if (command == "NAV:RESET:ALL") {
    resetMissionCounters(true);
    errorStatus = "NONE";
    queueBluetoothMessage("ACK:NAV:RESET:ALL", true);
    return;
  }
  if (command == "NAV:RESUME") {
    if (operatingMode == "AUTO" && missionState == MISSION_PAUSED) {
      wideBlackStartedAt = millis();
      if (!junctionArmed) beginJunctionAction(GO_STRAIGHT);
      else setMissionState(LINE_FOLLOW, "LINE_FOLLOW");
      errorStatus = "NONE";
      queueBluetoothMessage("ACK:NAV:RESUME", true);
    } else {
      errorStatus = "NOT_PAUSED_AT_JUNCTION";
      queueBluetoothMessage("NACK:NAV:RESUME", true);
    }
    return;
  }
  if (command.startsWith("NAV:ACTION:")) {
    NavigationAction action;
    if (!parseAction(command.substring(11), action)) { errorStatus = "BAD_NAV_ACTION"; queueBluetoothMessage("NACK:NAV:ACTION", true); return; }
    if (operatingMode != "AUTO" || missionState != WAITING_FOR_MISSION_COMMAND) {
      errorStatus = "NOT_WAITING_AT_JUNCTION";
      queueBluetoothMessage("NACK:NAV:NOT_WAITING", true);
      return;
    }
    errorStatus = "NONE";
    beginJunctionAction(action);
    queueBluetoothMessage("ACK:NAV:ACTION", true);
    return;
  }

  errorStatus = "UNKNOWN_COMMAND";
  queueBluetoothMessage("NACK:UNKNOWN_COMMAND", true);
}

void pollBluetoothCommands() {
  uint8_t bytesRead = 0;
  while (bytesRead < BT_RX_BYTES_PER_POLL && SerialBT.available() > 0) {
    const int value = SerialBT.read();
    if (value < 0) break;
    bytesRead++;
    const char incoming = (char)value;

    if (incoming == '\r') continue;
    if (incoming == '\n') {
      if (bluetoothRxDiscarding) {
        queueBluetoothMessage("NACK:" + String(bluetoothRxDiscardReason), true);
      } else if (bluetoothRxLength > 0) {
        bluetoothRxBuffer[bluetoothRxLength] = '\0';
        String completeCommand(bluetoothRxBuffer);
        completeCommand.trim();
        if (completeCommand.length() > 0) processCommand(completeCommand);
      }
      bluetoothRxLength = 0;
      bluetoothRxBuffer[0] = '\0';
      bluetoothRxDiscarding = false;
      bluetoothRxDiscardReason = "";
      continue;
    }

    if (bluetoothRxDiscarding) continue;
    const uint8_t byteValue = (uint8_t)incoming;
    if (byteValue == 0 || byteValue > 0x7E || (byteValue < 0x20 && incoming != '\t')) {
      bluetoothRxLength = 0;
      bluetoothRxBuffer[0] = '\0';
      bluetoothRxDiscarding = true;
      bluetoothRxDiscardReason = "BT_RX_INVALID_CHAR";
      errorStatus = "BT_RX_INVALID_CHAR";
      continue;
    }
    if (bluetoothRxLength < BT_RX_MAX_LENGTH) {
      bluetoothRxBuffer[bluetoothRxLength++] = incoming;
      bluetoothRxBuffer[bluetoothRxLength] = '\0';
    } else {
      bluetoothRxLength = 0;
      bluetoothRxBuffer[0] = '\0';
      bluetoothRxDiscarding = true;
      bluetoothRxDiscardReason = "BT_RX_OVERFLOW";
      errorStatus = "BT_RX_OVERFLOW";
    }
  }
}
// ==================== STATUS REPORTING ====================
void sendStatus() {
  if (!SerialBT.connected()) return;

  // Send a small status group each cycle so BT output cannot hold up control.
  const uint8_t group = statusReportGroup;
  statusReportGroup = (uint8_t)((statusReportGroup + 1) % 3);
  if (group == 0) {
    queueBluetoothMessage("STATE:" + robotState);
    queueBluetoothMessage("MODE:" + operatingMode);
    queueBluetoothMessage("MISSION:" + missionStateName());
    queueBluetoothMessage("CURRENT_PHASE:" + phaseName(currentPhase));
    queueBluetoothMessage("CURRENT_SEGMENT:" + currentSegment);
    queueBluetoothMessage("SPEED:" + String(motorSpeed));
    queueBluetoothMessage("ERROR:" + errorStatus);
  } else if (group == 1) {
    queueBluetoothMessage("IR:" + readIRSensors());
    queueBluetoothMessage("IR_PATTERN:" + readIRPattern());
    queueBluetoothMessage("JUNCTION_COUNT:" + String(totalJunctionCount));
    queueBluetoothMessage("MISSION_JUNCTION_COUNT:" + String(missionJunctionCount));
    queueBluetoothMessage("BALL_JUNCTION_COUNT:" + String(ballJunctionCount));
    queueBluetoothMessage("STATION_JUNCTION_COUNT:" + String(stationJunctionCount));
    queueBluetoothMessage("FINAL_JUNCTION_COUNT:" + String(finalJunctionCount));
    queueBluetoothMessage("ALL_BLACK_COUNT:" + String(allBlackEventCount));
    queueBluetoothMessage("LAST_JUNCTION_ACTION:" + lastJunctionActionName);
    queueBluetoothMessage("JUNCTION_TYPE:" + junctionTypeName());
    queueBluetoothMessage("JUNCTION_ARMED:" + String(junctionArmed ? "1" : "0"));
  } else {
    queueBluetoothMessage("TARGET_BALL:" + targetBall);
    queueBluetoothMessage("TARGET_STATION:" + targetStation);
    queueBluetoothMessage("TARGET_BASE:" + targetBase);
    queueBluetoothMessage("COLOR1:" + color1);
    queueBluetoothMessage("COLOR2:" + color2);
    queueBluetoothMessage("COLOR3:" + color3);
    queueBluetoothMessage("COLOR_CALIBRATED:" + String(COLOR_CALIBRATION_CONFIGURED ? "1" : "0"));
    queueBluetoothMessage("COLOR_STATUS:" + colorStatus);
    queueBluetoothMessage("FINISH_SEQUENCE:" + finalSequenceName());
    queueBluetoothMessage("FINISH_MARKER_COUNT:" + String(finishMarkerCount));
    queueBluetoothMessage("BALL_STATUS:" + ballStatus);
    queueBluetoothMessage("ACTUATOR_STATUS:" + actuatorStatus);
    queueBluetoothMessage("ACTUATOR_ANGLE:" + String(actuatorHomeConfigured ? actuatorCurrentAngle : -1));
    if (colorDataValid) {
      queueBluetoothMessage("RGB:" + String(colorRawR) + "," + String(colorRawG) + "," + String(colorRawB));
      queueBluetoothMessage("CLEAR:" + String(colorRawClear));
    }
  }
}
// ==================== SETUP / MAIN LOOP ====================
void setup() {
  operatingMode = "MANUAL";
  currentCommand = "S";
  missionState = MISSION_IDLE;
  Serial.begin(115200);
  Wire.begin(COLOR_SDA_PIN, COLOR_SCL_PIN);
  Wire.setClock(400000);
  oledReady = oledInit();
  initColorSensor();
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  for (int i = 0; i < 5; i++) pinMode(IR_PINS[i], INPUT);
  resetIRFilter();
  stopMotors();

  SerialBT.begin("Robot_BT");
  bluetoothAckQueue = xQueueCreate(32, sizeof(BluetoothTxMessage));
  bluetoothStatusQueue = xQueueCreate(32, sizeof(BluetoothTxMessage));
  if (bluetoothAckQueue != nullptr && bluetoothStatusQueue != nullptr) {
    if (xTaskCreate(bluetoothTxWorker, "bt_tx", 4096, nullptr, 1, &bluetoothTxTaskHandle) != pdPASS)
      errorStatus = "BT_TX_TASK_FAILED";
  } else {
    errorStatus = "BT_TX_QUEUE_FAILED";
  }
  Serial.println("ADAR Map-Aware Control Firmware Started");
  Serial.println("Bluetooth Name: Robot_BT");
}

void loop() {
  pollBluetoothCommands();

  if (operatingMode == "MANUAL" && currentCommand != "S" &&
      millis() - lastCommandTime > COMMAND_TIMEOUT) {
    currentCommand = "S";
    stopMotors();
  }

  // Keep filtered IR status fresh even when autonomous updates are paused/manual.
  if (millis() - lastManualIRSampleAt >= 20 &&
      (operatingMode == "MANUAL" || missionState == WAITING_FOR_MISSION_COMMAND ||
       missionState == MISSION_PAUSED || missionState == ERROR_STOP)) {
    lastManualIRSampleAt = millis();
    sampleIRFrame();
    updateAllBlackRearm(); // never consume/dispatch an autonomous marker in manual mode
  }

  updateAutonomous();
  pollColorSensor();
  updateColorScan(millis());
  refreshOled();

  if (millis() - lastStatusTime >= STATUS_INTERVAL) {
    lastStatusTime = millis();
    sendStatus();
  }

  // A disconnect never changes the latched mode. AUTO continues; MANUAL stops.
  // Only explicit MODE:M and MODE:A commands change operating mode.
  if (!SerialBT.connected() && operatingMode == "MANUAL") {
    currentCommand = "S";
    stopMotorOutputs();
    robotState = "IDLE";
  }
}
