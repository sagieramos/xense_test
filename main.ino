#include <Wire.h>
#include <SPI.h>
#include <RTClib.h> // DS3231
#include <RFM69.h>  // RFM69_LowPowerLab

// PIN CONFIG
#define RELAY_PIN 26
#define BUZZER_PIN 27

#define RF69_CS 5
#define RF69_INT 4
#define RF69_RST 2
#define RF69_FREQ RF69_433MHZ

// ADE7953 PIN CONFIGURATION - UPDATED FOR ESP32
#define ADE_CS_PIN 13
#define ADE_IRQ_PIN 34
#define ADE_RST_PIN 32
#define ADE_SPI_SPEED 1000000

// ESP32 SPI PINS (using default VSPI)
#define ADE_MOSI 23
#define ADE_MISO 19
#define ADE_SCK 18

// ADE7953 CONFIGURATION
#define ADE_XTAL_FREQ 8192000 // Nominal 3.579545 MHz crystal frequency
// For 8.192 MHz: use 8192000
// For 3.5 MHz: use 3579545

// ADE7953 REGISTERS - CORRECTED ADDRESSES
// Voltage/Current RMS (24-bit unsigned)
#define ADE_AVRMS 0x21C // Channel A RMS voltage
#define ADE_AIRMS 0x21A // Channel A RMS current
#define ADE_BVRMS 0x31C // Channel B RMS voltage
#define ADE_BIRMS 0x31A // Channel B RMS current

// Power Registers (24-bit signed for AWATT/BWATT/AVAR/BVAR; unsigned for AVA/BVA)
#define ADE_AWATT 0x212 // Channel A active power
#define ADE_BWATT 0x312 // Channel B active power
#define ADE_AVA 0x210   // Channel A apparent power
#define ADE_BVA 0x310   // Channel B apparent power
#define ADE_AVAR 0x214  // Channel A reactive power
#define ADE_BVAR 0x314  // Channel B reactive power

// Energy Registers (24-bit signed) - Corrected though unused
#define ADE_AWATTHR 0x21E // Channel A active energy
#define ADE_BWATTHR 0x31E // Channel B active energy
#define ADE_AVAHR 0x220   // Channel A apparent energy
#define ADE_BVAHR 0x320   // Channel B apparent energy

// Configuration
#define ADE_LCYCMODE 0x004 // Line cycle mode (8-bit)
#define ADE_LINECYC 0x101  // Number of half line cycles (16-bit)
#define ADE_CONFIG 0x102   // Configuration register (16-bit)
#define ADE_CFMODE 0x107   // CF mode (16-bit)
// Removed invalid COMPMODE 0x105

// Calibration (24-bit unsigned)
#define ADE_AVGAIN 0x381   // Channel A voltage gain
#define ADE_AIGAIN 0x380   // Channel A current gain
#define ADE_AWGAIN 0x382   // Channel A watt gain
#define ADE_AVARGAIN 0x383 // Channel A var gain
#define ADE_AVRMSOS 0x388  // Channel A voltage offset (24-bit signed)
#define ADE_AIRMSOS 0x386  // Channel A current offset (24-bit signed)

// Status/Control (24-bit)
#define ADE_IRQSTATA 0x22D // Interrupt status A
#define ADE_IRQSTATB 0x32D // Interrupt status B
#define ADE_IRQENA 0x22C   // Interrupt enable A
#define ADE_IRQENB 0x22F   // Interrupt enable B
#define ADE_RSTIRQSTATA 0x22E // Reset interrupt status A
#define ADE_PERIOD 0x10E   // Line period (16-bit)

// Product ID
#define ADE_PRODID 0x702 // Product ID (16-bit, 0x7953)

// Unlock registers
#define ADE_UNLOCK 0x00FE
#define ADE_LOCK_REG 0x0120

// MODULE INSTANCES
RTC_DS3231 rtc;
RFM69 radio(RF69_CS, RF69_INT);

// GLOBAL FLAGS & DATA
volatile bool adeIRQTriggered = true;
volatile bool rtcIRQTriggered = false;
volatile bool rfmIRQTriggered = false;

struct ADEData
{
  uint32_t vrmsA, vrmsB;
  uint32_t irmsA, irmsB;
  int32_t wattA, wattB;
  int32_t varA, varB;
  uint32_t vaA, vaB;
  uint16_t period; // Changed to 16-bit
  float frequency;
};

ADEData adeData;

// FUNCTION DECLARATIONS
bool testRTC();
bool testRFM69();
bool testRelay();
bool testBuzzer();
bool testADE();
void adeReset();
void adeInit();
void setupADEInterrupt();
void readADEData();
void displayADEData();
void calibrateADE();
void testADEAccuracy();
void testADEInterrupts();
uint16_t adeRead16(uint16_t reg);
uint32_t adeRead24(uint16_t reg);
int32_t adeRead32s(uint16_t reg);
void adeWrite16(uint16_t reg, uint16_t value);
void adeWrite32(uint16_t reg, uint32_t value);
void stressTestRelay();
void testBuzzerTones();

// INTERRUPTS

void IRAM_ATTR adeISR()
{
  adeIRQTriggered = true;
}

void IRAM_ATTR rtcISR()
{
  rtcIRQTriggered = true;
}

void IRAM_ATTR rfmISR()
{
  rfmIRQTriggered = true;
}

// SETUP

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n");
  Serial.println("╔════════════════════════════════════════════════╗");
  Serial.println("║  XENSE SMART SOCKET COMPREHENSIVE TEST         ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin();
  
  // Initialize SPI with custom pins for ADE7953 - MODE0
  SPI.begin(ADE_SCK, ADE_MISO, ADE_MOSI, ADE_CS_PIN);
  Serial.println("⚙️  SPI initialized with custom pins:");
  Serial.printf("    MOSI: %d, MISO: %d, SCK: %d, CS: %d\n", 
                ADE_MOSI, ADE_MISO, ADE_SCK, ADE_CS_PIN);

  Serial.println("\n⚙️  Starting Hardware Tests...\n");
  delay(500);

  // Run comprehensive tests
  bool rtcOK = testRTC();
  bool rfmOK = testRFM69();
  bool relayOK = testRelay();
  bool buzzerOK = testBuzzer();
  bool adeOK = testADE();

  // Summary
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║           TEST SUMMARY                     ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.printf("  RTC DS3231:    %s\n", rtcOK ? "✓ PASS" : "✗ FAIL");
  Serial.printf("  RFM69 Radio:   %s\n", rfmOK ? "✓ PASS" : "✗ FAIL");
  Serial.printf("  Relay:         %s\n", relayOK ? "✓ PASS" : "✗ FAIL");
  Serial.printf("  Buzzer:        %s\n", buzzerOK ? "✓ PASS" : "✗ FAIL");
  Serial.printf("  ADE7953:       %s\n", adeOK ? "✓ PASS" : "✗ FAIL");
  Serial.println("════════════════════════════════════════════\n");

  if (adeOK)
  {
    Serial.println("✓ ADE7953 is ready for continuous monitoring");
    Serial.println("  Real-time data will be displayed every 2 seconds\n");
  }
}

// LOOP

void loop()
{
  static unsigned long lastRead = 0;

  if (millis() - lastRead >= 2000)
  {
    readADEData();
    displayADEData();
    lastRead = millis();
  }

  // Check for interrupts
  if (adeIRQTriggered)
  {
    uint32_t irqA = adeRead24(ADE_IRQSTATA);
    uint32_t irqB = adeRead24(ADE_IRQSTATB);
    Serial.printf("⚡ ADE IRQ! StatusA: 0x%06X, StatusB: 0x%06X\n", irqA, irqB);
    
    // Clear interrupts (write back to reset registers)
    adeWrite24(ADE_RSTIRQSTATA, irqA);
    // adeWrite24(ADE_RSTIRQSTATB, irqB); // If needed
    
    adeIRQTriggered = false;
  }

  if (rtcIRQTriggered)
  {
    Serial.println("RTC Alarm Triggered!");
    rtcIRQTriggered = false;
  }

  if (rfmIRQTriggered)
  {
    Serial.println("RFM69 Interrupt!");
    rfmIRQTriggered = false;
  }
}

// RTC TEST

bool testRTC()
{
  Serial.println("┌─────────────────────────────────────┐");
  Serial.println("│  TEST 1: DS3231 RTC                 │");
  Serial.println("└─────────────────────────────────────┘");

  if (!rtc.begin())
  {
    Serial.println("✗ RTC not detected on I2C bus!");
    return false;
  }
  Serial.println("✓ RTC detected on I2C");

  if (rtc.lostPower())
  {
    Serial.println("⚠  RTC lost power - setting current time");
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  DateTime now = rtc.now();
  Serial.printf("✓ Current Time: %04d-%02d-%02d %02d:%02d:%02d\n",
                now.year(), now.month(), now.day(),
                now.hour(), now.minute(), now.second());

  // Test temperature sensor
  float temp = rtc.getTemperature();
  Serial.printf("✓ Temperature: %.2f°C\n", temp);

  // Verify time stability
  delay(1100);
  DateTime now2 = rtc.now();
  int diffSec = now2.unixtime() - now.unixtime();
  Serial.printf("✓ Time stability: %d second(s) elapsed\n", diffSec);

  if (diffSec < 1 || diffSec > 2)
  {
    Serial.println("⚠  Warning: Time drift detected!");
  }

  Serial.println("✓ RTC Test Complete\n");
  return true;
}

// RFM69 TEST

bool testRFM69()
{
  Serial.println("┌─────────────────────────────────────┐");
  Serial.println("│  TEST 2: RFM69 Radio Module         │");
  Serial.println("└─────────────────────────────────────┘");

  pinMode(RF69_RST, OUTPUT);
  digitalWrite(RF69_RST, HIGH);
  delay(50);
  digitalWrite(RF69_RST, LOW);
  delay(50);

  if (!radio.initialize(RF69_433MHZ, 1, 100))
  {
    Serial.println("✗ RFM69 initialization failed!");
    return false;
  }
  Serial.println("✓ RFM69 initialized");

  radio.setHighPower();
  radio.setPowerLevel(31);
  Serial.println("✓ High power mode enabled");

  // Test transmission
  const char *testMsg = "XENSE_TEST";
  radio.send(1, testMsg, strlen(testMsg));
  Serial.printf("✓ Test packet sent: '%s'\n", testMsg);

  // Read RSSI
  int16_t rssi = radio.readRSSI();
  Serial.printf("✓ RSSI: %d dBm\n", rssi);

  // Test promiscuous mode
  radio.spyMode(true);
  radio.writeReg(0x29, 0x00);
  radio.writeReg(0x2E, 0x88);
  Serial.println("✓ Promiscuous mode enabled");

  Serial.printf("✓ Temperature: %d°C\n", radio.readTemperature(0));

  Serial.println("✓ RFM69 Test Complete\n");
  return true;
}

// RELAY TEST

bool testRelay()
{
  Serial.println("┌─────────────────────────────────────┐");
  Serial.println("│  TEST 3: Relay Control              │");
  Serial.println("└─────────────────────────────────────┘");

  Serial.println("⚡ WARNING: AC LOAD TEST");
  Serial.println("   Ensure proper electrical safety!");
  delay(1000);

  // Basic switching test
  for (int i = 0; i < 5; i++)
  {
    Serial.printf("✓ Cycle %d: ON", i + 1);
    digitalWrite(RELAY_PIN, HIGH);
    delay(500);
    Serial.print(" -> OFF\n");
    digitalWrite(RELAY_PIN, LOW);
    delay(500);
  }

  // Stress test option
  Serial.println("\n⚡ Stress Test: 100 rapid cycles");
  stressTestRelay();

  Serial.println("✓ Relay Test Complete\n");
  return true;
}

void stressTestRelay()
{
  unsigned long startTime = millis();
  for (int i = 0; i < 100; i++)
  {
    digitalWrite(RELAY_PIN, HIGH);
    delay(50);
    digitalWrite(RELAY_PIN, LOW);
    delay(50);
    if (i % 20 == 0)
      Serial.printf("  %d/100...\n", i);
  }
  unsigned long duration = millis() - startTime;
  Serial.printf("✓ Completed in %lu ms\n", duration);
}

// BUZZER TEST

bool testBuzzer()
{
  Serial.println("┌─────────────────────────────────────┐");
  Serial.println("│  TEST 4: Buzzer                     │");
  Serial.println("└─────────────────────────────────────┘");

  // Basic beeps
  Serial.println("♪ Testing basic tones...");
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }

  // Pattern test
  Serial.println("♪ Testing alarm pattern...");
  testBuzzerTones();

  Serial.println("✓ Buzzer Test Complete\n");
  return true;
}

void testBuzzerTones()
{
  int pattern[] = {200, 100, 200, 100, 500};
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(pattern[i]);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

// ADE7953 COMPREHENSIVE TEST

bool testADE()
{
  Serial.println("┌─────────────────────────────────────┐");
  Serial.println("│  TEST 5: ADE7953 Energy Monitor     │");
  Serial.println("└─────────────────────────────────────┘");

  pinMode(ADE_CS_PIN, OUTPUT);
  digitalWrite(ADE_CS_PIN, HIGH);
  pinMode(ADE_RST_PIN, OUTPUT);
  digitalWrite(ADE_RST_PIN, HIGH);
  pinMode(ADE_IRQ_PIN, INPUT_PULLUP);

  Serial.printf("✓ ADE7953 pins configured: CS=%d, RST=%d, IRQ=%d\n", 
                ADE_CS_PIN, ADE_RST_PIN, ADE_IRQ_PIN);

  adeReset();
  delay(100);

  // Clear initial IRQ after reset
  adeWrite32(ADE_RSTIRQSTATA, 0x00100000); // Clear reset bit (bit 20)

  // Read Product ID
  uint16_t prodID = adeRead16(ADE_PRODID);
  Serial.printf("ℹ Product ID: 0x%04X ", prodID);

  if (prodID != 0x7953)
  {
    Serial.println("✗ Invalid! (Expected 0x7953)");
    return false;
  }
  Serial.println("✓ (Valid)");

  // Initialize ADE
  adeInit();
  Serial.println("✓ ADE7953 initialized");

  // Setup interrupts
  setupADEInterrupt();
  Serial.println("✓ Interrupt configured");

  // Test communication
  Serial.println("\n✓ Testing register access...");
  adeWrite16(ADE_LINECYC, 200);
  uint16_t readBack = adeRead16(ADE_LINECYC);
  Serial.printf("  Write: 200, Read: %d ", readBack);
  if (readBack == 200)
  {
    Serial.println("✓");
  }
  else
  {
    Serial.println("✗");
    return false;
  }

  // Calibration test
  Serial.println("\n✓ Running calibration check...");
  calibrateADE();

  // Accuracy test with AC power
  Serial.println("\n⚡ LIVE AC POWER TEST");
  Serial.println("   Connect known load for accuracy test");
  delay(2000);
  testADEAccuracy();

  // Interrupt test
  Serial.println("\n✓ Testing interrupt generation...");
  testADEInterrupts();

  Serial.println("✓ ADE7953 Test Complete\n");
  return true;
}

// ADE RESET
void adeReset()
{
  digitalWrite(ADE_RST_PIN, LOW);
  delay(10);
  digitalWrite(ADE_RST_PIN, HIGH);
  delay(100);
  Serial.println("✓ ADE7953 hardware reset");
}

// ADE INITIALIZATION - CORRECTED
void adeInit()
{
  // Unlock sequence
  adeWrite16(ADE_UNLOCK, 0x00AD);
  delay(10);
  adeWrite16(ADE_LOCK_REG, 0x0030);
  delay(100);

  // Clear LCYCMODE RSTREAD (bit 6=0)
  adeWrite16(ADE_LCYCMODE, 0x0001); // Line cycle accumulation mode, RSTREAD=0
  adeWrite16(ADE_LINECYC, 200);     // 200 half line cycles

  // Enable power calculation
  adeWrite16(ADE_CFMODE, 0x0300); // CF1 = active power, CF2 = reactive

  // Removed invalid COMPMODE write

  // Original CONFIG write (if needed; check purpose)
  adeWrite16(ADE_CONFIG, 0x0080);
  delay(100);

  // Lock SPI interface
  adeWrite16(ADE_CONFIG, 0x0000);

  Serial.printf("✓ ADE configured for %.2f MHz crystal\n", ADE_XTAL_FREQ / 1000000.0);
}

// ADE INTERRUPT SETUP - CORRECTED
void setupADEInterrupt()
{
  attachInterrupt(digitalPinToInterrupt(ADE_IRQ_PIN), adeISR, FALLING);

  // Enable zero-crossing (bit 15) and CYCEND (bit 2) interrupts
  uint32_t enA = 0x00008004; // ZX (0x8000) + CYCEND (0x0004)
  adeWrite32(ADE_IRQENA, enA);
  adeWrite32(ADE_IRQENB, 0x00000000);
}

// CALIBRATION
void calibrateADE()
{
  // Read current calibration values (24-bit, but read as 32 for consistency)
  int32_t avgain = adeRead32s(ADE_AVGAIN);
  int32_t aigain = adeRead32s(ADE_AIGAIN);

  Serial.printf("  Current gains - Voltage: %ld, Current: %ld\n", avgain, aigain);

  // For actual calibration:
  // 1. Apply known voltage/current
  // 2. Calculate gain factors
  // 3. Write back to gain registers

  Serial.println("ℹ  Using default calibration (adjust with known reference)");
}

// ACCURACY TEST
void testADEAccuracy()
{
  Serial.println("  Taking 10 measurements over 5 seconds...");

  float avgVoltage = 0, avgCurrent = 0, avgPower = 0;

  for (int i = 0; i < 10; i++)
  {
    readADEData();

    // Convert raw to actual values (calibration dependent)
    float voltage = adeData.vrmsA * 0.0001; // Example scaling
    float current = adeData.irmsA * 0.0001;
    float power = adeData.wattA * 0.001;

    avgVoltage += voltage;
    avgCurrent += current;
    avgPower += power;

    Serial.printf("    [%d] V=%.1f, I=%.3f, P=%.1f W\n",
                  i + 1, voltage, current, power);
    delay(500);
  }

  avgVoltage /= 10;
  avgCurrent /= 10;
  avgPower /= 10;

  Serial.printf("\n  📈 Averages: V=%.1f V, I=%.3f A, P=%.1f W\n",
                avgVoltage, avgCurrent, avgPower);

  // Power factor check
  if (avgCurrent > 0.001)
  {
    float apparentPower = avgVoltage * avgCurrent;
    float pf = avgPower / apparentPower;
    Serial.printf("  Power Factor: %.3f\n", pf);
  }
}

// INTERRUPT TEST
void testADEInterrupts()
{
  adeIRQTriggered = false;

  Serial.println("  Waiting for zero-crossing interrupt...");
  unsigned long timeout = millis() + 5000;

  while (!adeIRQTriggered && millis() < timeout)
  {
    delay(10);
  }

  if (adeIRQTriggered)
  {
    Serial.println("✓ Interrupt working!");
    adeIRQTriggered = false;
  }
  else
  {
    Serial.println("⚠ No interrupt detected");
  }
}

// READ ADE DATA - CORRECTED ADDRESSES
void readADEData()
{
  adeData.vrmsA = adeRead24(ADE_AVRMS);
  adeData.vrmsB = adeRead24(ADE_BVRMS);
  adeData.irmsA = adeRead24(ADE_AIRMS);
  adeData.irmsB = adeRead24(ADE_BIRMS);
  adeData.wattA = (int32_t)adeRead24(ADE_AWATT); // Signed
  adeData.wattB = (int32_t)adeRead24(ADE_BWATT);
  adeData.varA = (int32_t)adeRead24(ADE_AVAR);
  adeData.varB = (int32_t)adeRead24(ADE_BVAR);
  adeData.vaA = adeRead24(ADE_AVA);
  adeData.vaB = adeRead24(ADE_BVA);
  adeData.period = adeRead16(ADE_PERIOD); // 16-bit

  // Corrected frequency calculation
  if (adeData.period != 0)
  {
    float line_period_sec = (1.0 + (float)adeData.period) / 223750.0; // 223.75 kHz clock
    adeData.frequency = 1.0 / line_period_sec;
  } else {
    adeData.frequency = 0.0;
  }
}

// DISPLAY ADE DATA
void displayADEData()
{
  Serial.println("┌──────────────────────────────────────────────┐");
  Serial.println("│         ADE7953 LIVE MEASUREMENTS            │");
  Serial.println("├──────────────────────────────────────────────┤");
  Serial.printf("│ VRMS A: %10lu   VRMS B: %10lu │\n", adeData.vrmsA, adeData.vrmsB);
  Serial.printf("│ IRMS A: %10lu   IRMS B: %10lu │\n", adeData.irmsA, adeData.irmsB);
  Serial.printf("│ WATT A: %10ld   WATT B: %10ld │\n", adeData.wattA, adeData.wattB);
  Serial.printf("│  VAR A: %10ld    VAR B: %10ld │\n", adeData.varA, adeData.varB);
  Serial.printf("│   VA A: %10lu     VA B: %10lu │\n", adeData.vaA, adeData.vaB);
  Serial.printf("│ Frequency: %.2f Hz                            │\n", adeData.frequency);
  Serial.println("└──────────────────────────────────────────────┘\n");
}

// ADE SPI READ FUNCTIONS - CORRECTED PROTOCOL (MODE0)
uint16_t adeRead16(uint16_t reg)
{
  SPI.beginTransaction(SPISettings(ADE_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(ADE_CS_PIN, LOW);

  uint8_t addr_h = (reg >> 8) & 0xFF;
  uint8_t addr_l = reg & 0xFF;
  SPI.transfer(addr_h);
  SPI.transfer(addr_l);
  SPI.transfer(0x80); // Read command

  uint8_t high = SPI.transfer(0x00);
  uint8_t low = SPI.transfer(0x00);

  digitalWrite(ADE_CS_PIN, HIGH);
  SPI.endTransaction();
  return ((uint16_t)high << 8) | low;
}

uint32_t adeRead24(uint16_t reg)
{
  SPI.beginTransaction(SPISettings(ADE_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(ADE_CS_PIN, LOW);

  uint8_t addr_h = (reg >> 8) & 0xFF;
  uint8_t addr_l = reg & 0xFF;
  SPI.transfer(addr_h);
  SPI.transfer(addr_l);
  SPI.transfer(0x80); // Read command

  uint8_t b2 = SPI.transfer(0x00); // MSB
  uint8_t b1 = SPI.transfer(0x00);
  uint8_t b0 = SPI.transfer(0x00); // LSB

  digitalWrite(ADE_CS_PIN, HIGH);
  SPI.endTransaction();
  return ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;
}

int32_t adeRead32s(uint16_t reg)
{
  SPI.beginTransaction(SPISettings(ADE_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(ADE_CS_PIN, LOW);

  uint8_t addr_h = (reg >> 8) & 0xFF;
  uint8_t addr_l = reg & 0xFF;
  SPI.transfer(addr_h);
  SPI.transfer(addr_l);
  SPI.transfer(0x80); // Read command

  int32_t value = 0;
  int8_t b3 = (int8_t)SPI.transfer(0x00); // Sign extend MSB
  uint8_t b2 = SPI.transfer(0x00);
  uint8_t b1 = SPI.transfer(0x00);
  uint8_t b0 = SPI.transfer(0x00);

  digitalWrite(ADE_CS_PIN, HIGH);
  SPI.endTransaction();
  value = ((int32_t)b3 << 24) | ((uint32_t)b2 << 16) | ((uint32_t)b1 << 8) | b0;
  return value;
}

// Helper for 24-bit write (using 32-bit with upper=0)
void adeWrite24(uint16_t reg, uint32_t value)
{
  adeWrite32(reg, value & 0x00FFFFFF);
}

// ADE SPI WRITE FUNCTIONS - CORRECTED
void adeWrite16(uint16_t reg, uint16_t value)
{
  SPI.beginTransaction(SPISettings(ADE_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(ADE_CS_PIN, LOW);

  uint8_t addr_h = (reg >> 8) & 0xFF;
  uint8_t addr_l = reg & 0xFF;
  SPI.transfer(addr_h);
  SPI.transfer(addr_l);
  SPI.transfer(0x00); // Write command
  SPI.transfer((value >> 8) & 0xFF);
  SPI.transfer(value & 0xFF);

  digitalWrite(ADE_CS_PIN, HIGH);
  SPI.endTransaction();
}

void adeWrite32(uint16_t reg, uint32_t value)
{
  SPI.beginTransaction(SPISettings(ADE_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(ADE_CS_PIN, LOW);

  uint8_t addr_h = (reg >> 8) & 0xFF;
  uint8_t addr_l = reg & 0xFF;
  SPI.transfer(addr_h);
  SPI.transfer(addr_l);
  SPI.transfer(0x00); // Write command
  SPI.transfer((value >> 24) & 0xFF);
  SPI.transfer((value >> 16) & 0xFF);
  SPI.transfer((value >> 8) & 0xFF);
  SPI.transfer(value & 0xFF);

  digitalWrite(ADE_CS_PIN, HIGH);
  SPI.endTransaction();
}
