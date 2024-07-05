/* Hardware: Arduino Pro Mini */

/* References:
- https://developer.arm.com/documentation/ddi0222/b/
- https://www.xjtag.com/about-jtag/jtag-a-technical-overview/

*/

#define PIN_TDI 11
#define PIN_TCK 10 /* JTAG clock. Sampling on rising edge. */
#define PIN_TDO 9
#define PIN_TMS 8
#define PIN_RTCK 12

#define PIN_LED1 6

#define INSTRUCTION_IDCODE 0b1110 /* Connects the IDCODE register */
#define INSTRUCTION_SCAN_N 0b0010 /* The SCAN_N instruction connects the scan path select register between DBGTDI and DBGTDO */
#define INSTRUCTION_INTEST 0x1100 /* The INTEST instruction places the selected scan chain in test mode. */

#define LENGTH_OF_INSTRUCTION 4

#define TRUE_EXIT_SHIFT 1
#define DO_NOT_EXIT 0

void delayHalfClock(void) {
  //delay(1);
  _delay_us(10);
}

void additionalDelayForVisibility(void) {
  delayHalfClock(); delayHalfClock();
}

void goto_testRunIdle(void) {
  uint8_t i;
  digitalWrite(PIN_TMS, 1); /* multiple 1 lead to resetting the TAP state machine. */
  digitalWrite(PIN_TDI, 0); /* does not matter. */
  for (i=1; i<=5; i++) {
    delayHalfClock();
    digitalWrite(PIN_TCK, HIGH);
    delayHalfClock();
    digitalWrite(PIN_TCK, LOW);
  }
  /* now the TAP is in state "test logic reset". */
  digitalWrite(PIN_TMS, 0);
  delayHalfClock();
  digitalWrite(PIN_TCK, HIGH); /* this positive edge changes to "run test idle" */
  delayHalfClock();
  digitalWrite(PIN_TCK, LOW);

  /* just for better visibiliy on scope: additional delay: */
  additionalDelayForVisibility();
}

void goto_shiftIR(void) {
  uint8_t i;
  uint32_t tms;
  tms = 0b0011;
  for (i=0; i<4; i++) {
    digitalWrite(PIN_TMS, (tms & 1));
    delayHalfClock();
    digitalWrite(PIN_TCK, HIGH); /* shift on positive edge */
    delayHalfClock();
    digitalWrite(PIN_TCK, LOW);
    tms>>=1;
  }
  additionalDelayForVisibility();
}

void goto_shiftDR(void) {
  uint8_t i;
  uint32_t tms;
  tms = 0b001;
  for (i=0; i<3; i++) {
    digitalWrite(PIN_TMS, (tms & 1));
    delayHalfClock();
    digitalWrite(PIN_TCK, HIGH); /* shift on positive edge */
    delayHalfClock();
    digitalWrite(PIN_TCK, LOW);
    tms>>=1;
  }
  additionalDelayForVisibility();
}

uint32_t shiftData(uint32_t tdiData, uint8_t numberOfBits, uint8_t blExitTheShiftStateWhenFinished) {
  uint8_t i;
  uint32_t tdoMask = 1;
  uint32_t tdoData = 0;
  
  for (i=1; i<=numberOfBits; i++) {
    if ((i==numberOfBits) && (blExitTheShiftStateWhenFinished)) {
      digitalWrite(PIN_TMS, 1); /* In shift-DR and shift-IR, we set TMS=1 if we want to exit the state. */
    } else {
      digitalWrite(PIN_TMS, 0); /* In shift-DR and shift-IR, the TMS is 0, if we want to stay in this state. */
    }
    digitalWrite(PIN_TDI, (tdiData & 1));
    delayHalfClock();
    if (digitalRead(PIN_TDO)) { /* sampling of the TDO, half a clock after the falling "setup" edge. */
      tdoData |= tdoMask;
    }
    tdoMask<<=1;
    digitalWrite(PIN_TCK, HIGH); /* sample on positive edge */
    delayHalfClock();
    digitalWrite(PIN_TCK, LOW); /* setup on falling edge */
    tdiData>>=1;
  }
  additionalDelayForVisibility();
  return tdoData;
}

uint32_t IR(uint8_t instruction) {
  uint8_t oldInstruction;
    goto_testRunIdle();
    goto_shiftIR();
    oldInstruction = shiftData(instruction, 5, 1);
    Serial.print(" -> oldInstruction is "); Serial.print(oldInstruction); /* old instruction is 0b0001 according to chapter B.5.3 */
    goto_testRunIdle();
}

uint32_t DR32(uint32_t x) {
  uint32_t oldData;
    goto_testRunIdle();
    goto_shiftDR();
    oldData = shiftData(x, 32, 1);
    Serial.print(" -> oldData is "); Serial.print(oldData);
    goto_testRunIdle();
}



void readDataAfterAllInstructions(void) {
  uint32_t data32;
  uint8_t oldInstruction;
  uint8_t k;
  for (k=0; k<=31; k++) {
    goto_testRunIdle();
    goto_shiftIR();
    oldInstruction = shiftData(k /*INSTRUCTION_IDCODE*/, 5, 1);
    Serial.print(k); Serial.print(" -> oldInstruction is "); Serial.print(oldInstruction); /* old instruction is 0b0001 according to chapter B.5.3 */
    goto_testRunIdle();
    goto_shiftDR();
    data32 = shiftData(0x00000000, 32, 1);
    Serial.print(" data32 is 0x"); Serial.println(data32, HEX);
    delayHalfClock();delayHalfClock();delayHalfClock();delayHalfClock();delayHalfClock();delayHalfClock();
    delay(500);
  }
}

void test1(void) {
  uint8_t oldScanPath;
  uint32_t debugstatus;
  uint8_t kScanPath;
  for (kScanPath=0; kScanPath<=31; kScanPath++) {
    goto_testRunIdle();
    goto_shiftIR();
    shiftData(INSTRUCTION_SCAN_N, LENGTH_OF_INSTRUCTION, 1); /* connect the scan path selection register */
    goto_testRunIdle();
    goto_shiftDR();
    oldScanPath = shiftData(kScanPath, 5, 1); /* SCAN_N register has 5 bits. We select scan path 2, "EmbeddedICE-RT programming" */
    Serial.print(kScanPath); Serial.print(" old scan path was "); Serial.println(oldScanPath);

    /* it is not clear, whether it needs an INTEST instruction to connect the specified scan path, or not. */
    goto_testRunIdle();
    goto_shiftIR();
    shiftData(INSTRUCTION_INTEST, LENGTH_OF_INSTRUCTION, 1); /* connect the selected scan path, chapter B.4.4 */

  }

#if 0
  goto_shiftDR();
  shiftData(1, 1, DO_NOT_EXIT); /* 1 bit read/write */
  #define EMBEDDEDICE_ADDR_DEBUG_STATUS 1
  #define DONTCARE 0
  shiftData(EMBEDDEDICE_ADDR_DEBUG_STATUS, 5, DO_NOT_EXIT); /* 5-bit-address field */
  debugstatus = shiftData(DONTCARE, 32, TRUE_EXIT_SHIFT); /* 32-bit-data field */
  Serial.print("debugstatus 0x"); Serial.println(debugstatus, HEX);
#endif

}

void testClockingWithReturnClock(void) {
  uint8_t i;
  Serial.println("testClockingWithReturnClock");
  digitalWrite(PIN_TMS, 1); /* multiple 1 lead to resetting the TAP state machine. */
  for (i=0; i<10; i++) {
    delayHalfClock();
    digitalWrite(PIN_TCK, HIGH);
    delayHalfClock();
    digitalWrite(PIN_TCK, LOW);
  }
}

void testDigitalOutputs(void) {
  uint8_t k;
  uint32_t mask;
  /* preparation: DR, DR, IR(0) */
  DR32(0);
  delay(50);
  DR32(0);
  delay(50);
  IR(0);
  delay(50);
  for (k=0; k<32; k++) {
    mask = 1<<k;
    DR32(mask);
    delay(50);
    //DR32(0xffffffff);
    //delay(500);
  }
}

void testBlinkGPIO1_2_INT(void) {
  uint8_t k;
  uint32_t mask;
  /* preparation: DR, DR, IR(0) */
  //DR32(0);
  IR(1);
  delay(500);
  //DR32(0);
  IR(1);
  delay(500);
  IR(0);
  delay(500);
  #define GPIO1_H 0x100
  #define GPIO2_H 0x400
  #define INT_H 0x20000000
  #define PATTERN1 (GPIO1_H | GPIO2_H) /* means INT = low = LED on */
  #define PATTERN2 (GPIO2_H | INT_H) /* means GPIO1 = low = LED on */
  #define PATTERN3 (GPIO1_H | INT_H) /* means GPIO2 = low = LED on */

  for (k=0; k<5; k++) {
    DR32(PATTERN1);  delay(20);
    DR32(PATTERN2);  delay(20);
    DR32(PATTERN3);  delay(20);
    DR32(PATTERN2);  delay(20);
  }
}


void setup() {
    pinMode(PIN_TCK, OUTPUT);
    pinMode(PIN_TMS, OUTPUT);
    pinMode(PIN_TDI, OUTPUT);
    pinMode(PIN_TDO, INPUT_PULLUP);
    pinMode(PIN_RTCK, INPUT_PULLUP);
    Serial.begin(115200);
    Serial.println("This is JTAGexperiments.");
}

void loop() {
  //testClockingWithReturnClock();
  //readDataAfterAllInstructions();
  //testDigitalOutputs();
  testBlinkGPIO1_2_INT();
  delay(1000);

}