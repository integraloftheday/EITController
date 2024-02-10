
#include <SPI.h>
//#include <esp_now.h>
//#include <WiFi.h>

/*
 * Hold the AD9833 in RESET state.
 * Resets internal registers to 0, which corresponds to an output of
 * midscale - digital output at 0.
 * 
 * The difference between Reset() and EnableOutput(false) is that
 * EnableOutput(false) keeps the AD9833 in the RESET state until you
 * specifically remove the RESET state using EnableOutput(true).
 * With a call to Reset(), ANY subsequent call to ANY function (other
 * than Reset itself and Set/IncrementPhase) will also remove the RESET
 * state.
 */

// Reference frequency from the external clock chip (in Hz)
const uint32_t referenceFrequency = 25000000UL; // 25 MHZ 
uint16_t highRegister, lowRegister;

short control_data_LSB = 0x0000;
short control_data_MSB = 0x1000;

// off
short off = 0b0000000000000000;
short sleepy = 0b0000000010000000;
short square = 0b0000000001101000;

//~1kHz
//short frequencyLSB1 = 0b0110100111110001;
short frequencyLSB1 =   0b0110100111110010;
short frequencyMSB1 = 0b0100000000000000;

//~10kHz
short frequencyLSB2 = 0b0110001101101110;
short frequencyMSB2 = 0b0100000000000110;

//~100kHz
short frequencyLSB3 = 0b0110001001001110;
short frequencyMSB3 = 0b0100000001000001;

//short Phase        = 0x0000;
short Phase        = 0x0000;
short ExitReset    = 0x2000;

short switch1_pin     = 16; // B
short switch2_pin     = 21; // A
short howland_pin     = 17; // H
short test            = 32;

short syncConsole_pin = 26;


unsigned long millisPrevious = 0;
byte debounceInterval = 10;

int   delay1ms  = 1000;
int   delay2ms  = 2000;
int   delay3ms  = 3000;
int   delay5ms  = 5000;
int   delay10ms = 10000; 
int   delay11ms = 11000;
int   delay20ms = 20000; 

//int   delay1trig   = delay3ms;
int   delay1length = delay10ms;
//int   delay2trig   = delay1ms;
int   delay2length = delay10ms;
//int   delay3trig   = delay3ms;
int   delay3length = delay3ms;
//int   delay4trig   = delay1ms;
int   delay4length = delay3ms;


int   count = 0;

bool  temp_ConsoleValue = LOW;
bool  syncConsolePrev_value = LOW;
bool  syncConsoleCurrent_value = LOW;

short pos_neg = 0;

//bool debounce(){
//  static uint16_t state = 0;
//  state = (state<<1) | digitalRead(syncConsole_pin) | 0xFE00;
//  return (state == 0xFF00)
//}



void setup() {

//Initiate console and switch
pinMode(syncConsole_pin, INPUT); //input trigger from console to sync with pulse sequence
pinMode(test, INPUT);//

pinMode(switch1_pin, OUTPUT); // B
pinMode(switch2_pin, OUTPUT); // A
pinMode(howland_pin,OUTPUT);  // H


//initiate SPI pins
pinMode(SS, OUTPUT);
digitalWrite(MOSI, LOW);
digitalWrite(SS,  HIGH);
digitalWrite(SCK,  LOW);

//Begin SPI protocol predefined by the SPI chip on the ESP32
SPI.begin();
Serial.begin(9600);

//Enable switches to initial state
digitalWrite(switch1_pin,LOW);  // B
digitalWrite(switch2_pin,LOW);  // A
digitalWrite(howland_pin,HIGH); // H

  send16bits(control_data_LSB);
  send16bits(frequencyLSB1);
  send16bits(control_data_MSB);
  send16bits(frequencyMSB1);
  send16bits(Phase);
  //send16bits(square);
  send16bits(ExitReset);



  count = 0;
}

int frequency;
bool positive1;
float delayStart1;
float duration1;
bool positive2;
float delayStart2;
float duration2;
bool positive3;
float delayStart3;
float duration3;
bool positive4;
float delayStart4;
float duration4;

void loop() {
  // Serial communication
  if (Serial.available() > 0) {
    // Read and parse the serial message
    String serialMessage = Serial.readStringUntil('\n');
    int parsedItems = sscanf(serialMessage.c_str(), "%d-%d-%f-%f-%d-%f-%f-%d-%f-%f-%d-%f-%f", 
                             &frequency, &positive1, &delayStart1, &duration1,
                             &positive2, &delayStart2, &duration2,&positive3, &delayStart3, &duration3,
                             &positive4, &delayStart4, &duration4);

    // Check if all values were parsed successfully
    if (parsedItems == 13) {
      convertFrequencyToRegisters(static_cast<uint16_t>(frequency), highRegister, lowRegister);
      Serial.println("Successful Programming");
      printValues("Frequency", frequency, highRegister, lowRegister);
      printValues("Positive1", positive1, delayStart1, duration1);
      printValues("Positive2", positive2, delayStart2, duration2);
      printValues("Positive3", positive3, delayStart3, duration3);
      printValues("Positive4", positive4, delayStart4, duration4);
    } else {
      // Handle parsing error
      Serial.println("Error parsing serial message");
    }
  }

  // Correct layout of the calls
  syncConsolePrev_value = syncConsoleCurrent_value;
  syncConsoleCurrent_value = digitalRead(syncConsole_pin);

  
  if((syncConsoleCurrent_value == HIGH) && (syncConsolePrev_value == LOW)){
    count++;
    int pos_neg_values[4] = {positive1, positive2, positive3, positive4};
    float delayStart_values[4] = {delayStart1, delayStart2, delayStart3, delayStart4};
    float duration_values[4] = {duration1, duration2, duration3, duration4};
    processSignal(pos_neg_values[pos_neg], delayStart_values[pos_neg], duration_values[pos_neg]);
    pos_neg = (pos_neg + 1) % 4;
    //Serial.println(pos_neg);
  }
}

void printValues(const char* name, int value, int high, int low) {
  Serial.print(name);
  Serial.println(value);
  Serial.print(name);
  Serial.print(" High: ");
  Serial.println(high);
  Serial.print(name);
  Serial.print(" Low: ");
  Serial.println(low);
}

void processSignal(int positive, float delayStart, float duration) {
  delayMicroseconds(delayStart);
  digitalWrite(switch1_pin, positive ? HIGH : LOW);
  digitalWrite(howland_pin, HIGH);
  digitalWrite(switch2_pin, LOW);

  send16bits(control_data_LSB);
  send16bits(lowRegister);
  send16bits(control_data_MSB);
  send16bits(highRegister);
  send16bits(Phase);
  //send16bits(square);

  delayMicroseconds(duration);
  send16bits(sleepy);

  digitalWrite(howland_pin, LOW);
  digitalWrite(switch2_pin, HIGH);
}

void send16bits(short mydata){
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    digitalWrite(SS, LOW); //enable select
    SPI.transfer16(mydata); //Control Register Bootup
    digitalWrite(SS, HIGH); //Disable Select
    SPI.endTransaction();
}


void convertFrequencyToRegisters(uint32_t frequency, uint16_t &highRegister, uint16_t &lowRegister) {
  // Calculate the 28-bit frequency register value for the AD9833
  uint32_t frequencyRegisterValue = (frequency * (1LL << 28)) / referenceFrequency;
  Serial.println(frequencyRegisterValue);
  // Extract the upper 14 bits (MSBs) and lower 14 bits (LSBs)
  highRegister = (frequencyRegisterValue >> 14) & 0x3FFF;
  highRegister = highRegister | 0x4000;
  lowRegister = frequencyRegisterValue & 0x3FFF;
  lowRegister = lowRegister | 0x4000;
}
