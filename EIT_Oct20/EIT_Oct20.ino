
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
// serial communication 

if (Serial.available() > 0) {
    // Read the serial message
    String serialMessage = Serial.readStringUntil('\n');

    // Parse the serial message
    int parsedItems = sscanf(serialMessage.c_str(), "%d-%d-%f-%f-%d-%f-%f-%d-%f-%f-%d-%f-%f", 
                             &frequency, &positive1, &delayStart1, &duration1,
                             &positive2, &delayStart2, &duration2,&positive3, &delayStart3, &duration3,
                             &positive4, &delayStart4, &duration4);

    // Check if all values were parsed successfully
    if (parsedItems == 7) {
      // Process the parsed values as needed
      // You can use frequency, positive1, delayStart1, duration1,
      // positive2, delayStart2, and duration2 in your Arduino logic

      //convert to MSB and LSB for frequency 
      convertFrequencyToRegisters(static_cast<uint16_t>(frequency), highRegister, lowRegister);
      // For example, print the parsed values to the Serial monitor
      Serial.print("Sucessful Programming"); 
      Serial.print("Frequency: ");
      Serial.println(frequency);
      Serial.print("Frequency High: ");
      Serial.println(highRegister);
      Serial.print("Frequency Low: ");
      Serial.println(lowRegister);
      Serial.print("Positive1: ");
      Serial.println(positive1);
      Serial.print("DelayStart1: ");
      Serial.println(delayStart1);
      Serial.print("Duration1: ");
      Serial.println(duration1);
      Serial.print("Positive2: ");
      Serial.println(positive2);
      Serial.print("DelayStart2: ");
      Serial.println(delayStart2);
      Serial.print("Duration2: ");
      Serial.println(duration2);
    } else {
      // Handle parsing error
      Serial.println("Error parsing serial message");
    }
  }


//correct layout of the calls
syncConsoleCurrent_value = digitalRead(syncConsole_pin);
syncConsolePrev_value = syncConsoleCurrent_value;



//if((pulseIn(syncConsole_pin, HIGH) > 10)){ //USE ME FOR PRECISELY TRIGGERING FROM PULSES

//if((syncConsoleCurrent_value == LOW) && (syncConsolePrev_value == HIGH)){ //USE ME FOR TRYING MY BEST TO TRIGGER ON RISING EDGES

if((syncConsoleCurrent_value == HIGH) && (syncConsolePrev_value == LOW)){
  count = count + 1;
  
  if(pos_neg == 0){
    if(!positive1){
      delayMicroseconds(delayStart1);
      digitalWrite(switch1_pin,LOW); // B
      digitalWrite(howland_pin,HIGH); // H
      digitalWrite(switch2_pin,LOW);  // A
      
      send16bits(control_data_LSB);
      send16bits(lowRegister);
      send16bits(control_data_MSB);
      send16bits(highRegister);
      send16bits(Phase);
     
      delayMicroseconds(duration1); 
      send16bits(sleepy);  
      
      
      digitalWrite(howland_pin,LOW); // H
      digitalWrite(switch2_pin,HIGH);  // A
    }else{
        delayMicroseconds(delayStart1);   
        digitalWrite(switch1_pin,HIGH);
        digitalWrite(howland_pin,HIGH); // H
        digitalWrite(switch2_pin,LOW);  // A
    
      send16bits(control_data_LSB);
      send16bits(lowRegister);
      send16bits(control_data_MSB);
      send16bits(highRegister);
      send16bits(Phase);
      
        delayMicroseconds(duration1);
        send16bits(sleepy);     
    
        digitalWrite(howland_pin,LOW); // H
        digitalWrite(switch2_pin,HIGH);  // A
    }

    pos_neg = 1;
  }

 else if(pos_neg == 1){
  if(!positive2){
    delayMicroseconds(delayStart2);
    digitalWrite(switch1_pin,LOW);
    digitalWrite(howland_pin,HIGH); // H
    digitalWrite(switch2_pin,LOW);  // A
    
    send16bits(control_data_LSB);
    send16bits(lowRegister);
    send16bits(control_data_MSB);
    send16bits(highRegister);
    send16bits(Phase);
   
    delayMicroseconds(duration2); 
    send16bits(sleepy);  
    
    digitalWrite(howland_pin,LOW); // H
    digitalWrite(switch2_pin,HIGH);  // A
  }else{
    delayMicroseconds(delayStart2);
    digitalWrite(switch1_pin,HIGH);
    digitalWrite(howland_pin,HIGH); // H
    digitalWrite(switch2_pin,LOW);  // A

      send16bits(control_data_LSB);
      send16bits(lowRegister);
      send16bits(control_data_MSB);
      send16bits(highRegister);
      send16bits(Phase);
  
    delayMicroseconds(duration2);
    send16bits(sleepy);     

    digitalWrite(howland_pin,LOW); // H
    digitalWrite(switch2_pin,HIGH);  // A
  }

    pos_neg = 2;
  }

   else if(pos_neg == 2){
  if(!positive3){
    delayMicroseconds(delayStart3);
    digitalWrite(switch1_pin,LOW);
    digitalWrite(howland_pin,HIGH); // H
    digitalWrite(switch2_pin,LOW);  // A
    
    send16bits(control_data_LSB);
    send16bits(lowRegister);
    send16bits(control_data_MSB);
    send16bits(highRegister);
    send16bits(Phase);
   
    delayMicroseconds(duration3); 
    send16bits(sleepy);  
    
    digitalWrite(howland_pin,LOW); // H
    digitalWrite(switch2_pin,HIGH);  // A
  }else{
    delayMicroseconds(delayStart3);
    digitalWrite(switch1_pin,HIGH);
    digitalWrite(howland_pin,HIGH); // H
    digitalWrite(switch2_pin,LOW);  // A

      send16bits(control_data_LSB);
      send16bits(lowRegister);
      send16bits(control_data_MSB);
      send16bits(highRegister);
      send16bits(Phase);
  
    delayMicroseconds(duration3);
    send16bits(sleepy);     

    digitalWrite(howland_pin,LOW); // H
    digitalWrite(switch2_pin,HIGH);  // A
  }

    pos_neg = 3;
  }

   else if(pos_neg == 3){
  if(!positive2){
    delayMicroseconds(delayStart4);
    digitalWrite(switch1_pin,LOW);
    digitalWrite(howland_pin,HIGH); // H
    digitalWrite(switch2_pin,LOW);  // A
    
    send16bits(control_data_LSB);
    send16bits(lowRegister);
    send16bits(control_data_MSB);
    send16bits(highRegister);
    send16bits(Phase);
   
    delayMicroseconds(duration4); 
    send16bits(sleepy);  
    
    digitalWrite(howland_pin,LOW); // H
    digitalWrite(switch2_pin,HIGH);  // A
  }else{
    delayMicroseconds(delayStart4);
    digitalWrite(switch1_pin,HIGH);
    digitalWrite(howland_pin,HIGH); // H
    digitalWrite(switch2_pin,LOW);  // A

      send16bits(control_data_LSB);
      send16bits(lowRegister);
      send16bits(control_data_MSB);
      send16bits(highRegister);
      send16bits(Phase);
  
    delayMicroseconds(duration4);
    send16bits(sleepy);     

    digitalWrite(howland_pin,LOW); // H
    digitalWrite(switch2_pin,HIGH);  // A
  }

    pos_neg = 0;
  }
  }
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
