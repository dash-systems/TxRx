// this code is based on RadioHead69_RawDemoTXRX_OLED
// 5.9.19 p. stahlhuth

#include <SPI.h>
#include <RH_RF95.h>

// -------  servo   -------
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  125 //  this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 //  this is the 'maximum' pulse length count (out of 4096)

#define BUTTON_A 9
#define LED      13

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

#define RFM95_CS      8
#define RFM95_INT     7
#define RFM95_RST     4

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


void setup() {
  delay(500);
  Serial.begin(9600);   // Sends data for USB serial monitor
  rf95.setSignalBandwidth(125000);
  rf95.setCodingRate4(8);
  rf95.setSpreadingFactor(12); 

//  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_A, INPUT);
  
  pinMode(LED, OUTPUT);     a
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather LoRa RX Test!");

  // servo
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
}

void loop()
{  if (rf95.waitAvailableTimeout(100)) {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (! rf95.recv(buf, &len)) {
      Serial.println("Receive failed");
      return;
    }
    digitalWrite(LED, HIGH);
    rf95.printBuffer("Received: ", buf, len);
    buf[len] = 0;
    
    Serial.print("Got: "); Serial.println((char*)buf);
    Serial.print("RSSI: "); Serial.println(rf95.lastRssi(), DEC);

    Blink(LED, 40, 20); //blink LED 20 times, 40ms between blinks

    ServoCycle();

    digitalWrite(LED, LOW);
  }

  if (!digitalRead(BUTTON_A))
  {
    Serial.println("Button pressed!");

//    Blink(LED, 40, 20); //blink LED 20 times, 40ms between blinks
    
    char radiopacket[20] = "Button #";
//    if (!digitalRead(BUTTON_A)) radiopacket[8] = 'A';
    radiopacket[9] = 0;

    Serial.print("Sending "); Serial.println(radiopacket);
    rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf95.waitPacketSent();

    Blink(LED, 40, 20); //blink LED 20 times, 40ms between blinks
    
  }
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

void ServoCycle() {
//  pwm.setPWM(0, 0, 500);      // test line 3
//  delay(1000);                // test line 4
  pwm.setPWM(0, 0, 200);      // original 125 
  delay(1000);
  pwm.setPWM(0, 0, 450);      // origiinal 500
//  delay(1000);                 // test line 1
//  pwm.setPWM(0, 0, 500);       // test line 2
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}
