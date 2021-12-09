#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 64);

#define SERVO 23
#define PIN 25
#define POT A0
#define BUTTON 0
#define BUF_SIZE 1
// #define DELTA_TOLERANCE 10

// const byte RESET[4] = {' ', 'R', 'S', 'T'};

void print(String s) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(s);
    display.display();
}

void print(int s) {
    // display.clearDisplay();
    // display.setCursor(0, 0);
    display.print(s);
    display.display();
}

Servo servo;
int pot_input() {
    int input = analogRead(POT);
    double maxInput = 4095.0;

    int color = 0x0000FF;

    int rotation = 180 - (int)((input / maxInput) * 180);
    return rotation;
}

int setRotation(int position) {
    servo.write(position);
}

int pushTime = 0;
bool buttonPushed = true;
bool checkButtonPushed(int button) {
    // check if button pushed
    int curTime = millis();
    int val = digitalRead(button);
    if(val == HIGH) {
        pushTime = curTime + 50; // 50ms for button debouncing
        buttonPushed = true;
    } else if (val == LOW && curTime >= pushTime && buttonPushed) {
        buttonPushed = false;
    }

    return !buttonPushed;
}

byte* getSerialInput() {
// print("s");
    byte header[4];
    uint8_t *readBuf = display.getBuffer();
// print("h");
    if(Serial.available()) {
// print("d");
        Serial.readBytes(header, 4);
// print("f");
        byte i;
        for(i=0;i<4;i++) {
            if(header[i]!=i+61) break;  // verify frame header, should be 61,62,63,64
        }
// print("j");
        if(i==4) {  // valid frame header received
            Serial.readBytes(readBuf, BUF_SIZE); // read entire data
// print("v");
            return readBuf;
        } else {
// print("i");
            return NULL;
        }
// print("header valid");
    } else {
// print("q");
        return NULL;
    }
}

int defaultPosition = 0;
int calculateRotation(uint8_t *inputBuf) {
// print("C");
    int rotation;
    if(inputBuf == NULL){
// print("n");
        rotation = 0;
    } else if((char)*inputBuf == 'r') {
        // reset message
// print("r");
        return defaultPosition;
    } else if ((char)*inputBuf == 'R'){
        rotation = 1;
// print("R");
    } else if ((char)*inputBuf == 'L'){
        rotation = -1;
// print("L");
    } else if ((char)*inputBuf == 'N'){
        rotation = 0;
// print("N");
    }
    return (int) servo.read() + rotation;
}

void setup() {
    // init serial connection
    Serial.begin(115200);
    // init button
    pinMode(BUTTON, INPUT_PULLUP); 
    // init servo
    servo.attach(SERVO);
    // init OLED display
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextSize(1);
    display.setTextColor(WHITE);   
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Set the default position of the camera");
    display.println("Press button 1 to confirm");
    display.display();
}
int i = 0;
bool defaultPosSet = false;
int target_angle = 0;
void loop() {
    // setRotation(pot_input());
    int rotation;
    if(!defaultPosSet) {
        rotation = pot_input();
        if(checkButtonPushed(BUTTON)) {
            defaultPosSet = true;
            defaultPosition = pot_input();
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Position set!");
            display.display();
        }
    } else {
        rotation = calculateRotation(getSerialInput());
// print("M");
print("updating rotation: ");
    }
// print(".");

print(rotation);
    setRotation(rotation);
}