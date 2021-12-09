#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 64);

#define SERVO 23
#define PIN 25
#define POT A0
#define BUTTON 0

Servo servo;
int pot_input() {
    int input = analogRead(POT);
    double maxInput = 4095.0;

    int color = 0x0000FF;

    int rotation = 360 - (int)((input / maxInput) * 360);
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

int getSerialInput() {
    
}

int calculateRotation(int delta) {

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
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);   
    display.setCursor(0, 0);
    display.println("Set the default position of the camera");
    display.println("Press button 1 to confirm");
    display.display();
}
int i = 0;
int defaultPosition = 0;
bool defaultPosSet = false;
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
    } else (
        rotation = calculateRotation(getSerialInput())
    )

    setRotation(rotation);
}