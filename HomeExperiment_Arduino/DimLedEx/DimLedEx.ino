// Define pin connections
const int potPin = A0;
const int redLEDPin = 9;
const int greenLEDPin = 10;
const int blueLEDPin = 11;

void setup() {
  // Set LED pins as output
  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
}

void loop() {
  // Read the potentiometer value
  int potValue = analogRead(potPin);
  
  // Map the potentiometer value to the PWM range (0-255)
  int brightness = map(potValue, 0, 1023, 0, 255);
  
  // Set the brightness of each LED
  analogWrite(redLEDPin, brightness);
  analogWrite(greenLEDPin, brightness);
  analogWrite(blueLEDPin, brightness);
  
  // Add a small delay
  delay(10);
}
