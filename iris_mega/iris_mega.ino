#define light 6
#define light1 7
#define light2 8
#define fan 9
#define fan1 10

String inString;

void setup() {

  Serial3.begin(115200);
  pinMode(light, OUTPUT);
  digitalWrite(light, LOW);
  pinMode(light1, OUTPUT);
  digitalWrite(light1, LOW);
  pinMode(light2, OUTPUT);
  digitalWrite(light2, LOW);
  pinMode(fan, OUTPUT);
  digitalWrite(fan, LOW);
  pinMode(fan1, OUTPUT);
  digitalWrite(fan1, LOW);
}

void loop() {
}

void serialEvent3() {
  while (Serial3.available()) {

    char inChar = Serial3.read();

    Serial.write(inChar);

    inString += inChar;
    if (inChar == ']') {
      if (inString.indexOf("[on]") > 0) {
        digitalWrite(light, HIGH);
      }

      if (inString.indexOf("[off]") > 0) {
        digitalWrite(light, LOW);
      }

       if (inString.indexOf("[on1]") > 0) {
        digitalWrite(light1, HIGH);
      }

      if (inString.indexOf("[off1]") > 0) {
        digitalWrite(light1, LOW);
      }

       if (inString.indexOf("[on2]") > 0) {
        digitalWrite(light2, HIGH);
      }

      if (inString.indexOf("[off2]") > 0) {
        digitalWrite(light2, LOW);
      }

       if (inString.indexOf("[on3]") > 0) {
        digitalWrite(fan, HIGH);
      }

      if (inString.indexOf("[off3]") > 0) {
        digitalWrite(fan, LOW);
      }
      
       if (inString.indexOf("[on4]") > 0) {
        digitalWrite(fan1, HIGH);
      }

      if (inString.indexOf("[off4]") > 0) {
        digitalWrite(fan1, LOW);
      }
      inString = "";
    }
  }
}
