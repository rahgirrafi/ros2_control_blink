const int LED = 13;
int char_to_int(char ch);
void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() >= 1) {
    int signal = Serial.parseInt();

    Serial.println(signal);
    // 
    digitalWrite(LED, signal ? HIGH : LOW);
  }
}

int char_to_int(char ch){
  int a = ch - '0';
  return a;
}