#define INA_PIN 11
#define INB_PIN 9
#define PWM_PIN 10
#define RUN_MILLIS 300
#define PIN_PULLPIN 6
#define PIN_PULLPINDRIVER 5

void setup() {

  pinMode(PWM_PIN,OUTPUT);
  digitalWrite(PWM_PIN,LOW);
  pinMode(INA_PIN,OUTPUT);
  pinMode(INB_PIN,OUTPUT);

  pinMode(PIN_PULLPIN, INPUT_PULLUP);
  pinMode(PIN_PULLPINDRIVER, OUTPUT);
  digitalWrite(PIN_PULLPINDRIVER, LOW);
  
  digitalWrite(INB_PIN,LOW);
  digitalWrite(INA_PIN,LOW);
  //digitalWrite(PWM_PIN, HIGH);
  pinMode(LED_BUILTIN,OUTPUT);
   Serial.begin(9600);
}

void loop() {
  // Waits for pin to be pulled and replaced
  while(digitalRead(PIN_PULLPIN));
  delay(300);
  while(!digitalRead(PIN_PULLPIN));
  digitalWrite(LED_BUILTIN,HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PWM_PIN, LOW);
  digitalWrite(INB_PIN, LOW);
  digitalWrite(INA_PIN, HIGH);
  digitalWrite(PWM_PIN, HIGH);

  delay(RUN_MILLIS);

  digitalWrite(LED_BUILTIN,LOW);
  digitalWrite(INA_PIN,LOW);
  analogWrite(PWM_PIN, LOW);
}
