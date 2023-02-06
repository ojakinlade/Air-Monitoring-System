#define NO2_PIN A0
#define NH3_PIN A1
#define CO_PIN  A2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(NO2_PIN, INPUT_PULLUP);
  pinMode(NH3_PIN, INPUT_PULLUP);
  pinMode(CO_PIN, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t NO2_val = analogRead(NO2_PIN);
  uint16_t NH3_val = analogRead(NH3_PIN);
  uint16_t CO_val = analogRead(CO_PIN);

  double NO2_conc = map(NO2_val, 0, 1023, 0.05, 10);
  uint16_t NH3_conc = map(NH3_val, 0, 1023, 1, 500);
  uint16_t CO_conc = map(CO_val, 0, 1023, 1, 1000);
  
  Serial.print("NO2 conc(ppm): ");
  Serial.println(NO2_conc);
  Serial.print("NH3 conc(ppm): ");
  Serial.println(NH3_conc);
  Serial.print("CO conc(ppm): ");
  Serial.println(CO_conc);
  delay(2000);
}
