#define A 4
#define B 5

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A, INPUT);
  pinMode(B, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("A: ");
  Serial.println(digitalRead(A));
  Serial.print("B: ");
  Serial.println(digitalRead(B));
}
