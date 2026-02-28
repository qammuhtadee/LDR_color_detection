// Test Code: LED Cycle
void setup() {
  pinMode(3, OUTPUT); // Red
  pinMode(4, OUTPUT); // Green
  pinMode(5, OUTPUT); // Blue
}

void loop() {
  digitalWrite(3, HIGH); delay(500); digitalWrite(3, LOW); // Pulse Red
  digitalWrite(4, HIGH); delay(500); digitalWrite(4, LOW); // Pulse Green
  digitalWrite(5, HIGH); delay(500); digitalWrite(5, LOW); // Pulse Blue
}