void setup()
{
  pinMode(P1_4, OUTPUT); //write enable
  pinMode(P2_4, INPUT_PULLUP); //read input
  pinMode(P2_3, OUTPUT); //driving output
  pinMode(P2_5, OUTPUT);//LED inside
  pinMode(P1_5, OUTPUT);//LED outside
  pinMode(P2_0, INPUT_PULLUP);//btn inside
  pinMode(P2_1, INPUT_PULLUP);//btn outside
  digitalWrite(P1_4, HIGH);
  digitalWrite(P1_5, LOW);
}
//1.4 select
//2.3 output
int output = LOW;
void loop()
{
  digitalWrite(P2_3, output);
  digitalWrite(P2_5, output);
  digitalWrite(P1_5, 1 - output);
  output = 1 - output;
  delay(300);
}
