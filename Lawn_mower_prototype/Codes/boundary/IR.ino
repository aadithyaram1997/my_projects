void IRModule()
{
  IRvalueD1 = digitalRead(LS);
  IRvalueD2 = digitalRead(RS);
  IRvalueD3 = digitalRead(MS);
  Serial.print("IRvalueD1: ");
  Serial.println(IRvalueD1);
  Serial.print("IRvalueD2: ");
  Serial.println(IRvalueD2);
}
