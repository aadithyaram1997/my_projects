void motor_function(char m_state) {
  switch(m_state) {
    
    case 'S':
     digitalWrite(motor1Pin1, LOW);
     digitalWrite(motor1Pin2, LOW);
     digitalWrite(motor2Pin1, LOW);
     digitalWrite(motor2Pin2, LOW);      
      break;
      
    case 'F':
     digitalWrite(motor1Pin1, LOW);
     digitalWrite(motor1Pin2, HIGH); 
     digitalWrite(motor2Pin1, LOW);
     digitalWrite(motor2Pin2, HIGH); 
      break;
      
    case 'B':
     digitalWrite(motor1Pin1, HIGH);
     digitalWrite(motor1Pin2, LOW); 
     digitalWrite(motor2Pin1, HIGH);
     digitalWrite(motor2Pin2, LOW);
      break;
      
    case 'R':
     digitalWrite(motor1Pin1, HIGH);
     digitalWrite(motor1Pin2, LOW);
     digitalWrite(motor2Pin1, LOW);
     digitalWrite(motor2Pin2, HIGH);
      break;
      
    case 'L':
     digitalWrite(motor1Pin1, LOW);
     digitalWrite(motor1Pin2, HIGH);
     digitalWrite(motor2Pin1, HIGH);
     digitalWrite(motor2Pin2, LOW);
      break;

    default : 
     digitalWrite(motor1Pin1, LOW);
     digitalWrite(motor1Pin2, LOW);
     digitalWrite(motor2Pin1, LOW);
     digitalWrite(motor2Pin2, LOW);      
    
  }
}
