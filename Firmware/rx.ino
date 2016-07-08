unsigned long timer[4];
byte last_channel[4];



void rx_initialize() {
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8);
  PCMSK1 |= (1 << PCINT9);
  PCMSK1 |= (1 << PCINT10);
  PCMSK1 |= (1 << PCINT11);
  Serial.begin(9600);
}

void print_that_bitch() {
  print_values();
}

ISR(PCINT1_vect) {

  timer[0] = micros();//Lire la valeur PWM en microsecondes
  
  //////////////////////////CHANNEL1/////////////////////////////
  if(last_channel[0] == 0 && digitalRead(A0) == HIGH){//vaut digitalRead(HIGH); mais en plus rapide
    last_channel[0] = 1;//le state est HIGH
    timer[1] = timer[0];//Lire combien de temps HIGH
  }
  else if(last_channel[0] == 1 && digitalRead(A0) == LOW){
    last_channel[0] = 0;//le state est HIGH
    input[0] = timer[0] - timer[1];
  }



  //////////////////////////CHANNEL2/////////////////////////////
  if(last_channel[1] == 0 && digitalRead(A1) == HIGH){//Rising
    last_channel[1] = 1;//le state est HIGH
    timer[2] = timer[0];//Lire combien de temps HIGH
  }
  else if(last_channel[1] == 1 && digitalRead(A1) == LOW){//Falling
    last_channel[1] = 0;//le state est HIGH
    input[1] = timer[0] - timer[2];
  }

  //////////////////////////CHANNEL3/////////////////////////////
  if(last_channel[2] == 0 && digitalRead(A2) == HIGH){//Rising
    last_channel[2] = 1;//le state est HIGH
    timer[3] = timer[0];
  }
  else if(last_channel[2] == 1 && digitalRead(A2) == LOW){//Falling
    last_channel[2] = 0;//le state est HIGH
    input[2] = timer[0] - timer[3];
  }

  //////////////////////////CHANNEL4/////////////////////////////
  if(last_channel[3] == 0 && digitalRead(A3) == HIGH){//Rising
    last_channel[3] = 1;//le state est HIGH
    timer[4] = timer[0];
  }
  else if(last_channel[3] == 1 && digitalRead(A3) == LOW){//Falling
    last_channel[3] = 0;//le state est HIGH
    input[3] = timer[0] - timer[4];
  }
}

void print_values(){
  Serial.print(input[0]);//Ici on print les valeurs en microsecondes
  Serial.print(" - ");
  Serial.print(input[1]);
  Serial.print(" - ");
  Serial.print(input[2]);
  Serial.print(" - ");
  Serial.println(input[3]); 
}
