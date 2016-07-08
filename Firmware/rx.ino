unsigned long timer[4];
byte last_channel[4];



void rx_initialize() {
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  Serial.begin(9600);
}

void print_that_bitch() {
  print_values();
}

ISR(PCINT0_vect) {

  timer[0] = micros();//Lire la valeur PWM en microsecondes
  
  //////////////////////////CHANNEL1/////////////////////////////
  if(last_channel[0] == 0 && PINB & B00000001){//vaut digitalRead(HIGH); mais en plus rapide
    last_channel[0] = 1;//le state est HIGH
    timer[1] = timer[0];//Lire combien de temps HIGH
  }
  else if(last_channel[0] == 1 && !(PINB & B00000001)){
    last_channel[0] = 0;//le state est HIGH
    input[0] = timer[0] - timer[1];
  }



  //////////////////////////CHANNEL2/////////////////////////////
  if(last_channel[1] == 0 && PINB & B00000010){//Rising
    last_channel[1] = 1;//le state est HIGH
    timer[2] = timer[0];//Lire combien de temps HIGH
  }
  else if(last_channel[1] == 1 && !(PINB & B00000010)){//Falling
    last_channel[1] = 0;//le state est HIGH
    input[1] = timer[0] - timer[2];
  }

  //////////////////////////CHANNEL3/////////////////////////////
  if(last_channel[2] == 0 && PINB & B00000100){//Rising
    last_channel[2] = 1;//le state est HIGH
    timer[3] = timer[0];
  }
  else if(last_channel[2] == 1 && !(PINB & B00000100)){//Falling
    last_channel[2] = 0;//le state est HIGH
    input[2] = timer[0] - timer[3];
  }

  //////////////////////////CHANNEL4/////////////////////////////
  if(last_channel[3] == 0 && PINB & B00001000){//Rising
    last_channel[3] = 1;//le state est HIGH
    timer[4] = timer[0];
  }
  else if(last_channel[3] == 1 && !(PINB & B00001000)){//Falling
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
