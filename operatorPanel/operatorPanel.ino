
#define ROW_A 2
#define ROW_B 3
#define ROW_C 4
#define ROW_D 5

#define COL_1 6
#define COL_2 7
#define COL_3 8

#define LVL_1 9
#define LVL_2 10
#define LVL_3 11
#define LVL_4 12

int sequenceCount;
int isConnected;


void setup() {
  Serial.begin(9600);

  pinMode(ROW_A, OUTPUT);
  pinMode(ROW_B, OUTPUT);
  pinMode(ROW_C, OUTPUT);
  pinMode(ROW_D, OUTPUT);

  pinMode(COL_1, OUTPUT);
  pinMode(COL_2, OUTPUT);
  pinMode(COL_3, OUTPUT);

  pinMode(LVL_1, OUTPUT);
  pinMode(LVL_2, OUTPUT);
  pinMode(LVL_3, OUTPUT);
  pinMode(LVL_4, OUTPUT);

  sequenceCount = 0;

}

void runLightSequence(int state){
  //Reset all before L states
    if(state == 24){
      clearAll();
    }
    //Go clockwise around the Positions
    if(state < 12){
        clearAll();
        lightSingle(state+1);
    }else if(state >= 12 && state < 24){
        //Go counter-clockwise around the Positions
        clearAll();
        lightSingle(23-state);
    }else if(state >= 24 && state < 28){      
      //Go Up the levels
        lightLevel(abs(23-state));
    }else if(state < 32){      
      //Go down the levels
      offLevel(33-state);
    }else{
      clearAll();
    }
}

char buffer[6];

void loop() {
  sequenceCount++; //Count every 50ms loop
  if(isConnected == 0){ //if the driverstation is connected stop sequence
    runLightSequence(sequenceCount / 4);
  }
  if(sequenceCount >= 4*32){ //If the sequence is done restart
    sequenceCount = 0;
  }
  if (readPosition()) {
    isConnected = 1; //If it receives a message from driver station laptop then stop sequence
    Serial.print("Received: ");
    buffer[5] = '\0';
    Serial.println(buffer);
    Serial.flush();

    int level = buffer[1] - '0';
    int pos = ((buffer[3] - '0') * 10) + (buffer[4] - '0');

    clearAll();
    lightLevel(level);
    lightSingle(pos);
  }

  delay(50);
}

bool readPosition() {
  bool started = false;
  int index = 0;
  
  if (Serial.available() < 5) return false;

  while (index < 5) {
    char c = Serial.read();

    if (!started) {
      if (c == 'L') {
        started = true;
      } else {
        continue;
      }
    }

    //Serial.print("Reading: ");
    //Serial.println(c);

    buffer[index] = c;
    index++;
  }

  return (index >= 5);
}

void clearAll() {
  digitalWrite(ROW_A, LOW);
  digitalWrite(ROW_B, LOW);
  digitalWrite(ROW_C, LOW);
  digitalWrite(ROW_D, LOW);

  digitalWrite(COL_1, HIGH);
  digitalWrite(COL_2, HIGH);
  digitalWrite(COL_3, HIGH);

  digitalWrite(LVL_1, LOW);
  digitalWrite(LVL_2, LOW);
  digitalWrite(LVL_3, LOW);
  digitalWrite(LVL_4, LOW);
}

void lightSingle(int led) {
  if (led < 13) {
    switch (led) {
      case 1:
        digitalWrite(ROW_A, HIGH);
        digitalWrite(COL_1, LOW);
        break;
      case 2:
        digitalWrite(ROW_A, HIGH);
        digitalWrite(COL_2, LOW);
        break;
      case 3:
        digitalWrite(ROW_A, HIGH);
        digitalWrite(COL_3, LOW);
        break;
      case 4:
        digitalWrite(ROW_B, HIGH);
        digitalWrite(COL_1, LOW);
        break;
      case 5:
        digitalWrite(ROW_B, HIGH);
        digitalWrite(COL_2, LOW);
        break;
      case 6:
        digitalWrite(ROW_B, HIGH);
        digitalWrite(COL_3, LOW);
        break;
      case 7:
        digitalWrite(ROW_C, HIGH);
        digitalWrite(COL_1, LOW);
        break;
      case 8:
        digitalWrite(ROW_C, HIGH);
        digitalWrite(COL_2, LOW);
        break;
      case 9:
        digitalWrite(ROW_C, HIGH);
        digitalWrite(COL_3, LOW);
        break;
      case 10:
        digitalWrite(ROW_D, HIGH);
        digitalWrite(COL_1, LOW);
        break;
      case 11:
        digitalWrite(ROW_D, HIGH);
        digitalWrite(COL_2, LOW);
        break;
      case 12:
        digitalWrite(ROW_D, HIGH);
        digitalWrite(COL_3, LOW);
        break;
    }

  } else {
    digitalWrite(led - 4, HIGH);
  }
}

void lightLevel(int level) {
  digitalWrite(level + 8, HIGH);
}

void offLevel(int level) {
  digitalWrite(level + 8, LOW);
}
