// #
// # Editor(s)  :  Morgan ROGER from Centralesupelec
// # Last       :  Morgan ROGER from Centralesupelec
// # Version    :  25.10.2026

// # Description:
// # The sketch for testing the infrared sensor Sharp GP2Y0A21YK0F on the ST5 VAC robot

// # Connections:
// #        Sensor  -> Analog pin A6 (triple connector yellow, red, black)


/* used Arduino pins */
const int infraredPin = A6;  // pin number to connect the sensor to

/* sensor management */
float volts = 0;             // sensor analog value
int distanceInCm = 80;       // max distance detected

/* task management */
#define numtask 1    // number of tasks
int deltask[numtask] = {500};  // array defining the period of all tasks
int timtask[numtask] ;         // array with the next execution time of all tasks
bool taskon[numtask] = {false}; // array with activation state of all tasks
void task0();                  // declaring the function for each task
void (*NameTask[numtask])() = {  task0  }; // array with the name of all task functions

void TaskOn(int task) {
  taskon[task]=true;  
  timtask[task] = (int)millis()+deltask[task]; 
}


/* SETUP */
void setup() {
  Serial.begin(115200);   // init the Serial port and set the baud rate
  TaskOn(0);              // activate task #0
}

/* LOOP */
void loop() {
  for (int k=0;k<numtask;k++)
    if (taskon[k]) 
      if ((((int)millis()-timtask[k])>0) || (deltask[k]==0))  // if execution time has come
      {
        NameTask[k]() ;                   // launching task #k
        timtask[k]=timtask[k]+deltask[k] ;
      }
}

/* TASK #0 = print distance measured with IR */
void task0() {
  volts = analogRead(infraredPin)*5.0/1024.0;  // 10 bits over 5 volts
  Serial.print(volts);
  Serial.print(" V : ");
  distanceInCm = convertMeasure();   // translate volts to obstacle distance
  if (distanceInCm<8 && distanceInCm!= 0) {
	Serial.println("Too close!");
  } else if (distanceInCm>80 || distanceInCm== 0) {
	Serial.println("Out of range");
  } else {
    Serial.print(distanceInCm);
    Serial.println(" cm");
  }
}


/* convert volts to distance */
int convertMeasure() {  // approximate sensor characteristics
  if (volts<1) {
	distanceInCm = 28.0/volts;
  } else {
	volts -= 0.28;
	distanceInCm = 20.2/volts;
  }
  return distanceInCm;
}
