#include "pinout.h"
#include "robotGeometry.h"
#include "interpolation.h"
#include "fanControl.h"
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"
#include "limiter.h"

#include <Stepper.h>


RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
RobotGeometry geometry;
Interpolation interpolator;
Interpolation interpolator_axis;
Interpolation interpolator_rad;
Limiter limiter;
Queue<Cmd> queue(15);
Command command;
bool absolute=true;
int button=0;
int button_released=0;
bool useAxis=true;

void setup() {
  Serial.begin(115200);
  
  //various pins..
  pinMode(LED_PIN       , OUTPUT);
  
  pinMode(TEST_PIN,INPUT_PULLUP);
  
  // pinMode(ROTATE_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(LOWER_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(HIGHER_ENDSTOP_PIN, INPUT_PULLUP);

  
  //reduction of steppers..
  stepperHigher.setReductionRatio(5.1, 200 * 8);  //big gear: 32, small gear: 9, steps per rev: 200, microsteps: 16
  stepperLower.setReductionRatio( 5.1, 200 * 8);
  stepperRotate.setReductionRatio(5.1, 200 * 1);
 
  //start positions..
  //stepperHigher.setPositionRad(PI / 2.0);  //90°
  //stepperLower.setPositionRad(0);          // 0°
  //stepperRotate.setPositionRad(0);         // 0°
  //stepperExtruder.setPositionRad(0);

  //enable and init..
  // setStepperEnable(false);

  delay(50);
  
  // calibration();
  
  // interpolator.setInterpolation(0,THICK_ARM_LEN,THICK_ARM_LEN,0, 0,THICK_ARM_LEN,THICK_ARM_LEN,0);
  // interpolator.updateActualPosition();
  interpolator=interpolator_axis;
  interpolator.setCurrentPos(0,THICK_ARM_LEN,THICK_ARM_LEN,0);
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  if(!limiter.isAvailable(geometry.getRotRad(), geometry.getLowRad(), geometry.getHighRad())){
    Serial.println("bad init position");
  }
  stepperRotate.setPositionRad(geometry.getRotRad());
  stepperLower.setPositionRad(geometry.getLowRad());
  stepperHigher.setPositionRad(geometry.getHighRad());
  
  setStepperEnable(true);
  // interpolator.setInterpolation(0,THICK_ARM_LEN,THICK_ARM_LEN / 2.0,0);
  Serial.println("start");
  
}

void setStepperEnable(bool enable) {
  if(enable){
    Serial.println("echo Enable Stepper");
  }else{
    Serial.println("echo Disable Stepper");
  }
  stepperRotate.enable(enable);
  stepperLower.enable(enable);
  stepperHigher.enable(enable); 
}

void actionButton(){
  button = digitalRead(TEST_PIN);
  if(button == LOW && button_released == 0){
    button_released=1;
    
   Serial.println("echo Action Button pressed");
    if(queue.isEmpty()){
      testGCode();
    }else{
      Serial.println("echo clear command queue");
      queue.clear();
    }
  }
  if(button_released == 1 && button == HIGH){
    button_released=0;
    Serial.println("echo Button released");
    delay(700);
  }
}

void testGCode(){
  String gcode[]={"M17","G28","G1 Z120 Y120","G1 X5 Z-90 Y100","G4 T1","G1 Z-145 Y95","G1 Y132","M40","M18"};
  for (int a=0; a <9; a++){
      command.insertGcode(gcode[a]);
      queue.push(command.getCmd());
  }
}

void testMotor(){
  // Serial.println("stepperLower.stepRelative");
  // for(int i=1;i<20;i++){
  //   do{
  //     stepperLower.stepRelative(100);
  //     stepperLower.update();
  //     delay(100);
  //     Serial.print(".");
  //   }while(!stepperLower.isOnPosition());
  //   Serial.println("");
  // }

  Serial.println("stepperHigher.stepRelative");
  for(int i=1;i<20;i++){
    do{
      stepperHigher.stepRelative(10);
      stepperHigher.update();
      delay(100);
      Serial.print(".");
    }while(!stepperHigher.isOnPosition());
    Serial.println("");
  }

  Serial.println("stepperRotate.stepRelative");
  for(int i=1;i<20;i++){
    do{
      stepperRotate.stepRelative(10);
      stepperRotate.update();
      delay(100);
      Serial.print(".");
    }while(!stepperRotate.isOnPosition());
    Serial.println("");
  }

  Serial.println("testMotor done");
  delay(100);
}

void loop () {
  actionButton();

  if(useAxis){
  //update and Calculate all Positions, Geometry and Drive all Motors...
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  if(limiter.isAvailable(geometry.getRotRad(), geometry.getLowRad(), geometry.getHighRad())){
    stepperRotate.stepToPositionRad(geometry.getRotRad());
    stepperLower.stepToPositionRad (geometry.getLowRad());
    stepperHigher.stepToPositionRad(geometry.getHighRad());
  }else{
    Serial.print("skipped motion:rot ");
    Serial.print(geometry.getRotRad());
    Serial.print(", low:");
    Serial.print(geometry.getLowRad());
    Serial.print(", high:");
    Serial.print(geometry.getHighRad());
    Serial.print(", x:");
    Serial.print(interpolator.getXPosmm());
    Serial.print(", y:");
    Serial.print(interpolator.getYPosmm());
    Serial.print(", z:");
    Serial.print(interpolator.getZPosmm());
    Serial.println();
    geometry.from(stepperRotate.getPositionRad(), stepperLower.getPositionRad(), stepperHigher.getPositionRad());
    interpolator.setCurrentPos(geometry.getXmm(), geometry.getYmm(), geometry.getZmm(), 0);
    if(!absolute){
      queue.clear();
    }
    delay(1000);
  }
  // bool moved=false;
  // if(stepperRotate.getLeftDistance()!=0){
  //   Serial.print(" rot:");
  //   Serial.print(stepperRotate.getLeftDistance());
  //   moved=true;
  // }
  // if(stepperLower.getLeftDistance()!=0){
  //   Serial.print(" low:");
  //   Serial.print(stepperLower.getLeftDistance());
  //   moved=true;
  // }
  // if(stepperHigher.getLeftDistance()!=0){
  //   Serial.print(" high:");
  //   Serial.print(stepperHigher.getLeftDistance());
  //   moved=true;
  // }
  // if(moved){
  //   Serial.println();
  // }
  stepperRotate.update();
  stepperLower.update();
  stepperHigher.update(); 
  }else{
    interpolator.updateActualPosition();
    if(limiter.isAvailable(interpolator.getZPosmm(), interpolator.getYPosmm(), interpolator.getXPosmm())){
      stepperRotate.stepToPositionRad(interpolator.getZPosmm());
      stepperLower.stepToPositionRad (interpolator.getYPosmm());
      stepperHigher.stepToPositionRad(interpolator.getXPosmm());
    }else{
      Serial.print("skipped motion:rot ");
      Serial.print(interpolator.getZPosmm());
      Serial.print(", low:");
      Serial.print(interpolator.getYPosmm());
      Serial.print(", high");
      Serial.print(interpolator.getXPosmm());
      Serial.println();
    }
  }
  
  if (!queue.isFull()) {
    if (command.handleGcode()) {
      queue.push(command.getCmd());
      
    }
  }
  if ((!queue.isEmpty()) && interpolator.isFinished()) {
    executeCommand(queue.pop());
    printOk();
  }
    
  if (millis() %500 <250) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

void cmdHome(Cmd (&cmd)){
  interpolator.setInterpolation(0,THICK_ARM_LEN,THICK_ARM_LEN,0);
}

void cmdMove(Cmd (&cmd)) {
   if(absolute){
     Serial.print("echo move abs");
    interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE, cmd.valueF);
   }else{
    Serial.print("echo move rel:");
    Serial.print(" X:");
    Serial.print(interpolator.getXPosmm()+cmd.valueX);
    Serial.print(" Y:");
    Serial.print(interpolator.getYPosmm()+cmd.valueY);
    Serial.print(" Z:");
    Serial.print(interpolator.getZPosmm()+cmd.valueZ);
    Serial.print(" E:");
    Serial.println(interpolator.getEPosmm()+cmd.valueE);
    interpolator.setInterpolation(interpolator.getXPosmm()+cmd.valueX,interpolator.getYPosmm()+cmd.valueY, interpolator.getZPosmm()+cmd.valueZ, interpolator.getEPosmm()+cmd.valueE, cmd.valueF);
   }
    Serial.print(" X:");
    Serial.print(interpolator.getXPosmm());
    Serial.print(" Y:");
    Serial.print(interpolator.getYPosmm());
    Serial.print(" Z:");
    Serial.print(interpolator.getZPosmm());
    Serial.print(" E:");
    Serial.println(interpolator.getEPosmm());
}
void cmdDwell(Cmd (&cmd)) {
  delay(int(cmd.valueT * 1000));
}

void cmdStepperOn() {
  setStepperEnable(true);
}
void cmdStepperOff() {
  setStepperEnable(false);
}

void cmdGetPos() {
  Serial.print("X:");
  Serial.print(interpolator.getXPosmm());
  Serial.print(" Y:");
  Serial.print(interpolator.getYPosmm());
  Serial.print(" Z:");
  Serial.print(interpolator.getZPosmm());
  Serial.print(" E:");
  Serial.print(interpolator.getEPosmm());
  Serial.print(" rot:");
  Serial.print(stepperRotate.getPositionRad());
  Serial.print(" low:");
  Serial.print(stepperLower.getPositionRad());
  Serial.print(" high:");
  Serial.print(stepperHigher.getPositionRad());
  Serial.println();
}

void cmdSetPosition(Cmd (&cmd)){
  interpolator.setCurrentPos(cmd.valueX,cmd.valueY,cmd.valueZ,cmd.valueE);
}

void cmdUseRad(){
  useAxis=false;
  interpolator=interpolator_rad;
  interpolator.setCurrentPos(stepperHigher.getPositionRad(), stepperLower.getPositionRad(), stepperRotate.getPositionRad(),0);
}

void cmdUseAxis(){
  useAxis=true;
  interpolator=interpolator_axis;
  geometry.from(stepperRotate.getPositionRad(), stepperLower.getPositionRad(), stepperHigher.getPositionRad());
  interpolator.setCurrentPos(geometry.getXmm(), geometry.getYmm(), geometry.getZmm(), 0);
}

void handleAsErr(Cmd (&cmd)) {
  printComment("Unknown Cmd " + String(cmd.id) + String(cmd.num) + " (queued)"); 
  printFault();
}

void executeCommand(Cmd cmd) {
  if (cmd.id == -1) {
    String msg = "parsing Error";
    printComment(msg);
    handleAsErr(cmd);
    return;
  }
  
  if (isnan(cmd.valueX)) {
    cmd.valueX=0;
    if(absolute){
      cmd.valueX = interpolator.getXPosmm();
    }
  }
  if (isnan(cmd.valueY)) {
    cmd.valueY=0;
    if(absolute){
      cmd.valueY = interpolator.getYPosmm();
    }
  }
  if (isnan(cmd.valueZ)) {
    cmd.valueZ=0;
    if(absolute){
      cmd.valueZ = interpolator.getZPosmm();
    }
  }
  if (isnan(cmd.valueE)) {
    cmd.valueE=0;
    if(absolute){
      cmd.valueE = interpolator.getEPosmm();
    }
  }
  
   //decide what to do
  if (cmd.id == 'G') {
    switch (cmd.num) {
      case 0: cmdMove(cmd); break;
      case 1: cmdMove(cmd); break;
      case 4: cmdDwell(cmd); break;
      case 28: cmdHome(cmd); break;
      //case 21: break; //set to mm
      case 90: absolute=true; break;
      case 91: absolute=false; break;
      case 92: cmdSetPosition(cmd); break;
      default: handleAsErr(cmd); 
    }
  } else if (cmd.id == 'M') {
    switch (cmd.num) {
      //case 0: cmdEmergencyStop(); break;
      //case 3: cmdGripperOn(cmd); break;
      //case 5: cmdGripperOff(cmd); break;
      case 17: cmdStepperOn(); break;
      case 18: cmdStepperOff(); break;
      case 20: cmdUseRad(); break;
      case 21: cmdUseAxis(); break;
      case 105: Serial.println("Test echo "); break;
      case 106: testGCode(); break;
      case 107: testMotor(); break;
      // case 100: calibration(); break;
      //case 106: cmdFanOn(); break;
      //case 107: cmdFanOff(); break;
      case 114: cmdGetPos();break;
      default: handleAsErr(cmd); 
    }
  } else {
    handleAsErr(cmd); 
  }
}

void calibration(){
  float step = 0.0003;
  float i;

  for(i = 0;; i -= step)
  {
    if((digitalRead(LOWER_ENDSTOP_PIN) != HIGH) && (digitalRead(HIGHER_ENDSTOP_PIN) != HIGH)){
      stepperLower.stepToPositionRad(i);
      stepperLower.update();
      stepperHigher.stepToPositionRad(i);
      stepperHigher.update();
      delay(1);
    }
    else if((digitalRead(LOWER_ENDSTOP_PIN) != HIGH) && (digitalRead(HIGHER_ENDSTOP_PIN) == HIGH)){
      stepperLower.stepToPositionRad(i);
      stepperLower.update();
      delay(1);
    }
    else if((digitalRead(LOWER_ENDSTOP_PIN) == HIGH) && (digitalRead(HIGHER_ENDSTOP_PIN) != HIGH)){
      stepperHigher.stepToPositionRad(i);
      stepperHigher.update();
      delay(1);
    }
    else{
      break;
    }
 }

//  for(i = 0;; i -= step)
//   {
//     if(digitalRead(ROTATE_ENDSTOP_PIN) == HIGH){
//       break;
//     }
//     stepperRotate.stepToPositionRad(i);
//     stepperRotate.update();
//     delay(1);
//   }
}


