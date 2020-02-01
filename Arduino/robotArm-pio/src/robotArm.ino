#include "pinout.h"
#include "robotGeometry.h"
#include "interpolation.h"
#include "fanControl.h"
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"

#include <Stepper.h>


RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
RobotGeometry geometry;
Interpolation interpolator;
Queue<Cmd> queue(15);
Command command;
bool absolute=true;
int button=0;
int button_released=0;

void setup() {
  Serial.begin(115200);
  
  //various pins..
  pinMode(LED_PIN       , OUTPUT);
  
  pinMode(TEST_PIN,INPUT_PULLUP);
  
  // pinMode(ROTATE_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(LOWER_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(HIGHER_ENDSTOP_PIN, INPUT_PULLUP);

  
  //reduction of steppers..
  stepperHigher.setReductionRatio(32.0 / 9.0, 200 * 16);  //big gear: 32, small gear: 9, steps per rev: 200, microsteps: 16
  stepperLower.setReductionRatio( 32.0 / 9.0, 200 * 16);
  stepperRotate.setReductionRatio(32.0 / 9.0, 200 * 16);
 
  //start positions..
  //stepperHigher.setPositionRad(PI / 2.0);  //90°
  //stepperLower.setPositionRad(0);          // 0°
  //stepperRotate.setPositionRad(0);         // 0°
  //stepperExtruder.setPositionRad(0);

  //enable and init..
  setStepperEnable(false);

  delay(50);
  
  // calibration();
  
  //interpolator.setInterpolation(-18,4,140,0, -18,4,140,0);
  interpolator.setInterpolation(-41,11,141,0, -41,11,141,0);
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  stepperRotate.setPositionRad(geometry.getRotRad());
  stepperLower.setPositionRad(geometry.getLowRad());
  stepperHigher.setPositionRad(geometry.getHighRad());
  
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

void loop () {
  actionButton();

  //update and Calculate all Positions, Geometry and Drive all Motors...
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad (geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
  stepperRotate.update();
  stepperLower.update();
  stepperHigher.update(); 
  
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




void cmdMove(Cmd (&cmd)) {

   if(absolute){
    Serial.print("echo move abs");
    Serial.print(" X:");
    Serial.print(cmd.valueX);
    Serial.print(" Y:");
    Serial.print(cmd.valueY);
    Serial.print(" Z:");
    Serial.print(cmd.valueZ);
    Serial.print(" E:");
    Serial.println(cmd.valueE);
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
  Serial.print(geometry.getXmm());
  Serial.print(" Y:");
  Serial.print(interpolator.getYPosmm());
  Serial.print(" Z:");
  Serial.print(interpolator.getZPosmm());
  Serial.print(" E:");
  Serial.print(interpolator.getEPosmm());
  Serial.print(" ");
  Serial.print(stepperHigher.getPositionRad());
}

void cmdSetPosition(Cmd (&cmd)){
  interpolator.setCurrentPos(cmd.valueX,cmd.valueY,cmd.valueZ,cmd.valueE);
}

void cmdHome(Cmd (&cmd)){
  //Home always all axis because we do not have the passed XYZ letters here
  //Do homing by moving up the lower shank and disable the higher shank
  Serial.println(stepperLower.getPosition());
  stepperLower.setPosition(0);
  stepperHigher.enable(false); //Spring
  for(int i=0;i<250;i++){
    stepperLower.stepToPosition(i*-1);  
    //stepperHigher.stepToPosition(i*-1);  
    stepperLower.update();
    //stepperHigher.update();
    delay(10); 
    Serial.println(stepperLower.getPosition());
    //Serial.println(stepperHigher.getPosition());
  }
  stepperHigher.enable(true);
  interpolator.setCurrentPos(0,-20,125,0);
  cmd.valueX=0;
  cmd.valueY=120;
  cmd.valueZ=160;
  cmdMove(cmd);
}


void cmdOpen(Cmd (&cmd)){
  //Home always all axis because we do not have the passed XYZ letters here
  Serial.println(stepperLower.getPosition());
  stepperLower.setPosition(0);
  stepperHigher.setPosition(0);
  stepperHigher.enable(false);
  for(int i=0;i<200;i++){
    stepperLower.stepToPosition(i*-1);  
    stepperLower.update();

    if(i==100){
       Serial.println("echo High turbo");
      stepperHigher.enable(true);
    }
    if(i>100){
          stepperHigher.stepToPosition(i*0.05);  
          stepperHigher.update();
    }
    delay(15); 
  }
  stepperHigher.enable(true);
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
      case 105: Serial.println("Test echo "); break;
      case 106: testGCode(); break;
      case 100: calibration(); break;
      //case 106: cmdFanOn(); break;
      //case 107: cmdFanOff(); break;
      case 114: cmdGetPos();break;
      case 40: cmdOpen(cmd);break;
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


