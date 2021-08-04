#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <webots/PositionSensor.hpp>
#include <windows.h>
#include <webots/InertialUnit.hpp>
#include <webots/Camera.hpp>
// All the webots classes are defined in the "webots" namespace

int start=0;
int maze=0;
int box=0;
int pick=0;
int level=0;
int quadrant = 4;
int dot_line=0;
double ramp=0;
int pillar=0;
int gate=0;
int right=0;
int check=0;
int ir=0;

int diff=0;
int pillar_count=0;
int correction=0;
using namespace webots;

#define Time_Step 16
#define Max_Speed 10

double ir0_val;
double ir1_val;
double ir2_val;
double ir3_val;
double ir4_val;
double ir5_val;

double ir6_val;
double ir7_val;
double ir8_val;
double ir9_val;

double ir_left_val;
double ir_right_val;

double ps1_val;
double ps2_val;

double ds_val;
double ir_val;
double ds_left_val;
double ds_right_val;

double ds_front_val;

// create the Robot instance.
Robot *robot = new Robot();
Motor *leftMotor =robot->getMotor("motor2");
Motor *rightMotor =robot->getMotor("motor1");
DistanceSensor *ds_left = robot-> getDistanceSensor("ds_left"); 
DistanceSensor *ds_right = robot-> getDistanceSensor("ds_right"); 
DistanceSensor *ds_front = robot-> getDistanceSensor("ds_front"); 
// get the time step of the current world.
int timeStep = (int)robot->getBasicTimeStep();




//double meterperrad = 0.04;
//double radperdeg = 0.052;
void right_turn(int i,int s){
  std::cout<<"right turn\n";
  double j=0;
  while (robot->step(timeStep) != -1) {
    j++;
    leftMotor->setVelocity(s);
    rightMotor->setVelocity(-s);
    if (j==i){
      break;
    }
  }
}



void left_turn(int i,int s){
  std::cout<<"left turn\n";
  double j=0;
  while (robot->step(timeStep) != -1) {
    j++;
    leftMotor->setVelocity(-s);
    rightMotor->setVelocity(s);
    if (j==i){
      break;
    }
  }
}


void forward(int i,int rs, int ls){
  std::cout<<"forward function\n";
  int j=0;
   while (robot->step(timeStep) != -1) {
      j++;
      leftMotor->setVelocity(ls);
      rightMotor->setVelocity(rs);
      if (j==i){
        break;
      }  
   }
}


 

double get_error(){
    return  (3*ir0_val+2*ir1_val+ir2_val-ir3_val-2*ir4_val-3*ir5_val)/30;
}

double get_error2(){
    return  (2*ir6_val+ir7_val-ir8_val-2*ir9_val)/30;
}

void LineFollowing(){
  double error= get_error();
  double last_error = 0;
  double derivative = error - last_error;
  last_error = error;
  
  leftMotor->setVelocity(3-25*error-45*derivative);
  rightMotor->setVelocity(3+25*error+45*derivative);

}

// Wall following
void WallFollowing(){
   // PID parameters for right motor
    double target =60;
    double last_wr_error = 0;
    double r_position = ds_right_val;
    double wr_error = (r_position - target)/10;
    double wr_derivative = wr_error - last_wr_error;
    last_wr_error = wr_error;
    
   //PID parametres for left motor 
    double last_wl_error = 0;
    double l_position = ds_left_val;
    double wl_error = (l_position - target)/10;
    double wl_derivative = wl_error - last_wl_error;
    last_wl_error = wl_error;
    
    if ((ds_right_val <500)&&(ds_left_val==1000)){

    leftMotor->setVelocity(3+0.015*wr_error+ 0.2*wr_derivative);
    rightMotor->setVelocity(3-0.015*wr_error-0.2*wr_derivative);
    }
    else if ((ds_right_val <500)&&(ds_left_val< 500)){

    leftMotor->setVelocity(3+0.02*wr_error+ 0.5*wr_derivative);
    rightMotor->setVelocity(3-0.02*wr_error-0.5*wr_derivative);
   
    }
    else if((ds_right_val ==1000)&&(ds_left_val<500)){   
 
    leftMotor->setVelocity(3-0.015*wl_error-0.2*wl_derivative);
    rightMotor->setVelocity(3+0.015*wl_error+0.2*wl_derivative);
    }
    else{
    leftMotor->setVelocity(3);
    rightMotor->setVelocity(3);
    }
}

void box_checking(){
  double i=0;
  while (robot->step(timeStep) != -1) {
    i++;
    //std::cout<<"CHECKING THE BOX\n";
    leftMotor->setVelocity(-1);
    rightMotor->setVelocity(1);
    
    ds_left_val = ds_left->getValue();
    ds_right_val = ds_right-> getValue();
  
    if (i==39){
    break;
    } 
} 
double p=0;
 while (robot->step(timeStep) != -1) {
p++;
  //std::cout<<"CHECKING THE BOX\n";
  ds_left_val = ds_left->getValue();
  ds_right_val = ds_right-> getValue();
  leftMotor->setVelocity(1);
  rightMotor->setVelocity(-1);
  
 //IF BOX IS DETECTED 
 if( 430<ds_left_val && ds_left_val<1000){
   std::cout<<"box detected\n";
   box++;
   forward(34,5,5);
   left_turn(45,5);
   break;
}
  if (p==39){
  break;
  }
}

}

void DashedLineFollowing(){
  //PID parametres for line following
  double error= get_error();
  double last_error = 0;
  double derivative = error - last_error;
  last_error = error;
  
  //PID parametres for line following
  double error2= get_error2();
  double last_error2 = 0;
  double derivative2 = error2 - last_error2;
  last_error2 = error2;
  if(ir0_val ==0 && ir1_val == 0 && ir2_val == 0 && ir3_val == 0 && ir4_val ==0  && ir5_val == 0){
      leftMotor->setVelocity(3-25*error2-45*derivative2);
      rightMotor->setVelocity(3+25*error2+45*derivative2);
  }
  else{
      leftMotor->setVelocity(3-25*error-45*derivative);
      rightMotor->setVelocity(3+25*error+45*derivative);
  }




}

int main(int argc, char **argv) {

  
  //Position sensors
  PositionSensor *ps1 = robot->getPositionSensor("psensor1");
  PositionSensor *ps2 = robot->getPositionSensor("psensor2");
  ps1->enable(Time_Step);
  ps2->enable(Time_Step);
  
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  //ir sensors
  DistanceSensor *ir0 = robot-> getDistanceSensor("ir0"); 
  DistanceSensor *ir1 = robot-> getDistanceSensor("ir1"); 
  DistanceSensor *ir2 = robot-> getDistanceSensor("ir2"); 
  DistanceSensor *ir3 = robot-> getDistanceSensor("ir3"); 
  DistanceSensor *ir4 = robot-> getDistanceSensor("ir4"); 
  DistanceSensor *ir5 = robot-> getDistanceSensor("ir5"); 
  
  DistanceSensor *ir6 = robot-> getDistanceSensor("ir6"); 
  DistanceSensor *ir7 = robot-> getDistanceSensor("ir7"); 
  DistanceSensor *ir8 = robot-> getDistanceSensor("ir8"); 
  DistanceSensor *ir9 = robot-> getDistanceSensor("ir9"); 
  
  DistanceSensor *ir_left = robot-> getDistanceSensor("ir_left"); 
  DistanceSensor *ir_right = robot-> getDistanceSensor("ir_right"); 
  InertialUnit *iu=robot-> getInertialUnit("iu");
  

  ir0->enable(Time_Step);
  ir1->enable(Time_Step);
  ir2->enable(Time_Step);
  ir3->enable(Time_Step);
  ir4->enable(Time_Step);
  ir5->enable(Time_Step);
  
  ir6->enable(Time_Step);
  ir7->enable(Time_Step);
  ir8->enable(Time_Step);
  ir9->enable(Time_Step);
  
  
  ir_left->enable(Time_Step);
  ir_right->enable(Time_Step);
  
  ds_left->enable(Time_Step);
  ds_right->enable(Time_Step);
  ds_front->enable(Time_Step);
  
  iu->enable(Time_Step);
  



  while (robot->step(timeStep) != -1) {
 
    ir0_val= ir0->getValue();
    ir1_val= ir1->getValue();
    ir2_val= ir2->getValue();
    ir3_val= ir3->getValue();
    ir4_val= ir4->getValue();
    ir5_val= ir5->getValue();
    
    ir6_val= ir0->getValue();
    ir7_val= ir1->getValue();
    ir8_val= ir2->getValue();
    ir9_val= ir3->getValue();
    
    ir_left_val= ir_left->getValue();
    ir_right_val= ir_right->getValue();
    ps1_val = ps1->getValue();
    ps2_val = ps2->getValue();
    
    ds_left_val = ds_left->getValue();
    ds_right_val = ds_right-> getValue();
    ds_front_val = ds_front-> getValue();
    const double* angle_val=iu->getRollPitchYaw();
    
    // Digitalization
     if(ir0_val>700){
      ir0_val = 0;
      }else{
      ir0_val = 1;
      }
      if(ir1_val>700){
      ir1_val = 0;
      }else{
      ir1_val = 1;
      }
      if(ir2_val>700){
      ir2_val = 0;
      }else{
      ir2_val = 1;
      }
      if(ir3_val>700){
      ir3_val = 0;
      }else{
      ir3_val = 1;
      }
      if(ir4_val>700){
      ir4_val = 0;
      }else{
      ir4_val = 1;
      }
      if(ir5_val>700){
      ir5_val = 0;
      }else{
      ir5_val = 1;
      }
      
      if(ir_left_val>700){
      ir_left_val = 0;
      }else{
     ir_left_val = 1;
      }
      if(ir_right_val>700){
      ir_right_val= 0;
      }else{
      ir_right_val = 1;
      }




if(gate>0){

if (gate>1){

LineFollowing();

}


else{
if (ir0_val ==1 && ir1_val == 1 && ir2_val == 1 && ir3_val == 1 && ir4_val ==1  && ir5_val == 1){
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    if (475 < ds_front_val and ds_front_val<570){
      forward(10,6,6);
       gate++ ;
 
}
    
   
}
    
else{
    LineFollowing();
}
}

  if (ir_left_val==0 and ir_right_val== 0 and (ir0_val ==0 && ir1_val == 0 && ir2_val == 0 && ir3_val == 0 && ir4_val ==0  && ir5_val == 0)){
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
}
}



else if (correction>0){

if (ir==1){
ir_val= ir_right_val;
}
else if(ir==2){
ir_val= ir_left_val;
}

if(ir_val ==1 and check==0){
forward(20,5,5);
check++;
} 
else if (ir_val==1 and check==1){
if (ir==1){
    forward(30,5,5);
    right_turn(45,5);
    gate++;
}
else if(ir==2){
    forward(30,5,5);
    left_turn(45,5);
    gate++;
}
}

LineFollowing();

}




else if (pillar>0){
 if (diff==2){
    ds_val= ds_left-> getValue();
 }
 else if (diff==1){
    ds_val= ds_right-> getValue();
 }
 
 if (50<ds_val && ds_val<150){
    pillar_count++;
    forward(13,5,5);
  }


    std::cout<<"Pillar count - "<<pillar_count<<"\n";

 

  LineFollowing();
  
  if ((diff==2) and (ir_left_val==1)){
  if(pillar_count==2){
  
     forward(30,5,5);
    left_turn(45,5);
    gate++;
    
    }
   else{
   
      std::cout<<"checking\n";
   right_turn(110,5);
   correction++;
   ir=1;  // To enable right IR in correction code
   
   
   
   }
  }
   else if ((diff==1) and (ir_right_val==1)){
   if(pillar_count==2){
     forward(30,5,5);
    right_turn(45,5);
     gate++;
  }
  else{
       std::cout<<"checking\n";
   left_turn(110,5);
   correction++;
   ir=2; //To enable left IR in correction code
    
  }
 }
 
 }


//Ramp      
else if(ramp>0){
 if (ir0_val == 1 &&ir1_val == 1 && ir2_val == 1 && ir3_val == 1 && ir4_val ==1  && ir5_val == 1){
  // EVEN DIFFERENCE
  if (diff==2 or diff==0){
    std::cout<<"EVEN DIFF left TURN\n";
    forward(30,5,5);
    left_turn(45,5);
   }
 
  //IF ODD DIFFERENCE
 else {
   std::cout<<"odd DIFF right TURN\n";
   forward(30,5,5); 
   right_turn(45,5);
 }
   pillar++;
 }

 else{
   LineFollowing();
 }  
}

//Dashed line following
else if (dot_line>0){
  std::cout<<"Dash line ENTERED\n"; 
  DashedLineFollowing();
  if (angle_val[0]>0.1){
   ramp++;
   std::cout<<"RAMP DETECTED\n";
 
 }
}




 /*
 
 
From start to circle entering
 maze==0
 
 */


else if (maze==0){
  if (right==2 and (ir_right_val== 1 || ir_left_val == 1)){
   maze++;
 std::cout<<"CIRCLE ENTERED\n"; 
 std::cout<<"2 right after\n";
 forward(19,5,5);
 right_turn(31,5);
  }

  //Left turn 
  else if( ir_right_val== 0 && ir_left_val == 1){
  
    forward(30,5,5);
    left_turn(45,5);
  }  
  
  //Right Turn
  else if(ir_left_val == 0 && ir_right_val == 1  ){
    right++;
     std::cout<<"right - "<<right<<"\n";
    forward(39,5,5);
    right_turn(50,5);  
  } 
 

 //Wall following
 else if(ir0_val ==0 && ir1_val == 0 && ir2_val == 0 && ir3_val == 0 && ir4_val ==0  && ir5_val == 0 && ir_left_val==0 && ir_right_val==0) {
    std::cout<<"Wall following\n"; 
    WallFollowing();
 }
 
 
  

// DETECTING THE CIRCLE ENTRANCE    
 
 // NORMAL LINE FOLLOWING BEFORE CIRCLE    
 else{
  LineFollowing();
 }
}
 

 
 /*
 
 
 CIRCLE MAZE SOLVING
 if maze==1: it will start circular maze solving
 
 
 */
 


else if (maze>0){
//BOX PICKED
if (pick>0){
  
  
  
if (level==3 or level==4){
if (pick==1){
if(ir_right_val==1 || ir_left_val==1 ){
  std::cout<<"right turn after picking\n";
  forward(27,5,5);
  right_turn(38,5);
  pick++;  
}
else{
LineFollowing();
}
}
else if (pick>1){
if ((ir_right_val==0 && ir_left_val==1) || (ir0_val ==1 && ir1_val == 1 && ir2_val == 1 && ir3_val == 1 && ir4_val ==1  && ir5_val == 1)){
 std::cout<<"Left turn exit way\n";
 forward(20,5,5);
 left_turn(25,5);
 dot_line++;
}
else{
LineFollowing();
}
}

}



else{
if (pick==1){
if(ir_right_val==1 || ir_left_val==1 ){
  forward(27,5,5);
  std::cout<<"left  turn after picking\n";
  left_turn(38,5);  
  pick++;
 
}
else{
LineFollowing();
}
}
else if (pick>1){
if ((ir_right_val==1 && ir_left_val==0)|| (ir0_val ==1 && ir1_val == 1 && ir2_val == 1 && ir3_val == 1 && ir4_val ==1  && ir5_val == 1)){
  std::cout<<"right turn exit way\n";
  forward(20,5,5);
  right_turn(25,5);
  dot_line++;
}

else if(ir_left_val== 1 && ir_right_val==0 ){
  std::cout<<"UNWANTED LEFT DETECTED\n"; 
  forward(21,3,4);
}



else{
  LineFollowing();
 
 }
 }
}
}


//BOX DETECTED
else if( box>0){

if(ds_front_val<90) {
   std::cout<<"Box picking\n";

// ARM CODE WILL COME


  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
  


  bool boxFound = true;
  bool cameraOn = false;
     Motor*mBase = robot->getMotor("base_motor");
   Motor*mTop = robot->getMotor("arm1_motor");
   Motor*mArm = robot->getMotor("arm2_motor");
   Motor*mHand1 = robot->getMotor("r_motor");
   Motor*mHand2 = robot->getMotor("l_motor");
   
   Camera *cm1;
   cm1 = robot-> getCamera("camera1");
   cm1->enable(Time_Step); 
   
   Camera *cm2;
   cm2 = robot-> getCamera("camera2");
   cm2->enable(Time_Step); 
   
   std::cout <<"start box picking"<< std::endl;
   //to find the difference to turn the robot
   int val1=0;
   int val2=0;
   int dif =0;
   
   int counter = 0;
    const double position_base =-2.2;
    const double position_top = 0.5;
    const double position_arm =0.9;
    //const double position_hand =0.01;
 
  if (boxFound) {
 
    while (robot->step(Time_Step) != -1){
 

    
    if (counter==0){
    mBase->setPosition(position_base);
    mHand1->setPosition(-0.3);
    mHand2->setPosition(0.3);
    mTop->setPosition(0);
    mArm->setPosition(1);
   counter = counter;
       std::cout << "picking" << std::endl;
   }
   
    else if (counter==135){
    mTop->setPosition(position_top);
    std::cout << "picking" << std::endl;
    }
 
    else if (counter==215){
    mArm->setPosition(position_arm);
    std::cout << "picking" << std::endl;
    }
    
    else if (counter==375){
    mHand1->setPosition(0.03);
    mHand2->setPosition(-0.03);
    
    std::cout << "picking" << std::endl;
    }
    else if (counter==415){
     mBase->setPosition(0.6);
     
    std::cout << "picking" << std::endl;
    }
    else if (counter==535){
     mTop->setPosition(-1.45);
     
    std::cout << "picking" << std::endl;
    }
    
    else if (counter==575){
     mArm->setPosition(0.0);
     cameraOn = true;
     
    std::cout << "picking" << std::endl;
    }if( cameraOn && counter == 735){
        //front colour detection
         const unsigned char *imF = cm1->getImage();
         const int imF_width = cm1->getWidth();
         const int imF_height = cm1->getHeight();
     
         std::cout<<imF_width<<"....."<<imF_height<<std::endl;
        
        int rtF = 0;
        int gtF = 0;
        int btF = 0;
        for (int x = 0; x < imF_width; x++)
        for (int y = 0; y < imF_height; y++) {
          int r = cm1->imageGetRed(imF, imF_width, x, y);
          int g = cm1->imageGetGreen(imF, imF_width, x, y);
          int b = cm1->imageGetBlue(imF, imF_width, x, y);
          
          rtF = rtF + r;
          gtF = gtF + g;
          btF = btF + b;
          }
       
        std::cout<<rtF<<"..."<<gtF<<"..."<<btF<<std::endl;
        
        if(rtF > gtF && rtF > btF){
          val1=1;
          std::cout << "front colour is RED" << std::endl;
       }else if(gtF > rtF && gtF > btF){
         val1=2;
        std::cout << "front colour is GREEN" << std::endl;
       }else if(btF > rtF && btF > gtF){
         val1=3;
        std::cout << "front colour is BLUE" << std::endl;
     }
     
     
         //bottom colour detection
         const unsigned char *imB = cm2->getImage();
         const int imB_width = cm2->getWidth();
         const int imB_height = cm2->getHeight();
     
         std::cout<<imB_width<<"....."<<imB_height<<std::endl;
        
        int rtB = 0;
        int gtB = 0;
        int btB = 0;
        for (int x = 0; x < imB_width; x++)
        for (int y = 0; y < imB_height; y++) {
          int r = cm2->imageGetRed(imB, imB_width, x, y);
          int g = cm2->imageGetGreen(imB, imB_width, x, y);
          int b = cm2->imageGetBlue(imB, imB_width, x, y);
          
          rtB = rtB + r;
          gtB = gtB + g;
          btB = btB + b;
          }
       btB=btB;
        std::cout<<rtB<<"..."<<gtB<<"..."<<btB<<std::endl;
        
        if(rtB > gtB && rtB > btB){
          val2=1;
          std::cout << "bottom colour is RED" << std::endl;
       }else if(gtB > rtB && gtB > btB){
         val2=2;
        std::cout << "bottom colour is GREEN" << std::endl;
       }else if(btB > rtB && btB > gtB){
         val2=3;
        std::cout << "bottom colour is BLUE" << std::endl;
     }
       dif = abs(val1-val2);
       diff=dif;
       std::cout << "difference is "<< dif << std::endl;
       
       if(dif%2==0){
       std::cout << "even difference turn left"<< dif << std::endl;
       }else{
       std::cout << "odd difference turn right"<< dif << std::endl;
       }
        break;
     }
    counter ++; 
                     
    }

  }


  pick++;
}

else{
  std::cout<<"Moving toward the box\n";
  LineFollowing();
}
}

// IF EXIT WAY DETECTED BEFORE BOX DETECTION
else if(ir_right_val== 1 || (ir0_val ==1 && ir1_val == 1 && ir2_val == 1 && ir3_val == 1 && ir4_val ==1  && ir5_val == 1) ){
  std::cout<<"EXIT WAY DETECTED WHILE CHECKING BOX\n"; 
  forward(30,4,1);
}

//BOX CHECKING BY CLOCKWISE AND ANTI CLOCKWISE TURNING
else if(ir_right_val== 0 && ir_left_val == 1  ){

 
//// displaying quadrant//////
  
  if (level%2 ==0){
    std::cout<<"Quadrant : "<<quadrant<<"\n";
    quadrant -= 1;
  }
  
/////////////////////////////
  level++; 
  
  
  std::cout<<"level :" << level<< "\n";
  box_checking();
  std::cout<<"MOVING AFTER CHECKING\n"; 
  forward(14,3,4);
}

//lINE FOLLOWING
else{
  LineFollowing(); 
 }

}
}

  delete robot;
  return 0;
}
