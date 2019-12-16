
#include <Servo.h>    // ajout librairie pour servomoteur
  Servo servo; // déclaration du servomoteur
  
#include <AccelStepper.h> //ajout librairie pour les accelerations des stepper

#include <MultiStepper.h> // ajout librairie pour controler plrs stepper en même temps
  AccelStepper stepperZ(AccelStepper::DRIVER, 6, 7);
  AccelStepper stepperY(AccelStepper::DRIVER, 4, 5);  //déclaration des 3 steppers
  AccelStepper stepperX(AccelStepper::DRIVER, 2, 3);

MultiStepper steppers;

long positions[3];  // tableau de postision pour les 3 stepper
long MaxSpeed[3];   // //tableau des 3 vitesse max

int joyZ;
int joyX;   //valeurs des 3 potentiomètres
int joyY;

int etatbp;
int Aetatbp=1;
int positionPince=0;  //position de la pince: fermé = 1  ouvert = 0

int L1=21;
int L2=17;             //longueur des axes du bras

void setup() {
  servo.attach(9);  // ajout du servomoteurau pin 9
  Serial.begin(9600); // demarrage du port serie

  stepperZ.setMaxSpeed(100*16);  
  stepperX.setMaxSpeed(100*16);  // vitesse max des stepper en step/seconde (les drivers sont en 1/16 des step)
  stepperY.setMaxSpeed(100*16);

  steppers.addStepper(stepperZ);
  steppers.addStepper(stepperX);  //ajout des steppers dans le librairie
  steppers.addStepper(stepperY);

pinMode(10,INPUT); // pin Bp
pinMode(A0,INPUT);
pinMode(A1,INPUT); //pin des potentiomètre
pinMode(A2,INPUT);

servo.write(110);  // ouverture de la pince

}

void loop() {
etatbp=digitalRead(10);

if ((etatbp != Aetatbp) && etatbp == LOW){   // si le pb a changé d'etat et qu'il est enfoncé
  if(positionPince == 1){                    // si la pince est fermée 
    servo.write(40);                         // ouvrir la pince
    positionPince = 0;
  }
  else {                                  
    servo.write(110);                        //sinon la fermer
    positionPince = 1;
  }
    
}
Aetatbp = etatbp;




joyZ=analogRead(A2);
joyX=analogRead(A1);   //mesure des valeurs des potentiomètres
joyY=analogRead(A0);

  
Serial.print(joyZ);
Serial.print("      ");
Serial.print(joyX);           // affichage des valeurs dans le moniteur serie
Serial.print("      ");
Serial.print(joyY);
Serial.print("    ");
Serial.println(Aetatbp);

if (joyZ<=600 and joyZ>=400){positions[0]=stepperZ.currentPosition();}

if (joyZ<400){
  stepperZ.setMaxSpeed(map(joyZ,400,0,20*16,100*16));
  positions[0]=stepperZ.currentPosition()+50; }                          // commande du stepper Z

if ( joyZ>600){
  stepperZ.setMaxSpeed(map(joyZ,600,1000,20*16,100*16));
  positions[0]=stepperZ.currentPosition()-50;
   }



if (joyX<=600 and joyX>=400){positions[1]=stepperX.currentPosition();}

if (joyX<400){
  stepperX.setMaxSpeed(map(joyX,400,0,20*16,100*16));
  positions[1]=stepperX.currentPosition()-50; }                          // commande du stepper X

if ( joyX>600){
  stepperX.setMaxSpeed(map(joyX,600,1000,20*16,100*16));
  positions[1]=stepperX.currentPosition()+50;
   }



if (joyY<=600 and joyY>=400){positions[2]=stepperY.currentPosition();}

if (joyY<400){
  stepperY.setMaxSpeed(map(joyY,400,0,20*16,100*16));
  positions[2]=stepperY.currentPosition()+50; }                          // commande du stepper Y

if ( joyY>600){
  stepperY.setMaxSpeed(map(joyY,600,1000,20*16,100*16));
  positions[2]=stepperY.currentPosition()-50;
   }
   
steppers.moveTo(positions);    //envoi des commande au stepper
steppers.runSpeedToPosition(); // bloquer les steppers 



}

/*void convertirAngleRotation (float alpha,float beta){
  float[] rotation=new float[]{rotationAlpha,rotationBeta}
  return rotation;
}
void arriverHauteur (float h){
  float alpha = asin(h/(l1+l2)); //on détermine les angles alpha et beta pour que le bras se possitionne à une hauteur h
  float beta = alpha;
  float[] rotation=new float[]{float rotationAlpha,float rotationBeta}
  rotation=convertirAngleRotation (alpha,beta);
  return (rotationAlpha,rotationBeta);
  }
  
void arriverLongueur(float h,float x){
  
  }*/
