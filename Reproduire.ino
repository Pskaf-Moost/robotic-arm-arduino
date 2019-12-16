/* Presser joystick gauche pour sauvegarder une position
 * Un appui long pour déclancher la séquence
 * Pour enregistrer une nouvelle séquence il faut reboot l'arduino
 * Pensez à désactiver les moteurs lors de la saisie des positions grâce au switch sur le coté du boitier
 */






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

int etatBp;
int AetatBp;
int positionPince=0;  //position de la pince: fermé = 1  ouvert = 0
int a =0;

int pX;
int pY;             // Valeurs des potentiometres
int pZ;

int pX0;
int pY0;           // valeurs initiales des potentiometres
int pZ0;

long debutCpt;
int bloker = 0;
int origine[2];

long touteCoordo[40][3];    // liste des triplets de coordonnées
int compteurPos=1;          // nombres de position

void setup() {
  servo.attach(9);  // ajout du servomoteurau pin 9
  Serial.begin(9600); // demarrage du port serie

  stepperZ.setMaxSpeed(50*16);  
  stepperX.setMaxSpeed(80*16);  // vitesse max des stepper en step/seconde (les drivers sont en 1/16 des step)
  stepperY.setMaxSpeed(80*16);

  steppers.addStepper(stepperZ);
  steppers.addStepper(stepperX);  //ajout des steppers dans le librairie
  steppers.addStepper(stepperY);

pinMode(10,INPUT); // pin Bp
pinMode(A3,INPUT);
pinMode(A4,INPUT); //pin des potentiomètre
pinMode(A5,INPUT);

servo.write(110);  // ouverture de la pince

}

void loop() {

stepperZ.setCurrentPosition(0);
stepperX.setCurrentPosition(0);         
stepperY.setCurrentPosition(0);



etatBp=digitalRead(10);
pX=analogRead(A4);
pY=analogRead(A5);     //valeurs initiales des pot
pZ=analogRead(A3);
if(a==0){
  pZ0=pZ;
  pX0=pX;   // stockage des valeurs initiales des pot
  pY0=pY;
  a=1;
}

if ((etatBp != AetatBp) && etatBp == LOW){   // si le bp a changé d'etat et qu'il est enfoncé
    touteCoordo[compteurPos][0]=(pZ-pZ0)*64;
    touteCoordo[compteurPos][1]=(pX-pX0)*64;   // ajout d'une postion 
    touteCoordo[compteurPos][2]=(pY-pY0)*64;
    compteurPos+=1;
    bloker = 0; 
    }
    
debutCpt=millis();

while ((etatBp != AetatBp) && etatBp == LOW && bloker == 0){
etatBp=digitalRead(10);

  if ((millis() - debutCpt) >= 2000){   //si deux seconde se sont ecoulées
    
  bloker = 1;
      touteCoordo[0][0]=-touteCoordo[compteurPos-1][0];
      touteCoordo[0][1]=-touteCoordo[compteurPos-1][1];  //remise a la position de départ
      touteCoordo[0][2]=-touteCoordo[compteurPos-1][2];
      
   for (int i =0; i <compteurPos;i++){
    for(int n =0;n<=2;n++){
    Serial.println(touteCoordo[i][n]);
   }}

      touteCoordo[compteurPos][0]=0;
      touteCoordo[compteurPos][1]=0;
      touteCoordo[compteurPos][2]=0;       // suppression de la position de trop, lors de l'appui prolongé
      compteurPos = compteurPos-1;

 
    for (int i =0; i< compteurPos;i++){

      if (i==1){ 
        stepperZ.setCurrentPosition(0);
        stepperX.setCurrentPosition(0);    //mise à l'origine pour les stepper, afin que les steppers aient la meme origine que les potentiomètres
        stepperY.setCurrentPosition(0);
      }
      positions[0]=touteCoordo[i][0];
      positions[1]=touteCoordo[i][1];
      positions[2]=touteCoordo[i][2];
      
      steppers.moveTo(positions);    //envoi des commande au stepper+
      steppers.runSpeedToPosition(); // bloquer les steppers 

      
      delay(1000);  

      
      }
  }
  
}
AetatBp=etatBp; 


Serial.print(pZ);
Serial.print("      ");
Serial.print(pX);           // affichage des valeurs dans le moniteur serie
Serial.print("      ");
Serial.print(pY);
Serial.print("    ");
Serial.println(compteurPos);


}


  
