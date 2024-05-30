// Import des librairies
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <NewPing.h>

// Fonction de redémarrage
void(* resetFunc) (void) = 0;

// Déclaration des variables

// menu pour la valeur du menu
// menuDisp pour la vérification d'affichage menu
// wheel pour la valeur du potentiomètre de la roue de menu
// lap détection des tours réalisés par l'arbre
// rpm calcul du nombre de tours par minute de l'arbre
// tachyTime pour l'intervalle de mesure
int menu, menuDisp, wheel, lap = 0, rpm, tachyTime = 2000;

// startTime et stopTime pour la différence de temps des séquences moteur
// time1 pour la différence de temps du tachymètre
// value pour la valeur du capteur à ultrason
// led_motor_time pour faire clignoter la lampe du panneau synoptique relative au fonctionnement du moteur
double startTime, stopTime, time1, value, led_motor_time, UStime;

// state pour le capteur IR
// select_state et motor_butt_state pour éviter répétition
// cmdMotor commande moteur on/off
// operating si oui ou non une séquence moteur est lancée
bool state, select_state, motor_butt_state, cmdMotor, operating;



// Déclaration des bornes

// IR borne de la réception des données du capteur infrarouge
// motor_butt borne de réception du signal de lancement de la séquence moteur
// UStrig borne de trigger du capteur ultrason
// USecho borne de réception du signal du capteur ultrason
// fin_course borne de réception du signal du capteur fin de course pour la sécurité cylindre
// motor1 bornes d'alimentation des relais moteur
// menu_wheel attribution du port du potentiomètre de la roue de menu
// menu_select pour capter la position du bouton "sélectionner" du menu
// servo1 et servo2 borne d'envoi du signal de commande aux servos pour la protection cylindre
// led_motor pour la led de signalisation du moteur allumé sur le panneau synoptique
const int UStrig = 2, USecho = 3, fin_course = 4, menu_select = 5, menu_wheel = 6, motor_butt = 7, servo1 = 8, servo2 = 9, motor1 = 10, IR = A3;

// Librairie pour contrôler l'écran LCD, 0x27 pour écran 16 caractères 2 lignes
LiquidCrystal_I2C lcd(0x27,16,2); // Pour les bornes A5, A4
// Librairie pour utiliser capteur à Ultrasons. Borne UStrig pour envoyer signal, borne USecho pour réception du signal
NewPing sonar(UStrig,USecho,200);
// Librairie pour contrôler les servomoteurs de la protection cylindre
Servo cyl_protect1;
Servo cyl_protect2;

void setup() {
  Serial.begin(9600); // Canal de communication 9600 baud pour debug console
  // Initialisation de l'écran LCD
  lcd.begin();
  lcd.display();
  // Rétro éclairage
  lcd.setBacklight(1);
  // Effacer le contenu de l'écran
  lcd.clear();
  // Affichage écran de lancement
  lcd.setCursor(3,0);
  lcd.print("Protect'eau");
  lcd.setCursor(3,1);
  lcd.print("Starting");
  
  // Déclaration de la fonction des pins
  pinMode(menu_wheel, INPUT);
  pinMode(IR, INPUT);
  pinMode(fin_course, INPUT);
  pinMode(motor1, OUTPUT);
  pinMode(menu_select, INPUT);
  pinMode(motor_butt,INPUT);
  pinMode(led_motor, OUTPUT);

  // Définir le port du servomoteur
  cyl_protect1.attach(servo1);
  cyl_protect2.attach(servo2);

  // Délai de lancement et effaçage écran LCD
  delay(2000);
  lcd.clear();
  Serial.println("Start");
}

void loop() {
  // Lancement des séquences de démarrage / arrêt moteur en fonctionnement nominal
  if(cylindre() && ultrasonic() && digitalRead(motor_butt) == HIGH && motor_butt_state) {
    if(!cmdMotor) {
      cmdMotor = true;
      startTime = millis();
      Serial.println("Start Motor Action");
    } else if(cmdMotor) {
      cmdMotor = false;
      stopTime = millis();
      Serial.println("Stop Motor Action");
    }
    motor_butt_state = false;
  }

  // Fonction pour éviter la répétition du bouton moteur
  if(digitalRead(motor_butt) == LOW && !motor_butt_state) {
    motor_butt_state = true;
  }
  // Lancer la séquence d'arrêt si les conditions nominales ne sont pas remplies
  if(!cylindre() && cmdMotor) {
    stopTime = millis();
    cmdMotor = false;
    Serial.println("Abort Motor 1");
  } else if(!ultrasonic() && cmdMotor) {
    stopTime = millis();
    cmdMotor = false;
    Serial.println("Abort Motor 2");
  }
  
  // Conditions pour exécuter les fonctions de démarrage / arrêt moteur
  if(cmdMotor && millis() - startTime <2100 && millis() - startTime >=0) {
    startMotor();
  } else if(!cmdMotor && millis() - stopTime <2100 && millis() - stopTime >=0) {
    stopMotor();
  }

  // Vérification de l'ancienne valeur de menu pour savoir si actualiser écran LCD ou pas
  menuDisp = menu;
  // Récupération valeur potentiomètre
  wheel = digitalRead(menu_wheel) +1;

  // Si menu == 0 alors afficher le menu monitoring et si pas prendre valeur potentiomètre
  if(menu == 0) {
    monitoring();
  } else {
    menu = wheel;
  }

  // Actualisation affichage LCD dans le menu
  if(menu !=0 && menu != menuDisp) {
    displaymenu();
  }

  // Exécution des fonctions relatives au menu + fonction pour éviter la répétition du bouton
  if(digitalRead(menu_select) == HIGH && select_state) {
    if(menu == 0) {
      menu = wheel;
      displaymenu();
    } else if(menu == 1) {
      lcd.clear();
      menu = 0;
    } else if(menu == 2) {
      
      resetFunc();
    }
    select_state = false;
  } else if(digitalRead(menu_select) == LOW && !select_state) {
    select_state = true;
  }

  // Si inactivité de 5 minutes, redémarrer l'arduino pour éviter les problèmes de taille pour le stockage du millis()
  // et entre autre éviter d'autres problèmes / bugs
  if(millis() > 300000 && !cmdMotor && !operating) {
    resetFunc();
  }

  // Fonction pour faire clignoter la lampe du panneau synoptique relative au fonctionnement du moteur
  if(cmdMotor && !operating) {
    if(millis() - led_motor_time < 500) {
      digitalWrite(led_motor, HIGH);
    } else if(millis() - led_motor_time > 1000) {
      digitalWrite(led_motor, LOW);
    } else {
      led_motor_time = millis();
    }
  } else {
    digitalWrite(led_motor, LOW);
  }
}

void startMotor() {
  if(millis() - startTime < 2000) { // Séquence de fermeture des servos
    cyl_protect1.write(180);
    cyl_protect2.write(180);
    operating = true;
    Serial.println("Close Servos Action");
    lcd.setCursor(3,1);
    lcd.print("Lancement");
  } else if(millis() - startTime >= 2000 && millis() - startTime < 2100) { // Alimentation des relais moteur
    Serial.println("Motor ON");
    lcd.setCursor(3,1);
    lcd.print("Moteur ON !");
    operating = false;
    digitalWrite(motor1, HIGH);
    digitalWrite(motor2, HIGH);
  }
}

void stopMotor() {
  if(millis() - stopTime < 2000) { // Séquence de retrait de l'alimentation des relais moteur
    operating = true;
    Serial.println("Motor OFF");
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, LOW);
  } else if(millis() - stopTime >= 2000 && millis() - stopTime < 2100) { // Ouverture des servos
    Serial.println("Open Servos Action");
    cyl_protect1.write(0);
    cyl_protect2.write(0);
    operating = false;
  }
}
void displaymenu() { // Actualisation de l'affichage LCD
  lcd.clear();
  if(menu == 1) {
    lcd.setCursor(0, 0);
    lcd.print("1. Monitoring");
  } else if(menu == 2) {
    lcd.setCursor(0, 0);
    lcd.print("2. Reboot");
  }
}

void monitoring() { // Affichage RPM et erreurs
  lcd.setCursor(3,0);
  lcd.print("RPM: ");
  lcd.setCursor(8,0);
  lcd.print(tachy());
  lcd.print("   ");

  if(!cylindre()) {
    lcd.setCursor(3,1);
    lcd.print("Cylindre !    ");
  } else if(!ultrasonic()) {
    lcd.setCursor(3,1);
    lcd.print("Bac !        ");
  } else if(!cmdMotor) {
    lcd.setCursor(3,1);
    lcd.print("Pret !        ");
  }
}

int tachy() { // Fonction tachymètre avec capteur infrarouge
  // lecture à moins de 800 bits (calibration) et state = false pour compter un tour
  if(analogRead(IR)<800 && !state) {
    lap++;
    // Protection anti incrémentation en boucle quand le capteur détecte l'arbre
    state = true;
  } else if (analogRead(IR)>=800 && state) { // Si capteur ne détecte plus l'arbre et que state = true, alors retirer sécurité
    state = false;
  }
  
  // Différence de temps avec time1 pour compter jusqu'à tachyTime sans utiliser de délai
  if(millis() - time1 >= tachyTime) {
    // rpm = tours/(TempsDeMesure[s])*1minute
    rpm = lap/((millis()-time1)/1000)*60;
    // Nouveau départ pour time1
    time1 = millis();
    // Mise à zéro des tours
    lap = 0;
  }
  return rpm;
}

bool ultrasonic() { // Fonction de détection du bac de récupération avec un capteur ultrason
  value = sonar.ping_cm();
  if(value <= 10 && millis() - UStime >= 1000) {
    UStime = millis();
    return true;
  } else if(value > 10 && millis() - UStime >= 1000){
    return false;
  }
}

bool cylindre() { // Fonction de détection de présence du cylindre
  if(digitalRead(fin_course) == HIGH) {
    return true;    
  } else {
    return false;    
  }
}