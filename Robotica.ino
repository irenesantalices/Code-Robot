#include "math.h"

//Variables de control
bool marcha, paro;
bool marcha_anterior, paro_anterior;
bool griper_cerrado;

//Parámetros del robot
const double l1,l2,l3,l4;

//Asignacion de Pines
//Motores
int pinA_M1, pinB_M1;
int pinA_M2, pinB_M2;
int pinA_M3, pinB_M3;
int pinA_M4, pinB_M4;
int pinA_M5, pinB_M5;

//Encoder


//Variables de posición y orientacion
//Si de los encoder nos llegan las qs, podemos realimentar directamente las q
double q1_d, q2_d, q3_d, q4_d, q5_d;
double q1_r, q2_r, q3_r, q4_r, q5_r;
double err_q1, err_q2, err_q3, err_q4, err_q5;

double rango_error;

typedef struct punto{
  double x, y, z;
  double alfa, beta,gamma;
} punto;
punto almacen, posicion, objetivo;

//Estados
enum estado_general{reposo, recogida, deposito}; 
enum estado_motor{parado, moviendo, emergencia};

//Funciones
bool evaluar_flanco(bool pulsador, bool* pulsador_anterior);

bool abrir_gripper();

void mover_robot(punto punto_intermedio);   //orientar antes de mover (activar q5)
void ciclo_trabajo(punto objetivo);
bool control_motor(double q_d, double q_r, double q_err, int pinA, int pinB);

double realimentacion(double q_d, double q_r);

//Principal
void setup() {
  //Motores
  pinMode(pinA_M1, OUTPUT);
  pinMode(pinB_M1, OUTPUT);
  
  pinMode(pinA_M2, OUTPUT);
  pinMode(pinB_M2, OUTPUT);
  
  pinMode(pinA_M3, OUTPUT);
  pinMode(pinB_M3, OUTPUT);
  
  pinMode(pinA_M4, OUTPUT);
  pinMode(pinB_M4, OUTPUT);
  
  pinMode(pinA_M5, OUTPUT);
  pinMode(pinB_M5, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //ECUACION 
}

bool evaluar_FP(bool pulsador, bool* pulsador_anterior){
  bool flanco;

  flanco = (pulsador == true && *pulsador_anterior!=pulsador);
  *pulsador_anterior = pulsador;
  
  return flanco;
}

void ciclo_trabajo(punto objetivo){
  //definicion variables locales
  punto punto_intermedio1,punto_intermedio2;
  const double distancia_seguridad;
  //programa
  punto_intermedio1.x=posicion.x;
  punto_intermedio1.y=posicion.y;
  punto_intermedio1.z=posicion.z + distancia_seguridad;
  mover_robot(punto_intermedio1);

  
  punto_intermedio2.x=objetivo.x;
  punto_intermedio2.y=objetivo.y;
  punto_intermedio2.z=posicion.z;
  mover_robot(punto_intermedio2);
  
  mover_robot(objetivo);

  //funcion abrir gripper

  //Regreso
  mover_robot(punto_intermedio2);
  mover_robot(punto_intermedio1);
  mover_robot(almacen); //posibilidad de dejarlo fuera

  //Cerrar gripper
  
}

void mover_robot(punto punto_intermedio){
  bool fin1, fin2, fin3, fin4, fin5;

  //Calculamos las q necearias
  
  //Controlamos los motores
  do{
    fin1 = control_motor(q1_d, q1_r, err_q1, pinA_M1, pinB_M1);
    fin2 = control_motor(q2_d, q2_r, err_q2, pinA_M2, pinB_M2);
    fin3 = control_motor(q3_d, q3_r, err_q3, pinA_M3, pinB_M3);
    fin4 = control_motor(q4_d, q4_r, err_q4, pinA_M4, pinB_M4);
    fin5 = control_motor(q5_d, q5_r, err_q5, pinA_M5, pinB_M5);
  }while( fin1*fin2*fin3*fin4*fin5 != true);
}

double realimentacion(double q_d, double q_r){
  return q_d - q_r;
}

bool control_motor(double q_d, double q_r, double q_err, int pinA, int pinB){
  q_err = realimentacion(q_d, q_r);
  if(abs(q_err) < rango_error){
    //Apagar motor si no lo esta
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    return true; //Acabado
  }
  else if(q_err > 0){
    //Motor Sentido horario
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  }
  else{
    //Motor sentido antihorario
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  }
  return false; //Sigue funcionando
}
