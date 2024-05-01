#include "math.h"
#include "Encoder.h"

#define N_MOTORES 5

//Estados
enum estado_general{reposo, recogida, deposito}; 
enum estado_motor{moviendo, parado, emergencia};

//Variables de control
bool marcha, paro;
bool marcha_anterior, paro_anterior;
bool griper_cerrado;

//Parámetros del robot
const double l1,l2,l3,l4;

//Motores
typedef struct motor{
  
  //Pins
  int pinA, pinB;

  //Variables de posicion
  double q_d;
  double q_r;
  double q_err;

  //Variables de seguridad
  double Pmax;

  estado_motor estado;
  
}motor;

motor motores[N_MOTORES];

//Encoders
Encoder encoder_M1(1,2);
Encoder encoder_M2(3,4);
Encoder encoder_M3(5,6);
Encoder encoder_M4(6,7);
Encoder encoder_M5(7,8);

Encoder* encoders[N_MOTORES];


//Vatímetros
typedef struct vatimetro{
  //Pins
  int pin;

  //Variables
  double P;
  
} vatimetro;

//Encoders

//Variables de posición y orientacion
//Si de los encoder nos llegan las qs, podemos realimentar directamente las q
double rango_error;

typedef struct punto{
  double x, y, z;
  double alfa, beta,gamma;
} punto;
punto almacen, posicion, objetivo;


//Funciones
bool evaluar_flanco(bool pulsador, bool* pulsador_anterior);

bool abrir_gripper();

void mover_robot(punto punto_intermedio);   //orientar antes de mover (activar q5)
void ciclo_trabajo(punto objetivo);
bool control_motor(motor& M, Encoder E);

double realimentacion(double q_d, double q_r, Encoder E);

//Principal
void setup() {
  int i; //Control de bucle

  //Asignacion de potencia maxima
  motores[0].Pmax = motores[1].Pmax = motores[2].Pmax = 0 * 0.6; //Parametros del motor
  motores[3].Pmax =  0 * 0.6; //Parametros del motor
  motores[4].Pmax = 0 * 0.6;

  //Introducimos por comodidad los encoders en un vector
  encoders[0] = &encoder_M1;
  encoders[1] = &encoder_M2;
  encoders[2] = &encoder_M3;
  encoders[3] = &encoder_M4;
  encoders[4] = &encoder_M5;

  //Motores
  for(i=0;i<N_MOTORES;i++){
    pinMode(motores[i].pinA,OUTPUT);
    pinMode(motores[i].pinB,OUTPUT);
  }
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
  bool fin[N_MOTORES];
  
  //Control bucles
  bool finT;
  int i; 
  //Calculamos las q necearias
  
  //Controlamos los motores
  do{
    finT = true;
    for(i=0;i<N_MOTORES;i++){
      fin[i] = control_motor(motores[i],*encoders[i]);

      //Bloqueo
      if(motores[i].estado = emergencia){
        while(true){
          //Pozo hasta que los problemas esten solucionados
          //W.P = analogRead(W.pin);
        }
      }
      
      finT = finT * fin[i];
    }
  }while( finT != true);
}

double realimentacion(double q_d, double q_r, Encoder E){
  //Encoder
  q_r = E.read();
  return q_d - q_r;
}

bool control_motor(motor& M, Encoder E){

  //Parada Emergenciza
  //W.P = analogRead(W.pin);
  /*
  if( W.P >= M.Pmax){
    digitalWrite(M.pinA, LOW);
    digitalWrite(M.pinB, LOW);
    M.estado = emergencia;
    return false;
  }
  */
  M.q_err = realimentacion(M.q_d, M.q_r,E);
  if(abs(M.q_err) < rango_error){
    //Apagar motor si no lo esta
    digitalWrite(M.pinA, LOW);
    digitalWrite(M.pinB, LOW);
    M.estado = parado;
    return true; //Acabado
  }
  else if(M.q_err > 0){
    //Motor Sentido horario
    digitalWrite(M.pinA, HIGH);
    digitalWrite(M.pinB, LOW);
  }
  else{
    //Motor sentido antihorario
    digitalWrite(M.pinA, LOW);
    digitalWrite(M.pinB, HIGH);
  }
  M.estado = moviendo;
  return false; //Sigue funcionando
}
