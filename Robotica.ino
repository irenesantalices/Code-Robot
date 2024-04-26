#include "math.h"

//Variables de control
bool marcha, paro;
bool marcha_anterior, paro_anterior;
bool griper_cerrado;
//Parámetros del robot
const double l1,l2,l3,l4;

//Variables de posición y orientacion
double q1, q2, q3, q4, q5;
typedef struct punto{
  double x, y, z;
  double alfa, beta,gamma;
} punto;
punto almacen, posicion, objetivo;

double rango_error;

enum estados{reposo, recogida, deposito};

//Funciones
bool evaluar_flanco(bool pulsador, bool* pulsador_anterior);
bool abrir_gripper();
void mover_robot(punto punto_intermedio);   //orientar antes de mover (activar q5)
void ciclo_trabajo(punto objetivo);
//Principal
void setup() {
  
  
  
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



  
}
