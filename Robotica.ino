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
  //decalracion orientaciones
  const punto a={0,0,-1};
  const punto o={1,0,0};
  const punto n={0,1,0};
  
  //declaracion de angulos
  double q1,q2,q3,q4,q5;
  //declaracion de variables intermediass
  double A,B,c5,s5,q234,p,c2,s2;
  
  q1=atan2(punto_intermedio.y,punto_intermedio.x);
  
  c5=sin(q1)*o.x-cos(q1)*o.y;      //que es n y o??
  s5=sin(q1)*n.x-cos(q1)*n.y;
  q5=atan2(s5,c5);
  if(cos(q1)==0){
    q234=atan2(a.z,a.y/sin(q1));
    p=punto_intermedio.y/sin(q1);
  }
  else{
    q234=atan2(a.z,a.x/cos(q1));
    p=punto_intermedio.x/cos(q1); 
  }
  A=p-l4*cos(q234)-l5*cos(q234);
  B=punto_intermedio.z-l1-l4*sin(q234)-l5*sin(q234);
  q3=acos((A*A+B*B-l2*l2-l3*l3)/(2*l2*l3));

  c2=(A*(l2+l3*cos(q3))+B*sin(q3)*l3)/(pow((l2+l3*cos(q3)),2)+pow((sin(q3)*l3),2));
  s2=(B*(l2+l3*cos(q3))-A*l3*sin(q3))/(pow(l2+l3*cos(q3),2)+pow(sin(q3)*l3,2));
  q2=atan2(s2,c2);

  q4=q234-q3-q2;
  
}
  

