#include "math.h"
#include "Encoder.h"
#include "DFRobot_INA219.h"
#include "Wire.h"

#define N_MOTORES 5



//Estados
enum bloque{normal, largo};
bloque pieza = normal;

enum estado_motor{parado, moviendo, emergencia};
enum estado_general{ok, bloqueo};

//Pin pulsador inicio
const uint8_t pinPulsador = 34;


//Variables de control
bool pulsador = false;
bool pulsador_anterior = false;


//Parámetros del robot
const double l1 = 0.275, l2 = 0.20 , l3 = 0.19 , l4 = 0.09, l5 = 0.075;

//Motores
typedef struct motor{

  //Relacion de transformacion del motor/Reductora y Relacion PWM/Velocidad
  double relacion_reductora;
  double relacion_PWM;

  //Pins
  uint8_t pinA, pinB;

  //Variables de posicion
  double q_d;
  double q_r;
  double q_err;

  //Variables de seguridad
  double Imax;

  estado_motor estado;
  
}motor;

motor motores[N_MOTORES];

//Encoders
Encoder encoder_M1(24,25);
Encoder encoder_M2(26,27);
Encoder encoder_M3(28,29);
Encoder encoder_M4(30,31);
Encoder encoder_M5(32,33);

Encoder* encoders[N_MOTORES];


//Amperimetro
typedef struct amperimetro{
  //Pins
  uint8_t pin;

  //Variables
  double A_D;
  double Vo;
  double I;
  
} amperimetro;
amperimetro amperimetros[N_MOTORES];

//Gripper
typedef struct gripper{
  int pinA, pinB, pinW1 = 19, pinW2 = 18; //Pines del vatimetro no son necesarios, se dejan como documentacion

}gripper;
gripper g{0,1};

//Amperimetro/Wattimetro del gripper
DFRobot_INA219_IIC     ina219(&Wire, INA219_I2C_ADDRESS4);

//Calibracion Amperimetro
float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;

//Si de los encoder nos llegan las qs, podemos realimentar directamente las q
double rango_error = 0.1;

typedef struct punto{
  double x, y, z;
  //Orientacion cte.
} punto;
punto almacen, posicion, objetivo;


//Funciones
void inicializar_motor();
void inicializar_almacen();
void inicializar_objetivo();
void inicializar_amperimetro();
void inicializar_watt();

void definir_nuevo_objetivo();

bool evaluar_FP();

void abrir_gripper();
void cerrar_gripper();

estado_general mover_robot(punto punto_intermedio);   //orientar antes de mover (activar q5)
void ciclo_trabajo();
bool control_motor(motor& M, Encoder E, amperimetro& A);

double realimentacion(double q_d, double q_r, Encoder E);

int grados_a_PWM(double angulo, double relacion_reductora, double relacion_PWM);
void leer_I(amperimetro& A);


//Principal
void setup() {

 //Pulsador 
  pinMode(pinPulsador,INPUT);

  //Motor
  inicializar_motor();
  
  //Introducimos por comodidad los encoders en un vector
  encoders[0] = &encoder_M1;
  encoders[1] = &encoder_M2;
  encoders[2] = &encoder_M3;
  encoders[3] = &encoder_M4;
  encoders[4] = &encoder_M5;

  inicializar_amperimetro();
  inicializar_watt();

  inicializar_almacen();
  inicializar_objetivo();

}

void loop() {
  // put your main code here, to run repeatedly:
  if(evaluar_FP()==true){
    ciclo_trabajo();
  }
}

void inicializar_motor(){
  int i; //Control de bucle

  //Potencia
  motores[0].Imax = motores[4].Imax = 2.8 * 0.6;
  motores[1].Imax = motores[2].Imax = motores[3].Imax = 3.0 * 0.6;



  //Pines
  //Asignacion
  for(i=0; i<N_MOTORES; i++){
    motores[4-i].pinA=2+2*i;
    motores[4-i].pinB=3+2*i;
  }

  //Inicializacion
  for(i=0;i<N_MOTORES;i++){
    pinMode(motores[i].pinA,OUTPUT);
    pinMode(motores[i].pinB,OUTPUT);
  }
}

void inicializar_amperimetro(){
  int i; //Control de bucle

  //Pines
  for(i=0;i<N_MOTORES-1;i++){
    amperimetros[i].pin = 23 - i; //Cambiar al conectar
  }
  amperimetros[i].pin = 17;

  for(i=0;i<N_MOTORES;i++){
    
  }
}

void inicializar_almacen(){
  almacen.x = 0;
  almacen.y = 0.26;
  almacen.z = 0.03;
}

void inicializar_objetivo(){
  objetivo.x = 0;
  objetivo.y = 0;
  objetivo.z = 0;
}

void inicializar_watt(){
  while(ina219.begin() != true) {
    //ERROR
  }

  ina219.linearCalibrate(ina219Reading_mA, extMeterReading_mA);
}

void definir_nuevo_objetivo(){
  static int contador = 0;
  
  if(objetivo.x == objetivo.y == objetivo.z == 0){
    //CAMBIAR
    objetivo.x = 0.275;
    objetivo.y = -0.085;
    objetivo.z = 0.045;
    
    contador++;
    return;
  }

  switch(pieza){
    case largo:
      objetivo.y += 0.042*3;

      contador++;
      if(contador>=2){
        pieza = normal;
        contador = 0;
      }
      break;
    case normal:
      if(contador == 0){
        objetivo.z += 0.04;
        objetivo.y += -0.085-0.042/2;
      }
      else
        objetivo.y += 0.042*2;
      contador++;
      if(contador>=3){
        pieza = largo;
        contador = 0;
        inicializar_objetivo(); //Reset para construir la siguiente pared
      }
    break;
  }
}

bool evaluar_FP(){
  bool flanco;

  pulsador = digitalRead(pinPulsador);
  flanco = (pulsador && pulsador_anterior!=pulsador);
  pulsador_anterior = pulsador;
  
  return flanco;
}

void abrir_gripper(){
  while(abs(ina219.getCurrent_mA()) > 0.2){
    analogWrite(g.pinA, (255/2));
    digitalWrite(g.pinB, LOW);
  }
  //Motor apagado
  analogWrite(g.pinA, 0);
  analogWrite(g.pinB, 0);
  
}

void cerrar_gripper(){
  while(abs(ina219.getCurrent_mA()) > 0.2){
    analogWrite(g.pinA, 255/2);
    digitalWrite(g.pinB, LOW);
  }
  //Motor sentido antihorario
  digitalWrite(g.pinA, 0);
  analogWrite(g.pinB, 0);
  
}

void ciclo_trabajo(){
  //definicion variables locales
  punto punto_intermedio1,punto_intermedio2;
  const double distancia_seguridad = 0.05;
  //programa
  punto_intermedio1.x=almacen.x;
  punto_intermedio1.y=almacen.y;
  punto_intermedio1.z=almacen.z + distancia_seguridad;
  if(mover_robot(punto_intermedio1) == bloqueo){
    return;
  }

  if(mover_robot(almacen) == bloqueo){
    return;
  } //posibilidad de dejarlo fuera

  cerrar_gripper();
  
  definir_nuevo_objetivo();

  punto_intermedio2.x=objetivo.x;
  punto_intermedio2.y=objetivo.y;
  punto_intermedio2.z=objetivo.z;
  if(mover_robot(punto_intermedio2) == bloqueo){
    digitalWrite(34,HIGH);
    return;
  }
  
  if(mover_robot(objetivo) == bloqueo){
    return;
  }

  abrir_gripper();

  //Regreso
  if(mover_robot(punto_intermedio2) == bloqueo){
    return;
  }
  
}

estado_general mover_robot(punto punto_intermedio){
  bool fin[N_MOTORES];
  
  //Control bucles
  bool finT;
  int i, j; //control bucles 
  //Calculamos las q necearias
   //decalracion orientaciones
  const punto a={0,0,-1};
  const punto o={1,0,0};
  const punto n={0,1,0};
  
  //declaracion de variables intermediass
  double A,B,c5,s5,q234,p,c2,s2;
  
  motores[0].q_d =atan2(punto_intermedio.y,punto_intermedio.x);
  
  c5 = sin(motores[0].q_d)*o.x-cos(motores[0].q_d)*o.y;      //que es n y o??
  s5 = sin(motores[0].q_d)*n.x-cos(motores[0].q_d)*n.y;
  motores[4].q_d = atan2(s5,c5);
  if(cos(motores[0].q_d)==0){
    q234 = atan2(a.z,a.y/sin(motores[0].q_d));
    p = punto_intermedio.y/sin(motores[0].q_d);
  }
  else{
    q234 = atan2(a.z,a.x/cos(motores[0].q_d));
    p = punto_intermedio.x/cos(motores[0].q_d); 
  }
  A = p-l4*cos(q234)-l5*cos(q234);
  B = punto_intermedio.z-l1-l4*sin(q234)-l5*sin(q234);
  motores[2].q_d = acos((A*A+B*B-l2*l2-l3*l3)/(2*l2*l3));

  c2 = (A*(l2+l3*cos(motores[2].q_d))+B*sin(motores[2].q_d)*l3)/(pow((l2+l3*cos(motores[2].q_d)),2)+pow((sin(motores[2].q_d)*l3),2));
  s2 = (B*(l2+l3*cos(motores[2].q_d))-A*l3*sin(motores[2].q_d))/(pow(l2+l3*cos(motores[2].q_d),2)+pow(sin(motores[2].q_d)*l3,2));
  motores[1].q_d = atan2(s2,c2);

  motores[3].q_d = q234-motores[2].q_d-motores[1].q_d;


  //Controlamos los motores
  do{
    finT = true;
    for(i=0;i<N_MOTORES;i++){
      fin[i] = control_motor(motores[i], *encoders[i], amperimetros[i]);

      //Bloqueo
      if(motores[i].estado == emergencia){
        for(j=0;j<N_MOTORES;j++){
          digitalWrite(motores[j].pinA, LOW);
          digitalWrite(motores[j].pinB, LOW);
          return bloqueo;
        }
      }
      
      finT = finT && fin[i];
    }
  }while( finT != true);
  return;
}

double realimentacion(double q_d, double q_r, Encoder E){
  //Encoder
  q_r = E.read();
  return q_d - q_r;
}

bool control_motor(motor& M, Encoder E, amperimetro& A){

  leer_I(A);
  if(A.I > abs(M.Imax)){
    M.estado = emergencia;
    return false;
  }

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
    analogWrite(M.pinA, (255/2));
    digitalWrite(M.pinB, LOW);
  }
  else{
    //Motor sentido antihorario
    digitalWrite(M.pinA, LOW);
    analogWrite(M.pinB, (255/2));
  }
  M.estado = moviendo;
  return false; //Sigue funcionando
}

int grados_a_PWM(double angulo, double relacion_reductora, double relacion_PWM){
  return(round(angulo * relacion_reductora * relacion_PWM));
}


void leer_I(amperimetro& A){
  A.A_D = analogRead(A.pin);
  A.Vo = ((A.A_D/1024)*5000)-2500; //Conversion de la señal A/D de 1024 bit a una señal de voltaje
  A.Vo = A.Vo/185; // y Relacion mV/A de la hoja de caractaristicas
}