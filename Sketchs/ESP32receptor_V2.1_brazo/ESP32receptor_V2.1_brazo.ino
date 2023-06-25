/* LISTA DE TAREAS
 * 1 - Configurar la disposicion de pines para integrar al receptor los pulsadores de star y stop + la pantalla LCD 
 * 2 - (LISTO) Conseguir el rango de valores de todos los sensores 
 * 3 - Armar una funcion que haga la conversion del valor del sensor al valor necesarios para generar la señal PWM
 * 4 - Actualizar la funcio rutinaTiempoReal();
*/
// DECLARACION DE LIBRERIAS ****************************************************************************************
#include "FS.h"
#include "SPI.h"
#include "SD.h"
#include <esp_now.h>
#include <WiFi.h>
#include <LiquidCrystal.h>

// DECLARACION DE PINES PARA LA MEMORIA SD *************************************************************************
#define CS 5
#define MOSI 23
#define CLK 18
#define MISO 19

// PUERTOS DE LA PANTALLA LCD **************************************************************************************
#define RS_4 13 // >> 2 << 22
#define E_6 12 // >> 13 << 23
#define D4_11 14 // >> 15 << 5
#define D5_12 27 // >> 14 << 18
#define D6_13 26 // >> 12 << 19
#define D7_14 25 // >> 4 << 21

// PUERTOS DE LOS SERVOMOTORES *************************************************************************************
#define PIN_SERVO_1 16 //SERVO1 = hombro
#define PIN_SERVO_2 17 //SERVO2 = elevacion
#define PIN_SERVO_3 05 //SERVO3 = muñeca 
#define PIN_SERVO_4 18 //SERVO4 = meñique/anular
#define PIN_SERVO_5 19 //SERVO5 = mayor
#define PIN_SERVO_6 21 //SERVO6 = indice
#define PIN_SERVO_7 03 //SERVO7 = pulgar

// PUERTOS DE LOS POTENCIOMETROS Y PULSADORES **********************************************************************
#define POTE_SELECT_RUTINA 36
#define BTN_INICIAR_RUTINA 39
#define BTN_DETENER_RUTINA 34

// VALORES LIMITE DE LOS DISTINTOS SENSORES ************************************************************************
#define VAL_PULGAR_RELAX 3000
#define VAL_PULGAR_FLEX 2300
#define VAL_INDICE_RELAX 2700
#define VAL_INDICE_FLEX 1900
#define VAL_MEDIO_RELAX 2900
#define VAL_MEDIO_FLEX 1800
#define VAL_MENIQUE_RELAX 2800
#define VAL_MENIQUE_FLEX 1900

// CONFIGURACION DE LAS PROPIEDADES DE LA SEÑAL PWM ****************************************************************
#define CANAL_1  1 //PWM - hombro
#define CANAL_2  2 //PWM - elevacion
#define CANAL_3  3 //PWM - muñeca
#define CANAL_4  4 //PWM - meñique/anular
#define CANAL_5  5 //PWM - mayor
#define CANAL_6  6 //PWM - indice
#define CANAL_7  7 //PWM - pulgar
#define RESOLUCION_PWM  8  // La señal tendra una resolucion de 8 bits, 256 valores posibles
#define FRECUENCIA_PWM  50 // Los servos trabajan en periodos de 20ms con duty entre 1ms y 2ms

// CONSTANTES ******************************************************************************************************
#define DELAY_RECCONFIG 2000 // Duracion de la recepcion de la estructura myreceiveData

// ESTRUCTURAS *****************************************************************************************************
struct s_receiveData
{ // Estructura implementada para la transmision de datos
  short sensor, rutina, estado; // estado >> (1) tiempo real  ; (2) transmitir rutina ; (3) crear rutina ; (4) cambiar sensor ; (5) borrar rutina 
  float ax,ay,az,gx,gy,gz;
  float d1,d2,d3,d4,d5;
  bool b_esDato; // Booleano que indica si la estructura enviada son datos de lista o no
};
typedef struct s_receiveData s_ReceiveData; 

// DECLARACION DE VARIABLES ****************************************************************************************
File archivo;             // Referencia a los elementos dentro de la memoria
String sNombreArchivo;    // Nombre del archivo con el cual se desea trabajar (usar solo en las rutinas)
bool b_recibiendo;        // indica si en este loop se esta recibiendo informacion mediante el protocolo ESPNOW
bool b_cambioDeFase;      // indica si en este loop se cambio de fase
bool b_omitirOrden=false; // omite todas las acciones hasta llegar a fase 0
long ultimoDatoRecibido;  // indica el momento en el cual se ha recibido el ultimo dato desde el transmisor
short faseActual = 0;     // fase de recepcion de datos (1)-configuracion ; (2)-datos ; (3)-configuracion
short faseAnterior = 0;   // valor de la fase del loop anterior
short lista[18];          // Contenido actualizado del archivo Lista.txt con valores en ASCII >> r1s1r2s2r3s3...r8s8r9s9 (0-48 ; 9-57)
short sensorSeleccionado=0; // Indica la rutina seleccionada que el usuario desea reproducir
short modoDeReproduccion=0; // Indica si se esta reproduciendo alguna rutina en el brazo segun los siguientes valores
/* 0 - No se esta reproduciendo nada  ;  1 - Reproduciendo por orden del guante  ;  2 - Reproduciendo una rutina desde la memoria SD*/

s_ReceiveData myReceiveData; // Instanciamos las estructuras para almacenar los datos de la lista para la memoria, 

// CONFIGURACION DE LOS PUERTOS DE LA PANTALLA LCD *****************************************************************
LiquidCrystal lcd( RS_4 , E_6 , D4_11 , D5_12 , D6_13 , D7_14 );
/* LCD ESP32
    1   -  GND 
    2   -  5V
    3   -  Vo (pote contraste)
    4   -  D2
    5   -  GND
    6   -  D13
    11  -  D15
    12  -  D14
    13  -  D12
    14  -  D4
    15  -  5V
    16  -  GND 
*/

// CABECERAS DE FUNCIONES ******************************************************************************************

// Actualiza la variable modoDeReproduccion
void actualizarModoDeReproduccion();

// Configura los puertos de entrada/salida del ESP32
void configurarPuertos();

// Configura los puertos y canales PWM con las propiedades necesarias para funcionar con los motores del brazo 
void configurarPWM();

// Crea una nueva rutina dentro de la memoria SD
void crearNuevaRutina();

// Edita el contenido del archivo lista.txt segun lo almacenado en la estructura myReceiveData
void editarArchivoLista();

// Escribe en una nueva linea el String "contenido" al final del archivo "nombre"  
void escribirAlFinal(String nombre, String contenido);

// Carga los datos de la estructura dentro del archivo de la rutina correspondiente
void escribirArchivo();

// Actualiza el valor de los booleanos acorde a la estapa actual de cada orden
void gestorDeBooleanos();

// Actualiza el valor de la fase de recepcion de datos actual 0-nada ; 1-config ; 2-dato ; 3-config
void gestorDeFase(); 

// Busca y retorna el numero de la rutina vinculado el numero del sensor s
short getRoutine(short s);

// Lee el potenciometro selector de sensores y retorna el sensor seleccionado segun sea el valor del potenciometro
short getSensor();

// Configura el Esp32 del receptor para que pueda recibir datos normalmente
void iniciarESPNOW();

// Inicializa la memoria SD
void iniciarMemoriaSD();

// Actualiza el mensaje de la pantalla LCD
void LCD_actualizarPantalla();

// Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]
void leerLista();

// Realiza las modificaciones necesarias sobre el dato d1 y mueve con dicho dato el motor de PIN_SERVO_7 (pulgar)
void PWMd1(float valorSensor);

// Reproduce (si existe) la rutina numero "num" de la memoria SD 
void reproducirRutina(short num);

// Indica si se ha pulsado el boton BTN_DETENER_RUTINA
bool routineStopped();

// Indica si se ha pulsado el boton BTN_INICIAR_RUTINA
bool routineStarted();

// Modifica el archivo Rutina.txt eliminando el sensor de la rutina almacenada en myReceiveData
void rutinaBorrarRutina();

// Rutina que modifica el archivo Lista.txt y cambia el valor del sensor de la rutina indicada en myReceiveData
void rutinaCambiarSensor();  

// Gestiona las tareas correspondientes acorde a la creacion de una nueva rutina
void rutinaCrearRutina();

// Ejecuta las sub-rutinas acorde a la configuracion recibida desde el emisor 
void rutinaDeEstados(); 

// Rutina de reproduccion en tiempo real de la rutina transmitida
void rutinaReproducirRutina();

// Carga la estructura con los datos recibidos mediante el protocolo ESPNOW
// ----------------------------------------------------------------------------------------------------------------
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) 
{
  memcpy(&myReceiveData, incomingData, sizeof(myReceiveData));
  
  b_recibiendo = true; // indica si se esta recibiendo informacion mediante el protocolo ESPNOW
  ultimoDatoRecibido = millis();
}

// ----------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);
  configurarPuertos();
  configurarPWM();
  iniciarESPNOW();
  iniciarMemoriaSD();  
  //lcd.begin(16,2);
  //lcd.clear();
  //lcd.print("funcionando");

} 

// ----------------------------------------------------------------------------------------------------------------
void loop() 
{
  /*    
  gestorDeFase(); // Modifica la fase de recepcion de datos actual 0-nada ; 1-config ; 2-dato ; 3-config
  gestorDeBooleanos(); // Actualiza el valor de los booleanos acorde a la estapa actual de cada orden

  if(b_recibiendo)
  { // Codigo a ejecutar cuando estamos recibiendo datos desde el emisor
    //Serial.println("loop() >> Llego un paquete de datos >> rutinaDeEstados()");
    b_recibiendo = false; // Con esto me aseguro que entre solo en los loops donde se reciben datos
    rutinaDeEstados(); // Ejecuta las sub-rutinas acorde a la configuracion recibida desde el emisor 
  }
*/
/*  else
  { // Codigo a ejecutar por defecto - Lee los sensores conectados al receptor

    actualizarModoDeReproduccion();

    LCD_actualizarPantalla(); // Actualiza el mensaje de la pantalla LCD 
    
    if(modoDeReproduccion == 2)
    { // Ejecutamos cuando se ha pulsado el boton de reproduccion de rutinas BTN_INICIAR_RUTINA

      while(modoDeReproduccion != 0)
      { // Ejecutamos alguna de las rutinas mientras no forcemos la detencion de la rutina pulsando BTN_DETENER_RUTINA o acabe normalmente la rutina
        reproducirRutina(getRoutine(getSensor())); // Reproducimos la rutina vinculada al sensor marcado en la pantalla LCD
        // reproducirRutina(short num);  Reproduce (si existe) la rutina numero "num" de la memoria SD 
        // getRoutine(short s); Retorna el numero de la rutina vinculada al sensor s, segun el archivo Lista.txt
        // getSensor(); retorna el numero del sensor marcado en la pantalla LCD
      }
    }
  }*/
} 

// ----------------------------------------------------------------------------------------------------------------
void actualizarModoDeReproduccion()
{ // Actualiza la variable modoDeReproduccion
  if(modoDeReproduccion==0 && routineStarted()) modoDeReproduccion = 1;
  else if(routineStopped()) modoDeReproduccion = 0;
}

// ----------------------------------------------------------------------------------------------------------------
void configurarPuertos()
{ // Configura los puertos de entrada/salida del ESP32
  // ASIGNO LOS PUERTOS DE LAS SALIDAS PWM COMO PUERTOS DE ENTRADA
  pinMode(PIN_SERVO_1,OUTPUT);
  pinMode(PIN_SERVO_2,OUTPUT);
  pinMode(PIN_SERVO_3,OUTPUT);
  pinMode(PIN_SERVO_4,OUTPUT);
  pinMode(PIN_SERVO_5,OUTPUT);
  pinMode(PIN_SERVO_6,OUTPUT);
  pinMode(PIN_SERVO_7,OUTPUT);
  pinMode(POTE_SELECT_RUTINA,INPUT);
  pinMode(BTN_INICIAR_RUTINA,INPUT);
  pinMode(BTN_DETENER_RUTINA,INPUT);
}

// ----------------------------------------------------------------------------------------------------------------
void configurarPWM()
{ // Configura los puertos y canales PWM con las propiedades necesarias para funcionar con los motores del brazo 
// Configuracion de las funcionalidades de la señal PWM
  ledcSetup(CANAL_1, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcSetup(CANAL_2, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcSetup(CANAL_3, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcSetup(CANAL_4, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcSetup(CANAL_5, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcSetup(CANAL_6, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcSetup(CANAL_7, FRECUENCIA_PWM, RESOLUCION_PWM);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PIN_SERVO_1, CANAL_1);
  ledcAttachPin(PIN_SERVO_2, CANAL_2);
  ledcAttachPin(PIN_SERVO_3, CANAL_3);
  ledcAttachPin(PIN_SERVO_4, CANAL_4);
  ledcAttachPin(PIN_SERVO_5, CANAL_5);
  ledcAttachPin(PIN_SERVO_6, CANAL_6);
  ledcAttachPin(PIN_SERVO_7, CANAL_7);  
}

// ----------------------------------------------------------------------------------------------------------------
void crearNuevaRutina()
{ // Crea una nueva rutina dentro de la memoria SD

  //Serial.println("crearNuevaRutina() >> creando el archivo Rutina_R.txt");
  short rutina = myReceiveData.rutina; // Numero de la rutina que deseamos crear
  String nombreArchivo = "/Rutina_" + (String)rutina + ".txt"; // Nombre del archivo que quremos crear

  archivo = SD.open(nombreArchivo, FILE_WRITE);
  archivo.close(); 
}

// ----------------------------------------------------------------------------------------------------------------
void editarArchivoLista()
{ // Edita el contenido del archivo lista.txt segun lo almacenado en la estructura myReceiveData

  //Serial.println("editarArchivoLista() >> actualizando Lista.txt");
  
  short sensor = myReceiveData.sensor + 48; // Valor leido del sensor (en ascii)
  short rutina = myReceiveData.rutina; // Valor leido de la rutina

  archivo = SD.open("/Lista.txt",FILE_WRITE);
  lista[2*rutina-1] = sensor;
  for(int i=0;i<9;i++)
  { // Escribe los datos del string dentro del archivo lista.txt
    archivo.print((char)lista[2*i]); 
    archivo.print('-');  
    archivo.println((char)lista[2*i+1]);
  } // fin >> for(i=0;i<9;i++)
  archivo.close(); 
}

// ----------------------------------------------------------------------------------------------------------------
void escribirAlFinal(String nombre, String contenido)
{ // Escribe en una nueva linea el String "contenido" al final del archivo "nombre" 
  archivo = SD.open(nombre, FILE_APPEND); // Abrimos el archivo en modo reescritura
  if(archivo) archivo.println(contenido); // Si se abrio correctamente, agregamos contenido al final del mismo
  archivo.close();
}

// ----------------------------------------------------------------------------------------------------------------
void escribirArchivo()
{ // Carga los datos de la estructura myReceiveData dentro del archivo de la rutina correspondiente
  short rutina = myReceiveData.rutina; // Numero de la rutina que deseamos crear
  String nombreArchivo = "/Rutina_" + (String)rutina + ".txt"; // Nombre del archivo que quremos crear

  archivo = SD.open(nombreArchivo, FILE_APPEND); // Agregamos un dato al final
  archivo.println((float)myReceiveData.ax);
  archivo.println((float)myReceiveData.ay);
  archivo.println((float)myReceiveData.az);
  archivo.println((float)myReceiveData.gx);
  archivo.println((float)myReceiveData.gy);
  archivo.println((float)myReceiveData.gz);
  archivo.println((float)myReceiveData.d1);
  archivo.println((float)myReceiveData.d2);
  archivo.println((float)myReceiveData.d3);
  archivo.println((float)myReceiveData.d4);
  archivo.println((float)myReceiveData.d5);

  archivo.close(); 

}

// ----------------------------------------------------------------------------------------------------------------
void gestorDeBooleanos()
{ // Actualiza el valor de los booleanos acorde a la estapa actual de cada orden

  //************ Cambios de fase dentro del modo 2 ***************
  // Primer loop del modo 2 - fase 1
  if(faseActual != faseAnterior && faseActual == 1 && myReceiveData.estado == 2) 
  {
    b_cambioDeFase = true;
    //Serial.print("gestorDeBooleanos() >> "); Serial.print("b_cambioDeFase = "); Serial.println(b_cambioDeFase);
  }
  // Primer loop del modo 2 - fase 2
  else if(faseActual != faseAnterior && faseActual == 2 && myReceiveData.estado == 2) b_cambioDeFase = true;
  // Primer loop del modo 2 - fase 3
  else if(faseActual != faseAnterior && faseActual == 3 && myReceiveData.estado == 2) b_cambioDeFase = true;


  //*************** En caso de que no haya cambios de fase de ningun tipo *********************
  else {b_cambioDeFase=false;}

  faseAnterior=faseActual; // Actualizamos el valor de la faseActual anterior y terminamos el loop
}

// ----------------------------------------------------------------------------------------------------------------
void gestorDeFase()
{ // Modifica la faseActual de recepcion de datos actual 0-nada ; 1-config ; 2-dato ; 3-config
  if(faseActual!=0 && millis()-ultimoDatoRecibido > DELAY_RECCONFIG)
  { // Se ejecuta si ha pasado un tiempo DELAY_RECCONFIG desde que se recibio el ultimo dato
    faseActual = 0;
  }
  else if(faseActual==0 && !myReceiveData.b_esDato)
  { // Se ejecuta cuando pasamos de la faseActual 0 a la faseActual 1 
    faseActual = 1;
  }
  else if(faseActual == 1 && myReceiveData.b_esDato)
  { // Se ejecuta cuando pasamos de la faseActual 1 a la faseActual 2 
   faseActual = 2;
  }
  else if(faseActual == 2 && !myReceiveData.b_esDato)
  { // Se ejecuta cuando pasamos de la faseActual 2 a la faseActual 3 
   faseActual = 3;
  }
  /*
  Serial.print("faseActual = ");   Serial.println(faseActual);
  Serial.print("b_esDato = ");   Serial.println(myReceiveData.b_esDato);
  */
}

// ----------------------------------------------------------------------------------------------------------------
short getRoutine(short s)
{ // Busca y retorna el numero de la rutina vinculado el numero del sensor s, segun el archivo Lista.txt
  leerLista(); // Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]
  /*
  Serial.print("Lista actual: ");
  Serial.print(lista[0]-48); Serial.print("-");
  Serial.print(lista[1]-48); Serial.print(" ; ");
  Serial.print(lista[2]-48); Serial.print("-");
  Serial.print(lista[3]-48); Serial.print(" ; ");
  Serial.print(lista[4]-48); Serial.print("-");
  Serial.print(lista[5]-48); Serial.print(" ; "); 
  Serial.print(lista[6]-48); Serial.print("-");
  Serial.print(lista[7]-48); Serial.print(" ; ");
  Serial.print(lista[8]-48); Serial.print("-");
  Serial.print(lista[9]-48); Serial.print(" ; ");   
  Serial.print(lista[10]-48); Serial.print("-");
  Serial.print(lista[11]-48); Serial.print(" ; ");
  Serial.print(lista[12]-48); Serial.print("-");
  Serial.print(lista[13]-48); Serial.print(" ; ");  
  Serial.print(lista[14]-48); Serial.print("-");
  Serial.print(lista[15]-48); Serial.print(" ; ");  
  Serial.print(lista[16]-48); Serial.print("-");
  Serial.println(lista[17]-48);
  */
  short ret; // valor de retorno
  int i = 0;
  for(i==0; i<9; i++)
  {
    if(lista[2*i+1]-48 == s) 
    {
      ret = lista[2*i]-48;
      /*
      Serial.print("Rutina "); Serial.println(lista[2*i]-48);
      Serial.print("Sensor "); Serial.println(lista[2*i+1]-48);
      */
    }
  }
  //delay(500);
  return ret;
}

// ----------------------------------------------------------------------------------------------------------------
short getSensor()
{  // Lee el potenciometro selector de sensores y retorna el sensor seleccionado segun sea el valor del potenciometro
  short val = analogRead(POTE_SELECT_RUTINA);
  if(val < 455) return 1;
  else if(val < 910) return 2;
  else if(val < 1365) return 3;
  else if(val < 1820) return 4;
  else if(val < 2275) return 5;
  else if(val < 2730) return 6;
  else if(val < 3185) return 7;
  else if(val < 3640) return 8;
  else return 9;
}

// ----------------------------------------------------------------------------------------------------------------
void iniciarESPNOW()
{ // Configura el Esp32 del receptor para que pueda recibir datos normalmente
  WiFi.mode(WIFI_STA);
  SD.begin(CS);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);  
}

// ----------------------------------------------------------------------------------------------------------------
void iniciarMemoriaSD()
{ // Inicializa la memoria SD

  Serial.println("iniciarMemoriaSD >> Entrando a la función");
  
  SD.begin(CS);
  
  Serial.println("iniciarMemoriaSD >> Memoria sd inicializada");

  
  // Si no existe el archivo Lista.txt lo creamos y le escribimos un cero
  if(!SD.exists("/Lista.txt")) 
  { 
    escribirAlFinal("/Lista.txt","1-0"); 
    escribirAlFinal("/Lista.txt","2-0"); 
    escribirAlFinal("/Lista.txt","3-0"); 
    escribirAlFinal("/Lista.txt","4-0"); 
    escribirAlFinal("/Lista.txt","5-0"); 
    escribirAlFinal("/Lista.txt","6-0"); 
    escribirAlFinal("/Lista.txt","7-0"); 
    escribirAlFinal("/Lista.txt","8-0"); 
    escribirAlFinal("/Lista.txt","9-0"); 

    Serial.println("iniciarMemoriaSD >> Archivo base creado");
  }
    Serial.println("iniciarMemoriaSD >> Saliendo de la función");

}

// ----------------------------------------------------------------------------------------------------------------
void LCD_actualizarPantalla()
{ // Actualiza el mensaje en la pantalla LCD 
  int sensor = getSensor(); // Valor del sensor indicado por el potenciometro
  
  if(sensor != sensorSeleccionado && modoDeReproduccion == 0)
  {
    lcd.clear();
    lcd.print("Desea activar");
    lcd.setCursor(0,1);
    lcd.print("el sensor "+(String)sensor+"?");
    sensorSeleccionado = sensor;
  }
  if(modoDeReproduccion == 1)
  {
    lcd.clear();
    lcd.print("Reproduciendo:");
    lcd.setCursor(0,1);
    lcd.print("rut:"+(String)getRoutine(sensor)+" - sen:"+(String)sensor);
  }
}

// ----------------------------------------------------------------------------------------------------------------
void leerLista()
{ // Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]

  //Serial.println("leerLista() >> Cargando los valores de LIsta.txt dentro del arreglo lista[18]");
  
  short valorLeido;
  short i = 0;
  
  archivo = SD.open("/Lista.txt",FILE_READ);
  archivo.seek(0); // posicionamos el puntero al inicio del archivo 

  while(archivo.available()) 
  { // Lee mientras exista contenido dentro del archivo
    valorLeido = archivo.read(); // lee un solo caracter - los lee como valores ascii (los numeros estan entre 48(0) y 57(9))

    if(valorLeido >= 48 && valorLeido <= 57)
    { // Almacenamos el valor en el vector si el mismo vale entre 48(0) y 57(9)
      lista[i] = valorLeido;
      i++;   
    } // fin >> if(valorLeido >= 48 && valorLeido <= 57)
  } // fin >> while(archivo.available())
  archivo.close();
}

// ----------------------------------------------------------------------------------------------------------------
void PWMd1(float valorSensor)
{ // Realiza las modificaciones necesarias sobre el dato d1 y mueve con dicho dato el motor de PIN_SERVO_7 (pulgar)

  // Filtramos y recortamos segun los limites superior e inferior esperados del sensor del pulgar
  if(valorSensor > VAL_PULGAR_RELAX) valorSensor = VAL_PULGAR_RELAX;
  else if(valorSensor < VAL_PULGAR_FLEX) valorSensor = VAL_PULGAR_FLEX;

  // Realizamos la conversion para que VAL_PULGAR_RELAX sea equivalente a 13(8 bits) para un duty de 1ms y VAL_PULGAR_FLEX a 26 para un duty de 2ms
  float pwm = ((float)((VAL_PULGAR_RELAX - valorSensor)/(VAL_PULGAR_RELAX - VAL_PULGAR_FLEX))+1)*13; // debe variar entre 13 y 26
  ledcWrite(CANAL_7, pwm);
}

// ----------------------------------------------------------------------------------------------------------------
void PWMd2(float valorSensor)
{ // Realiza las modificaciones necesarias sobre el dato d1 y mueve con dicho dato el motor de PIN_SERVO_6 (indice)

  // Filtramos y recortamos segun los limites superior e inferior esperados del sensor del pulgar
  if(valorSensor > VAL_INDICE_RELAX) valorSensor = VAL_INDICE_RELAX;
  else if(valorSensor < VAL_INDICE_FLEX) valorSensor = VAL_INDICE_FLEX;

  // Realizamos la conversion para que VAL_INDICE_RELAX sea equivalente a 13(8 bits) para un duty de 1ms y VAL_INDICE_FLEX a 26 para un duty de 2ms
  float pwm = ((float)((VAL_INDICE_RELAX - valorSensor)/(VAL_INDICE_RELAX - VAL_INDICE_FLEX))+1)*13; // debe variar entre 13 y 26
  ledcWrite(CANAL_6, pwm);
}

// ----------------------------------------------------------------------------------------------------------------
void PWMd3(float valorSensor)
{ // Realiza las modificaciones necesarias sobre el dato d1 y mueve con dicho dato el motor de PIN_SERVO_6 (MEDIO)

  // Filtramos y recortamos segun los limites superior e inferior esperados del sensor del pulgar
  if(valorSensor > VAL_MEDIO_RELAX) valorSensor = VAL_MEDIO_RELAX;
  else if(valorSensor < VAL_MEDIO_FLEX) valorSensor = VAL_MEDIO_FLEX;

  // Realizamos la conversion para que VAL_MEDIO_RELAX sea equivalente a 13(8 bits) para un duty de 1ms y VAL_MEDIO_FLEX a 26 para un duty de 2ms
  float pwm = ((float)((VAL_MEDIO_RELAX - valorSensor)/(VAL_MEDIO_RELAX - VAL_MEDIO_FLEX))+1)*13; // debe variar entre 13 y 26
  ledcWrite(CANAL_5, pwm);
}

// ----------------------------------------------------------------------------------------------------------------
void PWMd4(float valorSensor)
{ // Realiza las modificaciones necesarias sobre el dato d1 y mueve con dicho dato el motor de PIN_SERVO_6 (MENIQUE)

  // Filtramos y recortamos segun los limites superior e inferior esperados del sensor del pulgar
  if(valorSensor > VAL_MENIQUE_RELAX) valorSensor = VAL_MENIQUE_RELAX;
  else if(valorSensor < VAL_MENIQUE_FLEX) valorSensor = VAL_MENIQUE_FLEX;

  // Realizamos la conversion para que VAL_MENIQUE_RELAX sea equivalente a 13(8 bits) para un duty de 1ms y VAL_MENIQUE_FLEX a 26 para un duty de 2ms
  float pwm = ((float)((VAL_MENIQUE_RELAX - valorSensor)/(VAL_MENIQUE_RELAX - VAL_MENIQUE_FLEX))+1)*13; // debe variar entre 13 y 26
  ledcWrite(CANAL_4, pwm);
}

// ----------------------------------------------------------------------------------------------------------------
void reproducirRutina(short num)
{ // Reproduce (si existe) la rutina numero "num" de la memoria SD 
  String nombreArchivo =  "/Rutina_" + (String)num + ".txt"; // Nombre del archivo que quremos leer
  String dato=""; // Dato armado de los char sucesivos que leemos del archivo
  bool b_forzarCierre = false; // Se usa para forzar el fin de la reproduccion de la rutina
  bool cValido; // Indica si el caracter leido es un numero, un punto o un signo menos
  short valorLeido; // Valor leido del archivo (se lee como un ASCII)
  short i=0; // Indica la cantidad de datos enviados, cuando i=11 (enviamos un pack de datos al puerto serie) hacemos un pequeño delay

  // Serial.println("Lista que se quiere abrir: " + nombreArchivo);
    
  if(SD.exists(nombreArchivo))
  { // Ejecutamos la rutina indicada por el sensor solo si esta existe
    archivo = SD.open(nombreArchivo,FILE_READ); // Abrimos el archivo en modo lectura
    while(archivo.available() && !routineStopped())
    { // Leemos el archivo mientras tengamos contenido en el mismo y no se fuerce el cierre de la rutina con BTN_DETENER_RUTINA
      valorLeido = archivo.read();  
      cValido = (valorLeido >= 48 && valorLeido <= 57) || valorLeido=='-' || valorLeido=='.';
      if(cValido) 
      { // Si el dato es valido lo almacenamos en el string y habilitamos el guardado de datos
        dato += (char)valorLeido;  
      }         
      else if(!cValido && dato!="")
      { // Si el caracter no fue valido, transmitimos lo almacenado hasta ahora y limpiamos el String dato
        if(i == 1)
        { // Leemos el dato ax
        } else if(i == 2)
        { // Leemos ay
        } else if(i == 3)
        { // Leemos az
        } else if(i == 4)
        { // Leemos gx
        } else if(i == 5)
        { // Leemos gy
        } else if(i == 6)
        { // Leemos gz
        } else if(i == 7)
        { // Leemos d1 (pulgar)
          PWMd1(dato.toFloat()); // Convierte el valor leido y mueve el motor con la señal PWM generada
        } else if(i == 8)
        { // Leemos d2 (indice)
          PWMd2(dato.toFloat()); // Convierte el valor leido y mueve el motor con la señal PWM generada
        } else if(i == 9)
        { // Leemos d3 (medio)
          PWMd3(dato.toFloat()); // Convierte el valor leido y mueve el motor con la señal PWM generada
        } else if(i == 10)
        { // Leemos d4 (anular) (ignorado)
        } else if(i == 11)
        { // Leemos d5 (menique)
          PWMd4(dato.toFloat()); // Convierte el valor leido y mueve el motor con la señal PWM generada
        }
        dato="";
        i++;
        delay(50);
      }      
      //forzarCierre = digitalRead(STOP); // IMPLEMENTAR EL BOTON PARA FORZAR LA FINALIZACION DE LA RUTINA
      if(i==11)
      { // Si i =11 entonces ya mandamos los siguientes datos (d1 d2 d3 d4 d5 ax ay az gx gy gz)
        delay(10);
        i=0;
      }
    } // fin >> while(archivo.available() && !b_forzarCierre)  
    archivo.close(); // Cerramos el archivo
  } // fin >> if(SD.exists(nombreArchivo))
}

// ----------------------------------------------------------------------------------------------------------------
bool routineStopped()
{ // Indica si se ha pulsado el boton BTN_DETENER_RUTINA
  return digitalRead(BTN_DETENER_RUTINA == HIGH);
}

// ----------------------------------------------------------------------------------------------------------------
bool routineStarted()
{ // Indica si se ha pulsado el boton BTN_INICIAR_RUTINA
  return digitalRead(BTN_INICIAR_RUTINA == HIGH);
}

// ----------------------------------------------------------------------------------------------------------------
void rutinaBorrarRutina()
{ // Modifica el archivo Rutina.txt eliminando el sensor de la rutina almacenada en myReceiveData

  if (faseActual==1)
  {
    leerLista(); // Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]

    /*Serial.println("rutinaCambiarSensor()");
    Serial.print("estado = "); Serial.println(myReceiveData.estado);
    Serial.print("rutina = "); Serial.println(myReceiveData.rutina);
    Serial.print("sensor = "); Serial.println(myReceiveData.sensor);
    Serial.print("faseActual = "); Serial.println(faseActual);
    */
    short rutina = myReceiveData.rutina; // Valor leido de la rutina
    String nombreArchivo = "/Rutina_" + (String)rutina + ".txt"; // Nombre del archivo que quremos eliminar
    lista[2*rutina-1] = 0;
    
    //SD.remove(nombreArchivo);
    archivo = SD.open("/Lista.txt",FILE_WRITE);
    for(int i=0;i<9;i++)
    { // Escribe los datos del string dentro del archivo lista.txt
      archivo.print((char)lista[2*i]); 
      archivo.print('-');  
      archivo.println((char)lista[2*i+1]);
    } // fin >> for(i=0;i<9;i++)
    
    archivo.close(); 
  
  }
}

// ----------------------------------------------------------------------------------------------------------------
void rutinaCambiarSensor()
{ // Rutina que modifica el archivo Lista.txt y cambia el valor del sensor de la rutina indicada en myReceiveData
  if (faseActual==1)
  {
    leerLista(); // Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]
/*
    Serial.println("rutinaCambiarSensor()");
    Serial.print("estado = "); Serial.println(myReceiveData.estado);
    Serial.print("rutina = "); Serial.println(myReceiveData.rutina);
    Serial.print("sensor = "); Serial.println(myReceiveData.sensor);
    Serial.print("faseActual = "); Serial.println(faseActual);
 */   
    short sensor = myReceiveData.sensor + 48; // Valor leido del sensor (en ascii)
    short rutina = myReceiveData.rutina; // Valor leido de la rutina
    String nombreArchivo = "/Rutina_" + (String)rutina + ".txt"; // Nombre del archivo que quremos eliminar

    if(SD.exists(nombreArchivo))
    {
    lista[2*rutina-1] = sensor;
    
    //SD.remove(nombreArchivo);
    archivo = SD.open("/Lista.txt",FILE_WRITE);
    for(int i=0;i<9;i++)
    { // Escribe los datos del string dentro del archivo lista.txt
      archivo.print((char)lista[2*i]); 
      archivo.print('-');  
      archivo.println((char)lista[2*i+1]);
    } // fin >> for(i=0;i<9;i++)
    
    archivo.close(); 
    }
  }
}

// ----------------------------------------------------------------------------------------------------------------
void rutinaCrearRutina()
{ // Gestiona las tareas correspondientes acorde a la creacion de una nueva rutina

  // LISTA DE TAREAS
  // 1 - Al recebir la configuracion (2-R-S) en la faseActual 1 se debe editar el archivo Lista.txt actualizando el sensor y
  //     creando el archivo Rutina_R.txt
  // 2 - En la faseActual 2 comenzara a llegar el contenido de la rutina que se desea transmitir, asi que de almacenara dicho contenido
  //     dentro del archivo Rutina_R.txt que se debe alojar en la memoria SD del receptor
  // 3 - En la faseActual 3 se cierra el archivo y se actualizan todas las variables que sean necesarias para volver al estado por defecto

  // (1) - Actualizar Lista.txt y crear archivo de rutina
  // (2) - Esperando a la faseActual 2
  // (3) - Leer dato entrante (dentro de la faseActual 2)
  // (4) - Esperando dato entrante (dentro de la faseActual 2)
  // (5) - faseActual 3 >> cerrar archivo
      
  switch (faseActual)
  {    
    case 1: // Edita el archivo Lista.txt y crea un archivo Rutina_R.txt
    //Serial.println("rutinaCrearRutina() >> faseActual = 1");
    if(b_cambioDeFase)
    { // Se ejecuta cuando cambiamos de faseActual 0 a faseActual 1 
      //Serial.println("------------------------------------------------------------------------------------------------");  
      leerLista(); // Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]
      editarArchivoLista(); // Lee el archivo Lista.txt y asigna los valores cargados en la estructura
      crearNuevaRutina(); // Crea una nueva rutina dentro de la memoria SD
    }   
    break;  
    case 2: // Carga los valores almacenados en la estructura dentro del archivo correspondiente
    //Serial.println("rutinaCrearRutina() >> faseActual = 2");
    escribirArchivo();// Carga los datos de la estructura dentro del archivo de la rutina correspondiente
    break;
  }
}

// ----------------------------------------------------------------------------------------------------------------
void rutinaDeEstados()
{ // Ejecuta las sub-rutinas acorde a la configuracion recibida desde el emisor
  Serial.println("rutinaDeEstados() >> Llego un paquete de datos");
   
  Serial.print("estado = "); Serial.println(myReceiveData.estado);
  Serial.print("rutina = "); Serial.println(myReceiveData.rutina);
  Serial.print("sensor = "); Serial.println(myReceiveData.sensor);
 
  switch(myReceiveData.estado)
  {
    case 1: // Reproduccion en tiempo real
    Serial.println("rutinaDeEstados() >> Reproducir en tiempo real >> rutinaTiempoReal()");
    rutinaTiempoReal();
    break;
    case 2: // Recibir y guardar en memoria una rutina desde el emisor
    Serial.println("rutinaDeEstados() >> Transmitir rutina >> rutinaCrearRutina()");
    rutinaCrearRutina();
    break;
    case 4: // Cambiar el numero del sensor de la rutina indicada
    //Serial.println("rutinaDeEstados() >> Cambiar sensor >> rutinaCambiarSensor()");
    rutinaCambiarSensor();    
    break;
    case 5: // Borrar la rutina seleccionada  
    //Serial.println("rutinaDeEstados() >> Borrar rutina >> rutinaBorrarRutina()");
    rutinaBorrarRutina();
    break;
    case 6: // Reproducir la rutina seleccionada  
    //Serial.println("rutinaDeEstados() >> Reproducir rutina >> rutinaTiempoReal()");
    rutinaTiempoReal();
    break;
  } 
}

// ----------------------------------------------------------------------------------------------------------------
void rutinaTiempoReal()
{ // Gestiona la rutina acorde a la reproduccion de datos en tiempo real

  //servo1.attach(M_GY, PULSOMIN, PULSOMAX);
  //servo1.attach(M_GY);
  

  int VALORPOTY;
  int ANGULOY;


  
  if(faseActual==2 && myReceiveData.b_esDato)
  {
    VALORPOTY = (int)(myReceiveData.ay*18);
    //ANGULOY = map(VALORPOTY, 0, 4095, 0, 180);
 
    //servo1.write(VALORPOTY);

    Serial.print("VALORPOT = ");    Serial.println(VALORPOTY);
    Serial.print("ANGULO = ");    Serial.println(ANGULOY);
    Serial.println("--------------------------------------");  
  }

//  ta = millis();
}
