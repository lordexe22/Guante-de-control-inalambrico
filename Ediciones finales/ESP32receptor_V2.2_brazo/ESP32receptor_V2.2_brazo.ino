// DECLARACION DE LIBRERIAS ****************************************************************************************
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <esp_now.h>
#include <WiFi.h>
#include <LiquidCrystal.h>

// PUERTOS DE LA PANTALLA LCD **************************************************************************************
#define RS_4     22 
#define E_6      23 
#define D4_11    5 
#define D5_12    18 
#define D6_13    19 
#define D7_14    21

// PUERTOS DE LA MEMORIA SD ****************************************************************************************
#define SD_SCK   14
#define SD_MISO  27
#define SD_MOSI  13
#define SD_CS    15
SPIClass spiSD(HSPI); // Especificamos que vamos a usar el puerto HSPI para la memoria SD

// PUERTOS DEL POTENCIOMETRO SELECTOR DE RUTINA Y PULSADORES DE INICIO Y PARADA DE REPRODUCCION ********************
#define POTE_SELECT_RUTINA 36
#define BTN_INICIAR_RUTINA 39
#define BTN_DETENER_RUTINA 34

// PUERTOS DE LOS SERVOMOTORES *************************************************************************************
#define PIN_SERVO_1 17 //SERVO1 = hombro
#define PIN_SERVO_2 16 //SERVO2 = elevacion
#define PIN_SERVO_3 4  //SERVO3 = muñeca 
#define PIN_SERVO_4 32 //SERVO4 = pulgar
#define PIN_SERVO_5 33 //SERVO5 = indice
#define PIN_SERVO_6 25 //SERVO6 = mayor
#define PIN_SERVO_7 26 //SERVO7 = meñique

// CONFIGURACION DE LAS PROPIEDADES DE LA SEÑAL PWM ****************************************************************
#define CANAL_1  1 //PWM - hombro
#define CANAL_2  2 //PWM - elevacion
#define CANAL_3  3 //PWM - muñeca
#define CANAL_4  4 //PWM - meñique/anular
#define CANAL_5  5 //PWM - mayor
#define CANAL_6  6 //PWM - indice
#define CANAL_7  7 //PWM - pulgar              D26
#define RESOLUCION_PWM  8  // La señal tendra una resolucion de 8 bits, 256 valores posibles
#define FRECUENCIA_PWM  50 // Los servos trabajan en periodos de 20ms con duty entre 1ms y 2ms

// ESTRUCTURAS *****************************************************************************************************
struct s_receiveData
{ // Estructura implementada para la transmision de datos
  short sensor, rutina, estado; // estado >> (1) tiempo real  ; (2) transmitir rutina ; (3) crear rutina ; (4) cambiar sensor ; (5) borrar rutina 
  float ax,ay,az,gx,gy,gz;
  float d1,d2,d3,d4,d5;
  bool b_esDato; // Booleano que indica si la estructura enviada son datos de lista o no
};
typedef struct s_receiveData s_ReceiveData; 

s_ReceiveData myReceiveData; // Instanciamos las estructuras para almacenar los datos de la lista para la memoria, 

// CONSTANTES ******************************************************************************************************
#define DELAY_RECCONFIG 2000 // Duracion de la recepcion de la estructura myreceiveData

// VALORES LIMITE DE LOS DISTINTOS SENSORES ************************************************************************
#define VAL_PULGAR_RELAX 3000
#define VAL_PULGAR_FLEX 2200
#define VAL_INDICE_RELAX 2700
#define VAL_INDICE_FLEX 1800
#define VAL_MEDIO_RELAX 2700
#define VAL_MEDIO_FLEX 1650
#define VAL_MENIQUE_RELAX 2850
#define VAL_MENIQUE_FLEX 1850
#define VAL_AY_MIN -10
#define VAL_AY_MAX 0


// DECLARACION DE VARIABLES ****************************************************************************************
File archivo; // Referencia a los archivos creados dentro de la memoria
LiquidCrystal lcd( RS_4 , E_6 , D4_11 , D5_12 , D6_13 , D7_14 ); 
short lista[18];            // Contenido actualizado del archivo Lista.txt con valores en ASCII >> r1s1r2s2r3s3...r8s8r9s9 (0-48 ; 9-57)
short sensorSeleccionado=0; // Indica el sensor seleccionado que el usuario desea activar
short faseActual = 0;       // fase de recepcion de datos (1)-configuracion ; (2)-datos ; (3)-configuracion
short faseAnterior = 0;     // valor de la fase del loop anterior
short modoDeReproduccion=0; // Indica si se esta reproduciendo alguna rutina en el brazo segun los siguientes valores
/* 0 - No se esta reproduciendo nada  ;  1 - Reproduciendo en tiempo real  ;  2 - Reproduciendo una rutina desde la memoria SD*/
bool b_recibiendo;          // indica si en este loop se esta recibiendo informacion mediante el protocolo ESPNOW
bool b_cambioDeFase;        // indica si en este loop se cambio de fase
long ultimoDatoRecibido;    // indica el momento en el cual se ha recibido el ultimo dato desde el transmisor

float ajusteFlexion = 9;     // flexion >> ajusta la sensibilidad
float ajusteRotacion = 9;    // muñeca >> ajusta la sensibilidad
float ajusteIndice = 13;          // indice >> ajusta el rango de movimiento  
float ajusteMenique = 13;         // menique >> ajusta el rango de movimiento  
float ajusteMedio = 30;           // Medio >> ajusta el rango de movimiento  
float ajustePulgar = 13;          // Pulgar >> ajusta el rango de movimiento  
float ajusteElevacion = 13;       // elevacion >> ajusta el rango de movimiento  


// CABECERAS DE FUNCIONES ******************************************************************************************
// Actualiza la variable modoDeReproduccion
void actualizarModoDeReproduccion();
// Configura los puertos de entrada y salida del ESP32
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
// Escribe en la pantalla LCD los String Col1 y Col2 en las columnas 1 y 2 respectivamente
void escribirEnPantallaLCD(String col1, String col2);
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
// Inicializacion de la memoria SD
void iniciarMemoriaSD();
// Inicializacion de la pantalla LCD
void iniciarPantallaLCD();
// Lee el valor del potenciometro, lo procesa y actualiza el valor de la rutina en la pantalla LCD acorde al valor indicado
void LCD_actualizarPantalla();
// Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]
void leerLista();
// Puesta a cero del brazo
void puestaCero();
// Genera una señal PWM y mueve el motor anexado al d1 
void PWMd1(float valorSensor);
// Genera una señal PWM y mueve el motor anexado al d2 
void PWMd2(float valorSensor);
// Genera una señal PWM y mueve el motor anexado al d3 
void PWMd3(float valorSensor);
// Genera una señal PWM y mueve el motor anexado al d5 
void PWMd5(float valorSensor);
// Genera una señal PWM y mueve el motor anexado al movimiento de elevacion del brazo 
void PWMelevacion(float ay);
// Genera una señal PWM y mueve el motor anexado al movimiento de rotacion del brazo
void PWMrotacion(float gy); 
// Genera una señal PWM y mueve el motor anexado al movimiento de flexion del brazo
void PWMflexion(float gx); 
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
// // Gestiona la rutina acorde a la reproduccion de datos en tiempo real
void rutinaTiempoReal();

// Carga la estructura con los datos recibidos mediante el protocolo ESPNOW
// ----------------------------------------------------------------------------------------------------------------
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) 
{
  memcpy(&myReceiveData, incomingData, sizeof(myReceiveData));
  b_recibiendo = true; // indica si se esta recibiendo informacion mediante el protocolo ESPNOW
  ultimoDatoRecibido = millis();
}

// ----------------------------------------------------------------------------------------------------------------
void pruebaDeRangoPWM(short minimo, short maximo){
  bool desdeBase = true , desdeTecho = false;
  float valor = minimo; 
  while(true){
    if(desdeBase){
      while(valor<=maximo){
        valor ++;
        ledcWrite(CANAL_7, valor);
        delay(100);
      }
      desdeTecho = true;
      desdeBase = false;
    } else if(desdeTecho){
        while(valor>=minimo){
        valor --;
        ledcWrite(CANAL_7, valor);
        delay(100);
      }
      desdeTecho = false;
      desdeBase = true;
    }
  }
}

// ----------------------------------------------------------------------------------------------------------------
void pruebaDeRotacion(){

  float gx = myReceiveData.gx; // flexion -3 , extension +3
  float gy = myReceiveData.gy; // palma arriba -7 , palma abajo +7
  float ax = myReceiveData.ax; // palma abajo -9.8 , palma arriba +9.8 
  float ay = myReceiveData.ay; // mano vertical -9.8 , mano horizontal 0 

  float ax_min = -9.8;
  float ax_max = 9.8;
  float ay_min = -9.8;
  float ay_max = 0;
  
  short gx_min = -3; 
  short gx_max = 3; 
  short gy_min = -7; 
  short gy_max = 7; 
  
  // MOVIMIENTO ELEVANDO EL BRAZO
  if(ay < ay_min) ay = ay_min; 
  else if(ay > ay_max) ay = ay_max; 

  //float pwm = ((float)((ay_max - ay)/(ay_max - ay_min))+1)*13; // debe variar entre 13 y 26
  float pwm = ((float)((ay_max - ay)/(ay_max - ay_min))+1)*17;
  ledcWrite(CANAL_7, pwm);
  
  delay(20);
  
}


void setup()
{
  Serial.begin(115200);
  iniciarESPNOW();
  iniciarPantallaLCD();
  iniciarMemoriaSD();
  configurarPuertos();
  configurarPWM();

  // pruebaDeRangoPWM(13,40);
  // pruebaDeRotacion();
  // pruebaDeFlexion();
}

void loop()
{
    
     
  gestorDeFase(); // Modifica la fase de recepcion de datos actual 0-nada ; 1-config ; 2-dato ; 3-config
  gestorDeBooleanos(); // Actualiza el valor de los booleanos acorde a la estapa actual de cada orden

  if(b_recibiendo)
  { // Codigo a ejecutar cuando estamos recibiendo datos desde el ESP32 emisor
    b_recibiendo = false; // Con esto me aseguro que entre solo en los loops donde se reciben datos
    rutinaDeEstados(); // Ejecuta las sub-rutinas acorde a la configuracion recibida desde el emisor 
  } 
  
  else
  { // Codigo a ejecutar por defecto - Lee los sensores conectados al receptor
    /*actualizarModoDeReproduccion();*/ 
    //0 - No reproduciendo nada  ;  1 - Reproduciendo en tiempo real  ;  2 - Reproduciendo rutina desde memoria SD

    LCD_actualizarPantalla(); // Actualiza el mensaje de la pantalla LCD 
    
    if(modoDeReproduccion == 2)
    { // Ejecutamos cuando se ha pulsado el boton de reproduccion de rutinas BTN_INICIAR_RUTINA    
      reproducirRutina(getRoutine(getSensor())); // Reproducimos la rutina vinculada al sensor marcado en la pantalla LCD
      // reproducirRutina(short num);  Reproduce (si existe) la rutina numero "num" de la memoria SD 
      // getRoutine(short s); Retorna el numero de la rutina vinculada al sensor s, segun el archivo Lista.txt
      // getSensor(); retorna el numero del sensor marcado en la pantalla LCD      
    }
  }
}

// ----------------------------------------------------------------------------------------------------------------
void actualizarModoDeReproduccion()
{ // Actualiza la variable modoDeReproduccion
  modoDeReproduccion = 0;
  if(routineStarted()) modoDeReproduccion = 2;
}
// ----------------------------------------------------------------------------------------------------------------
void configurarPuertos()
{ // Configura los puertos de entrada y salida del ESP32
  pinMode(POTE_SELECT_RUTINA,INPUT);
  pinMode(BTN_INICIAR_RUTINA,INPUT);
  pinMode(BTN_DETENER_RUTINA,INPUT);
  pinMode(PIN_SERVO_1, OUTPUT);
  pinMode(PIN_SERVO_2, OUTPUT);
  pinMode(PIN_SERVO_3, OUTPUT);
  pinMode(PIN_SERVO_4, OUTPUT);
  pinMode(PIN_SERVO_5, OUTPUT);
  pinMode(PIN_SERVO_6, OUTPUT);
  pinMode(PIN_SERVO_7, OUTPUT);
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
  short rutina = myReceiveData.rutina; // Numero de la rutina que deseamos crear
  String nombreArchivo = "/Rutina_" + (String)rutina + ".txt"; // Nombre del archivo que quremos crear
  archivo = SD.open(nombreArchivo, FILE_WRITE);
  archivo.close(); 
}
// ----------------------------------------------------------------------------------------------------------------
void editarArchivoLista()
{ // Edita el contenido del archivo lista.txt segun lo almacenado en la estructura myReceiveData

  Serial.println("editarArchivoLista()  >>  Iniciando la funcion");  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ACTIVAR PARA PRUEBAS                                                  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ACTIVAR PARA PRUEBAS
  
  short sensor = myReceiveData.sensor + 48; // Valor leido del sensor (en ascii)
  short rutina = myReceiveData.rutina; // Valor leido de la rutina

  Serial.println("editarArchivoLista()  >>  ""rutina:"+(String)rutina+" , sensor:"+(String)(sensor-48));  //!!!!!!!!!!!! ACTIVAR PARA PRUEBAS                                                     

  archivo = SD.open("/Lista.txt",FILE_WRITE);
  lista[2*rutina-1] = sensor;
  for(int i=0;i<9;i++)
  { // Escribe los datos del string dentro del archivo lista.txt
    archivo.print((char)lista[2*i]); 
    archivo.print('-');  
    archivo.println((char)lista[2*i+1]);
  } // fin >> for(i=0;i<9;i++)
  archivo.close(); 

  Serial.println("editarArchivoLista()  >>  lista.txt actualizado");  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ACTIVAR PARA PRUEBAS                                                     
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
void escribirEnPantallaLCD(String col1, String col2)
{ // Escribe en la pantalla LCD los String Col1 y Col2 en las columnas 1 y 2 respectivamente
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(col1);
  lcd.setCursor(0,1);
  lcd.print(col2);
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
void gestorDeBooleanos()
{ // Actualiza el valor de los booleanos acorde a la estapa actual de cada orden

  //************ Cambios de fase dentro del modo 2 ***************
  // Primer loop del modo 2 - fase 1
  if(faseActual != faseAnterior && faseActual == 1 && myReceiveData.estado == 2) b_cambioDeFase = true;
  // Primer loop del modo 2 - fase 2
  else if(faseActual != faseAnterior && faseActual == 2 && myReceiveData.estado == 2) b_cambioDeFase = true;
  // Primer loop del modo 2 - fase 3
  else if(faseActual != faseAnterior && faseActual == 3 && myReceiveData.estado == 2) b_cambioDeFase = true;

  //*************** En caso de que no haya cambios de fase de ningun tipo *********************
  else {b_cambioDeFase=false;}

  // Actualizamos el valor de la faseActual anterior y terminamos el loop
  faseAnterior=faseActual; 
}
// ----------------------------------------------------------------------------------------------------------------
void gestorDeFase()
{ // Modifica la faseActual de recepcion de datos actual 0-nada ; 1-config ; 2-dato ; 3-config
  if(faseActual!=0 && millis()-ultimoDatoRecibido > DELAY_RECCONFIG)
  { // Se ejecuta si ha pasado un tiempo DELAY_RECCONFIG desde que se recibio el ultimo dato
    faseActual = 0;
  } else if(faseActual==0 && !myReceiveData.b_esDato && b_recibiendo)
  { // Se ejecuta cuando pasamos de la faseActual 0 a la faseActual 1 en un loop en el cual recibimos datos
    faseActual = 1;
  } else if(faseActual == 1 && myReceiveData.b_esDato && b_recibiendo)
  { // Se ejecuta cuando pasamos de la faseActual 1 a la faseActual 2 en un loop en el cual recibimos datos
   faseActual = 2;
  } else if(faseActual == 2 && !myReceiveData.b_esDato && b_recibiendo)
  { // Se ejecuta cuando pasamos de la faseActual 2 a la faseActual 3 en un loop en el cual recibimos datos
   faseActual = 3;
  }
}
// ----------------------------------------------------------------------------------------------------------------
void iniciarESPNOW()
{ // Configura el Esp32 del receptor para que pueda recibir datos normalmente
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);  
}
// ----------------------------------------------------------------------------------------------------------------
void iniciarMemoriaSD()
{ // Inicializacion de la memoria SD
  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS); //SCK,MISO,MOSI,SS //HSPI1
  if (!SD.begin( SD_CS, spiSD )) {
    escribirEnPantallaLCD((String)"Problema al",(String)"iniciar memoria");
    delay(5000);
    escribirEnPantallaLCD((String)"Reinicie el",(String)"programa");
    while(true){}
  } else {
    if(!SD.exists("/Lista.txt")) { 
      escribirAlFinal("/Lista.txt","1-0"); 
      escribirAlFinal("/Lista.txt","2-0"); 
      escribirAlFinal("/Lista.txt","3-0"); 
      escribirAlFinal("/Lista.txt","4-0"); 
      escribirAlFinal("/Lista.txt","5-0"); 
      escribirAlFinal("/Lista.txt","6-0"); 
      escribirAlFinal("/Lista.txt","7-0"); 
      escribirAlFinal("/Lista.txt","8-0"); 
      escribirAlFinal("/Lista.txt","9-0"); 
    }
  }
}
// ----------------------------------------------------------------------------------------------------------------
void iniciarPantallaLCD()
{ // Inicializacion de la pantalla LCD
  lcd.begin(16,2);
  lcd.clear();
}
// ----------------------------------------------------------------------------------------------------------------
void LCD_actualizarPantalla()
{ // Lee el valor del potenciometro, lo procesa y actualiza el valor de la rutina en la pantalla LCD acorde al valor indicado
  int sensor; // Valor de la rutina indicado por el potenciometro
  int val = analogRead(POTE_SELECT_RUTINA);

  if(val < 455) sensor = 1;
  else if(val < 910) sensor = 2;
  else if(val < 1365) sensor = 3;
  else if(val < 1820) sensor = 4;
  else if(val < 2275) sensor = 5;
  else if(val < 2730) sensor = 6;
  else if(val < 3185) sensor = 7;
  else if(val < 3640) sensor = 8;
  else sensor = 9;

  if(digitalRead(BTN_DETENER_RUTINA) == HIGH || (sensor != sensorSeleccionado) && modoDeReproduccion == 0)
  {
    /*Serial.println("Activar sensor "+(String)sensor+"?");*/
    escribirEnPantallaLCD((String)"Activar",(String)"sensor "+(String)sensor+(String)"?");
    sensorSeleccionado = sensor;
    modoDeReproduccion = 0;
  }
  if(digitalRead(BTN_INICIAR_RUTINA) == HIGH && modoDeReproduccion == 0)
  {
    /*Serial.println("Reproduciendo rutina"+(String)getRoutine(sensor));*/
    escribirEnPantallaLCD((String)"Reproduciendo",(String)"rutina "+(String)getRoutine(sensor));
    
    modoDeReproduccion = 2;
  }
}
// ----------------------------------------------------------------------------------------------------------------
void leerLista()
{ // Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]

  Serial.println("leerista()  >>  Iniciando la funcion");  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ACTIVAR PARA PRUEBAS                                               //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ACTIVAR PARA PRUEBAS

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

  Serial.println("leerista()  >>  lista[18] cargado desde el archivo lista.txt");  //!!!!!!!!!!!!!!!!!!!!!!!!!!! ACTIVAR PARA PRUEBAS                                               //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ACTIVAR PARA PRUEBAS

  
}

// ----------------------------------------------------------------------------------------------------------------
void puestaCero()
{ // Puesta a cero del brazo robotico
  ledcWrite(CANAL_1, 13);
  ledcWrite(CANAL_2, 13);
  ledcWrite(CANAL_3, 13);
  ledcWrite(CANAL_4, 13);
  ledcWrite(CANAL_5, 13);
  ledcWrite(CANAL_6, 13);
  ledcWrite(CANAL_7, 13);
}
// ----------------------------------------------------------------------------------------------------------------
void PWMd1(float valorSensor)
{ // Realiza las modificaciones necesarias sobre el dato d1 y mueve con dicho dato el motor de PIN_SERVO_7 (pulgar)

  // Filtramos y recortamos segun los limites superior e inferior esperados del sensor del pulgar
  if(valorSensor > VAL_PULGAR_RELAX) valorSensor = VAL_PULGAR_RELAX;
  else if(valorSensor < VAL_PULGAR_FLEX) valorSensor = VAL_PULGAR_FLEX;

  // Realizamos la conversion para que VAL_PULGAR_RELAX sea equivalente a 13(8 bits) para un duty de 1ms y VAL_PULGAR_FLEX a 26 para un duty de 2ms
  // float pwm = ((float)((VAL_PULGAR_RELAX - valorSensor)/(VAL_PULGAR_RELAX - VAL_PULGAR_FLEX))+1)*13; // debe variar entre 13 y 26
  float pwm = 13 + ((float)(ajustePulgar*((VAL_PULGAR_RELAX - valorSensor)/(VAL_PULGAR_RELAX - VAL_PULGAR_FLEX))));

  ledcWrite(CANAL_7, pwm);
}
// ----------------------------------------------------------------------------------------------------------------
void PWMd2(float valorSensor)
{ // Realiza las modificaciones necesarias sobre el dato d2 y mueve con dicho dato el motor de PIN_SERVO_6 (indice)

  // Filtramos y recortamos segun los limites superior e inferior esperados del sensor del pulgar
  if(valorSensor > VAL_INDICE_RELAX) valorSensor = VAL_INDICE_RELAX;
  else if(valorSensor < VAL_INDICE_FLEX) valorSensor = VAL_INDICE_FLEX;

  // Realizamos la conversion para que VAL_INDICE_RELAX sea equivalente a 13(8 bits) para un duty de 1ms y VAL_INDICE_FLEX a 26 para un duty de 2ms
  // float pwm = ((float)((VAL_INDICE_RELAX - valorSensor)/(VAL_INDICE_RELAX - VAL_INDICE_FLEX))+1)*13; // debe variar entre 13 y 26
  float pwm = 13 + ((float)(ajusteIndice*((VAL_INDICE_RELAX - valorSensor)/(VAL_INDICE_RELAX - VAL_INDICE_FLEX))));

  ledcWrite(CANAL_6, pwm);
}
// ----------------------------------------------------------------------------------------------------------------
void PWMd3(float valorSensor)
{ // Realiza las modificaciones necesarias sobre el dato d3 y mueve con dicho dato el motor de PIN_SERVO_5 (MEDIO)

  // Filtramos y recortamos segun los limites superior e inferior esperados del sensor del pulgar
  if(valorSensor > VAL_MEDIO_RELAX) valorSensor = VAL_MEDIO_RELAX;
  else if(valorSensor < VAL_MEDIO_FLEX) valorSensor = VAL_MEDIO_FLEX;

  // Realizamos la conversion para que VAL_MEDIO_RELAX sea equivalente a 13(8 bits) para un duty de 1ms y VAL_MEDIO_FLEX a 26 para un duty de 2ms
  //float pwm = ((float)((VAL_MEDIO_RELAX - valorSensor)/(VAL_MEDIO_RELAX - VAL_MEDIO_FLEX))+1)*13; // debe variar entre 13 y 26
  float pwm = 13 + ((float)(ajusteMedio*((VAL_MEDIO_RELAX - valorSensor)/(VAL_MEDIO_RELAX - VAL_MEDIO_FLEX))));

  ledcWrite(CANAL_5, pwm);
}
// ----------------------------------------------------------------------------------------------------------------
void PWMd4(float valorSensor)
{ // Realiza las modificaciones necesarias sobre el dato d4 y mueve con dicho dato el motor de PIN_SERVO_4 (MENIQUE)
  int minimo = 16;
  // Filtramos y recortamos segun los limites superior e inferior esperados del sensor del pulgar
  if(valorSensor > VAL_MENIQUE_RELAX) valorSensor = VAL_MENIQUE_RELAX;
  else if(valorSensor < VAL_MENIQUE_FLEX) valorSensor = VAL_MENIQUE_FLEX;


    //float pwm = 13 + ((float)(ajusteElevacion*((ay_max - ay)/(ay_max - ay_min))));
    
  // Realizamos la conversion para que VAL_MENIQUE_RELAX sea equivalente a 13(8 bits) para un duty de 1ms y VAL_MENIQUE_FLEX a 26 para un duty de 2ms
//  float pwm = ((float)((VAL_MENIQUE_RELAX - valorSensor)/(VAL_MENIQUE_RELAX - VAL_MENIQUE_FLEX))+1)*13; // debe variar entre 13 y 26
  float pwm = minimo + ((float)(ajusteMenique*((VAL_MENIQUE_RELAX - valorSensor)/(VAL_MENIQUE_RELAX - VAL_MENIQUE_FLEX))));

  ledcWrite(CANAL_4, pwm);
}
// ----------------------------------------------------------------------------------------------------------------
void PWMelevacion(float ay)
{ // Genera una señal PWM y mueve el motor anexado al movimiento de elevacion del brazo
  float ay_min = -9.8;
  float ay_max = 0;
  
  // LIMITES DE LECTURA
  if(ay < ay_min) ay = ay_min; 
  else if(ay > ay_max) ay = ay_max; 

  float pwm = 20 + ((float)(ajusteElevacion*((ay_max - ay)/(ay_max - ay_min))));
  ledcWrite(CANAL_2, pwm);
}
// ----------------------------------------------------------------------------------------------------------------
void PWMflexion(float gx)
{ // Genera una señal PWM y mueve el motor anexado al movimiento de flexion del brazo

  // DECLARACION DE VARIABLES ---------------------------------------------------------------
  short anguloMinimo = 13;//------------------------- Rangos
  short anguloMaximo = 35;
  static long tiempoAnterior = 0;//------------------ Tiempos
  long tiempoActual;
  float dt;
  static float velocidadAnterior = 0;//-------------- Medidas
  float velocidadActual;
  float dv;
  static float anguloAnterior;//--------------------- Angulos calculados
  float anguloActual;
  float dAngulo;

  // LECTURA DE DATOS ------------------------------------------------------------------------
  tiempoActual = millis();
  velocidadActual = gx;

  // CALCULO ---------------------------------------------------------------------------------
  dt = (float)(tiempoActual - tiempoAnterior)/(float)1000;
  if(dt>300) dt = 100;
  dAngulo = -((float)velocidadActual*(float)dt)*(float)ajusteFlexion;
  if(abs(dAngulo)<0.1) dAngulo = 0;
  anguloActual = anguloAnterior + dAngulo;

  // FILTROS ---------------------------------------------------------------------------------
  if(anguloActual > anguloMaximo) anguloActual = anguloMaximo;
  else if (anguloActual < anguloMinimo) anguloActual = anguloMinimo;
  
  // CREACION DE LA SEÑAL PWM ---------------------------------------------------------------
  ledcWrite(CANAL_1, anguloActual);

  // ASIGNACION DE DATOS PARA EL PROXIMO LOOP -----------------------------------------------
  tiempoAnterior = tiempoActual;
  velocidadAnterior = velocidadActual;
  anguloAnterior = anguloActual;
}
// ----------------------------------------------------------------------------------------------------------------
void PWMrotacion(float gy)
{ // Genera una señal PWM y mueve el motor anexado al movimiento de rotacion del brazo

  // DECLARACION DE VARIABLES ---------------------------------------------------------------
  short anguloMinimo = 13;//------------------------- Rangos
  short anguloMaximo = 35;
  static long tiempoAnterior = 0;//------------------ Tiempos
  long tiempoActual;
  float dt;
  static float velocidadAnterior = 0;//-------------- Medidas
  float velocidadActual;
  float dv;
  static float anguloAnterior;//--------------------- Angulos calculados
  float anguloActual;
  float dAngulo;

  // LECTURA DE DATOS ------------------------------------------------------------------------
  tiempoActual = millis();
  velocidadActual = gy;

  // CALCULO ---------------------------------------------------------------------------------
  dt = (float)(tiempoActual - tiempoAnterior)/(float)1000;
  if(dt>300) dt = 100;
  dAngulo = -((float)velocidadActual*(float)dt)*(float)ajusteRotacion;
  if(abs(dAngulo)<0.1) dAngulo = 0;
  anguloActual = anguloAnterior + dAngulo;

  // FILTROS ---------------------------------------------------------------------------------
  if(anguloActual > anguloMaximo) anguloActual = anguloMaximo;
  else if (anguloActual < anguloMinimo) anguloActual = anguloMinimo;
  
  // CREACION DE LA SEÑAL PWM ---------------------------------------------------------------
  ledcWrite(CANAL_3, anguloActual);

  // ASIGNACION DE DATOS PARA EL PROXIMO LOOP -----------------------------------------------
  tiempoAnterior = tiempoActual;
  velocidadAnterior = velocidadActual;
  anguloAnterior = anguloActual;
}
// ----------------------------------------------------------------------------------------------------------------
void reproducirRutina(short num)
{ // Reproduce (si existe) la rutina numero "num" de la memoria SD 
  String nombreArchivo =  "/Rutina_" + (String)num + ".txt"; // Nombre del archivo que queremos leer
  sensorSeleccionado = 99;
  String dato=""; // Dato armado de los char sucesivos que leemos del archivo
  bool b_forzarCierre = false; // Se usa para forzar el fin de la reproduccion de la rutina
  bool cValido; // Indica si el caracter leido es un numero, un punto o un signo menos
  short valorLeido; // Valor leido del archivo (se lee como un ASCII)
  short i=0; // Indica la cantidad de datos leidos hasta el momenro en paquetes de 11 
   
  float datosEnMemoria[11]; // Contiene ax ay az gx gy gx d1 d2 d3 d4 d5
                            //          0  1  2  3  4  5  6  7  8  9  10
                            //               d1 = Meñique ... d5 = Pulgar

  /*ETAPA 1: Comenzar el proceso solamente si existe el archivo*******************************************************************************************/                          
  if(SD.exists(nombreArchivo))
  { // Ejecutamos la rutina indicada por el sensor solo si esta existe
    archivo = SD.open(nombreArchivo,FILE_READ); // Abrimos el archivo en modo lectura

    /*ETAPA 2: Leer ciclicamente el contenido de la rutina indicada y almacenarlo en el vector datosEnMemoria**********************************************/

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
        datosEnMemoria[i] = dato.toFloat(); // Almacenamos el dato dentro del vector 
        dato=""; // Limpiamos el string dato para almacenar el proximo numero de la memoria
        i++;
        
        if(i==11)
        { // Completamos un paquete de datos, en este condicionar se tienen que procesar los datos y ejecutarse para mover los motores

          // Movimiento del pulgar
          PWMd1(datosEnMemoria[6]);
          // Movimiento del indice
          PWMd2(datosEnMemoria[7]);          
          // Movimiento del medio
          PWMd3(datosEnMemoria[8]);          
          // Movimiento del meñique
          PWMd4(datosEnMemoria[10]);
          
          // Movimiento de elevacion
          PWMelevacion(datosEnMemoria[1]);
          // Movimiento de rotacion
          PWMrotacion(datosEnMemoria[4]);
          // Movimiento de flexion
          PWMflexion(datosEnMemoria[3]);
          
          // Reiniciamos la cuenta para el siguiente paquete de 11 datos
          i=0; 

          delay(200);  
        } // fin >> if(i==11)   HASTA ACA EJECUTA CUANDO LLENAMOS EL VECTOR CON LOS 11 DATOS
      } // fin >> else if(!cValido && dato!="")   HASTA ACA EJECUTA CUANDO EL CARACTER LEIDO NO ES VALIDO     
    } // fin >> while(archivo.available() && !routineStopped())  TERMINAMOS DE LEER LA RUTINA
    archivo.close(); // Cerramos el archivo
    
  } // fin >> if(SD.exists(nombreArchivo))
  modoDeReproduccion = 0;
  puestaCero();
}
// ----------------------------------------------------------------------------------------------------------------
bool routineStopped()
{ // Indica si se ha pulsado el boton BTN_DETENER_RUTINA
  return (bool)(digitalRead(BTN_DETENER_RUTINA) == HIGH);
}
// ----------------------------------------------------------------------------------------------------------------
bool routineStarted()
{ // Indica si se ha pulsado el boton BTN_INICIAR_RUTINA
  return (bool)(digitalRead(BTN_INICIAR_RUTINA) == HIGH);
}
// ----------------------------------------------------------------------------------------------------------------
void rutinaBorrarRutina()
{ // Modifica el archivo Rutina.txt eliminando el sensor de la rutina almacenada en myReceiveData
  if (faseActual==1)
  {
    leerLista(); // Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]
    short rutina = myReceiveData.rutina; // Valor leido de la rutina
    String nombreArchivo = "/Rutina_" + (String)rutina + ".txt"; // Nombre del archivo que quremos eliminar
    lista[2*rutina-1] = 48;

    //SD.remove("/Lista.txt");
    SD.remove(nombreArchivo);
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
  // 1 - Al recibir la configuracion (2-R-S) en la faseActual 1 se debe editar el archivo Lista.txt actualizando el sensor y
  //     creando el archivo Rutina_R.txt
  // 2 - En la faseActual 2 comenzara a llegar el contenido de la rutina que se desea transmitir, asi que de almacenara dicho contenido
  //     dentro del archivo Rutina_R.txt que se debe alojar en la memoria SD del receptor
  // 3 - En la faseActual 3 se cierra el archivo y se actualizan todas las variables que sean necesarias para volver al estado por defecto

  // (1) - Actualizar Lista.txt y crear archivo de rutina
  // (2) - Esperando a la faseActual 2
  // (3) - Leer dato entrante (dentro de la faseActual 2)
  // (4) - Esperando dato entrante (dentro de la faseActual 2)
  // (5) - faseActual 3 >> cerrar archivo
  
  switch(faseActual)
  {    
    case 1: // Edita el archivo Lista.txt y crea un archivo Rutina_R.txt
    if(b_cambioDeFase)
    { // Se ejecuta cuando cambiamos de faseActual 0 a faseActual 1    
      leerLista(); // Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]
      editarArchivoLista(); // Lee el archivo Lista.txt y asigna los valores cargados en la estructura
      crearNuevaRutina(); // Crea una nueva rutina dentro de la memoria SD
    }   
    break;  
    case 2: // Carga los valores almacenados en la estructura dentro del archivo correspondiente
    escribirArchivo(); // Carga los datos de la estructura dentro del archivo de la rutina correspondiente
    break;
  }
}
// ----------------------------------------------------------------------------------------------------------------
void rutinaDeEstados()
{ // Ejecuta las sub-rutinas acorde a la configuracion recibida desde el emisor 
  switch(myReceiveData.estado)
  {
    case 1: // Reproduccion en tiempo real
    rutinaTiempoReal();  
    break;
    case 2: // Recibir y guardar en memoria una rutina desde el emisor
    rutinaCrearRutina();
    break;
    case 4: // Cambiar el numero del sensor de la rutina indicada
    rutinaCambiarSensor();    
    break;
    case 5: // Borrar la rutina seleccionada  
    rutinaBorrarRutina();
    break;
    case 6: // Reproducir la rutina seleccionada  
    rutinaTiempoReal();
    break;
  } 
}
// ----------------------------------------------------------------------------------------------------------------
void rutinaTiempoReal()
{ // Gestiona la rutina acorde a la reproduccion de datos en tiempo real

  //d1 meñique
  //d2 anular
  //d3 medio
  //d4 indice
  //d5 pulgar

    // Movimiento del meñique
    //Serial.print("Meñique:");
    //Serial.print(myReceiveData.d1);
    //Serial.print("\t");
    PWMd1(myReceiveData.d1);

    // Movimiento del Medio
    //Serial.print("Medio:");
    //Serial.print(myReceiveData.d3);
    //Serial.print("\t");
    PWMd2(myReceiveData.d3);          

    // Movimiento del indice
    //Serial.print("Indice:");
    //Serial.print(myReceiveData.d4);
    //Serial.print("\t");
    PWMd3(myReceiveData.d4);          

    // Movimiento del pulgar
    //Serial.print("Pulgar:");
    //Serial.println(myReceiveData.d5);
    PWMd4(myReceiveData.d5);

    // Movimiento de elevacion
    PWMelevacion(myReceiveData.ay);

    // Movimiento de rotacion
    PWMrotacion(myReceiveData.gy);

    // Movimiento de flexion
    PWMflexion(myReceiveData.gx);
    //pruebaDeRotacion();

}
