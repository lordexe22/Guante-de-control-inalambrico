/* 
 *
 ******************************************************************************************************************
 * PROTOCOLO DE MENSAJES POR EL PUERTO SERIAL
 *    SINTAXIS >> A-R-S
 *    A >> ACCION ESTABLECIDA (1 = transmitir en tiempo real) 
 *                            (2 = transmitir una rutina de la memoria (crear rutina en el receptor))  
 *                            (3 = crear una nueva rutina (solo en la memoria del transmisor))
 *                            (4 = cambiar el numero del sensor de alguna de las rutinas )
 *                            (5 = borrar rutina) (de la memoria del guante y del brazo)
 * 
 *    R >> Numero de la rutina seleccionada para los estados (2, 4, 5), en caso de ser innecesaria vale cero
 * 
 *    S >> Numero del sensor seleecionado para los estados (3, 4), en caso de ser innecesaria vale cero
 ****************************************************************************************************************** 
 *
 * SECUENCIA DE GUARDADO DE DATOS
 * ax - ay - az - gx - gy - gz - d1 - d2 - d3 - d4 - d5 
 *
 */
// estado >> (0) tiempo real  ; (1) transmitir rutina ; (2) crear rutina ; (3) cambiar sensor ; (4) borrar rutina 

// LIBRERIAS ------------------------------------------------------------------------------------------------------
#include <esp_now.h>          // Libreria de comandos de transmision para el protocolo ESPNOW
#include <WiFi.h>             // Libreria de comandos para configurar el ESP32
#include <Adafruit_MPU6050.h> // Libreria de instrucciones para el control del MPU6050
#include <Adafruit_Sensor.h>  
#include "FS.h"
#include "SPI.h"
#include "SD.h"

// DEFINICIONES DE PUERTOS ----------------------------------------------------------------------------------------
// Pines
#define Rx 16          // Puerto de transmision para la comunicacion UART con el Atmega328P
#define Tx 17          // Puerto de recepcion para la comunicacion UART con el Atmega328P
#define D1 36          // Puerto conectado al sensor del dedo meñique
#define D2 39          // Puerto conectado al sensor del dedo anular
#define D3 34          // Puerto conectado al sensor del dedo medio
#define D4 35          // Puerto conectado al sensor del dedo indice
#define D5 32          // Puerto conectado al sensor del dedo pulgar
#define LED_OK 12      // Puerto conectado a un led indicador
#define LED_ERROR 13   // Puerto conectado a un led indicador
#define CS 5
#define MOSI 23
#define CLK 18
#define MISO 19

// CONSTANTES -----------------------------------------------------------------------------------------------------
#define DELAY_TRANSCONFIG 200 // Cada cuanto tiempo se transmite la estructuras mySendConfig
#define DELAY_TRANSDATA 100 // Cada cuanto tiempo se transmite la estructura mySendData
#define DELAY_SENDCONFIG 2000 // Duracion de la transmision de la estructura mySendconfig

// Teclas para la visualizacion de datos de la estructura >> metodo showData()
char tecla;
bool ac_X = false, ac_Y = false, ac_Z = false, gy_X = false, gy_Y = false, gy_Z = false;
bool ti_X = false, ti_Y = false, ti_Z = false;
bool D_1 = false, D_2 = false, D_3 = false, D_4 = false, D_5 = false; 
bool Mode = false;

// Direccion MAC del ESP32 receptor del mensaje
uint8_t RxMACaddress[] = {0x78 , 0xE3 , 0x6D , 0x0A , 0x31 , 0x14}; 

// ESTRUCTURAS ----------------------------------------------------------------------------------------------------
struct s_sendData
{ // Estructura implementada para la transmision de datos
  float ax,ay,az,gx,gy,gz;
  float d1,d2,d3,d4,d5;
  bool b_esDato = true; // Booleano que indica si la estructura enviada son datos de lista o no
};

typedef struct s_sendData s_SendData; 
typedef s_SendData* ptr_SendData;     // Definicion del tipo de datos de los punteros que se transmitiran hacia el receptor

struct s_sendConfig
{ // Estructura que informa el estado actual del programa e indica las tareas a realizar
  // rutina es el numero de la rutina que se va a enviar, sensor es el numero del sensor al que queda emparentado la rutina que se va a enviar
  short sensor, rutina, estado; // estado >> (1) tiempo real  ; (2) transmitir rutina ; (3) crear rutina ; (4) cambiar sensor ; (5) borrar rutina 
  bool b_esDato = false; // Booleano que indica si la estructura enviada son datos de lista o no
};

typedef struct s_sendConfig s_SendConfig; 
typedef s_SendConfig* ptr_SendConfig;     // Definicion del tipo de datos de los punteros que se transmitiran hacia el receptor

// VARIABLES ------------------------------------------------------------------------------------------------------
Adafruit_MPU6050 mpu;     // Referencia a las madidas realizadas por el modulo MPU6050
File archivo;             // Referencia a los elementos dentro de la memoria
String str_nombreArchivo; // Nombre del archivo con el cual se desea trabajar (usar solo en las rutinas)
String str_puertoSerial;  // Almacena la cadena enviada desde el Atmega328P 

short estadoAnterior;     // Almacena el valor de la variable estado del loop anterior
short lista[18];          // Contenido actualizado del archivo Lista.txt con valores en ASCII >> r1s1r2s2r3s3...r8s8r9s9 (0-48 ; 9-57)

s_SendData mySendData;     // Instanciamos las estructuras para almacenar los datos de la lista para la memoria, 
s_SendConfig mySendConfig; // los datos y la configuracion a enviar al receptor.

bool finEstado = false; // Indica el final del estado a mediante la secuencia_A()
bool B_E1_P1 = false; // Booleano auxiliar que indica que debemos enviar la primera parte del codigo mySendConfig (2s)
bool B_E1_P2 = false; // Booleano auxiliar que indica que debemos enviar la segunda parte del codigo mySendData
bool B_E1_P3 = false; // Booleano auxiliar que indica que debemos enviar la tercera y ultima parte del codigo mySendConfig (2s)
bool B_E1_P4 = false; // Booleano auxiliar que indica que debemos enviar la secuencia que indica el final de la rutina
bool B_E2_P1 = false; // Booleano auxiliar que indica que debemos transmitir mySendConfig antes de enviar la rutina
bool B_E2_P2 = false; // Booleano auxiliar que indica que debemos transmitir la rutina solicitada
bool B_E2_P3 = false; // Booleano auxiliar que indica que debemos transmitir mySendConfig despues de enviar la rutina
bool B_E3_P1 = false; // Booleano auxiliar que indica que debemos actualizar el archivo lista
bool B_E3_P2 = false; // Booleano auxiliar que indica que debemos crear y llenar con datos el nuevo archivo de la rutina
bool B_E4_P1 = false; // Booleano auxiliar que indica que debemos cambiar el numero del sensor de la rutina seleccionada
bool B_E4_P2 = false; // Booleano auxiliar que indica que debemos transmitir la informacion sobre los cambios recientes
bool B_E5_P1 = false; // Booleano auxiliar que indica que debemos borrar una rutina y actualizar el archivo Lista.txt
bool B_E5_P2 = false; // Booleano auxiliar que indica que debemos transmitir la informacion sobre los cambios recientes

// CABECERAS DE FUNCIONES -----------------------------------------------------------------------------------------

// Actualiza el archivo lista.txt con el cambio y eliminar el archivo de la rutina seleccionado
void borrarRutina();

// Configura los puertos de entrada y salida del ESP32 
void configurarPuertos(); 

// Crea una nueva rutina >> estado 3 parte 2
void crearNuevaRutina();

// Edita el contenido del archivo lista.txt segun lo almacenado en la estructura mySendConfig
void editarArchivoLista();

// Parpadea el LED_ERROR cada 1 segundo (duty 10%). Se activa mientras la memoria no inicie correctamente
void errorA();

// Parpadea el LED_ERROR cada 1 segundo (duty 90%). Titila 3 veces. Se activa cuando el MPU6050 no inicia correctamente 
void errorB();

// Parpadea el LED_ERROR y led OK cada 1 segundo (duty 50%). Se activa si se intenta asignar el mismo sensor en dos rutinas 
void errorC();

// Escribe en una nueva linea, al final del archivo "nombre" el String "contenido" 
void escribirAlFinal(String nombre, String contenido);

// Controla los booleanos del programa de manera que trabajen ordenadamente con el programa
void gestorDeBooleanos();

// Lee el String str_puertoSerial y retorna el valor de la rutina de la cadena almacenada
short getRoutine();

// Lee el String str_puertoSerial y retorna el valor del sensor de la cadena almacenada  
short getSensor();

// Lee el String str_puertoSerial y retorna un valor entero acorde al estado de la cadena almacenada
short getState();

// Guarda los datos de la estructura mySendData dentro del archivo especificado
void guardarDatos(String nombreArchivo);

// Inicializa el protocolo ESPNOW 
void iniciarESPNOW(); 

// Inicializa la memoria SD
void iniciarMemoriaSD();

// Inicializa el modulo MPU6050 
void iniciarMPU6050();

// Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]
void leerLista();

// Lee el contenido del puerto serial y asigna la cadena al String str_puertoSerial
void leerPuertoSerial();

// Lee todos los sensores y los asigna a su respectivo campo dentro de la estructura instanciada mySendData o myListData
void leerSensores();

// Lectura y manipulacion de datos del puerto serial
void rutina_A();

// Gestiona el funcionamiento del programa en el modo 1 - (transmision de datos en tiempo real)
void rutina_B();

// Gestiona el funcionamiento del programa en el modo 3 - (crear rutina)
void rutina_C();

// Gestiona el funcionamiento del programa en el modo 4 - (cambiar el sensor de alguna rutina)
void rutina_D();

// Gestiona el funcionamiento del programa en el modo 5 - (borrar rutina)
void rutina_E();

// Gestiona el funcionamiento del programa en el modo 2 - (Transmitir rutina desde la memoria SD)
void rutina_F();

// Secuencia que indica que se ha recibido la cedena 0-0-0 del puerto serial, indica el final de una orden
void secuencia_A();

// Transmite los datos almacenados en la instancia de la estructura mySendData
void transmitirConfiguracion();

// Transmite los datos almacenados en la instancia de la estructura mySendData
void transmitirDatos();

// transmite los datos contenidos en el archivo de rutina especificado mediante estructuras mySendData
void transmitirRutina();

// Retorna un booleano que indica si el mensaje cumple con la sintaxis preestablecida, en caso contrario lo descarta
bool validarMensaje();

// ----------------------------------------------------------------------------------------------------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{ // callback que se ejecuta cada vez que el dato es enviado
   if(status == ESP_NOW_SEND_SUCCESS)
  {
    digitalWrite(LED_OK, HIGH);
    delay(5);
    digitalWrite(LED_OK, LOW);
  }
  else if(status == !ESP_NOW_SEND_SUCCESS)
  {
    digitalWrite(LED_ERROR, HIGH);
    delay(5);
    digitalWrite(LED_ERROR, LOW);
  }
}

// ----------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);
  Serial2.begin(9600,SERIAL_8N1,Rx,Tx); // Inicializacion del puerto seral para que reciba datos del Atmega328P
 
  configurarPuertos();  // Configuracion de los puertos del ESP32
  iniciarMemoriaSD();   // Inicializa la memoria SD
  iniciarESPNOW();      // Inicializa el protocolo ESPNOW
  iniciarMPU6050();     // Inicializa el modulo MPU6050
}

// ----------------------------------------------------------------------------------------------------------------
void loop() 
{ 
  rutina_A(); // Rutina para leer el puerto serial y armar la estructura mySendConfig
  rutina_B(); // Rutina para ejecutar el modo o estado 1 - transmitir en tiempo real
  rutina_C(); // Rutina para ejecutar el modo o estado 3 - crear nueva rutina
  rutina_D(); // Rutina para ejecutar el modo o estado 4 - cambiar el sensor de una rutina
  rutina_E(); // Rutina para ejecutar el modo o estado 5 - borrar una rutina
  rutina_F(); // Rutina para ejecutar el modo o estado 2 - enviar rutina desde la memoria SD 
  gestorDeBooleanos(); // Coordina los booleanos para que funcionen las rutinas
}

// ----------------------------------------------------------------------------------------------------------------
void borrarRutina()
{ // Actualiza el archivo lista.txt con el cambio y elimina el archivo de la rutina seleccionado

  short rutina = mySendConfig.rutina; // Numero de la rutina que deseamos 
  String nombreArchivo = "/Rutina_" + (String)rutina + ".txt"; // Nombre del archivo que quremos eliminar

  short valorLeido;
  
  SD.remove(nombreArchivo);

  archivo = SD.open("/Lista.txt",FILE_WRITE);
  lista[2*rutina-1] = 48;
  for(int i=0;i<9;i++)
  { // Escribe los datos del string dentro del archivo lista.txt
    archivo.print((char)lista[2*i]); 
    archivo.print('-');  
    archivo.println((char)lista[2*i+1]);
  } // fin >> for(i=0;i<9;i++)
  archivo.close();
}

// ----------------------------------------------------------------------------------------------------------------
void configurarPuertos()
{ // Configura los puertos de entrada y salida del ESP32 
  pinMode(D1,INPUT);
  pinMode(D2,INPUT);
  pinMode(D3,INPUT);
  pinMode(D4,INPUT);
  pinMode(D5,INPUT);
  pinMode(LED_OK,OUTPUT);
  pinMode(LED_ERROR,OUTPUT);
}

// ----------------------------------------------------------------------------------------------------------------
void crearNuevaRutina()
{ // Crea una nueva rutina y comienza a almacenar los datos de los sensores >> estado 3 parte 2

  bool prenderLed; 
  
  short rutina = mySendConfig.rutina; // Numero de la rutina que deseamos crear
  String nombreArchivo = "/Rutina_" + (String)rutina + ".txt"; // Nombre del archivo que quremos crear
  if(SD.exists(nombreArchivo)) errorC(); // Marcamos errorC si intentamos crear una rutina que ya existe 
  else
  { // En caso de que no exista la rutina
    while(B_E3_P2)
    { // Mientras este activo el booleano de adquisicion de datos para la nueva rutina      
      prenderLed = !prenderLed;
      if(prenderLed) digitalWrite(LED_OK,HIGH);
      else digitalWrite(LED_OK,LOW);
      leerSensores(); // Leemos los sensores
      guardarDatos(nombreArchivo); // Guardamos los datos leidos en la memoria
      rutina_A(); // Leemos el puerto serial para ver si se termino el envio de datos
      gestorDeBooleanos(); // Actualizamos los valores de estado (para evitar un loop infinito en esta funcion)
      delay(DELAY_TRANSDATA); // Esperamos DELAY_TRANSDATA para volver a leer los sensores
    } // fin >> while(B_E3_P2)
  } // fin >> else
}

// ----------------------------------------------------------------------------------------------------------------
void editarArchivoLista()
{ // Edita el contenido del archivo lista.txt segun lo almacenado en la estructura mySendConfig
  short sensor = mySendConfig.sensor + 48; // Valor leido del sensor (en ascii)
  short rutina = mySendConfig.rutina; // Valor leido de la rutina

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
void errorA()
{ // Parpadea el LED_ERROR cada 1 segundo (duty 10%). Se activa mientras la memoria no inicie correctamente
  digitalWrite(LED_ERROR,HIGH); delay(100);
  digitalWrite(LED_ERROR,LOW); delay(900);
}

// ----------------------------------------------------------------------------------------------------------------
void errorB()
{ // Parpadea el LED_ERROR cada 1 segundo (duty 90%). Titila 3 veces. Se activa cuando el MPU6050 no inicia correctamente
  digitalWrite(LED_ERROR,HIGH); delay(900);
  digitalWrite(LED_ERROR,LOW); delay(100);
}

// ----------------------------------------------------------------------------------------------------------------
void errorC()
{ // Parpadea el LED_ERROR y led OK cada 1 segundo 3 veces (duty 50%). Se activa si se intenta asignar el mismo sensor en dos rutinas 
  for (int i=0; i<3 ; i++)
  {
    digitalWrite(LED_ERROR,HIGH); 
    digitalWrite(LED_OK,HIGH); delay(400);
    digitalWrite(LED_ERROR,LOW); 
    digitalWrite(LED_OK,LOW); delay(400);
  }
}

// ----------------------------------------------------------------------------------------------------------------
void escribirAlFinal(String nombre, String contenido)
{ // Escribe en una nueva linea el String "contenido" al final del archivo "nombre" 
  archivo = SD.open(nombre, FILE_APPEND); // Abrimos el archivo en modo reescritura
  if(archivo) archivo.println(contenido); // Si se abrio correctamente, agregamos contenido al final del mismo
  else errorB(); // En caso contrario ejecutamos el error B (falla al abrir el archivo) >> LED_ERROR titila 3 veces
  archivo.close();
}

// ----------------------------------------------------------------------------------------------------------------
short getRoutine()
{ // Lee el String str_puertoSerial y retorna el valor de la rutina de la cadena almacenada
  int i = 0;
  String rutina = "";
  
  while(str_puertoSerial[2+i] != '-')
  { // avanza en el string hasta identifica el primer guion
    rutina += str_puertoSerial[2+i];
    i++;
  } // fin >> while(!salir)

  return rutina.toInt();
}

// ----------------------------------------------------------------------------------------------------------------
short getSensor()
{ // Lee el String str_puertoSerial y retorna el valor del sensor de la cadena almacenada  
  int i = 0;
  while(str_puertoSerial[2+i] != '-') i++;
  i++;
  return (str_puertoSerial.substring(2+i)).toInt();
}

// ----------------------------------------------------------------------------------------------------------------
short getState()
{ // Lee el contenido del puerto serial, asigna la cadena al String str_puertoSerial y retorna un valor entero acorde a la cadena recibida 
  // 1 >> Transmision en tiempo real
  // 2 >> Enviar rutina desde la memoria
  // 3 >> Crear nueva rutina
  // 4 >> Cambiar el numero del sensor de una rutina
  // 5 >> Eliminar la rutina seleccionada
  if(str_puertoSerial[1]=='-')
  {
    if(str_puertoSerial[0]=='1') return 1;
    else if(str_puertoSerial[0]=='2') return 2;
    else if(str_puertoSerial[0]=='3') return 3;
    else if(str_puertoSerial[0]=='4') return 4;
    else if(str_puertoSerial[0]=='5') return 5;
    else return 0;
  }
  else return 0;
}

// ----------------------------------------------------------------------------------------------------------------
void gestorDeBooleanos()
{ // Controla los booleanos del programa de manera que trabajen ordenadamente con el programa
  // 1 - Si el estado acaba de cambiar de 0 a 1, 2, 4 o 5 transmito por 2 segundos mySendConfig  
  // 2 - Pasados los 2 segundos, transmitimos mySendData hasta que cambie el estado de 1 otro valor (fin de la transmision)
  // 3 - Al acabar la transmision se envia por 2 segundos mas mySendConfig
  // 4 - El estado 3 actializa lista.txt en el primer loop y luego guarda los valores leidos en los sensores en un nuevo archivo
  // 5 - El estado 4 actualiza lista/txt en el primer loop y luego transmite mySendConfig por los siguientes 2 segundos al receptor

  static long instanteDeCambio; // Almacena el instante donde se ejecuta un evento cualquiera
  short estadoActual = mySendConfig.estado; 
  
  // ---------- Gestion del estado 1 ----------
  if(estadoActual!=estadoAnterior && estadoActual==1)
  { // Se ejecuta si acabamos de cambiar al estado 1 >> ejecuta una ves
    // Mientras B_E1_P1 este activo, debemos transmitir la estructura mySendConfig en rutina_B()
    instanteDeCambio = millis(); // Almacenamos el instante donde se dio el cambio de estado
    B_E1_P1 = true;
  } 
  else if(B_E1_P1 && (millis() - instanteDeCambio > DELAY_SENDCONFIG )) 
  { // Se ejecuta despues de un tiempo DELAY_SENDCONFIG desde la ejecucion del primer loop del estado 1
    // Mientras B_E1_P2 este activo, debemos transmitir la estructura mySendData en rutina_B() hasta que se presione la pantalla
    // y se lea por el puerto serial la cadena 0-0-0
    B_E1_P1 = false;
    B_E1_P2 = true;
  } 
  else if(estadoActual!=estadoAnterior && estadoAnterior==1)
  { // Se ejecuta cuando se preciona la pantalla para terminar de transmitir en tiempo real
    // Mientras B_E1_P3 este activo, debemos transmitir la estructura mySendConfig en rutina_B()
    B_E1_P2 = false;
    B_E1_P3 = true;
    instanteDeCambio = millis(); // Almacenamos el instante donde se dio el cambio de estado
  } 
  else if(B_E1_P3 && (millis() - instanteDeCambio > DELAY_SENDCONFIG ))
  { // Se ejecuta despues de un tiempo DELAY_SENDCONFIG desde la ejecucion del condicional anterior
    B_E1_P3 = false;
    B_E1_P4 = true;
  } 
  else if(B_E1_P4)
  { // B_E1_P4 sera true solo en un ciclo para realizar la secuencia indicando el fin de la transmision
    B_E1_P4 = false;
  }
  
  // ---------- Gestion del estado 2 ----------
  if(estadoActual!=estadoAnterior && estadoActual==2)
  { // Si acaba de cambiar el estado al modo 2 >> Enviamos mySendConfig
    Serial.println("gestorDeBooleanos >> B_E2_P1 = true;");
    B_E2_P1 = true;
    instanteDeCambio = millis(); // Almacenamos el instante donde se dio el cambio de estado    
  } 
  else if(B_E2_P1 && (millis() - instanteDeCambio > DELAY_SENDCONFIG))
  { // Se activa para iniciar la transmision de datos acorde a la rutina seleccionada (transmitimos mySendData)
    Serial.println("gestorDeBooleanos >> B_E2_P1 = false;");
    Serial.println("gestorDeBooleanos >> B_E2_P2 = true;");
    B_E2_P1 = false;
    B_E2_P2 = true;
    instanteDeCambio = millis(); // Almacenamos el instante donde se dio el cambio de estado    
    // B_E2_P2 se desactiva al finalizar la transmision de datos en la funcion correspondiente (al leer el archivo completo se hace estado=0)
  } 
  else if(B_E2_P2 && estadoActual==0)
  { // Se activa para iniciar la transmision de la configuracion de la estructura mySendData, indicando el fin de la rutina
    Serial.println("gestorDeBooleanos >> B_E2_P2 = false;");
    Serial.println("gestorDeBooleanos >> B_E2_P3 = true;");
    B_E2_P2 = false;
    B_E2_P3 = true;
    instanteDeCambio = millis(); // Almacenamos el instante donde se dio el cambio de estado        
    // B_E2_P2 se desactiva el finalizar la transmision de datos en la funcion correspondiente 
  } 
  else if(B_E2_P3 && estadoActual==0 && (millis() - instanteDeCambio > DELAY_SENDCONFIG))
  { // Para finalizar volvemos a enviar la estructura mySendConfig
    Serial.println("gestorDeBooleanos >> B_E2_P3 = false;");
    B_E2_P3 = false;
    secuencia_A();
  } 

  // ---------- Gestion del estado 3 ----------
  if(estadoActual!=estadoAnterior && estadoActual==3)
  { // Si acaba de cambiar el estado al modo 3 >> actualizamos el archivo Lista.txt
    B_E3_P1 = true;
  } // fin >> if(estadoActual!=estadoAnterior && estadoActual==3)

  else if(estadoActual==estadoAnterior && estadoActual==3 && B_E3_P1)
  { // Se activa para iniciar la creacion de una nueva rutina y la asignacion de los datos leidos por los sensores
    B_E3_P1 = false;
    B_E3_P2 = true;
  } // fin >> if(estadoActual!=estadoAnterior && estadoActual==3)

  else if(estadoActual!=estadoAnterior && B_E3_P2)
  { // Finaliza la creacion de la nueva rutina
    B_E3_P2 = false; 
  } // fin >> else if(estadoActual!=estadoAnterior && B_E3_P2)

  // ---------- Gestion del estado 4 ----------
  if(estadoActual!=estadoAnterior && estadoActual==4)
  { // Si acaba de cambiar el estado al modo 4 >> cambiamos el sensor asignado en Lista.txt
    instanteDeCambio = millis();
    B_E4_P1 = true;
  } // fin >> if(estadoActual!=estadoAnterior && estadoActual==4)

  else if(B_E4_P1 && estadoActual==estadoAnterior && estadoActual==4)
  { // Se activa para iniciar la transmision de la rutina actualizada (transmitimos mySendCOnfig por 2 segundos)
    B_E4_P1 = false;
    B_E4_P2 = true;
  } // fin >> else if(B_E4_P1 && estadoActual==estadoAnterior && estadoActual==4)

  else if(B_E4_P2 && (millis() - instanteDeCambio > DELAY_SENDCONFIG ))
  { // Finaliza la transmision de la rutina actualizada
    B_E4_P2 = false; 
    secuencia_A();
  } // fin >> else if(B_E4_P2 && (millis() - instanteDeCambio < t_transmitirConfiguracion ))

  // ---------- Gestion del estado 5 ----------
  if(estadoActual!=estadoAnterior && estadoActual==5)
  { // Si acaba de cambiar el estado al modo 5 >> eliminamos la rutina de Lista.txt y borramos el archivo que contenia la lista
    B_E5_P1 = true;
  } // fin >> if(estadoActual!=estadoAnterior && estadoActual==5)

  else if(B_E5_P1 && estadoActual==estadoAnterior && estadoActual==5)
  { // Se activa para iniciar la transmision de la rutina actualizada (transmitimos mySendCOnfig por 2 segundos)
    B_E5_P1 = false;
    B_E5_P2 = true;
    instanteDeCambio = millis();
  } // else if(B_E5_P1 && estadoActual==estadoAnterior && estadoActual==5)

  else if(B_E5_P2 && (millis() - instanteDeCambio > DELAY_SENDCONFIG ))
  { // Finaliza la transmision de la rutina actualizada
    B_E5_P2 = false; 
    secuencia_A();
  } // fin >> else if(B_E5_P2 && (millis() - instanteDeCambio < t_transmitirConfiguracion ))

  estadoAnterior = estadoActual; // Asignamos al estado anterior el actual antes de iniciar el proximo loop
}

// ----------------------------------------------------------------------------------------------------------------
void guardarDatos(String nombreArchivo)
{ // Guarda los datos de la estructura mySendData dentro del archivo especificado
  escribirAlFinal(nombreArchivo, (String)mySendData.ax);
  escribirAlFinal(nombreArchivo, (String)mySendData.ay);
  escribirAlFinal(nombreArchivo, (String)mySendData.az);
  escribirAlFinal(nombreArchivo, (String)mySendData.gx);
  escribirAlFinal(nombreArchivo, (String)mySendData.gy);
  escribirAlFinal(nombreArchivo, (String)mySendData.gz);
  escribirAlFinal(nombreArchivo, (String)mySendData.d1);
  escribirAlFinal(nombreArchivo, (String)mySendData.d2);
  escribirAlFinal(nombreArchivo, (String)mySendData.d3);
  escribirAlFinal(nombreArchivo, (String)mySendData.d4);
  escribirAlFinal(nombreArchivo, (String)mySendData.d5);
}

// ----------------------------------------------------------------------------------------------------------------
void iniciarESPNOW()
{// Inicializa el protocolo ESPNOW
  WiFi.mode(WIFI_STA); // Activamos el ESP32 como una estacion de Wi-Fi
  esp_now_init();      // Inicializa el protocolo ESPNOW 
  esp_now_register_send_cb(OnDataSent); // Ejecuta la funcion OnDataSent cuanto va a transmitir un dato

  // Registramos el ESP32 receptor
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, RxMACaddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Agregamos el otro ESP32        
  esp_now_add_peer(&peerInfo);   
}

// ----------------------------------------------------------------------------------------------------------------
void iniciarMemoriaSD()
{ // Inicializa la memoria SD
  while(!SD.begin(CS))
  { // Ejecutamos el errorA mientras la memoria no inicie correctamente (generalmente por no estar conectada)
    errorA();
  }  
   // Si no existe el archivo nombre.txt lo creamos y le escribimos un cero
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
  }
}

// ----------------------------------------------------------------------------------------------------------------
void iniciarMPU6050()
{ // Inicializa el modulo MPU6050 
  while(!mpu.begin())
  { // Inicializamos el dispositivo y marcamos error en caso fallido
      errorB();     
  } 
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Rango de medicion de los acelerometros
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // Velocidad maxima del giroscopio en 500°/s (mide 8.75)
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);    // Ancho de banda del filtro del modulo
}

// ----------------------------------------------------------------------------------------------------------------
void leerLista()
{ // Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]

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
void leerPuertoSerial()
{ // Lee el contenido del puerto serial y asigna la cadena al String str_puertoSerial
  str_puertoSerial = "";
  if(Serial2.available())
  {
    while(Serial2.available()) str_puertoSerial += (char)Serial2.read();
    Serial.print("Comando leido desde el puerto serial: "); Serial.println(str_puertoSerial);
  }
}

// ----------------------------------------------------------------------------------------------------------------
void leerSensores()
{ // Lee todos los sensores y los asigna a su respectivo campo dentro de la estructura instanciada mySendData 
  // Leemos y obtenemos los valores de los sensores
  sensors_event_t a, g, temp;   // a > acelerometro, g > giroscopio, temp > temperatura(no usar)
  mpu.getEvent(&a, &g, &temp);  // Se almacenan en el objeto tipo mpu

  if(mySendConfig.estado == 1 || mySendConfig.estado == 3)
  { //En modo 1 (leer en tiempo real), almacenamos los datos en la estructura mySendData (datos para transmitir)
    // Almacenamos los datos del MPU6050 en la estructura
    mySendData.ax = a.acceleration.x;    
    mySendData.ay = a.acceleration.y;    
    mySendData.az = a.acceleration.z;    
    mySendData.gx = g.gyro.x;    
    mySendData.gy = g.gyro.y;    
    mySendData.gz = g.gyro.z;    
    // Lectura de los potenciometros conectados al ESP32 (los dedos)
    mySendData.d1 = analogRead(D1);  
    mySendData.d2 = analogRead(D2);  
    mySendData.d3 = analogRead(D3);  
    mySendData.d4 = analogRead(D4);  
    mySendData.d5 = analogRead(D5);  
  } 
}

// ----------------------------------------------------------------------------------------------------------------
void rutina_A()
{ // Lectura y manipulacion de datos del puerto serial
  
  // LISTA DE TAREAS
  // 1 - Esta funcion solo se activa si se detecta un mensaje en el puerto serial
  // 2 - Lee el contenido del puerto y lo almacena en el string str_puertoSerial >> leerPuertoSerial()
  // 3 - Analiza el str_puertoSerial y evalua si el mensaje tiene el formato correcto o no entra en conflicto con alguna rutina ya creada
  //     Si el formato es correcto, se conserva el String, en caso contrario se borra >> validarMensaje()
  //     Ademas del formato, la orden no debe tener conflictos con las rutinas previamente creadas.
  // 4 - Si el formato es correcto, separa el estado, el numero de la rutina y el numero del sensor del mensaje 
  //     >> getState() >> getRoutine() >> getSensor()
  // 5 - Almacena los valores obtenidos del string en los campos de la estructura ya instanciada mySendConfig
  // 6 - Si la cadena recibida es 0-0-0 ejecuta secuancia_A() para visualizar mediante los leds dicho evento

  if(Serial2.available()) 
  { // Sentencia de lectura que se ejecuta si el ESP32 resive un mensaje por el puerto serial
    delay(10); // Necesario para que se pueda leer por completo el puerto serial
    leerPuertoSerial();    

    if(str_puertoSerial == "0-0-0")
    { // El comando 0-0-0 indica el final de una orden >> ejecutamos la secuencia_A para visualizar dicho evento
      finEstado = true;
      mySendConfig.estado = 0;
      mySendConfig.rutina = 0;
      mySendConfig.sensor = 0;      
    }
    else if(validarMensaje())
    { // Si el mensaje tiene el formato adecuado, almacenamos los valores a la estructura ya instanciada mySendConfig
      mySendConfig.estado = getState();
      mySendConfig.rutina = getRoutine();
      mySendConfig.sensor = getSensor();
    } 
    else
    { // Si el mensaje no es valido, reiniciamos los campos de la estructura y ejecutamos el errorC (orden rechazada)
      mySendConfig.estado = 0;
      mySendConfig.rutina = 0;
      mySendConfig.sensor = 0;
      errorC();
    }
    str_puertoSerial="";
  } // fin >> if(Serial.available())
}

// ----------------------------------------------------------------------------------------------------------------
void rutina_B()
{ // Gestiona el funcionamiento del programa en el modo 1 - (transmision de datos en tiempo real)

  // LISTA DE TAREAS
  // 1 - Esta funcion solo se activa si mySendConfig.estado == 1 (transmision de datos en tiempo real)
  // 2 - Lee el valor de todos los sensores cada 100 ms y lo almacena en la estructura ya instanciada mySendData >> leerSensores()
  // 3 - Al almacenar los datos en la estructura, lo primero que hace en enviar algunas veces la instancia de la
  //     estructura mySendConfig para avisarle al receptor que se esta a punto de enviar una serie de datos y como debe operar
  //     con los mismos >> transmitirConfiguracion(). Posteriormente se envian los datos de interes >> transmitirDatos(). Para
  //     finalizar se vuelve a enviar la configuracion seteada en 0-0-0 >> transmitirConfiguracion().
 
  if(mySendConfig.estado == 1 || B_E1_P3)
  { // Se activa si estamos en el modo de operacion 1
    static long instanteDeCambio=0; // Toma el valor del instante donde se envio el ultimo dato
    
    if(B_E1_P1)
    { // Envia a lo largo de (DELAY_SENDCONFIG) segundos la estructura mySendConfig cada (DELAY_TRANSCONFIG) segundos  
      if(millis() - instanteDeCambio > DELAY_TRANSCONFIG)
      { // Envia un paquete cada (DELAY_TRANSCONFIG) segundos    
        transmitirConfiguracion();
        instanteDeCambio = millis();     
      }   
    } 
    else if(B_E1_P2)
    { // Se ejecuta si ya paso el tiempo en el que se envia la configuracion y transmite mySendData cada (DELAY_TRANSDATA) segundos  
      if(millis() - instanteDeCambio > DELAY_TRANSDATA)
      { // Envia datos mientras mySendConfig.estado == 1 una ves cada (DELAY_TRANSDATA) segundos
        leerSensores();
        transmitirDatos();
        instanteDeCambio = millis();
      } 
    } 
    else if(B_E1_P3)
    { // Envia a lo largo de (DELAY_SENDCONFIG) segundos la estructura mySendConfig cada (DELAY_TRANSCONFIG) segundos  
      if(millis() - instanteDeCambio > DELAY_TRANSCONFIG)
      { // Envia un paquete cada (DELAY_TRANSCONFIG) segundos    
        transmitirConfiguracion();
        instanteDeCambio = millis();     
      } 
    } 
  } // fin >> if(mySendConfig.estado == 1)
  else if(B_E1_P4 && finEstado)
  {
    finEstado = false;
    secuencia_A();
  }
}

// ----------------------------------------------------------------------------------------------------------------
void rutina_C()
{ // Gestiona el funcionamiento del programa en al modo 3 - (crear rutina)

  // LISTA DE TAREAS
  // 1 - Esta funcion solo estara activa si mySendConfig.estado=3
  // 2 - Lee el archivo Lista y crea un string por cada rutina que contiene el conjunto (n°rutina-n°sensor)
  // 3 - Cambia el conjunto asignado en el archivo lista
  // 4 - Crea un nuevo archivo con nombre Rutina_(n°rutina)
  // 5 - Lee los sensores y agrega los valores al archivo recien creado en el siguiente orden:
  //     ax-ay-az-gx-gy-gz-d1-d2-d3-d4-d5 separandolos por \n (un valor por renglon) 
  
  if(B_E3_P1)
  { // Fase uno del estado 3 >> actualizar el listado de rutinas
    editarArchivoLista(); // Lee el archivo y asigna los valores enviado por el puerto serial
  } // fin >> if(B_E3_P1)
  else if(B_E3_P2)
  { // En la fase, creamos el nuevo archivo de la rutina y le asignamos los valores leidos en los sensores
    crearNuevaRutina(); // Crea la rutina y la llena de datos leyendo los sensores (funcion ciclica)
    secuencia_A(); // Indicamos el final de la creacion de la rutina
  } // fin >> else if(B_E3_P2)
}

// ----------------------------------------------------------------------------------------------------------------
void rutina_D()
{ // Gestiona el funcionamiento del programa en al modo 4 - (cambiar el sensor de alguna rutina)

  // LISTA DE TAREAS
  // 1 - Esta funcion solo estara activa si mySendConfig.estado=4
  // 2 - Lee el archivo Lista y crea un string por cada rutina que contiene el conjunto (n°rutina-n°sensor)
  // 3 - Cambia el conjunto asignado en el archivo lista
  // 4 - Transmite durante un periodo de tiempo la estructura mySendConfig para tambien actualizar el cambio en el receptor
  
  if(B_E4_P1)
  { // Actualiza la lista agregando el nuevo valor del sensor con los valores leidos del puerto serial
    editarArchivoLista();
  } // fin >> if(B_E4_P1)
  else if(B_E4_P2)
  { // Transmitimos la estructura mySendConfig durante el tiempo que dura activo B_E4_P2
    transmitirConfiguracion();
    delay(DELAY_TRANSCONFIG);
  } // fin >> else if(B_E4_P2)
}

// ----------------------------------------------------------------------------------------------------------------
void rutina_E()
{ // Gestiona el funcionamiento del programa en al modo 5 - (borrar rutina)

  // LISTA DE TAREAS
  // 1 - Esta funcion solo estara activa si mySendConfig.estado=5
  // 2 - Lee el archivo Lista y crea un string por cada rutina que contiene el conjunto (n°rutina-n°sensor)
  // 3 - Cambia el conjunto asignado en el archivo lista y el valor del sensor pasa a ser cero
  // 4 - Borra el archivo en el cual se encontraba la rutina
  // 5 - Transmite durante un periodo de tiempo la estructura mySendConfig para tambien actualizar el cambio en el receptor

  if(B_E5_P1)
  { // Actualizamos el archivo Lista.txt y borramos la rutina designada en mySendConfig
    borrarRutina();
  } // fin >> if(B_E5_P1)
  else if(B_E5_P2)
  { // Transmitimos mySendConfig para atualizar los mismos campos en el receptor mientras este activo B_E5_P2
    transmitirConfiguracion();
    delay(DELAY_TRANSCONFIG);
  } // fin >> else if(B_E5_P2)
}

// ----------------------------------------------------------------------------------------------------------------
void rutina_F()
{ // Gestiona el funcionamiento del programa en el modo 2 - (Transmitir rutina desde la memoria SD)

  // LISTA DE TAREAS
  // 1 - Esta funcion solo estara activa si mySendConfig.estado=2
  // 2 - En el primer loop debe identificar la existencia de la rutina que deseamos transmitir y en caso de que exista
  //     enviamos durante algunas veces la estructura de datos mySendConfig
  // 3 - Despues de enviar la configuracion, procedemos a leer el archivo y almacenar los datos en la estructura mySendData
  //     finalmente enviamos el paquete y repetimos el paso 3 hasta finalizar la lectura del archivo
  // 4 - Para finalizar esta rutina, volvemos e enviar algunas veces el paquete de datos mySendConfig

  static long ultimoEnvio;
  
  if(B_E2_P1 && millis() - ultimoEnvio > DELAY_TRANSCONFIG) 
  { // Enviamos cada DELAY_TRANSCONFIG un pack del tipo mySendConfig
    Serial.println("rutina_F() >> transmitirConfiguracion(); PT1");
    transmitirConfiguracion();
    ultimoEnvio = millis();
  } 
  else if(B_E2_P2)
  { // Cargamos los datos en mySendData leyendo el archivo de la rutina especificado y lo transmitimos
    Serial.println("rutina_F() >> transmitirRutina()");
    transmitirRutina();
    ultimoEnvio = millis();
  }
  else if(B_E2_P3 && millis() - ultimoEnvio > DELAY_TRANSCONFIG)
  { // Enviamos cada DELAY_TRANSCONFIG un pack del tipo mySendConfig
    Serial.println("rutina_F() >> transmitirConfiguracion(); PT2");
    transmitirConfiguracion();
    ultimoEnvio = millis();
  } 
}

// ----------------------------------------------------------------------------------------------------------------
void secuencia_A()
{ // Secuencia que indica que se ha recibido la cedena 0-0-0 del puerto serial, indica el final de una orden
    digitalWrite(LED_OK,HIGH); delay(100);
    digitalWrite(LED_ERROR,HIGH); digitalWrite(LED_OK,LOW); delay(100);
    digitalWrite(LED_ERROR,LOW); digitalWrite(LED_OK,HIGH); delay(100);
    digitalWrite(LED_ERROR,HIGH); digitalWrite(LED_OK,LOW); delay(100);
    digitalWrite(LED_ERROR,LOW); digitalWrite(LED_OK,HIGH); delay(100);
    digitalWrite(LED_ERROR,HIGH); digitalWrite(LED_OK,LOW); delay(100);
    digitalWrite(LED_ERROR,LOW); digitalWrite(LED_OK,HIGH); delay(100);
    digitalWrite(LED_OK,LOW);    
}

// ----------------------------------------------------------------------------------------------------------------
void transmitirConfiguracion()
{ // Transmite los datos almacenados en la instancia de la estructura mySendData
  // Enviamos la estructura de datos leida mediante el protocolo ESPNOW
  esp_err_t result = esp_now_send(0, (uint8_t *) &mySendConfig, sizeof(mySendConfig));
}

// ----------------------------------------------------------------------------------------------------------------
void transmitirDatos()
{ // Transmite los datos almacenados en la instancia de la estructura mySendData
  // Enviamos la estructura de datos leida mediante el protocolo ESPNOW
  esp_err_t result = esp_now_send(0, (uint8_t *) &mySendData, sizeof(mySendData));
} 

// ----------------------------------------------------------------------------------------------------------------
void transmitirRutina()
{ // transmite los datos contenidos en el archivo de rutina especificado mediante estructuras mySendData

  bool cValido, cInvalido, cargarDato;
  short rutina = mySendConfig.rutina; // Numero de la rutina que deseamos leer
  String nombreArchivo = "/Rutina_" + (String)rutina + ".txt"; // Nombre del archivo que queremos crear
  
  String dato="";
  char valorLeido;
  short selector=1; // segun su valor, se le asignara el valor almacenado en dato a un campo concreto de mySendData
  
  // 1-ax ; 2-ay ; 3-az ; 4-gx ; 5-gy ; 6-gz ; 7-D1 ; 8-D2 ; 9-D3 ; 10-D4 ; 11-D5
  
  archivo = SD.open(nombreArchivo,FILE_READ);
  while(archivo.available()) // Ejecuta el ciclo mientras queden datos por leer dentro del archivo
  { // leemos hasta el final del archivo  
    valorLeido = archivo.read();
    // Verificamos que el valor leido sea un caracter valido
    if((valorLeido >= 48 && valorLeido <= 57) || valorLeido=='-' || valorLeido=='.' ) { cValido = true; cInvalido = false; }
    else{ cValido = false; cInvalido = true; }

    if(cValido) 
    { // Si el dato es valido lo almacenamos en el string y habilitamos el guardado de datos
      dato += valorLeido;  
      cargarDato=true;
    }     
    else if(cInvalido)
    { // Si leemos un dato invalido, guardamos el contenido leido hasta ahora en algun miembro de la estructura mySendData
      if(selector == 1 && cargarDato)       { mySendData.ax = dato.toInt(); selector = 2;  dato=""; }
      else if(selector == 2 && cargarDato)  { mySendData.ay = dato.toInt(); selector = 3;  dato=""; }
      else if(selector == 3 && cargarDato)  { mySendData.az = dato.toInt(); selector = 4;  dato=""; }
      else if(selector == 4 && cargarDato)  { mySendData.gx = dato.toInt(); selector = 5;  dato=""; }
      else if(selector == 5 && cargarDato)  { mySendData.gy = dato.toInt(); selector = 6;  dato=""; }
      else if(selector == 6 && cargarDato)  { mySendData.gz = dato.toInt(); selector = 7;  dato=""; }
      else if(selector == 7 && cargarDato)  { mySendData.d1 = dato.toInt(); selector = 8;  dato=""; }
      else if(selector == 8 && cargarDato)  { mySendData.d2 = dato.toInt(); selector = 9;  dato=""; }
      else if(selector == 9 && cargarDato)  { mySendData.d3 = dato.toInt(); selector = 10; dato=""; }
      else if(selector == 10 && cargarDato) { mySendData.d4 = dato.toInt(); selector = 11; dato=""; }
      else if(selector == 11 && cargarDato) { mySendData.d5 = dato.toInt(); selector = 1;  dato=""; transmitirDatos(); }   
      cargarDato=false;     
    }      
  } // fin >> while(archivo.available()) 
  archivo.close();
  mySendConfig.estado = 0;
}
// ----------------------------------------------------------------------------------------------------------------
bool validarMensaje()
{ // Retorna un booleano que indica si el mensaje cumple con la sintaxis preestablecida, si no cumple con la sintxis o
  // entra en conflicto con alguna rutina ya creada, se descarta el mensaje

  // Cargamos los valores leidos del mensaje del puerto serial
  short sensor = getSensor();
  short rutina = getRoutine();
  short estado = getState();
  short sensorASCII = sensor+48;

  // Contamos los guiones y evaluamos los caracteres para descartar contenido basura
  short tam = str_puertoSerial.length(); // Obtenemos el tamaño de la cadena
  short guiones = 0; // El mensaje debe tener dos guiones
    
  for(short i=0 ; i<=tam ; i++)
  { // Evalua y cuenta los guiones caracter a caracter ademas de verificar que los caracteres de la cadena sean validos
    if(str_puertoSerial[i]=='-') guiones++;
    if( str_puertoSerial[i]!='-' && str_puertoSerial[i]!='0' && str_puertoSerial[i]!='1' && str_puertoSerial[i]!='2'  && 
        str_puertoSerial[i]!='3' && str_puertoSerial[i]!='4' && str_puertoSerial[i]!='5' && str_puertoSerial[i]!='6'  &&
        str_puertoSerial[i]!='7' && str_puertoSerial[i]!='8' && str_puertoSerial[i]!='9' && str_puertoSerial[i]!='\n' &&
        str_puertoSerial[i]!='\0') return false;    
  } // fin >>   for(int i=0 ; i<=tam ; i++)

  if(guiones != 2)
  { // Evalua el numero de guiones leidos que deberia ser igual a dos
    str_puertoSerial = "";
    return false;  
  } // fin >> if(guiones != 2)

  // Evaluamos si el mensaje entra en conflicto con alguna de las rutinas creadas
  leerLista(); // Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]

  // Si se quiere transmitir una rutina (estado 2) que no existe
  if(estado==2 && lista[2*rutina-1]==48) return false;
  
  // Si se quiere crear una rutina (estado 3) con un numero de rutina que ya existe
  if(estado==3 && lista[2*rutina-1]!=48) return false; 

  // Si se quiere crear una rutina (estado 3) con un numero de sensor que ya esta en uso
  if(estado==3 && (lista[1] == sensorASCII || lista[3] == sensorASCII || lista[5] == sensorASCII || lista[7] == sensorASCII || 
  lista[9] == sensorASCII || lista[11] == sensorASCII || lista[13] == sensorASCII || lista[15] == sensorASCII || lista[17] == sensorASCII)) 
  return false;

  // Si se quiere transmitir una rutina (estado 2) que no ha sido creada aun
  if(estado==2 && lista[2*rutina-1]==48) return false; 

  // Si se quiere cambiar el numero del sensor (estado 4) por uno que ya este en uso
  if(estado==4 && (lista[1] == sensorASCII || lista[3] == sensorASCII || lista[5] == sensorASCII || lista[7] == sensorASCII || 
  lista[9] == sensorASCII || lista[11] == sensorASCII || lista[13] == sensorASCII || lista[15] == sensorASCII || 
  lista[17] == sensorASCII)) return false;  

  // Si se quiere cambiar el numero del sensor de una rutina que no ha sido creada
  if(estado==4 && lista[2*rutina-1]==48) return false;

  // Si se quiere borrar una rutina (estado 5) que no existe
  if(estado==5 && lista[2*rutina-1]==48) return false; 

  // Si llego aca cumplio todas las condiciones y la orden es valida
  return true; 
}
