// DECLARACION DE LIBRERIAS ****************************************************************************************
#include "FS.h"
#include "SPI.h"
#include "SD.h"
#include <esp_now.h>
#include <WiFi.h>

// DECLARACION DE PINES ********************************************************************************************
#define CS 5
#define MOSI 23
#define CLK 18
#define MISO 19

// CONSTANTES -----------------------------------------------------------------------------------------------------
#define DELAY_RECCONFIG 2000 // Duracion de la recepcion de la estructura myreceiveData


// ESTRUCTURAS ----------------------------------------------------------------------------------------------------
struct s_receiveData
{ // Estructura implementada para la transmision de datos
  short sensor, rutina, estado; // estado >> (1) tiempo real  ; (2) transmitir rutina ; (3) crear rutina ; (4) cambiar sensor ; (5) borrar rutina 
  float ax,ay,az,gx,gy,gz;
  float d1,d2,d3,d4,d5;
  bool b_esDato; // Booleano que indica si la estructura enviada son datos de lista o no
};

typedef struct s_receiveData s_ReceiveData; 

// DECLARACION DE VARIABLES *******************************************************************************************

File archivo;             // Referencia a los elementos dentro de la memoria
String sNombreArchivo;    // Nombre del archivo con el cual se desea trabajar (usar solo en las rutinas)
short faseActual = 0;     // fase de recepcion de datos (1)-configuracion ; (2)-datos ; (3)-configuracion
short faseAnterior = 0;   // valor de la fase del loop anterior
short lista[18];          // Contenido actualizado del archivo Lista.txt con valores en ASCII >> r1s1r2s2r3s3...r8s8r9s9 (0-48 ; 9-57)
bool b_recibiendo;        // indica si en este loop se esta recibiendo informacion mediante el protocolo ESPNOW
bool b_cambioDeFase;      // indica si en este loop se cambio de fase
bool b_omitirOrden=false; // omite todas las acciones hasta llegar a fase 0
long ultimoDatoRecibido;  // indica el momento en el cual se ha recibido el ultimo dato desde el transmisor

s_ReceiveData myReceiveData; // Instanciamos las estructuras para almacenar los datos de la lista para la memoria, 

// CABECERAS DE FUNCIONES ******************************************************************************************

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

// Configura el Esp32 del receptor para que pueda recibir datos normalmente
void iniciarESPNOW();

// Inicializa la memoria SD
void iniciarMemoriaSD();

// Lee el archivo Lista.txt y actualiza su contenido dentro del vector global >> lista[18]
void leerLista();

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
  iniciarESPNOW();
  iniciarMemoriaSD();  
} 

// ----------------------------------------------------------------------------------------------------------------
void loop() 
{  
  gestorDeFase(); // Modifica la fase de recepcion de datos actual 0-nada ; 1-config ; 2-dato ; 3-config
  gestorDeBooleanos(); // Actualiza el valor de los booleanos acorde a la estapa actual de cada orden

  if(b_recibiendo)
  { // Codigo a ejecutar cuando estamos recibiendo datos desde el emisor
    //Serial.println("loop() >> Llego un paquete de datos >> rutinaDeEstados()");
    b_recibiendo = false; // Con esto me aseguro que entre solo en los loops donde se reciben datos
    rutinaDeEstados(); // Ejecuta las sub-rutinas acorde a la configuracion recibida desde el emisor 
  }

  else
  { // Codigo a ejecutar por defecto - Lee los sensores conectados al receptor
  }

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
  SD.begin(CS);
  
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
  //Serial.println("rutinaDeEstados() >> Llego un paquete de datos");
/*   
  Serial.print("estado = "); Serial.println(myReceiveData.estado);
  Serial.print("rutina = "); Serial.println(myReceiveData.rutina);
  Serial.print("sensor = "); Serial.println(myReceiveData.sensor);
 */
  switch(myReceiveData.estado)
  {
    case 1: // Reproduccion en tiempo real
    //Serial.println("rutinaDeEstados() >> Reproducir en tiempo real >> rutinaTiempoReal()");
    rutinaTiempoReal();
    break;
    case 2: // Recibir y guardar en memoria una rutina desde el emisor
    //Serial.println("rutinaDeEstados() >> Transmitir rutina >> rutinaCrearRutina()");
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
  if(faseActual==2 && myReceiveData.b_esDato)
  { // Cuando estemos en la fase 2 cargamos los valores de la estructura
    // Metodo para ejecutar en simulacion (unity)
    Serial.println(myReceiveData.ax*(1));
    Serial.println(myReceiveData.ay*(1));
    Serial.println(myReceiveData.az*(1));
    Serial.println(myReceiveData.gx*(1));
    Serial.println(myReceiveData.gy*(1));
    Serial.println(myReceiveData.gz*(1));
    Serial.println(myReceiveData.d1*(1));
    Serial.println(myReceiveData.d2*(1));
    Serial.println(myReceiveData.d3*(1));
    Serial.println(myReceiveData.d4*(1));
    Serial.println(myReceiveData.d5*(1));
  }
}
