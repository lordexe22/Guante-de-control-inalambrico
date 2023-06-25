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

// LIBRERIAS ------------------------------------------------------------------------------------------------------
#include <Adafruit_GFX.h>      // Libreria de graficos
#include <Adafruit_TFTLCD.h>   // Libreria de LCD
#include <TouchScreen.h>       // Libreria del TouchScreen

#include <Fonts/FreeSerifBoldItalic12pt7b.h> // Incluye una fuente para el texto que se mostrara en pantalla

#if defined(__SAM3X8E__)
  #undef __FlashStringHelper::F(string_literal)
  #define F(string_literal) string_literal
#endif

// DEFINICIONES DE PUERTOS ----------------------------------------------------------------------------------------
#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

// CONSTANTES -----------------------------------------------------------------------------------------------------
// Es RGB 565: http://www.barth-dev.de/online/rgb565-color-picker/

// CONSTANTES DE LA PANTALLA LCD
#define RES_V 240 // pixeles verticales
#define RES_H 320 // pixeles horizontales

// CONSTANTES DEL TACTIL
#define TS_MINX 150  //150
#define TS_MINY 120  //120
#define TS_MAXX 920  //920
#define TS_MAXY 940  //940
#define MINPRESSURE 10
#define MAXPRESSURE 500 //1000

// CONSTANTES DE USO COMUN 
#define CUADRO 40  // = RES_V / 6 = RES_H / 8
#define ESPERA 500
// CONSTANTES REEDEFINIDAS MANUALMENTE 
#define TS_MINX_ 100 //195
#define TS_MAXX_ 1200 //870  >> 1200 anda bien
#define TS_MINY_ 250 //115
#define TS_MAXY_ 900 //860


// VARIABLES ------------------------------------------------------------------------------------------------------
// CONFIGURACÓN DE LA PANTALLA TFT
#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300); // Instancia el Touch Screen

// CONFIGURACION DEL LCD DE LA PANTALLA
#define LCD_CS A3   
#define LCD_CD A2   
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4 //opcional
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET); // Instancia LCD

bool setRutina=true;
bool setSensor=false;

short sensor=0;
short rutina=0;
short sensorA; // Valor anterior del sensor (se usa para sobreescribir el valor de la pantalla)
short rutinaA; // Valor anterior de la rutina (se usa para sobreescribir el valor de la pantalla)
short estadoActual; // Listado de estados posibles de funcionamiento para la pantalla tactil
// 0 >> Estado por defecto, no hay ningun modo activo
// 1 >> Modo TRANSMICION EN TIEMPO REAL activo
// 2 >> Modo CREAR NUEVA RUTINA activo
// 3 >> Modo ELIMINAR RUTINA activo
// 4 >> Modo CAMBIAR SENSOR activo
// 5 >> Modo TRANSMITIR RUTINA activo

String str_msgError="";
String str_msgErrorA="";

String str_zona; // Representa las distintas zonas de la pantalla (6 filas x 8 columnas)  
//  A1 A2 A3 A4 A5 A6 A7 A8
//  B1 B2 B3 B4 B5 B6 B7 B8
//  C1 C2 C3 C4 C5 C6 C7 C8
//  D1 D2 D3 D4 D5 D6 D7 D8
//  E1 E2 E3 E4 E5 E6 E7 E8
//  F1 F2 F3 F4 F5 F6 F7 F8

long ultimaLectura = 0; // Contiene el instante donde se preciono por ultima vez la pantalla tactil

// CABECERAS DE FUNCIONES -----------------------------------------------------------------------------------------

// Diseño de los botones de la pantalla
void dibujarBoton(short boton, short x0 ,short y0);

// Diseño de los iconos de la pantalla
void dibujarIcono(short icono, short x0 ,short y0);

// Gestiona el comportamiento del programa en funcion de los botones pulsados y envia los mensajes por puerto serial acorde
void gestorDeOperaciones();

// Actualiza los botones, iconos y texto de la pantalla a medida que se va presionando la misma
void gestorDePantallas(String str_zona);

// Inicializacion de la pantalla TFT
void inicializarPantalla();

// Genera un evento de lectura del TouchScreen con el cual podremos obtener las coordenadas y la presion
TSPoint leerTouchScreen(TouchScreen ts);

// Carga la pantalla y textos del menu principal
void pantallaPrincipal();

// Identifica la zona presionada sobre la pantalla, estas pueden verse usando la funcion cuadricula, x e y son las 
// coordenadas leidas del TS, retorna un string identificando la fila y la columna presionada 
// (NOTA) ESTA FUNCION SE DEBE USAR UNICAMENTE CON LA CERTEZA DE QUE SE PULSO LA PANTALLA 
String zona(short x, short y);

// ----------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(9600);
  inicializarPantalla();
  pantallaPrincipal();
}

// ----------------------------------------------------------------------------------------------------------------
void loop() 
{ 
  TSPoint p = leerTouchScreen(ts); // Leemos la presion sobre la pantalla
     
  if (p.z > ts.pressureThreshhold && (millis() - ultimaLectura > ESPERA) ) 
  { // Actualizamos la pantalla y enviamos informacion si detectamos que se ha presionado la misma
    ultimaLectura = millis();
    str_zona = zona((short)p.x, (short)p.y);
    gestorDePantallas(str_zona);
    gestorDeOperaciones();
  }
}

// ----------------------------------------------------------------------------------------------------------------
void dibujarBoton(short boton, short x0 ,short y0)
{ // dibuja en la pantalla uno de los diseños de los botones de la pantalla dentro de la zona establecida
  // boton(1)  >> boton de transmision en tiempo real (OFF) -  boton(2)  >> boton de transmision en tiempo real (ON)
  // boton(3)  >> boton de creacion de nueva rutina (OFF)   -  boton(4)  >> boton de creacion de nueva rutina (ON) 
  // boton(5)  >> boton de eliminacion de rutina (OFF)      -  boton(6)  >> boton de eliminacion de rutina (ON)
  // boton(7)  >> boton de cambio de sensor (OFF)           -  boton(8)  >> boton de cambio de sensor (ON)
  // boton(9)  >> boton de transmision de rutina (OFF)      -  boton(10) >> boton de transmision de rutina (ON)
  // boton(11) >> boton de confirmar accion (OFF)           -  boton(12) >> boton de confirmar accion (ON)

  // boton(13) numero

  // color >> RRRR RGGG GGGB BBBB
  short borde = 2;
  short redondeo = 5;
   
  switch (boton)
  {
    case 1: // Boton de transmision en tiempo real en OFF (OK)
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(128,0,0)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(64,0,0)); // Fondo OFF
    break;
    case 2: // Boton de transmision en tiempo real en ON (OK)
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(128,0,0)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(200,0,0)); // Fondo ON    
    break;
    case 3: // Boton de crear nueva rutina en OFF (OK)
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(0,64,0)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(0,32,0)); // Fondo OFF        
    break;
    case 4: // Boton de crear nueva rutina en ON (OK)
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(0,128,32)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(0,128,0)); // Fondo ON            
    break;
    case 5: // Boton de eliminacion de rutina en OFF (OK)
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(0,0,128)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(0,0,64)); // Fondo OFF            
    break;
    case 6: // Boton de eliminacion de rutina en ON (OK)
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(0,0,128)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(0,0,200)); // Fondo ON            
    break; 
    case 7: // Boton de cambio de sensor en OFF (OK)
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(170,140,0)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(120,90,0)); // Fondo OFF            
    break;
    case 8: // Boton de cambio de sensor en ON (OK)
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(170,140,0)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(230,180,50)); // Fondo ON            
    break;
    case 9: // Boton de transmision de rutina en OFF
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(170,0,140)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(120,0,90)); // Fondo OFF            
    break;
    case 10: // Boton de transmision de turina en ON
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(170,0,140)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(230,50,180)); // Fondo ON            
    break;
    case 11: // Boton de confirmacion en OFF
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(120,120,120)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(80,80,80)); // Fondo OFF            
    break;  
    case 12: // Boton de confirmacion en ON
    tft.fillRoundRect(x0, y0, 2*CUADRO, 2*CUADRO, redondeo, tft.color565(120,120,120)); // Borde
    tft.fillRoundRect(x0+borde, y0+borde, 2*CUADRO-2*borde, 2*CUADRO-2*borde, 5, tft.color565(180,180,180)); // Fondo OFF            
    break;  
    case 13: // Numero 1
    tft.fillCircle(x0, y0, 0.5*CUADRO-2, tft.color565(180,55,0));
    tft.fillCircle(x0, y0, 0.5*CUADRO-5, tft.color565(0,0,0));
  }
}

// ----------------------------------------------------------------------------------------------------------------
void dibujarIcono(short icono, short x0 ,short y0)
{ // Diseño de los iconos de la pantalla
    short radio1 = 20;
    short radio2 = 15;
    short alto = 40;
    short ancho = 30;   
  if(icono == 1)
  { // >> Modo tiempo real >> reloj
    tft.fillCircle(x0, y0, radio1, tft.color565(10,10,50));
    tft.fillCircle(x0, y0, radio2, tft.color565(120,120,120));
    for(int i=-1;i<2;i++){ tft.drawLine(x0+i,y0-14,x0+i,y0,tft.color565(0,0,0)); }
    for(int i=-1;i<2;i++){ tft.drawLine(x0,y0+i,x0+9,y0+i,tft.color565(0,0,0)); }
  }
  else if(icono == 2)
  { // Modo crear nueva rutina >> papel con icono +
    //papel
    tft.fillRect(x0-ancho/2,y0-alto/2,ancho,alto,tft.color565(160,160,160));
    //escrituras
    tft.drawLine(x0-ancho/3,y0-alto/3,x0+ancho/3,y0-alto/3,tft.color565(0,0,0));
    tft.drawLine(x0-ancho/3,y0-alto/3+5,x0+ancho/3-5,y0-alto/3+5,tft.color565(0,0,0));
    tft.drawLine(x0-ancho/3,y0-alto/3+10,x0+ancho/3-3,y0-alto/3+10,tft.color565(0,0,0));
    tft.drawLine(x0-ancho/3,y0-alto/3+15,x0+ancho/3,y0-alto/3+15,tft.color565(0,0,0));
    //cruz azul
    tft.fillRect(x0+ancho/4-8,y0+alto/4,21,5,tft.color565(0,75,150));      
    tft.fillRect(x0+ancho/4,y0+alto/4-8,5,21,tft.color565(0,75,150));      
    tft.fillRect(x0+ancho/4-8+2,y0+alto/4+1,17,3,tft.color565(0,90,200));      
    tft.fillRect(x0+ancho/4+1,y0+alto/4-8+2,3,17,tft.color565(0,90,200));      
  }
  else if(icono == 3)
  { // Modo eliminar rutina >> papel con icono X
    //papel
    tft.fillRect(x0-ancho/2,y0-alto/2,ancho,alto,tft.color565(160,160,160));
    //escrituras
    tft.drawLine(x0-ancho/3,y0-alto/3,x0+ancho/3,y0-alto/3,tft.color565(0,0,0));
    tft.drawLine(x0-ancho/3,y0-alto/3+5,x0+ancho/3-5,y0-alto/3+5,tft.color565(0,0,0));
    tft.drawLine(x0-ancho/3,y0-alto/3+10,x0+ancho/3-3,y0-alto/3+10,tft.color565(0,0,0));
    tft.drawLine(x0-ancho/3,y0-alto/3+15,x0+ancho/3,y0-alto/3+15,tft.color565(0,0,0));
    //cruz roja
    for(int i=-3; i<4; i++){ tft.drawLine(x0+ancho/6+i-5,y0+alto/4,x0+ancho/6+i+12,y0+alto/4+12,tft.color565(120,15,15)); }
    for(int i=-3; i<4; i++){ tft.drawLine(x0+ancho/6+i-5,y0+alto/4+12,x0+ancho/6+12+i,y0+alto/4,tft.color565(120,15,15)); }
    for(int i=-1; i<2; i++){ tft.drawLine(x0+ancho/6+i-5+2,y0+alto/4+2,x0+ancho/6+i+12-2,y0+alto/4+12-1,tft.color565(180,15,15)); }
    for(int i=-1; i<2; i++){ tft.drawLine(x0+ancho/6+i-5+2,y0+alto/4+12-1,x0+ancho/6+12+i-2,y0+alto/4+2,tft.color565(180,15,15)); }
  }
  else if(icono == 4 && estadoActual == 0)
  { // Modo cambiar sensor >> icono sensor
    tft.fillCircle(x0, y0, 30, tft.color565(70,30,90)); // Color violeta
    tft.fillCircle(x0, y0, 25, tft.color565(120,90,0)); // Fondo amarillo oscuro
    tft.fillCircle(x0, y0, 20, tft.color565(70,30,90)); // Color violeta
    tft.fillCircle(x0, y0, 15, tft.color565(120,90,0)); // Fondo amarillo oscuro
    tft.fillCircle(x0, y0, 5, tft.color565(70,30,90)); // Color violeta
    tft.fillTriangle(x0, y0-5, x0-25, y0-35, x0+25, y0-35, tft.color565(120,90,0)); // Fondo amarillo oscuro
    tft.fillTriangle(x0, y0+5, x0-25, y0+35, x0+25, y0+35, tft.color565(120,90,0)); // Fondo amarillo oscuro
  }
  else if(icono == 4 && estadoActual == 4)
  { // Modo cambiar sensor >> icono sensor
    tft.fillCircle(x0, y0, 30, tft.color565(70,30,90)); // Color violeta
    tft.fillCircle(x0, y0, 25, tft.color565(230,180,50)); // Fondo amarillo claro
    tft.fillCircle(x0, y0, 20, tft.color565(70,30,90)); // Color violeta
    tft.fillCircle(x0, y0, 15, tft.color565(230,180,50)); // Fondo amarillo claro
    tft.fillCircle(x0, y0, 5, tft.color565(70,30,90)); // Color violeta
    tft.fillTriangle(x0, y0-5, x0-25, y0-35, x0+25, y0-35, tft.color565(230,180,50)); // Fondo amarillo claro
    tft.fillTriangle(x0, y0+5, x0-25, y0+35, x0+25, y0+35, tft.color565(230,180,50)); // Fondo amarillo claro
  }
  else if(icono == 5)
  { // Modo enviar rutina 
    short alto = 40;
    short ancho = 30;   
    //papel
    tft.fillRect(x0-ancho/2,y0-alto/2,ancho,alto,tft.color565(160,160,160));
    //escrituras
    tft.drawLine(x0-ancho/3,y0-alto/3,x0+ancho/3,y0-alto/3,tft.color565(0,0,0));
    tft.drawLine(x0-ancho/3,y0-alto/3+5,x0+ancho/3-5,y0-alto/3+5,tft.color565(0,0,0));
    tft.drawLine(x0-ancho/3,y0-alto/3+10,x0+ancho/3-3,y0-alto/3+10,tft.color565(0,0,0));
    tft.drawLine(x0-ancho/3,y0-alto/3+15,x0+ancho/3,y0-alto/3+15,tft.color565(0,0,0));
    //flecha verde
    tft.fillRect(x0+ancho/5-5,y0+alto/4+3,15,5,tft.color565(15,80,15)); 
    tft.fillTriangle(x0+ancho/3,y0+alto/4+10,x0+ancho/3,y0+alto/4,x0+ancho/3+10,y0+alto/4+5,tft.color565(15,80,15)); 
    tft.fillRect(x0+ancho/5-4,y0+alto/4+4,13,3,tft.color565(15,130,15)); 
    tft.fillTriangle(x0+ancho/3+2,y0+alto/4+8,x0+ancho/3+2,y0+alto/4+2,x0+ancho/3+6,y0+alto/4+5,tft.color565(20,130,20)); 
  }
  else if(icono==6 && estadoActual == 0)
  { // Iniciar operacion seleccionada
    tft.fillCircle(x0, y0, radio1+10, tft.color565(100,10,10));
    tft.fillTriangle(x0-12,y0-10,x0-12,y0+10,x0+12,y0,tft.color565(80,80,80)); 
  }
  else if(icono==6 && (estadoActual == 7 || estadoActual == 8 || estadoActual == 9 || estadoActual == 10 || estadoActual == 11))
  { // Iniciar operacion seleccionada
    tft.fillCircle(x0, y0, radio1+10, tft.color565(160,10,10));
    tft.fillTriangle(x0-12,y0-10,x0-12,y0+10,x0+12,y0,tft.color565(180,180,180)); 
  }
  if(icono==7)
  { // CheckBox sin seleccionar (para indicar si se va a escribir el numero sobre rutina o sobre sensor)
    tft.fillCircle(x0, y0, radio1/2, tft.color565(200,200,200));
    tft.fillCircle(x0, y0, radio1/2-3, tft.color565(10,10,10));
  }
  if(icono==8)
  { // Checkbox seleccionado (para indicar si se va a escribir el numero sobre rutina o sobre sensor)
    tft.fillCircle(x0, y0, radio1/2, tft.color565(200,200,200));
    tft.fillCircle(x0, y0, radio1/2-3, tft.color565(10,10,10));
    tft.fillCircle(x0, y0, radio1/2-7, tft.color565(200,200,200));    
  }

}

// ----------------------------------------------------------------------------------------------------------------
void gestorDeOperaciones()
{ // Gestiona el comportamiento del programa en funcion de los botones pulsados y envia los mensajes por puerto serial acorde
  static bool transmitiendo;
  bool estadoDeTransmision = estadoActual==7 || estadoActual==8 || estadoActual==9 || estadoActual==10 || estadoActual==11;
  String codigo;
  
  if(estadoDeTransmision && !transmitiendo)
  { // Si se preciono el boton de iniciar accion junto a otro boton 
    transmitiendo = true;
    if(estadoActual==7)
    { // transmitiendo en tiempo real
      Serial.print("1-0-0");
    }
    else if(estadoActual==8)
    { // crear una nueva rutina
      codigo = "3-"+(String)rutina+"-"+(String)sensor;
      Serial.print(codigo);
    }
    else if(estadoActual==9)
    { // borrar rutina
      codigo = "5-"+(String)rutina+"-0";
      Serial.print(codigo);
    }    
    else if(estadoActual==10)
    { // cambiar el sensor
      codigo = "4-"+(String)rutina+"-"+(String)sensor;
      Serial.print(codigo);
    }        
    else if(estadoActual==11)
    { // transmitir rutina
      codigo = "2-"+(String)rutina+"-0";
      Serial.print(codigo);
    }        
  } // fin >> f(estadoDeTransmision %% !transmitiendo)

  else if(transmitiendo)
  { // si ya se habia ejecutado la tarea y queremos detener la operacion
    transmitiendo = false;
    rutina = 0;
    rutinaA = 0;
    sensor = 0;
    sensorA = 0;
    estadoActual = 0;
    Serial.print("0-0-0");
    tft.fillScreen(tft.color565(0,0,0));
    tft.setTextColor(tft.color565(200,200,200));
    setRutina=true;
    setSensor=false;
    pantallaPrincipal();

  }
  
}

// ----------------------------------------------------------------------------------------------------------------
void gestorDePantallas(String str_zona)
{ // actualiza los botones, iconos y texto de la pantalla a medida que se va presionando la misma

  //Botones:    btn1 btn4
  //            btn2 btn5
  //            btn3 btn6

  bool btn1 = str_zona=="A1" || str_zona=="A2" || str_zona=="B1" || str_zona=="B2"; 
  bool btn2 = str_zona=="C1" || str_zona=="C2" || str_zona=="D1" || str_zona=="D2";
  bool btn3 = str_zona=="E1" || str_zona=="E2" || str_zona=="F1" || str_zona=="F2";
  bool btn4 = str_zona=="A3" || str_zona=="A4" || str_zona=="B3" || str_zona=="B4";
  bool btn5 = str_zona=="C3" || str_zona=="C4" || str_zona=="D3" || str_zona=="D4";
  bool btn6 = str_zona=="E3" || str_zona=="F3" || str_zona=="E4" || str_zona=="F4";
  
  
  if(btn1 && estadoActual==0)
  { // Se presiono el boton (en off) de transmitir en tiempo real
    dibujarBoton(2,0,0);
    dibujarIcono(1,CUADRO,CUADRO);
    estadoActual=1;
    ultimaLectura = millis();
  }
  else if(btn1 && estadoActual==1)
  { // Se presiono el boton (en on) de crear una nueva rutina
    dibujarBoton(1,0,0);
    dibujarIcono(1,CUADRO,CUADRO);
    estadoActual=0;
    ultimaLectura = millis();
  }
  else if(btn2 && estadoActual==0 )
  { // Se presiono el boton (en off) de crear una nueva rutina
    dibujarBoton(4,0,2*CUADRO);
    dibujarIcono(2,CUADRO,3*CUADRO);
    estadoActual=2;
    ultimaLectura = millis();
  }
  else if(btn2 && estadoActual==2 )
  { // Se presiono el boton (en on) de crear una nueva rutina
    dibujarBoton(3,0,2*CUADRO);
    dibujarIcono(2,CUADRO,3*CUADRO);
    estadoActual=0;
    ultimaLectura = millis();
  }  
  else if(btn3 && estadoActual==0 )
  { // Se presiono el boton (en off) de eliminar una rutina
    dibujarBoton(6,0,4*CUADRO);
    dibujarIcono(3,CUADRO,5*CUADRO); 
    estadoActual=3;
    ultimaLectura = millis();
  }
  else if(btn3 && estadoActual==3 )
  { // Se presiono el boton (en on) eliminar una rutina
    dibujarBoton(5,0,4*CUADRO);
    dibujarIcono(3,CUADRO,5*CUADRO); 
    estadoActual=0;
    ultimaLectura = millis();
  }    
  else if(btn4 && estadoActual==0)
  { // Se presiono el boton (en off) de cambiar el numero del sensor
    estadoActual=4;
    dibujarBoton(8,2*CUADRO,0);
    dibujarIcono(4,3*CUADRO,CUADRO); 
    ultimaLectura = millis();
  }
  else if(btn4 && estadoActual==4)
  { // Se presiono el boton (en on) de cambiar el numero del sensor
    estadoActual=0;
    dibujarBoton(7,2*CUADRO,0);
    dibujarIcono(4,3*CUADRO,CUADRO); 
    ultimaLectura = millis();
  }
  else if(btn5 && estadoActual==0 )
  { // Se presiono el boton (en off) de crear una nueva rutina
    dibujarBoton(10,2*CUADRO,2*CUADRO);
    estadoActual=5;
    dibujarIcono(5,3*CUADRO,3*CUADRO); 
    ultimaLectura = millis();
  }
  else if(btn5 && estadoActual==5 )
  { // Se presiono el boton (en on) de crear una nueva rutina
    dibujarBoton(9,2*CUADRO,2*CUADRO);
    estadoActual=0;
    dibujarIcono(5,3*CUADRO,3*CUADRO); 
    ultimaLectura = millis();
  }  
  else if(btn6 && (estadoActual==1 || estadoActual==2 || estadoActual==3 || estadoActual==4 || estadoActual==5))
  { // Se presiono el boton (en off) de ejecutar funcion de transmision en tiempo real 
    if((estadoActual==2 || estadoActual==3 || estadoActual==4 || estadoActual==5) && rutina == 0)
    { // no se especifico la rutina con la cual trabajar
      tft.setCursor(4.5*CUADRO+5,5.5*CUADRO+9);
      tft.setTextColor(tft.color565(0,0,0));
      tft.print(str_msgErrorA);    
      str_msgError = "Falta rutina";   
      tft.setCursor(4.5*CUADRO+5,5.5*CUADRO+9);
      tft.setTextColor(tft.color565(200,200,200));
      tft.print(str_msgError);       
      str_msgErrorA = str_msgError; 
    }
    else if((estadoActual==2 || estadoActual==4 ) && sensor == 0)
    { // no se especifico el sensor con el cual trabajar
      tft.setCursor(4.5*CUADRO+5,5.5*CUADRO+9);
      tft.setTextColor(tft.color565(0,0,0));
      tft.print(str_msgErrorA);    
      str_msgError = "Falta sensor";   
      tft.setCursor(4.5*CUADRO+5,5.5*CUADRO+9);
      tft.setTextColor(tft.color565(200,200,200));
      tft.print(str_msgError);       
      str_msgErrorA = str_msgError;       
    }
    else
    { // si no hay errores operamos normalmente
      estadoActual+=6;
      dibujarBoton(12,2*CUADRO,4*CUADRO);
      dibujarIcono(6,3*CUADRO,5*CUADRO); 
      ultimaLectura = millis();
      tft.setCursor(4.5*CUADRO+5,5.5*CUADRO+9);
      tft.setTextColor(tft.color565(0,0,0));
      tft.print(str_msgErrorA);   
      tft.setCursor(4.5*CUADRO+5,5.5*CUADRO+9);
      tft.setTextColor(tft.color565(200,200,200));
      tft.print("Operando...");   

      str_msgErrorA="";
    }
  } 

  // LISTA DE BOTONES NUMERICOS 

  else if(str_zona=="A6")
  { // Se presiono el boton 1
    if(setRutina) rutina=1;
    else if(setSensor) sensor=1;
  }
  else if(str_zona=="A7")
  { // Se presiono el boton 2
    if(setRutina) rutina=2;
    else if(setSensor) sensor=2;
  }  
  else if(str_zona=="A8")
  { // Se presiono el boton 3
    if(setRutina) rutina=3;
    else if(setSensor) sensor=3;
  }
  else if(str_zona=="B6")
  { // Se presiono el boton 4
    if(setRutina) rutina=4;
    else if(setSensor) sensor=4;
  }
  else if(str_zona=="B7")
  { // Se presiono el boton 5
    if(setRutina) rutina=5;
    else if(setSensor) sensor=5;
  }
  else if(str_zona=="B8")
  { // Se presiono el boton 6
    if(setRutina) rutina=6;
    else if(setSensor) sensor=6;
  }
  else if(str_zona=="C6")
  { // Se presiono el boton 7
    if(setRutina) rutina=7;
    else if(setSensor) sensor=7;
  }
  else if(str_zona=="C7")
  { // Se presiono el boton 8
    if(setRutina) rutina=8;
    else if(setSensor) sensor=8;
  }
  else if(str_zona=="C8")
  { // Se presiono el boton 9
    if(setRutina) rutina=9;
    else if(setSensor) sensor=9;
  }
  else if(str_zona=="D5")
  { // Si hago check en Rutina
    setRutina=true;
    setSensor=false;
    dibujarIcono(8,4.5*CUADRO,3.5*CUADRO); 
    dibujarIcono(7,4.5*CUADRO,4.5*CUADRO); 
  }
  else if(str_zona=="E5")
  { // Si hago check en Sensor
    setRutina=false;
    setSensor=true;    
    dibujarIcono(7,4.5*CUADRO,3.5*CUADRO); 
    dibujarIcono(8,4.5*CUADRO,4.5*CUADRO); 
  }
  
  // Mensajes por pantalla

  if(str_zona=="A6" || str_zona=="A7" || str_zona=="A8" || str_zona=="B6" || str_zona=="B7" || str_zona=="B8" || 
     str_zona=="C6" || str_zona=="C7" || str_zona=="C8")
  { // Si presionaron alguno de los numeros se sobreescriben y actualizan los valores
  tft.setCursor(7*CUADRO+5,3.5*CUADRO+7);
  tft.setTextColor(tft.color565(0,0,0));
  tft.print(rutinaA);    
  tft.setCursor(7*CUADRO+5,3.5*CUADRO+7);
  tft.setTextColor(tft.color565(200,200,200));
  tft.print(rutina);       
  tft.setCursor(7*CUADRO+5,4.5*CUADRO+8);
  tft.setTextColor(tft.color565(0,0,0));
  tft.print(sensorA); 
  tft.setCursor(7*CUADRO+5,4.5*CUADRO+8);
  tft.setTextColor(tft.color565(200,200,200));
  tft.print(sensor);    
  rutinaA = rutina;
  sensorA = sensor;
  }
}
// ----------------------------------------------------------------------------------------------------------------
void inicializarPantalla()
{ // Inicializacion de la pantalla TFT
  tft.reset();
  tft.begin(0x9341); // Iniciamos el LCD especificando el controlador ILI9341.
  tft.setRotation(1); //Orientacion Horizontal
  tft.fillScreen(tft.color565(0,0,0));
  tft.setFont(&FreeSerifBoldItalic12pt7b); 
  tft.setTextColor(tft.color565(180,180,180));

  pinMode(13, OUTPUT); // El pin 13 debe variar entre HIGH y LOW para leer correctamente la pantalla cuando se la
                       // usa como pantalla tactil >> ver funcion leerTouchScreen
}

// ----------------------------------------------------------------------------------------------------------------
TSPoint leerTouchScreen(TouchScreen ts)
{ // Genera un evento de lectura del TouchScreen con el cual podremos obtener las coordenadas y la presion

  pinMode(XM, INPUT);
  pinMode(YP, INPUT);
  digitalWrite(13, HIGH);

  TSPoint p = ts.getPoint();
  
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  digitalWrite(13, LOW);

  return p;
}

// ----------------------------------------------------------------------------------------------------------------
void pantallaPrincipal()
{ // Carga la pantalla y textos del menu principal
  dibujarBoton(1,0,0);
  dibujarIcono(1,CUADRO,CUADRO);
  dibujarBoton(3,0,2*CUADRO);
  dibujarIcono(2,CUADRO,3*CUADRO);
  dibujarBoton(5,0,4*CUADRO);
  dibujarIcono(3,CUADRO,5*CUADRO);  
  dibujarBoton(7,2*CUADRO,0);
  dibujarIcono(4,3*CUADRO,CUADRO);    
  dibujarBoton(9,2*CUADRO,2*CUADRO);
  dibujarIcono(5,3*CUADRO,3*CUADRO); 
  dibujarBoton(11,2*CUADRO,4*CUADRO);  
  dibujarIcono(6,3*CUADRO,5*CUADRO); 
  // Teclado numerico
  dibujarBoton(13,5.5*CUADRO,0.5*CUADRO);  
  dibujarBoton(13,6.5*CUADRO,0.5*CUADRO);  
  dibujarBoton(13,7.5*CUADRO,0.5*CUADRO);  
  dibujarBoton(13,5.5*CUADRO,1.5*CUADRO);  
  dibujarBoton(13,6.5*CUADRO,1.5*CUADRO);  
  dibujarBoton(13,7.5*CUADRO,1.5*CUADRO);  
  dibujarBoton(13,5.5*CUADRO,2.5*CUADRO);  
  dibujarBoton(13,6.5*CUADRO,2.5*CUADRO);  
  dibujarBoton(13,7.5*CUADRO,2.5*CUADRO);  
  //Numeros
  tft.setCursor(5.35*CUADRO,0.65*CUADRO);
  tft.print("1");
  tft.setCursor(6.35*CUADRO,0.65*CUADRO);
  tft.print("2");
  tft.setCursor(7.35*CUADRO,0.65*CUADRO);
  tft.print("3");
  tft.setCursor(5.35*CUADRO,1.65*CUADRO);
  tft.print("4");
  tft.setCursor(6.35*CUADRO,1.65*CUADRO);
  tft.print("5");
  tft.setCursor(7.35*CUADRO,1.65*CUADRO);
  tft.print("6");  
  tft.setCursor(5.35*CUADRO,2.65*CUADRO);
  tft.print("7");
  tft.setCursor(6.35*CUADRO,2.65*CUADRO);
  tft.print("8");
  tft.setCursor(7.35*CUADRO,2.65*CUADRO);
  tft.print("9");  
  // Barras separadoras
  for(int i=-1;i<2;i++)
  {  
    tft.drawFastVLine(4.4*CUADRO+i, 0, 3*CUADRO, tft.color565(180,55,0));
    tft.drawFastVLine(4.6*CUADRO+i, 0, 3*CUADRO, tft.color565(180,55,0));  
  }
  // Texto 
  tft.setCursor(5*CUADRO+5,3.5*CUADRO+7);
  tft.print("Rutina:");    
  tft.setCursor(5*CUADRO+5,4.5*CUADRO+8);
  tft.print("Sensor:");  
  tft.setCursor(7*CUADRO+5,3.5*CUADRO+7);
  tft.print(rutina);   
  tft.setCursor(7*CUADRO+5,4.5*CUADRO+8);
  tft.print(sensor);  
  dibujarIcono(8,4.5*CUADRO,3.5*CUADRO); 
  dibujarIcono(7,4.5*CUADRO,4.5*CUADRO);     
}

// ----------------------------------------------------------------------------------------------------------------
String zona(short x, short y)
{ // Identifica la zona presionada sobre la pantalla, estas pueden verse usando la funcion cuadricula, x e y son las 
  // coordenadas leidas del TS, retorna un string identificando la fila y la columna presionada 
  // (NOTA) ESTA FUNCION SE DEBE USAR UNICAMENTE CON LA CERTEZA DE QUE SE PULSO LA PANTALLA 
  //  A1 A2 A3 A4 A5 A6 A7 A8
  //  B1 B2 B3 B4 B5 B6 B7 B8
  //  C1 C2 C3 C4 C5 C6 C7 C8
  //  D1 D2 D3 D4 D5 D6 D7 D8
  //  E1 E2 E3 E4 E5 E6 E7 E8
  //  F1 F2 F3 F4 F5 F6 F7 F8
  short alto = (short)(TS_MAXX_ - TS_MINX_)/8;   //Altura de cada cuadricula de la pantalla
  short ancho = (short)(TS_MAXY_ - TS_MINY_)/6;  //Ancho de cada cuadricula de la pantalla

  String F , C;

  if(x > TS_MINX_ && x < TS_MINX_ + alto*1 ){F = "A";}                  // DETECTA FILA A
  else if(x >= TS_MINX_ + alto*1 && x < TS_MINX_ + alto*2) {F = "B";}   // DETECTA FILA B
  else if(x >= TS_MINX_ + alto*2 && x < TS_MINX_ + alto*3) {F = "C";}   // DETECTA FILA C
  else if(x >= TS_MINX_ + alto*3 && x < TS_MINX_ + alto*4) {F = "D";}   // DETECTA FILA D
  else if(x >= TS_MINX_ + alto*4 && x < TS_MINX_ + alto*5) {F = "E";}   // DETECTA FILA E
  else if(x >= TS_MINX_ + alto*5 && x < TS_MAXX_) {F = "F";}            // DETECTA FILA F
  
  if(y < TS_MAXY_ && y > TS_MAXY_ - ancho*1) {C = "1";}                 // DETECTA COLUMNA 1
  else if(y <= TS_MAXY_ - ancho*1 && y > TS_MAXY_ - ancho*2) {C = "2";} // DETECTA COLUMNA 2
  else if(y <= TS_MAXY_ - ancho*2 && y > TS_MAXY_ - ancho*3) {C = "3";} // DETECTA COLUMNA 3
  else if(y <= TS_MAXY_ - ancho*3 && y > TS_MAXY_ - ancho*4) {C = "4";} // DETECTA COLUMNA 4
  else if(y <= TS_MAXY_ - ancho*4 && y > TS_MAXY_ - ancho*5) {C = "5";} // DETECTA COLUMNA 5
  else if(y <= TS_MAXY_ - ancho*5 && y > TS_MAXY_ - ancho*6) {C = "6";} // DETECTA COLUMNA 6
  else if(y <= TS_MAXY_ - ancho*6 && y > TS_MAXY_ - ancho*7) {C = "7";} // DETECTA COLUMNA 7
  else if(y <= TS_MAXY_ - ancho*7 && y > TS_MAXY_ - ancho*8) {C = "8";} // DETECTA COLUMNA 8
  
  return F+C;
}
