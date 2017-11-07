/************************************/
/* Proyecto Medidas Electronicas II */
/* -------------------------------- */
/*   Control de motor paso a paso   */
/*      para stub "automatico"      */
/************************************/

#include <TimerOne.h>

/********************************/
//Pines del driver motor paso a paso DRV8825
#define _EN     13    //Enable: nivel bajo
#define _DIR    8     //Direccion: sentido de giro del motor
#define _STEP   9     //Step: entrada de pulsos (t_min=2useg y frec_max=250khz)
#define _MOD0   12    //Modo 0: microstep
#define _MOD1   11    //Modo 1: microstep
#define _MOD2   10    //Modo 2: microstep
#define _FAULT  5     //Fault: (salida del drv8825) seÃ±al logica bajo por sobretemperatura o sobrecorriente
/* |MODE2|MODE1|MODE0||
*  |  0  |  0  |  0  ||  Full Step
*  |  0  |  0  |  1  ||  1/2  Step
*  |  0  |  1  |  0  ||  1/4  Step
*  |  0  |  1  |  1  ||  8   uStep
*  |  1  |  0  |  0  ||  16  uStep
*  |  1  |  0  |  1  ||  32  uStep
*/

//Pines endstop
#define _ENDSTOP_DER 2   //Endstop 1 
#define _ENDSTOP_IZQ 3   //Endstop 2

//Sentido de giro
#define DERECHA   1
#define IZQUIERDA 0
#define MOTOR_NORMAL    'N'
#define MOTOR_USTEP     'U'    
#define MOTOR_FSTEP     'F' 

#define MICROPASOS  1000  //Por debajo de este valor usa uStep

#define _MAX_COLA   5     //Maximo de instrucciones en cola, de sobrepasarlo se sobreescribira el siguiente en cola circular (osea el primero)

/********************************/
//Variables
char inputString[_MAX_COLA][10]={0};         // a string to hold incoming data
boolean stringComplete = false, izq_endstop = false, der_endstop = false, flag_10p=false, flag_delay=false;  // whether the string is complete
boolean first_step = true, start_motor = false, started_motor = false, periodo = false,  dir_motor = 1;
unsigned long pasos = 0, pasos_total = 0, actual_time = 0, time_delay = 0;
unsigned char vel_motor='U', indice=0, indice_string=0, indi_str_cola=0, pin=0;

/********************************/

void setup() 
{
    //Seteo de entradas
    pinMode(_FAULT, INPUT); 
    
    //Seteo de salidas
    pinMode(_EN, OUTPUT);
    pinMode(_DIR, OUTPUT);         
    pinMode(_STEP, OUTPUT);  
    pinMode(_MOD0, OUTPUT);    
    pinMode(_MOD1, OUTPUT);
    pinMode(_MOD2, OUTPUT);
    digitalWrite(_EN, 1);

    //Seteo interrupciones externas
    pinMode(_ENDSTOP_DER, INPUT_PULLUP);
    pinMode(_ENDSTOP_IZQ, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_ENDSTOP_DER), fend_int, FALLING);   //LOW, RISING, FALLING
    attachInterrupt(digitalPinToInterrupt(_ENDSTOP_IZQ), fend_int, FALLING);   //LOW, RISING, FALLING
    
    //Seteo comunicacion serie
    Serial.begin(9600);               // initialize serial
    
    //Seteo Interrupcion Timer       
    Timer1.initialize(1000);            // 10 microseg
    Timer1.attachInterrupt(timerIsr); // attach the service routine here
}

void loop() 
{
    if(Serial.available())
        serial_evento();
    if(start_motor == false)
    {
        if(stringComplete) 
        {
            read_serial_queue();                                              //lee cola de instrucciones y setea dir_motor, vel_motor y pasos
            start_motor = true;                                               //subo flag de arranque del motor
            if((digitalRead(_ENDSTOP_IZQ) == 0) && (dir_motor == IZQUIERDA))
            {
                Serial.println("END_IZQ");
                pasos = 0;
                start_motor = false;
            }
            if((digitalRead(_ENDSTOP_DER) == 0) && (dir_motor == DERECHA))
            {
                Serial.println("END_DER");
                pasos = 0;
                start_motor = false;
            }
        }      
    }
    else
    {
        switch(vel_motor)
        {
            case 'N':
                fmotor_normal();
                break;
            case 'U':
                fmotor_ustep();
                break;
            case 'F':
                fmotor_fstep();
                break;
        }  
    }
}

void fmotor_normal()
{
    if(first_step)
    {
        if(flag_delay)
        {
            actual_time = millis();
            if((actual_time - time_delay) > 100)
            {
                first_step = false;
                started_motor =true;
                flag_delay = false;
            }
        }
        else
        {
            digitalWrite(_DIR, dir_motor);
            digitalWrite(_EN, 0);
            Timer1.initialize(1000);
            time_delay = millis();
            flag_delay = true;
        }
    }
    else
    {
        if(pasos == 0)
        {
            if(flag_delay)
            {
                actual_time = millis();
                if((actual_time - time_delay) > 100)
                {
                    start_motor = false;
                    first_step = true;
                    digitalWrite(_EN, 1);
                    flag_delay = false;
                }
            }
            else
            {
                started_motor = false;
                digitalWrite(_STEP, 0);
                if(digitalRead(_ENDSTOP_IZQ))
                    izq_endstop = false;
                if(digitalRead(_ENDSTOP_DER))
                    der_endstop = false;
                Serial.println(pasos_total);
                pasos_total=0;
                
                time_delay = millis();
                flag_delay = true;
            }
        }
    }
}

void fmotor_ustep()
{
    if(first_step)
    {
        if(flag_delay)
        {
            actual_time = millis();
            if((actual_time - time_delay) > 100)
            {
                pasos = pasos*4 + 1;
                first_step = false;
                started_motor =true;
                flag_delay = false;
            }
        }
        else
        {
            digitalWrite(_DIR, dir_motor);
            digitalWrite(_EN, 0);
            digitalWrite(_MOD1, 1);
            Timer1.initialize(250);
            
            time_delay = millis();
            flag_delay = true;
        }
    }
    else
    {
        if(pasos == 0)
        {
            if(flag_delay)
            {
                actual_time = millis();
                if((actual_time - time_delay) > 100)
                {
                    start_motor = false;
                    first_step = true;
                    digitalWrite(_EN, 1);
                    flag_delay = false;
                }
            }
            else
            {
                started_motor = false;
                digitalWrite(_STEP, 0);
                digitalWrite(_MOD1, 0);
                if(digitalRead(_ENDSTOP_IZQ))
                    izq_endstop = false;
                if(digitalRead(_ENDSTOP_DER))
                    der_endstop = false;
                Serial.println(pasos_total/4);
                pasos_total=0;
                                
                time_delay = millis();
                flag_delay = true;
            }
        }
    }
}

void fmotor_fstep()
{
    if(first_step)
    {   
        if(flag_delay)
        {
            actual_time = millis();
            if((actual_time - time_delay) > 100)
            {
                pasos = pasos*8 + 1;
                first_step = false;
                started_motor =true;
                flag_delay = false;
            }
        }
        else
        {
            digitalWrite(_DIR, dir_motor);
            digitalWrite(_EN, 0);
            digitalWrite(_MOD1, 1);
            digitalWrite(_MOD0, 1);
            Timer1.initialize(250);
            
            time_delay = millis();
            flag_delay = true;
        }
    }
    else
    {
        if(pasos == 0)
        {            
            if(flag_delay)
            {
                actual_time = millis();
                if((actual_time - time_delay) > 100)
                {
                    start_motor = false;
                    first_step = true;
                    digitalWrite(_EN, 1);
                    flag_delay = false;
                }
            }
            else
            {
                started_motor = false;
                digitalWrite(_STEP, 0);
                digitalWrite(_MOD1, 0);
                digitalWrite(_MOD0, 0);
                if(digitalRead(_ENDSTOP_IZQ))
                    izq_endstop = false;
                if(digitalRead(_ENDSTOP_DER))
                    der_endstop = false;
                Serial.println(pasos_total/8);
                pasos_total=0;
                                
                time_delay = millis();
                flag_delay = true;
            }
        }
    }
}

void timerIsr()
{
    if(started_motor == true)
    {
        if(pin)
        {
          pin=0;
          digitalWrite(_STEP, 0);
          if(pasos > 0)
              pasos--;
        }
        else
        {
            if(pasos)
            {
                pin=1;
                digitalWrite(_STEP, 1);
                pasos_total++;
            }
        }
    }
}

void serial_evento() 
{
    char inChar = (char)Serial.read();  //lectura caracter
    inputString[indice_string][indice] = inChar;
    indice++;
    if(inChar == '\n')
    {
        indice = 0;
        if(strncmp(inputString[indice_string],"STOP",4) == 0)
        {
            pasos = 0;
            Serial.println("STOP");
            indice_string = 0;
            indi_str_cola = 0;
            stringComplete = false;
        }
        else
        {
            if(strncmp(inputString[indice_string],"STATUS",6) == 0)
            {
                if(digitalRead(_ENDSTOP_DER) == 0)
                    Serial.println("END_DER");
                else
                {
                    if(digitalRead(_ENDSTOP_IZQ) == 0)
                        Serial.println("END_IZQ");
                    else
                        Serial.println(pasos_total);
                }
            }
            else
            {    
                stringComplete = true;
                if(indice_string < (_MAX_COLA - 1))
                    indice_string++;
                else
                    indice_string = 0;
            }
        }
    }
}

void read_serial_queue()
{
    char  caracter;
    boolean lectura = true;
    int indi = 0;

    pasos = 0;
    while(lectura)
    {
        caracter = inputString[indi_str_cola][indi];
        switch(caracter)
        {
          case 'D':
              if(dir_motor == IZQUIERDA)
                  flag_10p = true;
              dir_motor = DERECHA;          //Direccion seteado de pc sentido derecho
              break;
          case 'I':
              if(dir_motor == DERECHA)
                  flag_10p = true;
              dir_motor = IZQUIERDA;        //Direccion seteado de pc sentido izquierdo
              break;
          case 'N':
              vel_motor = MOTOR_NORMAL;
              break;
          case 'U':
              vel_motor = MOTOR_USTEP;
              break;
          case 'F':
              vel_motor = MOTOR_FSTEP;
              break;    
          default:    
              if(('0' <= caracter) && (caracter <= '9'))        //Si el caracter es un numero
                  pasos = pasos * 10 + (caracter-'0');  //Ascci-to-num y multiplico por decena, centena...
        }    
        indi++;
        if(caracter == '\n')
        {
            lectura = false;
            if(flag_10p)
            {
                flag_10p = false;
                pasos += 10;
            }
            Serial.write(inputString[indi_str_cola], indi);
            if(indi_str_cola < (_MAX_COLA - 1))
                indi_str_cola++;
            else
                indi_str_cola = 0;
            if(indi_str_cola == indice_string)
                stringComplete = false;
        }
    }
}

void fend_int()
{
    if(digitalRead(_ENDSTOP_IZQ) == 0)
    {
        if(dir_motor == IZQUIERDA)
        { 
            if(izq_endstop == false)
                Serial.println("END_IZQ");
            digitalWrite(_EN, 1);
            pasos = 0;
            izq_endstop = true;
        }
    }
    if(digitalRead(_ENDSTOP_DER) == 0)
    {
        if(dir_motor == DERECHA)
        {
            if(der_endstop == false)
                Serial.println("END_DER");
            digitalWrite(_EN, 1);
            pasos = 0;
            der_endstop = true;
        }
    }
}

