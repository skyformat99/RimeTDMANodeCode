/*
************************************************************************************************
* Filename   	: WatchDog.c
* Programmer : JiangJun
* Description	: Driver SHT7x(Sensor of Humidity and Temperature) 
* Date           : 2015-08-09 10:00
* Copyright    : (c) RimeLink (www.rimelink.com)
************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                                          INCLUDE FILES
*********************************************************************************************************
*/
#include "stm8l15x_clk.h"
#include "stm8l15x_gpio.h"
#include <math.h>  
#include <stdio.h>
#include "Util.h"
#include "SHT7x.h"

/*
*********************************************************************************************************
*                                            				     DEFINITIONS / TYPEDEFS
*********************************************************************************************************
*/

//----------------------------------------------------------------------------------
// modul-var
//----------------------------------------------------------------------------------
/* Configure pins of DATA(PA5) and SCK(PA6) */
#define DATA_IOPORT    GPIOA
#define DATA_PIN    GPIO_Pin_5

#define SCK_IOPORT    GPIOA
#define SCK_PIN    GPIO_Pin_6

#define SET_PIN(Port, Pin)    ((Port)->ODR |= (Pin))
#define RESET_PIN(Port, Pin)    ((Port)->ODR &= (uint8_t)(~(Pin)))
#define GET_PIN(Port, Pin)    ((Port)->IDR & (uint8_t)(Pin))

/* Operate pins of DATA and SCK */
#define GET_DATA()    (GET_PIN(DATA_IOPORT, DATA_PIN) ? 1 : 0)
#define SET_DATA()    SET_PIN(DATA_IOPORT, DATA_PIN)
#define RESET_DATA()    RESET_PIN(DATA_IOPORT, DATA_PIN)

#define SET_SCK()    SET_PIN(SCK_IOPORT, SCK_PIN)
#define RESET_SCK()    RESET_PIN(SCK_IOPORT, SCK_PIN)

#define SET_DATA_INPUT()    GPIO_Init(DATA_IOPORT, DATA_PIN, GPIO_Mode_In_PU_No_IT)
#define SET_DATA_OUTPUT()    GPIO_Init(DATA_IOPORT, DATA_PIN, GPIO_Mode_Out_PP_Low_Fast)

#define INIT_SCK_PIN()    GPIO_Init(SCK_IOPORT, SCK_PIN, GPIO_Mode_Out_PP_Low_Fast)

#define noACK 0
#define ACK   1

//adr  command  r/w
#define STATUS_REG_W 0x06   //000   0011    0
#define STATUS_REG_R 0x07   //000   0011    1
#define MEASURE_TEMP 0x03   //000   0001    1
#define MEASURE_HUMI 0x05   //000   0010    1
#define RESET        0x1e   //000   1111    0


/*
*********************************************************************************************************
*                                                                        FUNCTION PROTOTYPE
*********************************************************************************************************
*/
//----------------------------------------------------------------------------------
static char s_write_byte(unsigned char value)
//----------------------------------------------------------------------------------
// writes a byte on the Sensibus and checks the acknowledge 
{ 
  unsigned char i,error=0;  

  SET_DATA_OUTPUT();
  for (i = 0x80;i > 0;i >>= 1)             //shift bit for masking
  { 
    if (i & value) 
      SET_DATA();          //masking value with i , write to SENSI-BUS
    else 
      RESET_DATA();                        
    ASM_NOP();                        //observe setup time

    SET_SCK();                          //clk for SENSI-BUS
    ASM_NOP();
    ASM_NOP();
    ASM_NOP();        //pulswith approx. 5 us  	

    RESET_SCK();
    ASM_NOP();                         //observe hold time
  }
  
  SET_DATA();                           //release GET_DATA()-line
  ASM_NOP();                          //observe setup time

  SET_DATA_INPUT();
  SET_SCK();                            //clk #9 for ack 
  error=GET_DATA();                       //check ack (GET_DATA() will be pulled down by SHT11)
  RESET_SCK();        

  return error;                     //error=1 in case of no acknowledge
}

//----------------------------------------------------------------------------------
static char s_read_byte(unsigned char ack)
//----------------------------------------------------------------------------------
// reads a byte form the Sensibus and gives an acknowledge in case of "ack=1" 
{ 
  unsigned char i,val=0;
  
  SET_DATA_OUTPUT();
  SET_DATA();                           //release DATA-line

  SET_DATA_INPUT();
  for (i = 0x80; i > 0; i >>= 1)             //shift bit for masking
  { 
    SET_SCK();                          //clk for SENSI-BUS
    if (GET_DATA()) 
      val = (val | i);        //read bit  
    RESET_SCK();  					 
  }

  SET_DATA_OUTPUT();
  if (ack) 
    RESET_DATA();    //in case of "ack==1" pull down DATA-Line
  else 
    SET_DATA();
  ASM_NOP();                          //observe setup time

  SET_SCK();                            //clk #9 for ack
  ASM_NOP();
  ASM_NOP();
  ASM_NOP();          //pulswith approx. 5 us 
  RESET_SCK();

  ASM_NOP();                          //observe hold time						    
  SET_DATA();                           //release DATA-line

  return val;
}

//----------------------------------------------------------------------------------
static void s_transstart(void)
//----------------------------------------------------------------------------------
// generates a transmission start 
//         _____           ________
// DATA:       |______|
//               ___      ___
// SCK : ___|   |___|   |______
{  
   SET_DATA_OUTPUT();

   SET_DATA();

   RESET_SCK();                   //Initial state
   ASM_NOP();
   SET_SCK();
   ASM_NOP();

   RESET_DATA();
   ASM_NOP();

   RESET_SCK();  
   ASM_NOP();
   ASM_NOP();
   ASM_NOP();
   SET_SCK();
   ASM_NOP();

   SET_DATA();		   
   ASM_NOP();

   RESET_SCK();		   
}

//----------------------------------------------------------------------------------
static char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum)
//----------------------------------------------------------------------------------
// reads the status register with checksum (8-bit)
{ 
  unsigned char error=0;

  s_transstart();                   //transmission start
  error=s_write_byte(STATUS_REG_R); //send command to sensor
  *p_value=s_read_byte(ACK);        //read status register (8-bit)
  *p_checksum=s_read_byte(noACK);   //read checksum (8-bit)  

  return error;                     //error=1 in case of no response form the sensor
}

//----------------------------------------------------------------------------------
static char s_write_statusreg(unsigned char *p_value)
//----------------------------------------------------------------------------------
// writes the status register with checksum (8-bit)
{ 
  unsigned char error=0;

  s_transstart();                   //transmission start
  error+=s_write_byte(STATUS_REG_W);//send command to sensor
  error+=s_write_byte(*p_value);    //send value of status register

  return error;                     //error>=1 in case of no response form the sensor
}
 							   

//----------------------------------------------------------------------------------
void s_connectionreset(void)
//----------------------------------------------------------------------------------
// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//       ____________________________________________________         ________
// DATA:                                                                                   |_____|
//              _      _      _      _      _      _      _      _      _            ___      ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
{  
  unsigned char i; 

  SET_DATA_OUTPUT();
  
  SET_DATA(); 
  RESET_SCK();                    //Initial state

  for (i = 0; i < 9; i++)                  //9 SCK cycles
  { 
    SET_SCK();
    RESET_SCK();
  }

  s_transstart();                   //transmission start
}

//----------------------------------------------------------------------------------
char s_softreset(void)
//----------------------------------------------------------------------------------
// resets the sensor by a softreset 
{ 
  unsigned char error=0;
  
  s_connectionreset();              //reset communication
  error+=s_write_byte(RESET);       //send RESET-command to sensor
  
  return error;                     //error=1 in case of no response form the sensor
}

//----------------------------------------------------------------------------------
char s_measure(unsigned char *p_value, unsigned char *p_checksum, unsigned char mode)
//----------------------------------------------------------------------------------
// makes a measurement (humidity/temperature) with checksum
{ 
  unsigned long    lCnt;
  unsigned char error=0;

  s_transstart();                   //transmission start

  switch(mode){                     //send command to sensor
    case TEMP: 
      error += s_write_byte(MEASURE_TEMP); 
      break;
    case HUMI: 
      error+=s_write_byte(MEASURE_HUMI); 
      break;
    default: 
      break;	 
  }

  //wait until sensor has finished the measurement
  SET_DATA_INPUT();
  for (lCnt = 0; lCnt < 250000L; ++lCnt) /* Wait about 400ms */
  {
    if (0 == GET_DATA())
    {
      break;
    }
  }

  if (GET_DATA())
    error += 1;  // or timeout (~2 sec.) is reached

  *(p_value)  =s_read_byte(ACK);    //read the first byte (MSB)
  *(p_value+1)=s_read_byte(ACK);    //read the second byte (LSB)
  *p_checksum =s_read_byte(noACK);  //read checksum

  return error;
}

//----------------------------------------------------------------------------------------
void calc_sth11(float *p_humidity ,float *p_temperature)
//----------------------------------------------------------------------------------------
// calculates temperature [¡æ] and humidity [%RH] 
// input :  humi [Ticks] (12 bit) 
//          temp [Ticks] (14 bit)
// output:  humi [%RH]
//          temp [¡æ]
{
  const float C1=-2.0468;           // for 12 Bit RH
  const float C2=+0.0367;           // for 12 Bit RH
  const float C3=-0.0000015955;     // for 12 Bit RH
  const float T1=+0.01;             // for 12 Bit RH
  const float T2=+0.00008;          // for 12 Bit RH	

  float rh=*p_humidity;             // rh:      Humidity [Ticks] 12 Bit 
  float t=*p_temperature;           // t:       Temperature [Ticks] 14 Bit
  float rh_lin;                     // rh_lin:  Humidity linear
  float rh_true;                    // rh_true: Temperature compensated humidity
  float t_C;                        // t_C   :  Temperature [°C]

  t_C=t*0.01 - 40.1;                //calc. temperature [°C] from 14 bit temp. ticks @ 5V
  rh_lin=C3*rh*rh + C2*rh + C1;     //calc. humidity from ticks to [%RH]
  rh_true=(t_C-25)*(T1+T2*rh)+rh_lin;   //calc. temperature compensated humidity [%RH]
  
  if(rh_true>100)rh_true=100;       //cut if the value is outside of
  if(rh_true<0.1)rh_true=0.1;       //the physical possible range

  *p_temperature=t_C;               //return temperature [°C]
  *p_humidity=rh_true;              //return humidity[%RH]
}

//--------------------------------------------------------------------
float calc_dewpoint(float h,float t)
//--------------------------------------------------------------------
// calculates dew point
// input:   humidity [%RH], temperature [¡æ]
// output:  dew point [¡æ]
{ float k,dew_point ;
  
  k = (log10(h)-2)/0.4343 + (17.62*t)/(243.12+t);
  dew_point = 243.12*k/(17.62-k);

  return dew_point;
}

//--------------------------------------------------------------------
void sht7x_Init(void)
//--------------------------------------------------------------------
// MUST enable RTIMER before call this procedure.
{
  SET_DATA_OUTPUT();
  INIT_SCK_PIN();

  s_softreset(); /* Resets the interface, clears the status register to default values. */
  util_DelayMs(12); /* Wait minimum 11 ms before next command. */

  return;
}

