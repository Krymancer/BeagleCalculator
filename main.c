#include "uart_irda_cir.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "board.h"
#include "beaglebone.h"
#include "gpio_v2.h"
#include "consoleUtils.h"
#include "delay.h"


/*****************************************************************************
**                INTERNAL MACRO DEFINITIONS
*****************************************************************************/
#define	PIN_HIGH   				1
#define PIN_LOW						0
#define WDT_WSPR 					0x48
#define GPIO_IRQSTATUS_0 	0x2c
#define	HIGH   						1
#define LOW								0
#define OUTPUT						1
#define INPUT							0
/* Values denoting the Interrupt Line number to be used. */
#define GPIO_INTC_LINE_1                  (0x0)
#define GPIO_INTC_LINE_2                  (0x1)
/* Values used to enable/disable interrupt generation due to level
** detection on an input GPIO pin.*/
#define GPIO_INTC_TYPE_NO_LEVEL           (0x01)
#define GPIO_INTC_TYPE_LEVEL_LOW          (0x04)
#define GPIO_INTC_TYPE_LEVEL_HIGH         (0x08)
#define GPIO_INTC_TYPE_BOTH_LEVEL         (0x0C)
/* Values used to enable/disable interrupt generation due to edge
** detection on an input GPIO pin.*/
#define GPIO_INTC_TYPE_NO_EDGE            (0x80)
#define GPIO_INTC_TYPE_RISE_EDGE          (0x10)
#define GPIO_INTC_TYPE_FALL_EDGE          (0x20)
#define GPIO_INTC_TYPE_BOTH_EDGE          (0x30)

#define TIMER_INITIAL_COUNT            (0xFF000000u)
#define TIMER_RLD_COUNT                (0xFFFFFF83u) //(0xFF000000u)

#define T_1MS_COUNT                     (0x5DC0u)
#define OVERFLOW                        (0xFFFFFFFFu)
#define TIMER_1MS_COUNT                 (0x5DC0u)

#define conf_lcd_data0                                  0x8A0   //GPIO2_6
#define conf_lcd_data1                                  0x8A4   //GPIO2_7
#define conf_lcd_data2                                  0x8A8   //GPIO2_8
#define conf_lcd_data3                                  0x8AC   //GPIO2_9
#define conf_lcd_data4                                  0x8B0   //GPIO2_10
#define conf_lcd_data5                                  0x8B4   //GPIO2_11
#define conf_lcd_data6                                  0x8B8   //GPIO2_12
#define conf_lcd_data7                                  0x8BC   //GPIO2_13

#define conf_lcd_data8                                  0x8C0   //GPIO2_14
#define conf_lcd_data9                                  0x8C4   //GPIO2_15

#define LCD_DO                                          6  // DATA 0 PIN
#define LCD_D1                                          7  // DATA 1 PIN
#define LCD_D2                                          8  // DATA 2 PIN
#define LCD_D3                                          9  // DATA 3 PIN
#define LCD_D4                                          10 // DATA 4 PIN
#define LCD_D5                                          11 // DATA 5 PIN
#define LCD_D6                                          12 // DATA 6 PIN
#define LCD_D7                                          13 // DATA 7 PIN
#define LCD_RS                                          14 // REGISTER SELECT PIN
#define LCD_EN                                          15 // ENABLE PIN

/*------------------------------------------------- LCD COMMANDS  ------------------------------------------------- */
#define COMMAND                                         0x0
#define DATA                                            0x1
#define LCD_CLEAR                                       0x01  //CLEAR AND RETURN HOME
#define LCD_HOME                                        0x11  //SET CURSOR TO HOME
/*------------------------------------------------- END OF COMMANDS  ------------------------------------------------- */


//Functions
static void 		initSerial(void);
static void 		initLed(unsigned int, unsigned int, unsigned int);
static void 		initButton(unsigned int, unsigned int, unsigned int);
static void			gpioAintcConf(void);
static void 		gpioIsr(void);
static int			gpioPinIntConf(unsigned int, unsigned int, unsigned int);
static void 		gpioPinIntEnable(unsigned int, unsigned int, unsigned int);
static void 		gpioIntTypeSet(unsigned int, unsigned int, unsigned int);
static void 		disableWachtDog();

//Keypad functions
static int 			verifyLine();
static char * 	itoa (int value, char *result, int base);

//Lcd Functions
static void 		setLow();
static void 		toogleEnable();
static void 		writeLcdPins();
static void 		sendData(unsigned char cmd,unsigned int FLAG);
static void 		LCDInit();
static void 		initPins();
static void 		sendText(char* text);
static void  		initPin(unsigned int baseAdd, unsigned int module, unsigned int pin);

//Flags
static int 			flagISR=0;
static int 			flagOPR=0;
static int 			flagFNS=0;
//Operands
static char 		n1;
static char 		n2;
static char 		op;
static int 			ch;
//Lcd's aux strings
static char     strn1[16] = "                ";
static char     strn2[16] = "                ";
static char 		strrst[16] = "                ";
static char 		strcls[] = "                ";

//Lcd control variables
static int data[8],i;
static int lcdPins[] =  {LCD_DO, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7, LCD_RS, LCD_EN};

int main(){
	GPIOModuleClkConfig(1); //Keypad
	GPIOModuleClkConfig(2); //LCD

	IntMasterIRQEnable();

	initSerial();

	initLed(SOC_GPIO_1_REGS, 1, 28); //Init Row1
	initLed(SOC_GPIO_1_REGS, 1, 18); //Init Row2
	initLed(SOC_GPIO_1_REGS, 1, 19); //Init Row3
	initLed(SOC_GPIO_1_REGS, 1, 16); //Init Row4
	initButton(SOC_GPIO_1_REGS, 1, 12); //Init Column1
	initButton(SOC_GPIO_1_REGS, 1, 13); //Init Column1
	initButton(SOC_GPIO_1_REGS, 1, 14); //Init Column1
	initButton(SOC_GPIO_1_REGS, 1, 15); //Init Column1

	LCDInit();

	// ENABLE PINS TO INTERRUPT
	gpioAintcConf();
	gpioPinIntConf(SOC_GPIO_1_REGS, GPIO_INTC_LINE_1, 12);
	gpioIntTypeSet(SOC_GPIO_1_REGS, 12, GPIO_INTC_TYPE_RISE_EDGE);
	gpioPinIntConf(SOC_GPIO_1_REGS, GPIO_INTC_LINE_1, 13);
	gpioIntTypeSet(SOC_GPIO_1_REGS, 13, GPIO_INTC_TYPE_RISE_EDGE);
	gpioPinIntConf(SOC_GPIO_1_REGS, GPIO_INTC_LINE_1, 14);
	gpioIntTypeSet(SOC_GPIO_1_REGS, 14, GPIO_INTC_TYPE_RISE_EDGE);
	gpioPinIntConf(SOC_GPIO_1_REGS, GPIO_INTC_LINE_1, 15);
	gpioIntTypeSet(SOC_GPIO_1_REGS, 15, GPIO_INTC_TYPE_RISE_EDGE);

	DelayTimerSetup();
	disableWachtDog();

	GPIOPinWrite(SOC_GPIO_1_REGS,28,1);	//Row1 HIGH
	GPIOPinWrite(SOC_GPIO_1_REGS,18,1);	//Row2 HIGH
	GPIOPinWrite(SOC_GPIO_1_REGS,19,1);	//Row3 HIGH
	GPIOPinWrite(SOC_GPIO_1_REGS,16,1);	//Row4 HIGH


	sendData(0x80,COMMAND);
	sendData(0x30,COMMAND);
	sendData(0x38,COMMAND);
	sendData(0x0f,COMMAND);
	sendData(0x01,COMMAND);
	while (1) {

		//GET THE FRIST NUMBER
		while(flagISR==0);
		if((ch < 40) && (flagFNS!=1)){
			if(flagOPR==0){
				n1 = n1*10 + ch;
				ConsoleUtilsPrintf("%d\n",n1);
				itoa(n1,strn1,10);
				sendData(strn1[0],DATA);
				ConsoleUtilsPrintf("NUMERO1\n");
				delay(300);
				flagISR=0;
			}
		}

		//GET THE OPERATION
		if((ch > 40) && (ch < 60) && (flagFNS!=1)){
			while(flagISR==0);
			op = ch;
			ConsoleUtilsPrintf("%c\n",op);
			sendData(op,DATA);
			ConsoleUtilsPrintf("OP\n");
			delay(300);
			flagISR=0;
			flagOPR=1;
		}

		//GET THE SECOND NUMBER
		while(flagISR==0);
		if(ch < 40){
			if(flagOPR==1){
				n2 = n2*10 + ch;
				ConsoleUtilsPrintf("%d\n",n2);
				itoa(n2,strn2,10);
				sendData(strn2[0],DATA);
				ConsoleUtilsPrintf("NUMERO2\n");
				delay(300);
				flagISR=0;
				flagFNS=1;
			}
		}

		if((ch==61) && (flagFNS==1)){ // =
			switch (op) {
				case '+':{
					ConsoleUtilsPrintf("%d\n",n1+n2);
					itoa(n1+n2,strrst,10);
					//sendData(0x01,COMMAND);
				  sendData('=',DATA);
					sendData(strrst[0],DATA);
					if((n1+n2) > 9){
					sendData(strrst[2],DATA);
					}
					if((n1+n2) > 19){
					sendData(strrst[2],DATA);
					}
					//sendData(strrst[1],DATA);
					//sendData(strrst[2],DATA);
					//sendData(strrst[3],DATA);
					flagISR=0;
					break;
				}
				case '-':{
					ConsoleUtilsPrintf("%d\n",n1-n2);
					itoa(n1-n2,strrst,10);
					sendData('=',DATA);
					sendData(strrst[0],DATA);
					sendData(strrst[1],DATA);
					if((n1-n2) < -9){
					sendData(strrst[2],DATA);
					}
					flagISR=0;
					break;
				}
				case '*':{
					ConsoleUtilsPrintf("%d\n",n1*n2);
					itoa(n1*n2,strrst,10);
					sendData('=',DATA);
					sendData(strrst[0],DATA);
					if((n1*n2) > 9){
					sendData(strrst[1],DATA);
					}
					if((n1*n2) > 99){
					sendData(strrst[2],DATA);
					}
					flagISR=0;
					break;
				}
				case '/':{
					if(n2 == 0){
						sendText("Div by zero err");
						break;
					}
					ConsoleUtilsPrintf("%d\n",n1/n2);
					itoa(n1/n2,strrst,10);
					sendData('=',DATA);
					sendData(strrst[0],DATA);
					if((n1*n2) > 9){
					sendData(strrst[1],DATA);
					}
					if((n1*n2) > 99){
					sendData(strrst[2],DATA);
					}
					flagISR=0;
					break;
				}
				default:{
					sendData(0x01,COMMAND);
					sendText("Eror");
					ConsoleUtilsPrintf("ERRO OPERANDO\n");
					break;
				}
			}
			flagFNS = 0;
			flagOPR = 0;
			n1 = 0;
			n2 = 0;
			op = 0;

			while(ch!=128);
			sendData(0x0f,COMMAND);
			sendData(0x01,COMMAND);
		}

		//while(ch!=128){

		//}
	}

	return(0);
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to disable wacht dog.
*END*-----------------------------------------------------------*/
static void disableWachtDog(){
  HWREG(SOC_WDT_1_REGS+WDT_WSPR) = 0xAAAA;
  delay(30);
  HWREG(SOC_WDT_1_REGS+WDT_WSPR) = 0x5555;
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to get the number pressed o the
* keypad.
*END*-----------------------------------------------------------*/
static int verifyLine(){
	GPIOPinWrite(SOC_GPIO_1_REGS,28,1);
	if(GPIOPinRead(SOC_GPIO_1_REGS,12)){
		return 1;
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,13)){
		return 2;
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,14)){
		return 3;
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,15)){
		return 43; // 43 (char) = +
	}
	GPIOPinWrite(SOC_GPIO_1_REGS,28,0);

	GPIOPinWrite(SOC_GPIO_1_REGS,18,1);
	if(GPIOPinRead(SOC_GPIO_1_REGS,12)){
		return 4;
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,13)){
		return 5;
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,14)){
		return 6;
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,15)){
		return 45; // 45 (char) = -
	}
	GPIOPinWrite(SOC_GPIO_1_REGS,18,0);

	GPIOPinWrite(SOC_GPIO_1_REGS,19,1);
	if(GPIOPinRead(SOC_GPIO_1_REGS,12)){
		return 7;
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,13)){
		return 8;
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,14)){
		return 9;
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,15)){
		return 42; // 42 (char) = *
	}
	GPIOPinWrite(SOC_GPIO_1_REGS,19,0);

	GPIOPinWrite(SOC_GPIO_1_REGS,16,1);
	if(GPIOPinRead(SOC_GPIO_1_REGS,12)){
		return 128; // 46 (char) = .
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,13)){
		return 0;
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,14)){
		return 61; // 46 (char) = =
	}else if(GPIOPinRead(SOC_GPIO_1_REGS,15)){
		return 47; // 43 (char) = /
	}
	GPIOPinWrite(SOC_GPIO_1_REGS,16,0);
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to convert a in to char*.
*END*-----------------------------------------------------------*/
char * itoa (int value, char *result, int base){
    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to initialize UART.
*END*-----------------------------------------------------------*/
static void initSerial(){
	/* Initialize console for communication with the Host Machine */
    	ConsoleUtilsInit();

    	/* Select the console type based on compile time check */
    	ConsoleUtilsSetType(CONSOLE_UART);
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to initialize GPIO like LED.
*END*-----------------------------------------------------------*/
static void initLed(unsigned int baseAdd, unsigned int module, unsigned int pin){

    	/* Selecting GPIO pin for use. */
    	GPIOPinMuxSetup(module, pin);

    	/* Setting the GPIO pin as an output pin. */
    	GPIODirModeSet(baseAdd, pin, GPIO_DIR_OUTPUT);
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to initialize GPIO like BUTTON.
*END*-----------------------------------------------------------*/
static void initButton(unsigned int baseAdd, unsigned int module, unsigned int pin){

    	/* Selecting GPIO pin for use. */
    	GPIOPinMuxSetup(module, pin);

    	/* Setting the GPIO pin as an output pin. */
    	GPIODirModeSet(baseAdd, pin, GPIO_DIR_INPUT);
}

/*FUNCTION*-------------------------------------------------------
*
* Function Name : gpioAintcconfigure
* Comments      : Do the necessary gpio configurations on to AINTC.
*END*-----------------------------------------------------------*/
static void gpioAintcConf(void){

    /* Initialize the ARM interrupt control */
    IntAINTCInit();

    /* Registering gpioIsr */
    IntRegister(SYS_INT_GPIOINT1A, gpioIsr);

    /* Set the priority */
    IntPrioritySet(SYS_INT_GPIOINT1A, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the system interrupt */
    IntSystemEnable(SYS_INT_GPIOINT1A);

}

/*FUNCTION*-------------------------------------------------------
*
* Function Name : gpioIsr
* Comments      : DMTimer interrupt service routine.
*END*-----------------------------------------------------------*/
static void gpioIsr(void) {
    	/*	Clear wake interrupt	*/
	//HWREG(SOC_GPIO_1_REGS + 0x3C) = 0x1000;
	//HWREG(SOC_GPIO_1_REGS + 0x34) = 0x1000;
	//HWREG(SOC_GPIO_1_REGS + 0x2C) = 0x10000;
	flagISR = 1;

	GPIOPinWrite(SOC_GPIO_1_REGS,28,0);
	GPIOPinWrite(SOC_GPIO_1_REGS,18,0);
	GPIOPinWrite(SOC_GPIO_1_REGS,19,0);
	GPIOPinWrite(SOC_GPIO_1_REGS,16,0);
	ch = verifyLine();
	GPIOPinWrite(SOC_GPIO_1_REGS,28,1);
	GPIOPinWrite(SOC_GPIO_1_REGS,18,1);
	GPIOPinWrite(SOC_GPIO_1_REGS,19,1);
	GPIOPinWrite(SOC_GPIO_1_REGS,16,1);

	HWREG(SOC_GPIO_1_REGS + GPIO_IRQSTATUS_0) = (1<<12);
	HWREG(SOC_GPIO_1_REGS + GPIO_IRQSTATUS_0) = (1<<13);
	HWREG(SOC_GPIO_1_REGS + GPIO_IRQSTATUS_0) = (1<<14);
	HWREG(SOC_GPIO_1_REGS + GPIO_IRQSTATUS_0) = (1<<15);

}

/*FUNCTION*-------------------------------------------------------
*
* Function Name : gpioPinIntConfig
* Comments      :
*END*-----------------------------------------------------------*/
static int gpioPinIntConf(unsigned int baseAdd, unsigned int intLine,
                  unsigned int pinNumber){

    	/* Setting interrupt GPIO pin. */
    	gpioPinIntEnable(baseAdd,
               intLine,
               pinNumber);

    	return(0);
}

/*FUNCTION*-------------------------------------------------------
*
* Function Name : GPIOPinIntEnable
* Comments      : This API enables the configured interrupt event on a specified input
* GPIO pin to trigger an interrupt request.
*
* \param  baseAdd     The memory address of the GPIO instance being used
* \param  intLine     This specifies the interrupt request line on which the
*                     interrupt request due to the transitions on a specified
*                     pin be propagated
* \param  pinNumber   The number of the pin in the GPIO instance
*
* 'intLine' can take one of the following two values:
* - GPIO_INT_LINE_1 - interrupt request be propagated over interrupt line 1\n
* - GPIO_INT_LINE_2 - interrupt request be propagated over interrupt line 2\n
*
* 'pinNumber' can take one of the following values:
* (0 <= pinNumber <= 31)\n
*
* \return None
*
*END*-----------------------------------------------------------*/
static void gpioPinIntEnable(unsigned int baseAdd,
                      unsigned int intLine,
                      unsigned int pinNumber){
    if(GPIO_INTC_LINE_1 == intLine){
        HWREG(baseAdd + GPIO_IRQSTATUS_SET(0)) = (1 << pinNumber);
    }else{
        HWREG(baseAdd + GPIO_IRQSTATUS_SET(1)) = (1 << pinNumber);
    }
}

/*FUNCTION*-------------------------------------------------------
*
* Function Name : gpioAintcconfigure
* Comments      : This API configures the event type for a specified
* input GPIO pin. Whenever the selected event occurs on that GPIO pin
* and if interrupt generation is enabled for that pin, the GPIO module
* will send an interrupt to CPU.
*
* \param  baseAdd    The memory address of the GPIO instance being used
* \param  pinNumber  The number of the pin in the GPIO instance
* \param  eventType  This specifies the event type on whose detection,
*                    the GPIO module will send an interrupt to CPU,
*                    provided interrupt generation for that pin is enabled.
*
* 'pinNumber' can take one of the following values:
* (0 <= pinNumber <= 31)\n
*
* 'eventType' can take one of the following values:
* - GPIO_INT_TYPE_NO_LEVEL - no interrupt request on occurence of either a
*   logic LOW or a logic HIGH on the input GPIO pin\n
* - GPIO_INT_TYPE_LEVEL_LOW - interrupt request on occurence of a LOW level
*   (logic 0) on the input GPIO pin\n
* - GPIO_INT_TYPE_LEVEL_HIGH - interrupt request on occurence of a HIGH level
*   (logic 1) on the input GPIO pin\n
* - GPIO_INT_TYPE_BOTH_LEVEL - interrupt request on the occurence of both the
*   LOW level and HIGH level on the input GPIO pin\n
* - GPIO_INT_TYPE_NO_EDGE -  no interrupt request on either rising or
*   falling edges on the pin\n
* - GPIO_INT_TYPE_RISE_EDGE - interrupt request on occurence of a rising edge
*   on the input GPIO pin\n
* - GPIO_INT_TYPE_FALL_EDGE - interrupt request on occurence of a falling edge
*   on the input GPIO pin\n
* - GPIO_INT_TYPE_BOTH_EDGE - interrupt request on occurence of both a rising
*   and a falling edge on the pin\n
*
* \return  None
*
* \note  A typical use case of this API is explained below:
*
*        If it is initially required that interrupt should be generated on a
*        LOW level only, then this API can be called with
*        'GPIO_INT_TYPE_LEVEL_LOW' as the parameter.
*        At a later point of time, if logic HIGH level only should be made as
*        the interrupt generating event, then this API has to be first called
*        with 'GPIO_INT_TYPE_NO_LEVEL' as the parameter and later with
*        'GPIO_INT_TYPE_LEVEL_HIGH' as the parameter. Doing this ensures that
*        logic LOW level trigger for interrupts is disabled.
*END*-----------------------------------------------------------*/
static void gpioIntTypeSet(unsigned int baseAdd,
                    unsigned int pinNumber,
                    unsigned int eventType){
    eventType &= 0xFF;

    switch(eventType)
    {

        case GPIO_INT_TYPE_NO_LEVEL:

            /* Disabling logic LOW level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) &= ~(1 << pinNumber);

            /* Disabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_LEVEL_LOW:

            /* Enabling logic LOW level detect interrupt geenration. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) |= (1 << pinNumber);

            /* Disabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) &= ~(1 << pinNumber);

            /* Disabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) &= ~(1 << pinNumber);

            /* Disabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_LEVEL_HIGH:

            /* Disabling logic LOW level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) &= ~(1 << pinNumber);

            /* Enabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) |= (1 << pinNumber);

            /* Disabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) &= ~(1 << pinNumber);

            /* Disabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_BOTH_LEVEL:

            /* Enabling logic LOW level detect interrupt geenration. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) |= (1 << pinNumber);

            /* Enabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) |= (1 << pinNumber);

            /* Disabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) &= ~(1 << pinNumber);

            /* Disabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_NO_EDGE:

            /* Disabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) &= ~(1 << pinNumber);

            /* Disabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_RISE_EDGE:

            /* Enabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) |= (1 << pinNumber);

            /* Disabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) &= ~(1 << pinNumber);

            /* Disabling logic LOW level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) &= ~(1 << pinNumber);

            /* Disabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_FALL_EDGE:

            /* Disabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) &= ~(1 << pinNumber);

            /* Enabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) |= (1 << pinNumber);

            /* Disabling logic LOW level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) &= ~(1 << pinNumber);

            /* Disabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_BOTH_EDGE:

            /* Enabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) |= (1 << pinNumber);

            /* Enabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) |= (1 << pinNumber);

            /* Disabling logic LOW level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) &= ~(1 << pinNumber);

            /* Disabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) &= ~(1 << pinNumber);

        break;

        default:
        break;
    }
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to initialize the LCD.
*END*-----------------------------------------------------------*/
void LCDInit(){
  initPins();
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to send Data to LCD.
*END*-----------------------------------------------------------*/
void sendData(unsigned char cmd,unsigned int FLAG){
  //SETTING RS PIN TO RECIVE A COMMAND OR A DATA ===> 0 IS COMMAND 1 IS DATA
  GPIOPinWrite(SOC_GPIO_2_REGS,LCD_RS,FLAG);
  for(i=0;i<8;i++){
    //SETTING 1 OR HIGHER IF THE BIT i OF CHAR IS HIGH ELSE SET 0
    data[i] = cmd & (1<<i);
  }
  writeLcdPins();
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to write on the pins of the lcd.
*END*-----------------------------------------------------------*/
void writeLcdPins(){
  //SET ALL DATA PINS TO REFERENCE DATA VECTOR
  for(i=0;i<8;i++){
      if(data[i]!=0){
        //IF DATA VECTOR HAS 1 OR > 1 VALUE SET THE DATA PIN TO HIGH
        GPIOPinWrite(SOC_GPIO_2_REGS,lcdPins[i],HIGH);
      }else{
        //IF DATA VECTOR HAS 0 VALUE SET THE DATA PIN TO LOW
        GPIOPinWrite(SOC_GPIO_2_REGS,lcdPins[i],LOW);
      }
    }
    toogleEnable();
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to rise and fall the Enable pin of lcd.
*END*-----------------------------------------------------------*/
void toogleEnable(){
  //PULSE ENABLE TO SEND COMMANDS/DATA
  GPIOPinWrite(SOC_GPIO_2_REGS,LCD_EN,HIGH);
  for(i=1000;i--;){}
  GPIOPinWrite(SOC_GPIO_2_REGS,LCD_EN,LOW);
  //SET ALL DATA BITS TO LOW
  setLow();
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to low all pins of lcd.
*END*-----------------------------------------------------------*/
void setLow(){
  //AFTER SEND ANY COMMAND OR DATA RESET ALL DATA BITS TO 0 (ZERO)
  for(i=0;i<8;i++){
    GPIOPinWrite(SOC_GPIO_2_REGS,lcdPins[i],LOW);
  }
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to initialize all pins of LCD
*END*-----------------------------------------------------------*/
void initPins(){
  //INITIALIZE ALL PINS USED FROM LCD
  for(i=0;i<10;i++){
		initPin(SOC_GPIO_2_REGS, 2, lcdPins[i]);
  }
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to initialize a pin of LCD.
*END*-----------------------------------------------------------*/
static void initPin(unsigned int baseAdd, unsigned int module, unsigned int pin){

    	/* Selecting GPIO pin for use. */
    	GPIOPinMuxSetup(module, pin);

    	/* Setting the GPIO pin as an output pin. */
    	GPIODirModeSet(baseAdd, pin, GPIO_DIR_OUTPUT);
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to send text for the lcd.
*END*-----------------------------------------------------------*/
void sendText(char* text){
  for(i=0;i<8;i++){
		if(text[i] > 48){
    sendData(text[i],DATA);
	}else{
		sendData(' ',DATA);
	}
  }
}

/* End of file */
