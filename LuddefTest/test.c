#include <inc/hw_types.h>		// tBoolean
#include <utils/uartstdio.h>	// input/output over UART

#include <RASLib/inc/uart.h> 
#include <RASLib/inc/init.h> 
#include <RASLib/inc/time.h>
#include <RASLib/inc/luddef.h>

#define M_PI 3.14159265358979323846

int main(void) {
    tPose pose = {0};
    tEncoder *rightEnc, *leftEnc;
    float x = 3;
    
    InitializeMCU();
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);				
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);	
	UARTStdioInit(0);
    
    UARTprintf("hi! %04d\n", (int)x);

    rightEnc = InitializeEncoder(PIN_B0, PIN_B1, 0);
    leftEnc = InitializeEncoder(PIN_B2, PIN_B3, 1);
    
    // units = inches
    InitDeadReckoning(&pose, 7.125, 65.6, .1, leftEnc, rightEnc);
    
    while (1) {
        GetCurrentPose(&pose);
        
        UARTprintf("x*100: %04d   y*100: %04d   heading: %04d\r", 
            (int)(pose.x*100.0f),
            (int)(pose.y*100.0f),
            (int)(pose.heading*180.0f/(float)M_PI));
        
        Wait(.1);
    }
}
