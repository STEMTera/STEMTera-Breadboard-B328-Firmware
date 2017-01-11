/*
			LUFA Library
	Copyright (C) Dean Camera, 2010.
			
dean [at] fourwalledcubicle [dot] com
	www.fourwalledcubicle.com
*/

/*
Copyright 2010  Dean Camera (dean [at] fourwalledcubicle [dot] com)

Permission to use, copy, modify, distribute, and sell this 
software and its documentation for any purpose is hereby granted
without fee, provided that the above copyright notice appear in 
all copies and that both that the copyright notice and this
permission notice and warranty disclaimer appear in supporting 
documentation, and that the name of the author not be used in 
advertising or publicity pertaining to distribution of the 
software without specific, written prior permission.

The author disclaim all warranties with regard to this
software, including all implied warranties of merchantability
and fitness.  In no event shall the author be liable for any
special, indirect or consequential damages or any damages
whatsoever resulting from loss of use, data or profits, whether
in an action of contract, negligence or other tortious action,
arising out of or in connection with the use or performance of
this software.
*/

/** \file
*
*  Main source file for the STEMTera-usbserial project. This file contains the main tasks of
*  the project and is responsible for the initial application hardware configuration.
*/

#include "STEMTera-Breadboard-usbserial.h"


#ifdef STEMTERA_FACTORY
#define ATMEGA_FAILED	0xFD
#define ATMEGA_PASSED	0xFE

/* Text stored in PROGMEM (Flash) */

#if defined(__AVR_ATmega32U2__)
	const char StartText[] PROGMEM = "\r\nSTEMTera Breadboard Factory Test.\r\nATMEGA32U2 ";
#elif defined(__AVR_ATmega16U2__)
	const char StartText[] PROGMEM = "\r\nSTEMTera Breadboard Factory Test.\r\nATMEGA16U2 ";
#endif

const char DeviceIdText[] PROGMEM = "\r\nDevice ID: ";
const char SerialText[] PROGMEM = "Serial No: ";
const char NotEqual[] PROGMEM = " != ";
const char TestText[] PROGMEM = "TEST ";
const char Failed[] PROGMEM = " FAILED!";
const char BoardFailed[] PROGMEM = "\r\nWARNING!!! FAILED one or multiple tests. WARNING!!!\r\n\r\n";
const char BoardPassed[] PROGMEM = "\r\nBoard PASSED.\r\n";
const char Port PROGMEM = 'P';
const char Or[] PROGMEM = " or ";
const char Ok[] PROGMEM = "Ok.\r\n";
const char HEX[] PROGMEM = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
bool TestActivated = false;
bool HasFailed = false;
bool Passed = false;
int TestNumber = 0;
long DelayCount = 0;
#endif



/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
RingBuff_t USBtoUSART_Buffer;

/** Circular buffer to hold data from the serial port before it is sent to the host. */
RingBuff_t USARTtoUSB_Buffer;

/** Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type */
volatile struct
{
	uint8_t TxLEDPulse; /**< Milliseconds remaining for data Tx LED pulse */
	uint8_t RxLEDPulse; /**< Milliseconds remaining for data Rx LED pulse */
	uint8_t PingPongLEDPulse; /**< Milliseconds remaining for enumeration Tx/Rx ping-pong LED pulse */
} PulseMSRemaining;

/** LUFA CDC Class driver interface configuration and state information. This structure is
*  passed to all CDC Class driver functions, so that multiple instances of the same class
*  within a device can be differentiated from one another.
*/
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
	.Config = 
	{
		.ControlInterfaceNumber         = 0,

		.DataINEndpointNumber           = CDC_TX_EPNUM,
		.DataINEndpointSize             = CDC_TXRX_EPSIZE,
		.DataINEndpointDoubleBank       = false,

		.DataOUTEndpointNumber          = CDC_RX_EPNUM,
		.DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
		.DataOUTEndpointDoubleBank      = false,

		.NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
		.NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
		.NotificationEndpointDoubleBank = false,
	},
};

/** Main program entry point. This routine contains the overall program flow, including initial
*  setup of all components and the main program loop.
*/
int main(void)
{
	SetupHardware();
	
	RingBuffer_InitBuffer(&USBtoUSART_Buffer);
	RingBuffer_InitBuffer(&USARTtoUSB_Buffer);

	sei();
	
	for (;;)
	{
		/* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
		if (!(RingBuffer_IsFull(&USBtoUSART_Buffer)))
		{
			int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

			/* Read bytes from the USB OUT endpoint into the USART transmit buffer */
			if (!(ReceivedByte < 0))
			RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
		}
		
		/* Check if the UART receive buffer flush timer has expired or the buffer is nearly full */
		RingBuff_Count_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
		if ((TIFR0 & (1 << TOV0)) || (BufferCount > BUFFER_NEARLY_FULL))
		{
			TIFR0 |= (1 << TOV0);

			if (USARTtoUSB_Buffer.Count) {
				LEDs_TurnOnLEDs(LEDMASK_TX);
				PulseMSRemaining.TxLEDPulse = TX_RX_LED_PULSE_MS;
			}

			/* Read bytes from the USART receive buffer into the USB IN endpoint */
			while (BufferCount--)
			CDC_Device_SendByte(&VirtualSerial_CDC_Interface, RingBuffer_Remove(&USARTtoUSB_Buffer));
			
			/* Turn off TX LED(s) once the TX pulse period has elapsed */
			if (PulseMSRemaining.TxLEDPulse && !(--PulseMSRemaining.TxLEDPulse))
			LEDs_TurnOffLEDs(LEDMASK_TX);

			/* Turn off RX LED(s) once the RX pulse period has elapsed */
			if (PulseMSRemaining.RxLEDPulse && !(--PulseMSRemaining.RxLEDPulse))
			LEDs_TurnOffLEDs(LEDMASK_RX);
		}
		
		/* Load the next byte from the USART transmit buffer into the USART */
		if (!(RingBuffer_IsEmpty(&USBtoUSART_Buffer))) {
			Serial_TxByte(RingBuffer_Remove(&USBtoUSART_Buffer));
			
			LEDs_TurnOnLEDs(LEDMASK_RX);
			PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;
		}
		
		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
#ifdef STEMTERA_FACTORY
		DoTest();
#endif
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Hardware Initialization */
#ifdef STEMTERA_FACTORY
	SetupTestPins();
#endif
	Serial_Init(9600, false);
	LEDs_Init();
	USB_Init();
	
	/* Start the flush timer so that overflows occur rapidly to push received bytes to the USB interface */
	TCCR0B = (1 << CS02);
	
	/* Pull target /RESET line high */
	AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
	AVR_RESET_LINE_DDR  |= AVR_RESET_LINE_MASK;
	

}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Unhandled Control Request event. */
void EVENT_USB_Device_UnhandledControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
*
*  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
*/
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
	case CDC_PARITY_Odd:
		ConfigMask = ((1 << UPM11) | (1 << UPM10));		
		break;
	case CDC_PARITY_Even:
		ConfigMask = (1 << UPM11);		
		break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
	ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
	case 6:
		ConfigMask |= (1 << UCSZ10);
		break;
	case 7:
		ConfigMask |= (1 << UCSZ11);
		break;
	case 8:
		ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
		break;
	}

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	/* Special case 57600 baud for compatibility with the ATmega328 bootloader. */	
	UBRR1  = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600)
	? SERIAL_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS)
	: SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);	

	UCSR1C = ConfigMask;
	UCSR1A = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600) ? 0 : (1 << U2X1);
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
*  for later transmission to the host.
*/
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	if (USB_DeviceState == DEVICE_STATE_Configured)
	RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}

/** Event handler for the CDC Class driver Host-to-Device Line Encoding Changed event.
*
*  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
*/
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	bool CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);

	if (CurrentDTRState)
	AVR_RESET_LINE_PORT &= ~AVR_RESET_LINE_MASK;
	else
	AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
}

#ifdef STEMTERA_FACTORY
void SetupTestPins(void) {
	// Setup PORTB with internal pull-up
	PORTB = 0xff;
	// PB7, PB6, PB3 and PB2 set as OUTPUT
	DDRB  = (1<<7)|(1<<6)|(1<<3)|(1<<2);
	
	// Setup PORTC with internal pull-up	
	PORTC = 0xff;
	// PC6 and PC5 set as OUTPUT
	DDRC = (1<<6)|(1<<5);
	
	PORTD = 0xff;
	// Leave PORTD to default
}

void DoTest(void) {	
	uint8_t temp=0;
	uint8_t serial = 0;
#if defined(__AVR_ATmega32U2__)
	uint8_t DeviceID[]={0x1e,0x95,0x8a};	// DEVICE ID for ATMEGA32U2
#elif defined(__AVR_ATmega16U2__)
	uint8_t DeviceID[]={0x1e,0x94,0x89};	// DEVICE ID for ATMEGA16U2
#endif
	
	if (HasFailed) {
		DelayCount++;
		if (DelayCount>=5000) {
			LEDs_ToggleLEDs(LEDMASK_TX | LEDMASK_RX);
			DelayCount=0;
		}
	}
	
	if (Passed) {
		DelayCount++;
		if (DelayCount>=50000) {
			LEDs_ToggleLEDs(LEDMASK_TX | LEDMASK_RX);
			DelayCount=0;
		}
	}
	
	// When PD6 and PC7 are both pulled low by test switch
	if (!TestActivated) {
		if ((!(PIND & (1<<6))) & (!(PINC & (1<<7)))){	
			PORTB ^= (1<<3);
			_delay_ms(30);
			if ((!(PIND & (1<<6))) & (!(PINC & (1<<7)))){
				while( ((!(PIND & (1<<6))) & (!(PINC & (1<<7))))) {}
				TestNumber=0;
				TestActivated=true;
				Passed=false;
				HasFailed=false;
			}
			
			PrintProgmemText(StartText);

			// SERIAL NUMBER
			PrintProgmemText(SerialText);
			for (uint8_t i = 14; i < 24; i += 1) {
				serial = boot_signature_byte_get(i);
				if (USB_DeviceState == DEVICE_STATE_Configured) {
					RingBuffer_Insert(&USARTtoUSB_Buffer, pgm_read_byte(&HEX[(serial & 0xf)]));			
					RingBuffer_Insert(&USARTtoUSB_Buffer, pgm_read_byte(&HEX[(serial>>4)]));
				}
			}

			// DEVICE ID
			temp=0;
			PrintProgmemText(DeviceIdText);
			for (uint8_t i = 0; i < 5; i += 2) {			
				serial = boot_signature_byte_get(i);
				if (serial != DeviceID[i/2]) {
					HasFailed=true;
					PrintProgmemText(Failed);
					RingBuffer_Insert(&USARTtoUSB_Buffer,' ');
					RingBuffer_Insert(&USARTtoUSB_Buffer, pgm_read_byte(&HEX[(serial>>4)&0x0f]));
					RingBuffer_Insert(&USARTtoUSB_Buffer, pgm_read_byte(&HEX[(serial & 0xf)]));			
					PrintProgmemText(NotEqual);
					RingBuffer_Insert(&USARTtoUSB_Buffer, pgm_read_byte(&HEX[(DeviceID[i/2]>>4)&0x0f]));
					RingBuffer_Insert(&USARTtoUSB_Buffer, pgm_read_byte(&HEX[(DeviceID[i/2] & 0xf)]));			
					
					break;
				}
				if (USB_DeviceState == DEVICE_STATE_Configured) {
					RingBuffer_Insert(&USARTtoUSB_Buffer, pgm_read_byte(&HEX[(serial>>4)&0x0f]));
					RingBuffer_Insert(&USARTtoUSB_Buffer, pgm_read_byte(&HEX[(serial & 0xf)]));			
				}
			}
			
			
			if (USB_DeviceState == DEVICE_STATE_Configured) {
				RingBuffer_Insert(&USARTtoUSB_Buffer, '\r');
				RingBuffer_Insert(&USARTtoUSB_Buffer, '\n');			
			}
			
		}
	}
	
	if (TestActivated) {
		TestNumber++;
		//		if (USB_DeviceState == DEVICE_STATE_Configured) RingBuffer_Insert(&USARTtoUSB_Buffer, 0x30+TestNumber);			
		switch (TestNumber) {
		case 1:	
			TestPins(&PORTB,7, &PINB, 5);
			break;
		case 2: 
			TestPins(&PORTB,6, &PINB, 4);
			break;
		case 3: 
			TestPins(&PORTB,3, &PINB, 1);
			break;
		case 4: 
			TestPins(&PORTB,2, &PINB, 0);
			break;
		case 5: 
			TestPins(&PORTC,7, &PIND, 6);
			break;
		case 6: 
			TestPins(&PORTC,6, &PINC, 4);
			break;
		case 7: 
			TestPins(&PORTC,5, &PINC, 2);
			break;
		case 8: 
			TestPins(&PORTD,5, &PIND, 1);
			break;
		case 9: 
			TestPins(&PORTD,4, &PIND, 0);
			break;
		case 10:
			if (HasFailed) { 
				temp=ATMEGA_FAILED;
			}
			else {
				temp=ATMEGA_PASSED;
				Passed=true;
			}
			RingBuffer_Insert(&USBtoUSART_Buffer, temp);
			_delay_ms(100);
			break;
			
		default:
			TestNumber=0;
			TestActivated=false;
			break;
		}
	}
}

void TestPins(volatile uint8_t *OutPort, uint8_t OutBit, volatile uint8_t *InPort, uint8_t InBit) {
	uint8_t SavedStatus=0;
	
	PrintProgmemText(TestText);

	if (USB_DeviceState == DEVICE_STATE_Configured) RingBuffer_Insert(&USARTtoUSB_Buffer, 0x30+TestNumber);			
	if (USB_DeviceState == DEVICE_STATE_Configured) RingBuffer_Insert(&USARTtoUSB_Buffer, ' ');			
	if (USB_DeviceState == DEVICE_STATE_Configured) RingBuffer_Insert(&USARTtoUSB_Buffer, '-');			
	if (USB_DeviceState == DEVICE_STATE_Configured) RingBuffer_Insert(&USARTtoUSB_Buffer, ' ');			

	SavedStatus = *(OutPort-1);	// Save DDR Register
	*(OutPort-1) |= (1<<OutBit);
	*OutPort |= (1<<OutBit);
	_delay_ms(1);
	if (*InPort & (1<<InBit)) {
		*OutPort &= ~(1<<OutBit);
		_delay_ms(1);
		if (*InPort & (1<<InBit)) {
			HasFailed = true;
			PrintFailed(OutPort,InPort,OutBit,InBit);
			//TestNumber=0;
			//TestActivated=false;
		} else {
			PrintProgmemText(Ok);
		}
	} else {
		HasFailed = true;
		PrintFailed(OutPort,InPort,OutBit,InBit);
	}
	
	*(OutPort-1) = SavedStatus;
}

void PrintProgmemText(const char * text) {
	uint8_t TextLen = 0;
	TextLen=strlen_P(&text[0]);
	for (int i=0;i<TextLen;i++) {
		if (USB_DeviceState == DEVICE_STATE_Configured) {
			RingBuffer_Insert(&USARTtoUSB_Buffer,  pgm_read_byte(&text[i]));			
		}
	}	
}

void PrintFailed(volatile uint8_t *FirstPort, volatile uint8_t *SecondPort, uint8_t FirstBit, uint8_t SecondBit) {
	char OutP='X';
	char InP='X';
	
	if (FirstPort == &PORTB) OutP='B';
	if (FirstPort == &PORTC) OutP='C';
	if (FirstPort == &PORTD) OutP='D';

	if (SecondPort == &PINB) InP='B';
	if (SecondPort == &PINC) InP='C';
	if (SecondPort == &PIND) InP='D';


	if (USB_DeviceState == DEVICE_STATE_Configured) {
		RingBuffer_Insert(&USARTtoUSB_Buffer,  Port);
	}	
	if (USB_DeviceState == DEVICE_STATE_Configured) {
		RingBuffer_Insert(&USARTtoUSB_Buffer,  OutP);
	}
	if (USB_DeviceState == DEVICE_STATE_Configured) {
		RingBuffer_Insert(&USARTtoUSB_Buffer,  0x30+FirstBit);
	}
	PrintProgmemText(Or);
	
	if (USB_DeviceState == DEVICE_STATE_Configured) {
		RingBuffer_Insert(&USARTtoUSB_Buffer,  Port);
	}		
	if (USB_DeviceState == DEVICE_STATE_Configured) {
		RingBuffer_Insert(&USARTtoUSB_Buffer,  InP);
	}
	if (USB_DeviceState == DEVICE_STATE_Configured) {
		RingBuffer_Insert(&USARTtoUSB_Buffer,  0x30+SecondBit);
	}
	
	PrintProgmemText(Failed);
	
	if (USB_DeviceState == DEVICE_STATE_Configured) {
		RingBuffer_Insert(&USARTtoUSB_Buffer, '\r');
		RingBuffer_Insert(&USARTtoUSB_Buffer, '\n');			
	}	
	RingBuff_Count_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
	/* Read bytes from the USART receive buffer into the USB IN endpoint */
	while (BufferCount--)
	CDC_Device_SendByte(&VirtualSerial_CDC_Interface, RingBuffer_Remove(&USARTtoUSB_Buffer));	
}
#endif