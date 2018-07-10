/*
159.270 Assignment 6 - Terry Rozmus 94131529
*/
// NOTE: This setup assumes the Nucleo program
// has either been started first or been reset
// before the dialog interface is started
#include "mbed.h"
#include "uart.h"
#include <iostream>
#include <sstream>
#include <string>

#define LCD_DATA 1
#define LCD_INSTRUCTION 0

using namespace std;

// Set up pin to read state of the user button  
DigitalIn button(PC_13);

// Set up pins to control the serial communication line
//RawSerial pc(PA_2, PA_3);

// Set up UART communication
Uart uart(USBTX, USBRX);

// Set up pins to write the state of Green and Blue LEDs
DigitalOut greenLED(PA_5);
DigitalOut blueLED(PA_7);

// Set up pins for writing data to the LCD screen
BusOut lcdData(PA_9, PA_8, PB_10, PB_4);
DigitalOut lcdEN(PC_7), lcdRS(PB_6);

// Set up pins for taking distance measurements with the ultrasonic sensor
PwmOut usTrig(PB_3);
DigitalIn usEcho(PB_5);

// Set up pin for controlling the servo motor
PwmOut servo(PB_0);


class LEDs
{
	public:
		/*
		Public Member Functions
		*/		
		// LEDs class constructor method declaration
		LEDs() {
			// Set the state variables for each LED
			greenOn = true;
			blueOn = true;
			
			// Make sure the LEDs are initally turned on
			greenLED.write(1);
			blueLED.write(1);
		}
		
		// LED Toggles
		void toggleGreen() {
			greenOn = greenOn ^ true;
			greenLED.write(greenOn);
		}
		void toggleBlue() {
			blueOn = blueOn ^ true;
			blueLED.write(blueOn);
		}			
	
	private:
		// LED on/off state variables
		bool greenOn;
		bool blueOn;
};

class Stepper
{
	public:
		/*
		Public Member Functions
		*/		
		// Stepper class constructor method declaration
		Stepper(PinName pin0, PinName pin1, PinName pin2, PinName pin3, int delay = 50) :
		    stepperMagnets(pin0, pin1, pin2, pin3) 
		{
			// Initial direction forward
			direction = 1;
			
			// Initial active pole
			activeMagnet = 0;
			
			// Initial delay
			this->delay = delay;
			
			// initilise time data members
			now = 0;
			lastStepTime = 0;
			
			// Activate the stepper motor
			activated = false;
			toggleActivated();
		}
		
		// Energise the next pole of the stepper motor with
		// all other poles de-energised.
		void nextMagnet();
		
		// Toggle activated
		void toggleActivated(void);
		
		// Toggle the direction
		void toggleDirection(void) {
			direction = direction * -1;
		}

		// Set stepper delay
		void setDelay(int delay) {
			this->delay = delay;
			// Stepper will not work with a delay of 
			// less than 2ms
			if (this->delay < 2) this->delay = 2;
		}
		
		// Increment now time.
		void run (void) {
			now++;
			nextMagnet();
		}
		
	private:
		/*
		Private Data Members
		*/
		// Set up stepper control bus
		BusOut stepperMagnets; 
		
		// Attribute to store direction.
		signed char direction;
		
		// Activated flag
		bool activated;
		
		// Stepper delay
		int delay;
		
		// Establish which pole is 
		// currently energised.
		signed char activeMagnet;
		
		// Current time
		int now;
		
		// The last stepper motor step time
		int lastStepTime;	
		
		// Ticker object
		Ticker t;
		
		/*
		Private Member Functions
		*/
		// Update last step time
		void updateLastStepTime(void) {
			lastStepTime = lastStepTime + this->delay;
		}
};

class LCD
{
	public:
		/*
		Public Member Functions
		*/		
		// Stepper class constructor method declaration
		LCD() {
			lcdEN.write(0); //-- GPIO_WriteBit(GPIOC, LCD_EN, Bit_RESET);
			wait_us(15000);  //-- delay for >15msec second after power on

			lcdInit8Bit(0x30);   //-- we are in "8bit" mode
			wait_us(4100);       //-- 4.1msec delay
			lcdInit8Bit(0x30);   //-- but the bottom 4 bits are ignored
			wait_us(100);        //-- 100usec delay
			lcdInit8Bit(0x30);
			lcdInit8Bit(0x20);
			lcdCommand(0x28);      //-- we are now in 4bit mode, dual line
			lcdCommand(0x08);      //-- display off
			lcdCommand(0x01);      //-- display clear
			wait_us(2000);         //-- needs a 2msec delay !!
			lcdCommand(0x06);      //-- cursor increments
			lcdCommand(0x0f);      //-- display on, cursor(blinks) on
		}
		
		void lcdCommand(unsigned char command);
		void putChar(unsigned char c);
		void putString(char *s, int offset);
		void clearDisplay();

		void lcdSetRS(int mode); //-- mode is either LCD_DATA or LCD_INSTRUCTION
		void lcdPulseEN(void);
		void lcdInit8Bit(unsigned char command); 		
};

class UltrasonicSensor
{
	public:
		UltrasonicSensor(void) {
			usTrig.period_us(100000);
			usTrig.pulsewidth_us(10);
		}
		
		void measure(void);			// Measure if enough time has passed
		void setReady(void);		// Signal that enough time has passed to take the next measurement
		void toggleActivated(void); // Toggle measuring on/off
		
	private:
		int pulseWidth;   // pulse width in microseconds
		bool ready;		  // indicates the system is ready to make an ultrasonic measurement
		bool activated;	  // indicates the the ultrasonic sensor is in active measuring mode
		Ticker t;
};

class ServoMotor
{
	public:
		ServoMotor(void) {
			servo.period_us(20000);  //-- 20 ms time period
		}
		
		void setAngle(int);
};


/* 
Stepper class member functions
*/
// Method declaration for a function which energises the 
// next pole of the stepper motor with all other poles 
// de-energised.
void Stepper::nextMagnet(void)
{
	// Return if not enough time has elapsed since the 
	// last magnet change
	if (now - lastStepTime < delay) return;
	
	// Move to the next active magnet (wrap around 4 to 0 using modulus if in forward direction mode) 
	activeMagnet = (activeMagnet + direction) % 4;
	
	// Wrap around -1 to 3 if in reverse direction mode
	if (activeMagnet == -1) activeMagnet = 3;
	
	// Switch to the next configuration of the stepper motor magnets
	stepperMagnets = 1 << activeMagnet;
	
	// Get ready for processing the next step time
	updateLastStepTime();
}

// Toggle activated
void Stepper::toggleActivated(void) {
	activated = activated ^ true;
	
	if (activated) {
		// Set up the ticker object 
		t.attach_us(callback(this, &Stepper::run), 1000);
	} else {
		// De-magnetise all magnets and stop the ticker
		stepperMagnets = 0;
		t.detach();
	}
}

/*
LCD Class member functions
*/
void LCD::lcdSetRS(int mode)
{
    lcdRS.write(mode);
}

void LCD::lcdPulseEN(void)
{
    lcdEN.write(1);
    wait_us(1);      //-- enable pulse must be >450ns
    lcdEN.write(0);
    wait_us(1);
}

void LCD::lcdInit8Bit(unsigned char command)
{
    lcdSetRS(LCD_INSTRUCTION);
    lcdData.write(command>>4); //-- bottom 4 bits
    lcdPulseEN();
    wait_us(37);             //-- let it work on the data
}

void LCD::lcdCommand(unsigned char command)
{
    lcdSetRS(LCD_INSTRUCTION);
    lcdData.write(command>>4);
    lcdPulseEN();              //-- this can't be too slow or it will time out
    lcdData.write(command & 0x0f);
    lcdPulseEN();
    wait_us(37);             //-- let it work on the data
}

void LCD::putChar(unsigned char c)
{
    lcdSetRS(LCD_DATA);
    lcdData.write(c>>4);
    lcdPulseEN();              //-- this can't be too slow or it will time out
    lcdData.write(c & 0x0f);
    lcdPulseEN();
    wait_us(37);             //-- let it work on the data
}

void LCD::putString(char *s, int offset = 0) {
	int i = 0;
	while (i < offset) {
		i++;
		s++;
	}
	i = 0;
	while (*s > 31) {
		if (i == 16) LCD::lcdCommand(0xC0); // Move to the next line
		LCD::putChar(*s);
		s++;
		i++;
	}
	LCD::lcdCommand(0xC0);
}

void LCD::clearDisplay(void) {
	lcdCommand(0x08);      //-- display off
	lcdCommand(0x01);      //-- display clear
	wait_us(2000);         //-- needs a 2msec delay !!
	lcdCommand(0x06);      //-- cursor increments
	lcdCommand(0x0C);      //-- display on, cursor(blinks) off
}

/*
UltrasonicSensor class members
*/
void UltrasonicSensor::measure(void) {
	// Return if the ticker has not reached the next ready state
	// or the measuring is not currently activated
	if (!activated || !ready) {
		return;
	}
	// Reset ready state
	ready = false;
	
	Timer stopwatch;
	while (! usEcho.read()); //-- wait for Echo signal to go high
	stopwatch.start();
	while (usEcho.read()); //-- wait for Echo signal to go low
	stopwatch.stop();
	pulseWidth = stopwatch.read_us();
	//printf("U %d\n", pulseWidth);  // Raw serial-based version
	
	// Put command letter and number in char array
	// and set this to the UART
	char * s = new char[10];
	int l;
    stringstream ss;
	ss << pulseWidth;
	ss >> s;
	l = strlen(s);
	// Put "U " in front of the char array.
	for (int i = 0; i < l; i++) {
		s[i + 2] = s[i];
	}
	s[0] = 'U';
	s[1] = ' ';
	s[l + 2] = '\n';
	s[l + 3] = '\0';
	uart.writeLine(s);
	delete [] s;
}

// Toggle activated
void UltrasonicSensor::toggleActivated(void) {
	activated = activated ^ true;
	
	if (activated) {
		// Set up the ticker object 
		t.attach_us(callback(this, &UltrasonicSensor::setReady), 100000);
	} else {
		t.detach();
	}
}

void UltrasonicSensor::setReady(void) {
	// Set state indicating the system is ready to receive a
	// a reading from the ultrasonic sensor
	ready = true;
}

/*
ServoMotor class member functions
*/
void ServoMotor::setAngle(int angle) {
	servo.pulsewidth_us(1000 + angle * 10); // *10 is pretty close to the correct multiplier for my servo
}

		
int main()
{
	// Instantiate Nucleo board control objects
	Stepper step(PC_0, PC_1, PC_2, PC_3, 50); // Start the stepper motor
	LEDs leds;
	LCD screen;
	UltrasonicSensor range;
	ServoMotor servo;
	
	char com[34];  // Buffer for commands from the serial port
	bool errors;   // Flag for errors
	
	//int pressedCount = 0;
	while (1) {
		// Measure the distance using the ultrasonic sensor if it is currently
		// activated for measuring and sufficient time has passed since the
		// the last measurement
		range.measure();
		
		// Read the user button and write 'B' to the serial if pressed
		/* This approach didn't work while the Ultrasonic sensor was running
		// Send 'B' using UART if user button is pressed and held
		if (!button.read()) {
			pressedCount++;
		} else {
			pressedCount = 0;
		}
		if (pressedCount == 100000) {
			// printf("B\n"); // Raw serial-based version
			uart.writeLine("B\n\0");
		}
		*/
		if (!button.read()) {
			uart.writeLine("B\n\0");
		}
		 
		// Check if there is any data on the serial line
		// before checking for commands to process
		// if (pc.readable())  // Raw serial version
		if (uart.canReadLine())
		{
			// Get the command and echo it back to the 
			// terminal
			errors = false;
			
			// Raw serial-based communication
			//fgets(com, 34, stdin);
			//printf("%s\n", com);
			
			// Get line using UART
			uart.readLine(com);
			
			// Process valid commands
			if (com[0] == 'G' || com[0] == 'g') {
				leds.toggleGreen();
			} else
			if (com[0] == 'B' || com[0] == 'b') {
				leds.toggleBlue();
			} else 
			if (com[0] == 'S' || com[0] == 's') {
				step.toggleActivated();
			} else
			if (com[0] == 'T' || com[0] == 't') {
				step.toggleDirection();
			} else
			if (com[0] == 'D' || com[0] == 'd') {
				int i = 1, t = 0;
				// Ignore up to one leading space
				if (com[i] == 32) {
					i++;
				}
				// Process the next continuous string of
				// digits into an integer and set this
				// as the stepper delay
				while (com[i] > 47 && com[i] < 58) {
					// Ignore leading spaces
					//if (com[i] == 32) break; 
					t = t * 10 + com[i] - 48;
					i++;
				}
				if (t > 0) { 
					step.setDelay(t);
				} else {
					errors = true;
				}
			} else
			if (com[0] == 'M' || com[0] == 'm') {
				int i = 1;
				// Ignore up to one leading space
				if (com[i] == 32) {
					i++;
				}
				screen.clearDisplay();
				screen.putString(com, i);
			} else
			if (com[0] == 'U' || com[0] == 'u') {
				range.toggleActivated();
			} else
			if (com[0] == 'A' || com[0] == 'a') {
				int i = 1, a = 0;
				// Ignore up to one leading space
				if (com[i] == 32) {
					i++;
				}
				// Process the next continuous string of
				// digits into an integer and set this
				// as the servo motor angle
				while (com[i] > 47 && com[i] < 58) {
					// Ignore leading spaces
					//if (com[i] == 32) break; 
					a = a * 10 + com[i] - 48;
					i++;
				}
				// Set the servo angle
				servo.setAngle(a);
			} else {
				errors = true;
			}
			if (errors) {
				// printf("Your command had an error.\n"); // Serial version
				uart.writeLine("Your command had an error.\n\0");
			}
		}
	}
}