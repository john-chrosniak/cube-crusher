// Main.c
// Runs on LM4F120/TM4C123
// You may use, edit, run or distribute this file 
// You are free to change the syntax/organization of this file

// Jonathan W. Valvano 2/20/17, valvano@mail.utexas.edu
// Modified by Sile Shu 10/4/17, ss5de@virginia.edu
// Modified by Mustafa Hotaki 7/29/18, mkh3cf@virginia.edu

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "OS.h"
#include "tm4c123gh6pm.h"
#include "LCD.h"
#include <string.h> 
#include "UART.h"
#include "FIFO.h"
#include "joystick.h"
#include "PORTE.h"

// Constants
#define BGCOLOR     					LCD_BLACK
#define CUBECOLOR 						LCD_WHITE
#define CROSSSIZE            	5
#define PERIOD               	4000000   // DAS 20Hz sampling period in system time units
#define PSEUDOPERIOD         	8000000
#define LIFETIME             	1000
#define RUNLENGTH            	600 // 30 seconds run length
#define VERTICALNUM 6
#define HORIZONTALNUM 6
#define NUMCUBES 5
#define CUBESIZE 17
#define XGRIDSIZE 102
#define YGRIDSIZE 102

uint16_t EXPIRATIONTIME_MS = 5000;
uint16_t CUBEMOVETIME_MS = 100;
uint16_t level = 1;

typedef struct {
 uint32_t position[2];
 Sema4Type BlockFree;
} block;
block BlockArray[HORIZONTALNUM][VERTICALNUM];

typedef struct {
 //position is the location in the 6x6 grid of blocks. Takes on values from 0-5
 uint8_t position[2];
 //size is the number of pixels from the center on each side (size of 3 would make the cube be 7x7 pixels, 4 would be 9x9, etc.)
 uint8_t size;
 int16_t color;
 //has the cube been yeeted?
 bool is_alive;
 uint8_t direction;
 Sema4Type CubeFree;
} cube;
cube CubeArray[NUMCUBES];

extern Sema4Type LCDFree;
Sema4Type scoreFree, lifeFree;
uint16_t origin[2]; 	// The original ADC value of x,y if the joystick is not touched, used as reference
int16_t x = 63;  			// horizontal position of the crosshair, initially 63
int16_t y = 63;  			// vertical position of the crosshair, initially 63
int16_t prevx, prevy;	// Previous x and y values of the crosshair
uint8_t select;  			// joystick push
uint8_t area[2];
uint32_t PseudoCount;
int16_t life = 3;
int16_t score = 0;
bool spawner_active = false;
bool game_started = false;
uint16_t high_score = 0;

unsigned long NumCreated;   		// Number of foreground threads created
unsigned long NumSamples;   		// Incremented every ADC sample, in Producer
unsigned long UpdateWork;   		// Incremented every update on position values
unsigned long Calculation;  		// Incremented every cube number calculation
unsigned long DisplayCount; 		// Incremented every time the Display thread prints on LCD 
unsigned long ConsumerCount;		// Incremented every time the Consumer thread prints on LCD
unsigned long Button1RespTime; 	// Latency for Task 2 = Time between button1 push and response on LCD 
unsigned long Button2RespTime; 	// Latency for Task 7 = Time between button2 push and response on LCD
unsigned long Button1PushTime; 	// Time stamp for when button 1 was pushed
unsigned long Button2PushTime; 	// Time stamp for when button 2 was pushed

//---------------------User debugging-----------------------
unsigned long DataLost;     // data sent by Producer, but not received by Consumer
long MaxJitter;             // largest time jitter between interrupts in usec
#define JITTERSIZE 64
unsigned long const JitterSize=JITTERSIZE;
unsigned long JitterHistogram[JITTERSIZE]={0,};
unsigned long TotalWithI1;
unsigned short MaxWithI1;

void Device_Init(void){
	UART_Init();
	BSP_LCD_OutputInit();
	BSP_Joystick_Init();
}

void Mic_Init(void){
	/**
	 * Title: ADC TM4C123G Tiva C Launchpad – Measure Analog Voltage Signal
	 * URL: Reference: https://microcontrollerslab.com/adc-tm4c123g-tiva-c-launchpad-measure-analog-voltage-signal/
	 **/ 
	SYSCTL_RCGCGPIO_R |= 0x00000008; 
	while((SYSCTL_PRGPIO_R&0x8) != 0x8){};
	// PD.0 Configure for GPIO
	GPIO_PORTD_AFSEL_R |= 0x01;
	GPIO_PORTD_DEN_R &= ~0x01;
	GPIO_PORTD_AMSEL_R |= 0x01;
	// ADC 0 Init Channel 7
	ADC0_ACTSS_R &= ~0x8; // SS3 Disable
	ADC0_EMUX_R &= ~0xF000; // Software Trigger
	ADC0_SSMUX3_R = 7; // Analog Channel
	ADC0_SSCTL3_R |= (1<<1)|(1<<2); // One Sample
	ADC0_ACTSS_R |= 0x8; // SS3 Enable
}

uint16_t Sample_Microphone(void){
	ADC0_PSSI_R |= 0x8; // Start sampling
	while ((ADC0_RIS_R & 0x8) == 0){}; // Wait for sample
	uint16_t adc_value = ADC0_SSFIFO3_R; // Read ADC value
	ADC0_ISC_R |= 0x8; // Clear sample flag
	return adc_value;
}

void Random_Init(){
	Mic_Init();
	// Use Mic to set seed
	uint16_t seed = Sample_Microphone();
	srand(seed);
}

uint8_t getRandomNumber(void) {
	return (uint8_t) rand();
}



//------------------Task 1--------------------------------
// background thread executed at 20 Hz
//******** Producer *************** 
int UpdatePosition(uint16_t rawx, uint16_t rawy, jsDataType* data){
	if (rawx > origin[0]){
		x = x + 2*((rawx - origin[0]) >> 9);
	}
	else{
		x = x - 2*((origin[0] - rawx) >> 9);
	}
	if (rawy < origin[1]){
		y = y + 2*((origin[1] - rawy) >> 9);
	}
	else{
		y = y - 2*((rawy - origin[1]) >> 9);
	}
	if (x > 127-(128-XGRIDSIZE)/2){
		x = 127-(128-XGRIDSIZE)/2;}
	if (x < (128-XGRIDSIZE)/2){
		x = (128-XGRIDSIZE)/2;}
	if (y > YGRIDSIZE-1){
		y = YGRIDSIZE-1;}
	if (y < 0){
		y = 0;}
	data->x = x; data->y = y;
	return 1;
}

void Producer(void){
	uint16_t rawX,rawY; // raw adc value
	uint8_t select;
	jsDataType data;
	unsigned static long LastTime;  // time at previous ADC sample
	unsigned long thisTime;         // time at current ADC sample
	long jitter;                    // time between measured and expected, in us
	BSP_Joystick_Input(&rawX,&rawY,&select);
	thisTime = OS_Time();       // current time, 12.5 ns
	UpdateWork += UpdatePosition(rawX,rawY,&data); // calculation work
	NumSamples++;               // number of samples
	if(JsFifo_Put(data) == 0){ // send to consumer
		DataLost++;
	}
//calculate jitter
	if(UpdateWork > 1){    // ignore timing of first interrupt
		unsigned long diff = OS_TimeDifference(LastTime,thisTime);
		if(diff > PERIOD){
			jitter = (diff-PERIOD+4)/8;  // in 0.1 usec
		}
		else{
			jitter = (PERIOD-diff+4)/8;  // in 0.1 usec
		}
		if(jitter > MaxJitter){
			MaxJitter = jitter; // in usec
		}       // jitter should be 0
		if(jitter >= JitterSize){
			jitter = JITTERSIZE-1;
		}
		JitterHistogram[jitter]++; 
	}
	LastTime = thisTime;
}

//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 2 sec and die
// ***********ButtonWork*************
// void ButtonWork(void){
	
// 	OS_bWait(&LCDFree);
// 	Button1RespTime = OS_MsTime() - Button1PushTime; // LCD Response here
// 	BSP_LCD_FillScreen(BGCOLOR);
// 	//Button1FuncTime = OS_MsTime() - Button1PushTime;
// 	//Button1PushTime = 0;
// 	while (ElapsedTime < LIFETIME){
// 		CurrentTime = OS_MsTime();
// 		ElapsedTime = CurrentTime - StartTime;
// 		BSP_LCD_Message(0,5,0,"Life Time:",LIFETIME);
// 		BSP_LCD_Message(1,0,0,"Horizontal Area:",x);
// 		BSP_LCD_Message(1,1,0,"Vertical Area:",y);
// 		BSP_LCD_Message(1,2,0,"Elapsed Time:",ElapsedTime);
// 		OS_Sleep(50);

// 	}
// 	BSP_LCD_FillScreen(BGCOLOR);
// 	OS_bSignal(&LCDFree);
//   OS_Kill();  // done, OS does not return from a Kill
// } 


//--------------end of Task 2-----------------------------

//------------------Task 3--------------------------------

//******** Consumer *************** 
// foreground thread, accepts data from producer
// Display crosshair and its positions
// inputs:  none
// outputs: none
void Consumer(void){
	while(1){
		jsDataType data;
		JsFifo_Get(&data);
		OS_bWait(&LCDFree);
			
		BSP_LCD_DrawCrosshair(prevx, prevy, LCD_BLACK); // Draw a black crosshair
		BSP_LCD_DrawCrosshair(data.x, data.y, LCD_RED); // Draw a red crosshair

		ConsumerCount++;
		OS_bSignal(&LCDFree);
		prevx = data.x; 
		prevy = data.y;
		//OS_Suspend();
	}
  OS_Kill();  // done
}


//--------------end of Task 3-----------------------------

//************ Display *************** 
// foreground thread, do some pseudo works to test if you can add multiple periodic threads
// inputs:  none
// outputs: none
void Display(void){
	while(1){
		OS_bWait(&LCDFree);
		BSP_LCD_Message(1, 5, 0, "Life:",life);		
		BSP_LCD_Message(1, 5, 9, "Score:",score);
		BSP_LCD_Message(0, 0, 6, "Level:", level);
		//BSP_LCD_Message(1,4,0,"PseudoCount: ",PseudoCount);
		DisplayCount++;
		OS_bSignal(&LCDFree);
		//OS_Sleep(1);
		//OS_Suspend();

	}
  OS_Kill();  // done
}

//--------------end of Task 6-----------------------------

//------------------Task 8--------------------------------
// 
// 
// This task implements the motions of the cubes
void CubeThread (void){
	int i;
	cube * c = 0;
	for (i=0; i<NUMCUBES; i++){
		if (OS_bTry(&(CubeArray[i].CubeFree)) && !CubeArray[i].is_alive){
			c = &(CubeArray[i]);
			c->is_alive = true;
			break;
		}
	}
	if (c == 0){
		OS_Kill();
	}
	bool found_start = false;
	while (!found_start){
		uint8_t cube_posx = getRandomNumber()/(255/HORIZONTALNUM+1);
		uint8_t cube_posy = getRandomNumber()/(255/VERTICALNUM+1);
		if (OS_bTry(&(BlockArray[cube_posy][cube_posx].BlockFree))){
			c->position[0] = cube_posy;
			c->position[1] = cube_posx;
			OS_bWait(&LCDFree);
			BSP_LCD_Cube(CUBESIZE*c->position[1]+CUBESIZE/2+13, CUBESIZE*c->position[0]+CUBESIZE/2, CUBESIZE, CUBECOLOR);
			OS_Signal(&LCDFree);
			c->direction = getRandomNumber()/64;
			found_start = true;
		}

	}
	unsigned long cube_start_time = OS_MsTime();
	unsigned long last_move_time = OS_MsTime();
	while (c->is_alive && life){
		// first, check if the object is hit by the crosshair
		if((c->position[0] == (y-4) / CUBESIZE  && c->position[1] == (x - 13) / CUBESIZE) ||
		   (c->position[0] == (y+4) / CUBESIZE  && c->position[1] == (x - 13) / CUBESIZE) || 
		   (c->position[0] == y / CUBESIZE  && c->position[1] == (x - 17) / CUBESIZE) || 
		   (c->position[0] == y / CUBESIZE  && c->position[1] == (x - 9) / CUBESIZE)){
			// Increase the score
			c->is_alive = false;
			OS_bWait(&LCDFree);
			BSP_LCD_Cube(CUBESIZE*c->position[1]+CUBESIZE/2+13, CUBESIZE*c->position[0]+CUBESIZE/2, CUBESIZE, BGCOLOR);
			OS_CreateSound(262, 1);
			OS_Signal(&LCDFree);
			OS_bWait(&scoreFree);
			score++;
			if (score % 10 == 0) {
				if (EXPIRATIONTIME_MS > 3000) {
					EXPIRATIONTIME_MS -= 200;
					CUBEMOVETIME_MS -= 5;
					level++;
				}
			}
			OS_bSignal(&scoreFree);
			OS_bSignal(&(BlockArray[c->position[0]][c->position[1]].BlockFree));
			OS_bSignal(&(c->CubeFree));
		}
		// second, check if the object is expired
		else if (OS_MsTime() - cube_start_time > EXPIRATIONTIME_MS){
			// Decrease the life
			c->is_alive = false;
			OS_bWait(&LCDFree);
			BSP_LCD_Cube(CUBESIZE*c->position[1]+CUBESIZE/2+13, CUBESIZE*c->position[0]+CUBESIZE/2, CUBESIZE, BGCOLOR);
			OS_Signal(&LCDFree);
			OS_bWait(&lifeFree);
			if (life > 0){
				life--;
			}
			OS_bSignal(&lifeFree);
			OS_bSignal(&(BlockArray[c->position[0]][c->position[1]].BlockFree));
			OS_bSignal(&(c->CubeFree));
		}
		else{
			// if the object is neither hit nor expired,
			// update the cube information
			// then, display the object
			// last,decide next direction
			while (OS_MsTime() - last_move_time < CUBEMOVETIME_MS){
				OS_Suspend();
			}
			uint8_t next_x = c->position[1] + (c->direction % 2) * ((c->direction/2) * 2 - 1);
			uint8_t next_y = c->position[0] + (1 - c->direction % 2) * ((c->direction/2) * 2 - 1);
			bool found_pos = false;
			while (!found_pos){
				next_x = c->position[1] + (c->direction % 2) * ((c->direction/2) * 2 - 1);
				next_y = c->position[0] + (1 - c->direction % 2) * ((c->direction/2) * 2 - 1);
				if (next_x < HORIZONTALNUM && next_y < VERTICALNUM && OS_bTry(&(BlockArray[next_y][next_x].BlockFree))){
					OS_bSignal(&(BlockArray[c->position[0]][c->position[1]].BlockFree));
					OS_bWait(&LCDFree);
					BSP_LCD_Cube(CUBESIZE*c->position[1]+CUBESIZE/2+13, CUBESIZE*c->position[0]+CUBESIZE/2, CUBESIZE, BGCOLOR);
					BSP_LCD_Cube(CUBESIZE*next_x+CUBESIZE/2+13, CUBESIZE*next_y+CUBESIZE/2, CUBESIZE, CUBECOLOR);
					OS_Signal(&LCDFree);
					c->position[0] = next_y;
					c->position[1] = next_x;
					found_pos = true;
				}
				else{
					c->direction = getRandomNumber()/64;
				}
			}
			last_move_time = OS_MsTime();
		}
	}
	if (c->is_alive){
		c->is_alive = false;
		OS_bWait(&LCDFree);
		BSP_LCD_Cube(CUBESIZE*c->position[1]+CUBESIZE/2, CUBESIZE*c->position[0]+CUBESIZE/2, CUBESIZE, BGCOLOR);
		OS_Signal(&LCDFree);
		OS_bSignal(&(BlockArray[c->position[0]][c->position[1]].BlockFree));
		OS_bSignal(&(c->CubeFree));
	}
	OS_Kill(); // Cube should disappear, kill the thread
}

// //--------------end of Task 8-----------------------------

//------------------Task 9--------------------------------
// 
// 
// This task implements the motions of the cubes
void CubeSpawner (void){
	spawner_active = true;
	while(life){ // Implement until the game is over
		bool blocksExist = true;
		while(blocksExist){
			int i;
			blocksExist = false;
			for (i = 0; i < NUMCUBES; i++){
				if (CubeArray[i].is_alive){
					blocksExist = true;
					break;
				}
			}
			if (!blocksExist){
				uint8_t num_cubes = getRandomNumber()/(255/(NUMCUBES)+1)+1;
				uint8_t j;
				for (j=0; j<num_cubes; j++){
					NumCreated += OS_AddThread(&CubeThread, 128, 1);
				}
			}
			OS_Suspend();
		}
	}
	int noteArray[9] = {415, 415, 415, 311, 311, 208, 208, 233, 233};
	int tempoArray[9] = {3, 3, 1, 3, 3, 3, 3, 2, 3};
	OS_Music(noteArray, tempoArray);
	spawner_active = false;
	game_started = false;
	OS_Sleep(50); // wait
	OS_bWait(&LCDFree);
	BSP_LCD_FillScreen(BGCOLOR);
	BSP_LCD_FillScreen(BGCOLOR);
	if (score > high_score) {
		high_score = score;
		BSP_LCD_DrawString(3,2,"New High Score!",LCD_WHITE);
	}
	BSP_LCD_Message(0, 4, 8, "Score:",score);
	BSP_LCD_Message(0, 5, 3, "High Score:",high_score);
	BSP_LCD_DrawString(5,7,"Press S2 to",LCD_WHITE);
	BSP_LCD_DrawString(5,8,"Play Again!",LCD_WHITE);
	while (!game_started);
	OS_bSignal(&LCDFree);
	OS_Kill(); //Life = 0, game is over, kill the thread
}

// //--------------end of Task 9-----------------------------

//------------------Task 7--------------------------------
// background thread executes with button2
// one foreground task created with button push
// ***********ButtonWork2*************
void Restart(void){
	uint32_t StartTime,CurrentTime,ElapsedTime;
	game_started = true;
	OS_Sleep(50); // wait
	StartTime = OS_MsTime();
	ElapsedTime = 0;
	OS_bWait(&LCDFree);
	Button2RespTime = OS_MsTime() - Button2PushTime; // Response on LCD here
	BSP_LCD_FillScreen(BGCOLOR);
	while (ElapsedTime < 500){
		CurrentTime = OS_MsTime();
		ElapsedTime = CurrentTime - StartTime;
		BSP_LCD_DrawString(5,6,"Restarting",LCD_WHITE);
	}
	BSP_LCD_FillScreen(BGCOLOR);
	OS_bSignal(&LCDFree);
	// restart
	OS_bWait(&lifeFree);
	life = 3;
	OS_bSignal(&lifeFree);
	OS_bWait(&scoreFree);
	score = 0;
	level = 1;
	EXPIRATIONTIME_MS = 5000;
	CUBEMOVETIME_MS = 100;
	OS_bSignal(&scoreFree);
	x = 63; y = 63;
	int noteArray[9] = {311, 155, 233, 233, 208, 208, 155, 311, 233};
	int tempoArray[9] = {2, 1, 2, 2, 1, 1, 2, 2, 3};
	OS_Music(noteArray, tempoArray);
	if (!spawner_active){
		NumCreated += OS_AddThread(&CubeSpawner,128,2); 
	}
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
	game_started = true;	
}

//************SW2Push*************
// Called when Button2 pushed
// Adds another foreground task
// background threads execute once and return
void SW2Push(void){
  if(OS_MsTime() > 20 ){ // debounce
    if(OS_AddThread(&Restart,128,4)){
			OS_ClearMsTime();
      NumCreated++; 
    }
    OS_ClearMsTime();  // at least 20ms between touches
		Button2PushTime = OS_MsTime(); // Time stamp
  }
}

//--------------end of Task 7-----------------------------

// Fill the screen with the background color
// Grab initial joystick position to bu used as a reference
void CrossHair_Init(void){
	BSP_LCD_FillScreen(BGCOLOR);
	BSP_LCD_FillScreen(BGCOLOR);
	BSP_Joystick_Input(&origin[0],&origin[1],&select);
}



//******************* Main Function**********
int main(void){ 
	OS_InitBuzzer();     //	initialize buzzer hardware
	OS_Init();           // initialize, disable interrupts
	Device_Init();
	CrossHair_Init();
	Random_Init();
	OS_InitSemaphore(&scoreFree, 1);
	OS_InitSemaphore(&lifeFree, 1);
	uint8_t i;
	uint8_t j;
	for (i=0; i<NUMCUBES; i++){
		OS_InitSemaphore(&(CubeArray[i].CubeFree), 1);
	}
	for (i=0; i<HORIZONTALNUM; i++){
		for (j=0; j<VERTICALNUM; j++){
			OS_InitSemaphore(&(BlockArray[j][i].BlockFree), 1);
		}
	}
	DataLost = 0;        // lost data between producer and consumer
	NumSamples = 0;
	MaxJitter = 0;       // in 1us units
	PseudoCount = 0;
	BSP_LCD_FillScreen(BGCOLOR);
	BSP_LCD_DrawString(1, 6, "Press S1 to Start!", LCD_WHITE);
	OS_AddSW1Task(&SW1Push, 4);

	while (!game_started);
	BSP_LCD_FillScreen(BGCOLOR);
	BSP_LCD_FillScreen(BGCOLOR);
	//********initialize communication channels
	JsFifo_Init();

	//*******attach background tasks***********
	OS_AddSW2Task(&SW2Push, 4);
	// OS_AddPeriodicThread(&PeriodicUpdater, PSEUDOPERIOD, 3);
	OS_AddPeriodicThread(&Producer, PERIOD, 1); // 2 kHz real time sampling of PD3

	NumCreated = 0 ;
	// create initial foreground threads
	NumCreated += OS_AddThread(&Consumer, 128, 1); 
	NumCreated += OS_AddThread(&Display, 128, 1);
	
	int noteArray[9] = {311, 155, 233, 233, 208, 208, 155, 311, 233};
	int tempoArray[9] = {32, 16, 32, 32, 16, 16, 32, 32, 48};
	OS_Music(noteArray, tempoArray);
	NumCreated += OS_AddThread(&CubeSpawner,128,2);
	//   NumCreated += OS_AddThread(&Interpreter, 128, 2); 
	// NumCreated += OS_AddThread(&CubeNumCalc, 128, 3); 

	OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
	return 0;            // this never executes
}
