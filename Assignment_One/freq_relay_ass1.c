#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>

#include "sys/alt_irq.h"
#include "io.h"

#include "altera_avalon_pio_regs.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"

#include "freq_relay_ass1.h"
#include "FSM.h"

//loads connected
#define LOADS_CONNECTED 5

//Global Variables
double freq[100];
double dfreq[100];
TickType_t timeArray[100] = {0};
TickType_t avgTime[5] = {0};
TickType_t mostRecentTime[5] = {0};
TickType_t maxTime[5] = {0,0,0,0,0};
TickType_t minTime[5] = {9,9,9,9,9};
volatile float rocThreshold = 30;
volatile float lowerThreshold = 50;
int indexFreq = 99;
int timeIndex = 0;

//ISR variable
volatile int timer_expired = 0;
int buttonValue = 0;

//For frequency plot
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 3	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)
//RoC plot
#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 3
#define ROCPLT_ORI_Y 299
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 //minimum frequency to draw

typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;

//Priorities
#define PRVGADraw_Task_P      				1
#define updateThreshold_Task_P 				2
#define updateSystemState_Task_P			3
#define updateLoadConnections_Task_P		4
#define auditFrequency_Task_P				4

//handlers
TaskHandle_t PRVGADraw;
TaskHandle_t updateThreshold_handler;
TaskHandle_t updateSystem_handler;
TaskHandle_t updateLoad_handler;
TaskHandle_t auditFreq_handler;
TimerHandle_t timer;

//Queues
static QueueHandle_t Q_stability_data;
static QueueHandle_t Q_freq_data;
static QueueHandle_t Q_time_data;
static QueueHandle_t Q_keyb_data;
static QueueHandle_t Q_audit_data;

//Semaphore
SemaphoreHandle_t freqCalc;
SemaphoreHandle_t thresholdCalc;
SemaphoreHandle_t timeCalc;
SemaphoreHandle_t stateMutex;

//init state machine
state STATE_MATRIX[3][4] = {
    //LD_FLAG0 LD_FLAG1 MT_BTTN TH_CONF
    {{IDLE}, {BAL}, {MAIN}, {IDLE}},	//IDLE
    {{IDLE}, {BAL}, {MAIN}, {BAL}},		//BAL
    {{MAIN},{MAIN}, {MAIN}, {IDLE}}		//MAIN
};

state CURRENT_STATE = IDLE;

//task declarations
void VGAOutputTask(void *pvParameters);
void updateThresholdTask(void *pvParameters);
void updateSystemStateTask(void *pvParameters);
void updateLoadConnectionsTask(void *pvParameters);
void auditFrequencyTask(void *pvParameters);

void timer_500ms_isr(xTimerHandle t_timer) {
	timer_expired = 1;
}

void mt_bt_isr(void* context, alt_u32 id)
{
    // when button is pressed
	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);


	if (*temp == 4) { //key3 push button is pushed to enter maintenance
		printf("BUTTON   %d   \n", *temp);
		stateEval(MT_BTTN);
	}
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

//retrieve freq data and push into q
void freq_relay(){
	#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);

	TickType_t interruptTime = xTaskGetTickCountFromISR();
	xQueueSendToBackFromISR( Q_time_data, &interruptTime, pdFALSE );

	xQueueSendToBackFromISR( Q_freq_data, &temp, pdFALSE );
	return;
}

void ps2_isr (void* context, alt_u32 id)
{
  char ascii;
  int keyboardDebounce;
  int status = 0;
  unsigned char key = 0;
  KB_CODE_TYPE decode_mode;
  status = decode_scancode (context, &decode_mode , &key , &ascii) ;

  if ( status == 0) //success
  {
	    switch ( decode_mode )
	    {
	      case KB_ASCII_MAKE_CODE :
	        printf ( "ASCII   : %x\n", key ) ; // arrow keys
	        xQueueSendToBackFromISR(Q_keyb_data, &key, pdFALSE);
	        break ;
	      case KB_BINARY_MAKE_CODE :	// enter key
	    	xQueueSendToBackFromISR(Q_keyb_data, &key, pdFALSE);
	    	printf ( "BINARY   : %x\n", key ) ;
	        break ;
	      default :
	        printf ( "DEFAULT   : %x\n", key ) ;
	        break ;
	    }
  }
}

void updateSystemStateTask(void* pvParameters) {

	state stateHold;

    while (1) {
    	if (xSemaphoreTake(freqCalc, (TickType_t)0) && xSemaphoreTake(thresholdCalc, (TickType_t) 0)) { // wait till freqCalc & thresholdCalc semphores are free, so that state changes dont happen in the middle of calculation

			if (CURRENT_STATE != stateHold) {
				stateHold = CURRENT_STATE;	// stateHold allows the cases to only be run once (when state is updated)
				switch (stateHold) {
					case IDLE:
						xQueueReset(Q_freq_data);
						xQueueReset(Q_audit_data);

						vTaskResume(auditFreq_handler);
						vTaskResume(updateLoad_handler);

						vTaskSuspend(updateThreshold_handler);
						break;

					case BAL:
						vTaskResume(auditFreq_handler);
						vTaskResume(updateLoad_handler);

						vTaskSuspend(updateThreshold_handler);
						break;

					case MAIN:
						xQueueReset(Q_keyb_data);

						vTaskResume(updateThreshold_handler);
						vTaskResume(updateLoad_handler);

						vTaskSuspend(auditFreq_handler);


						break;
					default:

						break;
				}
			}

			xSemaphoreGive(freqCalc);
			xSemaphoreGive(thresholdCalc);
			vTaskDelay(3);
    	}
    }
}

void updateThresholdTask(void *pvParameters)
{
	unsigned char key;
	int valueUpdated = 0; // debounce

	while (1)
	{

		if (xQueueReceive(Q_keyb_data, &key, portMAX_DELAY) == pdTRUE
			&& valueUpdated == 0
			&& xSemaphoreTake(thresholdCalc, (TickType_t) 0)) {

			valueUpdated = 1;
			if(key == 0x75) { //up
				if (lowerThreshold + 0.1 <= 52) {
					lowerThreshold += 0.1;
				}
			} else if (key == 0x72) { // down
				if (lowerThreshold - 0.1 >= 45) {
					lowerThreshold -= 0.1;
				}
			} else if (key == 0x6b) { // left
				if (rocThreshold - 0.5 >= 0) {
					rocThreshold -= 0.5;
				}
			} else if (key == 0x74) { // right
				if (rocThreshold + 0.5 <= 90) {
					rocThreshold += 0.5;
				}
			} else if (key == 0x5a) { // enter key
				xSemaphoreTake(stateMutex, (TickType_t) 0);
				stateEval(TH_CONF);
				xSemaphoreGive(stateMutex);
			}

		} else if (valueUpdated == 1) {
			valueUpdated = 0;
		}
		xSemaphoreGive(thresholdCalc);
		vTaskDelay(4);
	}
}

void auditFrequencyTask(void *pvParameters) {

	bool stability = true; //true stable -- false unstable
	bool prevSystemState = stability;
	xTimerReset(timer, 5);

	while (1) {
		//process frequency data from interrupt
		while(uxQueueMessagesWaiting( Q_freq_data ) != 0) {

			xSemaphoreTake(freqCalc, (TickType_t) 0);

			xQueueReceive( Q_freq_data, freq+indexFreq, 0 );
			xQueueReceive( Q_time_data, timeArray+indexFreq, (TickType_t)2);

			if(indexFreq==0){	// calculating rate of change
				dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
			}else{
				dfreq[indexFreq] = (freq[indexFreq]-freq[indexFreq-1]) * 2.0 * freq[indexFreq]* freq[indexFreq-1] / (freq[indexFreq]+freq[indexFreq-1]);
			}

			if (dfreq[indexFreq] > 100.0){
				dfreq[indexFreq] = 100.0;
			}

			indexFreq =	++(indexFreq)%100;	// point to the next data (oldest) to be overwritten
											// interrupt triggers task, if in maintenance clear queue if triggered
										    // if system state changes values reset timer and ensure variable is
			if (stability != prevSystemState) {
				xTimerReset(timer, 5);
				timer_expired = 0;
				prevSystemState = stability;
			}
			
			xSemaphoreTake(thresholdCalc, (TickType_t) 0); // for threshold values
			if (fabs(dfreq[(indexFreq+99)%100] > rocThreshold || freq[(indexFreq+99)%100] < lowerThreshold)) {
				stability = false;
				xQueueSendToBack(Q_stability_data, 'U', pdFALSE); // [U]nstable send notification to GUI, overwriting
				// if system is unstable determine which load to shed when timer expires
				if (timer_expired == 1) {
					timeIndex = (indexFreq+99)%100;
					timer_expired = 0;
					xQueueSendToBack(Q_audit_data, 'D', pdFALSE);
					xSemaphoreTake(stateMutex, (TickType_t) 0);
					stateEval(LD_FLAG1);
					xSemaphoreGive(stateMutex);
					xTimerReset(timer, 5);
				}
			} else {
				stability = true;
				xQueueSendToBack(Q_stability_data, 'S', pdFALSE); // [S]table send notification to GUI, overwriting
				// if system is stable determine which load to connect when timer expires
				if (timer_expired == 1) {
					timer_expired = 0;
					xQueueSendToBack(Q_audit_data, 'C', pdFALSE);
					xTimerReset(timer, 5);
				}
			}
			xSemaphoreGive(thresholdCalc);
			xSemaphoreGive(freqCalc);
		}
		vTaskDelay(4);
	}
}

void updateLoadConnectionsTask(void *pvParameters) {

	int i;
	int switchValue;
	int redL;
	int greenL;
	TickType_t temp;
	unsigned char data;

	switchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);	// initializing
	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, (switchValue & ((1<<LOADS_CONNECTED)-1))); // connect all five loads;
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);

	while(1) {

		switchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);

		//redL stores states of the loads via LED_BASE
		redL = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);	// loads connected
		greenL = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE); // loads shed

		if (CURRENT_STATE == MAIN) { // if maintenance
			xSemaphoreTake(timeCalc, (TickType_t)0);

			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, (switchValue & ((1<<LOADS_CONNECTED)-1))); // connect all five loads;
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);
			for(i = 0; i < LOADS_CONNECTED; i++) {
				minTime[i] = 9;
				maxTime[i] = 0;
				avgTime[i] = 0;
				mostRecentTime[i] = 0;
			}

			xQueueReset(Q_audit_data);
			xSemaphoreGive(timeCalc);

		} else {
			if (redL == switchValue) {
				timer_expired = 1;
				xSemaphoreTake(stateMutex, (TickType_t) 0);
				stateEval(LD_FLAG0);	// request to go back to IDLE, no loads to manage
				xSemaphoreGive(stateMutex);
			}

			for (i = 0; i < LOADS_CONNECTED; i++){
				if (!(switchValue & (1 << i))){  // if a toggle was switched off
					IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, redL & ~(1 << i));
					IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenL & ~(1 << i));
					redL = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
					greenL = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);
				}
			}

			if (xQueueReceive(Q_audit_data, &data, portMAX_DELAY) == pdTRUE) {
				switch(data) {
				case 0x15: // [D]isconnect
					for (i = 0; i < LOADS_CONNECTED; i++){
						if(redL & (1 << i)){  // if the load is on, shed that one

							xSemaphoreTake(timeCalc, (TickType_t) 0);

							IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, redL & ~(1 << i));
							IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenL | (1 << i));
							greenL = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);
							redL = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);

							xSemaphoreTake(freqCalc, (TickType_t) 0);	// to access timeIndex
							temp = xTaskGetTickCount() - timeArray[timeIndex]; // response tiem from when INT happened to when load was shed
							xSemaphoreGive(freqCalc);


							mostRecentTime[i] = temp;

							avgTime[i] = (avgTime[i] + temp)/2;

							if (temp < minTime[i]) {
								minTime[i] = temp;
							}

							if (temp > maxTime[i]) {
								maxTime[i] = temp;
							}

							xSemaphoreGive(timeCalc);

							break;
						}
					}
					break;
				case 0xd9: // [C]onnect
					for (i = LOADS_CONNECTED -1; i >= 0; i--){
						if ((switchValue & (1 << i)) ^ (redL & (1 << i))){  // if toggle is on but load is off, unshed that load
							IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, redL | (1 << i));
							IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenL & ~(1 << i));
							greenL = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);
							redL = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
							break;
						}
					}
					break;
				default:
					break;
				}

			}
		}
		vTaskDelay(3);
	}
}

void VGAOutputTask(void *pvParameters)
{
	char str[3]; //holds float conversion output to be written in string
	unsigned char stability;

	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("Cannot find pixel buffer device\n");
		return;
	}

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
		return;
	}

	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);
	alt_up_char_buffer_clear(char_buf);

	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 400, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 400, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	int j = 0;
	int redL;
	char temp_buf[6];
	unsigned int milisec;
	Line line_freq, line_roc;

	while (1)
	{
		//clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 410, 299, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 410, 199, 0, 0);
		//draw threshold lines

		if (xSemaphoreTake(freqCalc, (TickType_t) 0)) {

			alt_up_pixel_buffer_dma_draw_line(pixel_buf,
					101,
					(int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (lowerThreshold - MIN_FREQ)),
					400,
					(int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (lowerThreshold - MIN_FREQ)),
					0x3ff << 10, 0);

			alt_up_pixel_buffer_dma_draw_line(pixel_buf,
					101,
					(int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * fabs(rocThreshold)),
					400,
					(int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * fabs(rocThreshold)),
					0x3ff << 10, 0);

			for(j=0;j<99;++j){ //indexFreq here points to the oldest data, j loops through all the data to be drawn on VGA
				if (((int)(freq[(indexFreq+j)%100]) > MIN_FREQ) && ((int)(freq[(indexFreq+j+1)%100]) > MIN_FREQ)){
					//Calculate coordinates of the two data points to draw a line in between
					//Frequency plot
					line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
					line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(indexFreq+j)%100] - MIN_FREQ));

					line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
					line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(indexFreq+j+1)%100] - MIN_FREQ));

					//Frequency RoC plot
					line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
					line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * fabs(dfreq[(indexFreq+j)%100]));

					line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
					line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * fabs(dfreq[(indexFreq+j+1)%100]));

					//Draw
					alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
					alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
				}
			}
			xSemaphoreGive(freqCalc);
		}


		alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
		alt_up_char_buffer_string(char_buf, "52", 10, 7);
		alt_up_char_buffer_string(char_buf, "50", 10, 12);
		alt_up_char_buffer_string(char_buf, "48", 10, 17);
		alt_up_char_buffer_string(char_buf, "46", 10, 22);

		alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
		alt_up_char_buffer_string(char_buf, "90", 10, 31);
		alt_up_char_buffer_string(char_buf, "60", 10, 33);
		alt_up_char_buffer_string(char_buf, "30", 10, 35);
		alt_up_char_buffer_string(char_buf, "0", 10, 37);

		//clear values to update
		alt_up_char_buffer_string(char_buf, "            ", 38, 40);
		alt_up_char_buffer_string(char_buf, "       ", 38, 44);
	    alt_up_char_buffer_string(char_buf, "       ", 38, 46);
	    alt_up_char_buffer_string(char_buf, "           ", 38, 48);

	    alt_up_char_buffer_string(char_buf, "Current System State:", 16, 40);
		alt_up_char_buffer_string(char_buf, "Lower Threshold (Hz):", 16, 44);
		alt_up_char_buffer_string(char_buf, "RoC Threshold (Hz/sec):", 14, 46);
		alt_up_char_buffer_string(char_buf, "System Status:", 23, 48);

		//alt_up_char_buffer_string(char_buf, "(Up/Down keys)", 48, 40); //lower threshold
		//alt_up_char_buffer_string(char_buf, "(Right/Left keys)", 48, 42);

		if (xSemaphoreTake(freqCalc, (TickType_t) 0)) { // protects variable reads inside
			alt_up_char_buffer_string(char_buf, STATE_STRING[CURRENT_STATE], 38, 40);
			alt_up_char_buffer_string(char_buf, gcvt(lowerThreshold, 3, str), 38, 44); // lower threshold
			alt_up_char_buffer_string(char_buf, gcvt(rocThreshold, 3, str), 38, 46); // RoC threshold

			xSemaphoreGive(freqCalc);
		}

		xQueueReceive( Q_stability_data, &stability, 1 ); // wait one tick to check Queue, do not overwrite

		if (stability == 0xda) { // 0xda hex val for S -> [S]table
			alt_up_char_buffer_string(char_buf, "STABLE", 38, 48); //system state
		} else {
			alt_up_char_buffer_string(char_buf, "UNSTABLE", 38, 48);
		}

		alt_up_char_buffer_string(char_buf, "INSTRUCTIONS", 60, 7);

		alt_up_char_buffer_string(char_buf, "Maintenance Mode:", 58, 10);
		alt_up_char_buffer_string(char_buf, "Push button3 to enter mode", 53, 12);
		alt_up_char_buffer_string(char_buf, "         (key3)", 53, 13);
		alt_up_char_buffer_string(char_buf, "Alter thresholds/loads", 53, 15);
		alt_up_char_buffer_string(char_buf, "Press 'EnterKey' to resume", 53, 17);
		alt_up_char_buffer_string(char_buf, "     normal operations", 53, 18);

		alt_up_char_buffer_string(char_buf, "Altering Thresholds:", 55, 21);
		alt_up_char_buffer_string(char_buf, "(only during maintenance)", 53, 22);
		alt_up_char_buffer_string(char_buf, "Lower - up/down arrow key", 53, 24);
		alt_up_char_buffer_string(char_buf, "RoC - left/right arrow key", 53, 26);

		alt_up_char_buffer_string(char_buf, "TOTAL RUN TIME:", 58, 31);
		milisec = xTaskGetTickCount();
		sprintf(temp_buf, "%02d:%02d:%02d.%1d", (milisec/3600000) % 24, (milisec/60000) % 60, (milisec/1000) % 60, (milisec/100) % 10);
		alt_up_char_buffer_string(char_buf, temp_buf, 60, 33);

		alt_up_char_buffer_string(char_buf, "LOAD REACTION TIME(ms):", 54, 35);
		alt_up_char_buffer_string(char_buf, "       Recent |Avg_mx_mi|", 54, 37);
		alt_up_char_buffer_string(char_buf, "LD 1:      _   |_  _  _  |", 53, 39);
		alt_up_char_buffer_string(char_buf, "LD 2:      _   |_  _  _  |", 53, 41);
		alt_up_char_buffer_string(char_buf, "LD 3:      _   |_  _  _  |", 53, 43);
		alt_up_char_buffer_string(char_buf, "LD 4:      _   |_  _  _  |", 53, 45);
		alt_up_char_buffer_string(char_buf, "LD 5:      _   |_  _  _  |", 53, 47);

		if (xSemaphoreTake(timeCalc, (TickType_t) 0)) {

			sprintf(temp_buf, "%d", mostRecentTime[0]);
			alt_up_char_buffer_string(char_buf, temp_buf, 66, 39);
			sprintf(temp_buf, "%d", mostRecentTime[1]);
			alt_up_char_buffer_string(char_buf, temp_buf, 66, 41);
			sprintf(temp_buf, "%d", mostRecentTime[2]);
			alt_up_char_buffer_string(char_buf, temp_buf, 66, 43);
			sprintf(temp_buf, "%d", mostRecentTime[3]);
			alt_up_char_buffer_string(char_buf, temp_buf, 66, 45);
			sprintf(temp_buf, "%d", mostRecentTime[4]);
			alt_up_char_buffer_string(char_buf, temp_buf, 66, 47);
			sprintf(temp_buf, "%d", avgTime[0]);
			alt_up_char_buffer_string(char_buf, temp_buf, 70, 39);
			sprintf(temp_buf, "%d", avgTime[1]);
			alt_up_char_buffer_string(char_buf, temp_buf, 70, 41);
			sprintf(temp_buf, "%d", avgTime[2]);
			alt_up_char_buffer_string(char_buf, temp_buf, 70, 43);
			sprintf(temp_buf, "%d", avgTime[3]);
			alt_up_char_buffer_string(char_buf, temp_buf, 70, 45);
			sprintf(temp_buf, "%d", avgTime[4]);
			alt_up_char_buffer_string(char_buf, temp_buf, 70, 47);
			sprintf(temp_buf, "%d", maxTime[0]);
			alt_up_char_buffer_string(char_buf, temp_buf, 73, 39);
			sprintf(temp_buf, "%d", maxTime[1]);
			alt_up_char_buffer_string(char_buf, temp_buf, 73, 41);
			sprintf(temp_buf, "%d", maxTime[2]);
			alt_up_char_buffer_string(char_buf, temp_buf, 73, 43);
			sprintf(temp_buf, "%d", maxTime[3]);
			alt_up_char_buffer_string(char_buf, temp_buf, 73, 45);
			sprintf(temp_buf, "%d", maxTime[4]);
			alt_up_char_buffer_string(char_buf, temp_buf, 73, 47);
			sprintf(temp_buf, "%d", minTime[0]);
			alt_up_char_buffer_string(char_buf, temp_buf, 76, 39);
			sprintf(temp_buf, "%d", minTime[1]);
			alt_up_char_buffer_string(char_buf, temp_buf, 76, 41);
			sprintf(temp_buf, "%d", minTime[2]);
			alt_up_char_buffer_string(char_buf, temp_buf, 76, 43);
			sprintf(temp_buf, "%d", minTime[3]);
			alt_up_char_buffer_string(char_buf, temp_buf, 76, 45);
			sprintf(temp_buf, "%d", minTime[4]);
			alt_up_char_buffer_string(char_buf, temp_buf, 76, 47);

			xSemaphoreGive(timeCalc);
		}

		redL = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);

		if (redL&(1 << 0)) {
			alt_up_char_buffer_string(char_buf, "on ", 61, 39);
		} else {
			alt_up_char_buffer_string(char_buf, "off", 61, 39);
		}
		if (redL&(1 << 1)) {
			alt_up_char_buffer_string(char_buf, "on ", 61, 41);
		} else {
			alt_up_char_buffer_string(char_buf, "off", 61, 41);
		}
		if (redL&(1 << 2)) {
			alt_up_char_buffer_string(char_buf, "on ", 61, 43);
		} else {
			alt_up_char_buffer_string(char_buf, "off", 61, 43);
		}
		if (redL&(1 << 3)) {
			alt_up_char_buffer_string(char_buf, "on ", 61, 45);
		} else {
			alt_up_char_buffer_string(char_buf, "off", 61, 45);
		}
		if (redL&(1 << 4)) {
			alt_up_char_buffer_string(char_buf, "on ", 61, 47);
		} else {
			alt_up_char_buffer_string(char_buf, "off", 61, 47);
		}
	}

}

int main()
{
	  alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

	  //create q
	  Q_stability_data = xQueueCreate(1, sizeof(unsigned char));
	  Q_keyb_data = xQueueCreate(5, sizeof(unsigned char));
	  Q_freq_data = xQueueCreate( 100, sizeof(double) );
	  Q_time_data = xQueueCreate(100, sizeof(TickType_t)); //corresponding time to freq interrupt
	  Q_audit_data = xQueueCreate(10, sizeof(unsigned char));
	  timer = xTimerCreate("Timer_500ms", 500, pdFALSE, NULL, timer_500ms_isr);

	  freqCalc = xSemaphoreCreateMutex();
	  thresholdCalc = xSemaphoreCreateMutex();
	  timeCalc = xSemaphoreCreateMutex();
	  stateMutex = xSemaphoreCreateMutex();

	  xSemaphoreGive(freqCalc);
	  xSemaphoreGive(thresholdCalc);
	  xSemaphoreGive(timeCalc);
	  xSemaphoreGive(stateMutex);

	  //register the PS/2 interrupt
	  IOWR_8DIRECT(PS2_BASE,4,1);
	  alt_up_ps2_clear_fifo (ps2_device) ;
	  alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);
	  alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

	  // clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.
	  IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

	  // enable interrupts for all buttons
	  IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);

	  // register the ISR
	  alt_irq_register(PUSH_BUTTON_IRQ, (void*)&buttonValue, mt_bt_isr);

	  xTaskCreate( VGAOutputTask, "VGAOutput", configMINIMAL_STACK_SIZE, NULL, PRVGADraw_Task_P, &PRVGADraw);
	  xTaskCreate( updateThresholdTask, "UpdateThreshold", configMINIMAL_STACK_SIZE, NULL, updateThreshold_Task_P, &updateThreshold_handler);
	  xTaskCreate( updateSystemStateTask, "UpdateSystemState", configMINIMAL_STACK_SIZE, NULL, updateSystemState_Task_P, &updateSystem_handler);
	  xTaskCreate( updateLoadConnectionsTask, "UpdateLoadConnections", configMINIMAL_STACK_SIZE, NULL, updateLoadConnections_Task_P, &updateLoad_handler);
	  xTaskCreate( auditFrequencyTask, "AuditFrequency", configMINIMAL_STACK_SIZE, NULL, auditFrequency_Task_P, auditFreq_handler);

	  vTaskStartScheduler();

	  while(1) {

	  }

	  return 0;

}
