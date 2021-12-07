/*
 * PizzaOvenXMegaA4.c
 *
 * Created: 27/10/2021 14:45:05
 * Author : Patrick Tietz
 */ 

// Define CPU speed
#define F_CPU 2000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

#include "inc/avr_compiler.h"
#include "inc/usart_driver.h"
#include "inc/eeprom_driver.h"


//Define Heating Element Outputs
#define PLATE_HEATER_ON PORTC.OUTSET = PIN5_bm
#define PLATE_HEATER_OFF PORTC.OUTCLR = PIN5_bm
#define DOME_HEATER_ON PORTC.OUTSET = PIN6_bm
#define DOME_HEATER_OFF PORTC.OUTCLR = PIN6_bm

//Define Forward and Reverse VSD Outputs
#define VSD_FORWARD_ON PORTE.OUTSET = PIN3_bm
#define VSD_FORWARD_OFF PORTE.OUTCLR = PIN3_bm
#define VSD_REVERSE_ON PORTE.OUTSET = PIN2_bm
#define VSD_REVERSE_OFF PORTE.OUTCLR = PIN2_bm

//Define Oven Lamp Output
#define OVEN_LAMP_ON PORTR.OUTSET = PIN0_bm
#define OVEN_LAMP_OFF PORTR.OUTCLR = PIN0_bm

//Define Buzzer Output
#define BUZZER_ON PORTC.OUTSET = PIN7_bm
#define BUZZER_OFF PORTC.OUTCLR = PIN7_bm

//Define Fan Start and Run Outputs
//#define FAN_START_ON PORTA.OUTSET = PIN0_bm
//#define FAN_START_OFF PORTA.OUTCLR = PIN0_bm
#define FAN_RUN_ON PORTA.OUTSET = PIN1_bm
#define FAN_RUN_OFF PORTA.OUTCLR = PIN1_bm

//Number of bytes to send in test example. 
#define NUM_BYTES  4
//Define that selects the USART used in example. 
#define USART USARTD0
//Define Baud rate of the UART 
#define BAUD 9600
//Define Number of chimes x2 for the Buzzer when the timer runs out 
#define CHIMES 6
//The IDLE and Max speed of the rotation plate
#define IDLE_SPEED 50
#define MAX_SPEED 100

//Define ADC Buffer Sizes
#define SAMPLE_COUNT 100
#define BUFFER_SIZE 20

//Number of ADC channels used
#define  CHAN_NUM 2

//ADC offset value
#define  ADC_OFFSET 190

//ADC Ref voltage
#define VREF 10

//The temperature over and underflows for the temperature max and min swell points
#define TEMP_OVERFLOW_PLATE 0
#define TEMP_OVERFLOW_DOME 0
#define TEMP_UNDERFLOW_PLATE 3
#define TEMP_UNDERFLOW_DOME 5

//Define Main Cont Output
#define MAIN_CONT_ON PORTA.OUTSET = PIN0_bm
#define MAIN_CONT_OFF PORTA.OUTCLR = PIN0_bm

//Address Bytes for saving values to the EEPROM
//Values to be saved are the ref temp for dome and plate, plate rotation speed, bake time and buzzer on/off
//Note since the temperatures are saved as 16bit variables we split the data over 2x 8bit fields
#define TEMP_D_ADDR_1 0x00
#define TEMP_D_ADDR_2 0x08
#define TEMP_P_ADDR_1 0x10
#define TEMP_P_ADDR_2 0x18
//Address of 32bit Page Buffer used for temperature storage
#define TEMP_PAGE_ADDR 0
//Addresses of remaining data storage values (speed, bake time, buzzer)
#define ROT_SPEED_ADDR 0x00
#define BAKE_TIME_M_ADDR 0x08
#define BAKE_TIME_S_ADDR 0x10
#define BUZZER_ADDR	0x18
//Address of 32bit Page Buffer used for the remaining storage values (speed, bake time, buzzer)
#define REMAIN_PAGE_ADDR 2

//--------------------------------Function Definitions----------------------------//
void init_timer0();
void init_timer1();
void init_uart(uint32_t baudrate);
uint8_t * uart_receive_bytes();
void  uart_send_bytes(char data[20]);
void initialize_values();
void data_received_handler(uint8_t data[NUM_BYTES]);
uint16_t combine_bytes(uint8_t b1, uint8_t b2);
void add_int_to_str(int value, char *str[50]);
void turn_plate_left(uint8_t command);
void turn_plate_right(uint8_t command);
void toggle_oven_lamp(uint8_t input);
void baketime_finished();
void preheat_finished();
void init_ADC();
void start_ADC();
uint16_t* adc_get_mean(uint8_t counter);
void send_temp(uint16_t temp, char command[30]);
void set_VSD_speed(uint8_t spd);
void init_DAC();
void check_bake_state();
void check_door_state();
void heating_oven();
void run_fan();
void check_heating_elements();
void set_temp_swellpoints(char temp);
void write_to_EEPROM_16(uint8_t page_addr ,uint8_t byte1_addr, uint8_t byte2_addr, uint16_t data);
void write_to_EEPROM_8(uint8_t page_addr ,uint8_t byte1_addr, uint16_t data);
uint16_t read_from_EEPROM_16(uint8_t page_addr ,uint8_t byte1_addr, uint8_t byte2_addr);
uint8_t read_from_EEPROM_8(uint8_t page_addr ,uint8_t byte1_addr);
void reset_EEPROM_values();
bool remainder_test(uint16_t value, uint8_t divider );

//-------------------------------Global Variables---------------------------------//
//USART data struct used in example. 
USART_data_t USART_data;
//pointer to Array in which received data is in. 
uint8_t* received;
//Rx received flag
bool rx_received = false;
//Define temperature variable structs
typedef struct
{
	uint16_t setpoint;
	uint16_t max_swell_point;
	uint16_t min_swell_point;
	
	}temperature;
	
temperature temp_dome;
temperature temp_plate;
//Actual temperature variables sensed by the PT100
uint16_t PT100_Dome = 0;
uint16_t PT100_Plate = 0;
//bool to see if max temp was reached
bool reached_max_dome = false;
bool reached_max_plate = false;
//Define plate speed (0 - 100%)
uint8_t plate_speed = 0;
//Bake timer timeout flag
bool bake_timeout_flag = false;
//bool used to repeat the timeout sound every 30sec until stopped
bool repeat_chime = false;
//tick counter used for chime
uint32_t tick_chime = 0;
//Preheat chime
bool preheat_chime = false;
//door switch
bool dooropen = false;
//timer ticks for debouncing door switch
uint32_t door_ticks_ref= 0;
uint32_t door_ticks = 0;
//Bake time variable
typedef struct
{
	uint8_t minutes;
	uint8_t seconds;
	}timer;
	
timer bake_time;
//Buzzer ON variable
bool buzzer_on = true;
//timer0 variables 
uint32_t tick_current = 0;
uint32_t adc_tick_count = 0;
uint32_t element_ticks = 0;
//baking state (0->OFF, 1->RUN, 2->PREHEAT, 3->IDLE)
uint8_t bake_state = 3;
//bool to turn main contact on or off
bool main_contact_on = false;
//Buffer Array variables for the adcs
uint16_t adc_buffer[CHAN_NUM][BUFFER_SIZE];
//ADC Buffer Counters
uint16_t adc_buffer_cnt = 0;
//Final Mean ADC value updated every second
uint16_t* adc_mean = 0;
//Flag to fetch adc values
bool fetch_adc = false;
//bool for fan On or OFF
bool fan_on = false;
//flag used to keep the fan start on for approx. 2 sec
//bool fan_start_mode = true;
//Fan start ticks 
//uint32_t fan_start_ticks = 0;
//tick counter to check if heating elements are working within 20min
uint32_t heating_error_ticks = 0;
//flag set to true if heating elements are working
bool elements_work = false;
//flag set when error detected
bool send_error = false;
//initial variables for Dome and Plate temp, rotation speed, baketime and buzzer
//These variables get updated via the EEPROM values and will thus retain the values from the previous bake
uint16_t initial_temp_p = 180;
uint16_t initial_temp_d = 180;
uint8_t initial_rot_spd = 50;
uint8_t initial_bake_time_m = 3;
uint8_t initial_bake_time_s = 0;
uint8_t initial_buzzer_on = 1;

#warning if this flag is set to true all EEPROM values will be reset to the first initial values as seen above
//Reset EEPROM flag
bool reset_EEPROM = false;

//-------------------------------Main Loop---------------------------------//
int main(void)
{
	//Wait for Nextion to startup
	_delay_ms(2000);
	
	//Set Buzzer as output on PC7
	PORTC.DIR |= PIN7_bm;
	
	//Define Heating elements as output on PC5 (Plate) and PC6 (Dome)
	PORTC.DIR |= PIN6_bm;
	PORTC.DIR |= PIN5_bm;
	
	//Set LAMP as output on PORTR PIN0
	PORTR.DIR |= PIN0_bm;
	
	//Set Door switch as input on PORTA PIN2 with pull up resistor
	PORTA.DIRCLR = PIN2_bm;
	PORTA.PIN2CTRL = PORT_OPC_PULLUP_gc; 
	
	//Set Fan as output
	//PORTA.DIR |= PIN0_bm;
	PORTA.DIR |= PIN1_bm;
	
	//Set VSD For and Rev as Output
	PORTE.DIR |= PIN2_bm;
	PORTE.DIR |= PIN3_bm;
	
	//Set Main Cont as output
	PORTA.DIR |= PIN0_bm;
	
	//Initialize UARTSS
	init_uart(BAUD);
	
	//Initialize Timers
	init_timer0();
	init_timer1();
	
	//Initialize ADC 
	init_ADC();
	
	//Initialize DAC
	init_DAC();
	
	// Enable PMIC interrupt level low
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
	
	// Enable global interrupts.
	sei();
	
	//start ADC
	start_ADC();
	
	EEPROM_DisableMapping();
	
	if(reset_EEPROM == true){
		//reset the values within the EEPROM 
		reset_EEPROM_values();
	}
	
	//delay
	_delay_ms(200);
	//initialize oven values - dome temp, plate temp and rotation speed
	initialize_values();
	
	//set tick value for heating elements
	heating_error_ticks = tick_current;
	
	//Main loop
	while(1){
		//Run Fan
		run_fan();
		
		//check heating element state after 20min
		check_heating_elements();
		
		//Set Main Contact On or off
		if(main_contact_on ==true){
			MAIN_CONT_ON;
		}
		else{
			MAIN_CONT_OFF;
		}
		
		//Read the UART string send from the Nextion
		if(rx_received == true)
		{
			//Call UART receiver function if UART receive flag is set
			received = uart_receive_bytes();
			//Send UART data to the receiver handler
			data_received_handler(received);
			//set UART receive flag to false
			rx_received = false;
		}
		
		//Once adc fetch flag is set by the timer (after 1 sec) retrieve the adc mean value
		if(fetch_adc == true){
			adc_mean = adc_get_mean(adc_buffer_cnt);
			adc_buffer_cnt++;
			//reset the buffer counter once it reaches the Buffer Size
			if(adc_buffer_cnt == BUFFER_SIZE)
			{
				adc_buffer_cnt = 0;
			}
			fetch_adc = false;
			//Calculate temperature
			PT100_Dome = adc_mean[0] / VREF;
			PT100_Plate = adc_mean[1] / VREF;
			//Send TempP to Nextion
			send_temp(PT100_Plate, "page0.actual_tempP.val=");
			//Send TempD to Nextion
			send_temp(PT100_Dome, "page0.actual_tempD.val=");
		}
		
		//If the timer goes off start oven chime
		if(bake_timeout_flag)
		{
			//Turn buzzer sound ON
			baketime_finished();
			repeat_chime = true;
			bake_timeout_flag = false;
			
		}
		
		//If Preheat is finished set off chime
		if(preheat_chime)
		{
			preheat_finished();
			preheat_chime = false;
		}
		
		//Check if door was opened
		check_door_state();
		
		//Check if start was enabled
		check_bake_state();
		
		//heat the oven
		heating_oven();
		
	}
}

//-------------------------------General Functions--------------------------------------//

void heating_oven()
{
	//This function is used to control the heat within the oven
	//For both plate and dome the heater should be ON until it reaches the max temp swell point (temp_setpint + TEMP_OVERFLOW)
	//Once The max swell point is reached the heating elements should be switched OFF until the min swell point is reached (temp_setpoint - TEMP_UNDERFLOW)
	
	//------------------------Dome heater Controller----------------------------//
	//Dome heater ON State
	if(PT100_Dome < temp_dome.max_swell_point && reached_max_dome == false){
		//Heat the dome until max temperature is reached
		DOME_HEATER_ON;
	}
	else if(PT100_Dome >= temp_dome.max_swell_point)
	{
		reached_max_dome = true;
	}
	//Dome heater OFF State
	if(PT100_Dome > temp_dome.min_swell_point && reached_max_dome == true)
	{
		//Turn off Dome heater and wait until min swell point is reached
		DOME_HEATER_OFF;
	}
	else if(PT100_Dome <= temp_dome.min_swell_point)
	{
		reached_max_dome = false;
	}
	
	//------------------------Plate heater Controller----------------------------//
	//Plat heater ON State
	if(PT100_Plate < temp_plate.max_swell_point && reached_max_plate == false){
		//Heat the plate until max temperature is reached
		PLATE_HEATER_ON;
	}
	else if(PT100_Plate >= temp_plate.max_swell_point)
	{
		reached_max_plate = true;
	}
	//Plate heater OFF State
	if(PT100_Plate > temp_plate.min_swell_point && reached_max_plate == true)
	{
		//Turn off Plate heater and wait until min swell point is reached
		PLATE_HEATER_OFF;
	}
	else if(PT100_Plate <= temp_plate.min_swell_point)
	{
		reached_max_plate = false;
	}
	
}

void add_int_to_str(int value, char *str[50])
{
	//function used to combine an integer and a string into a string
	char temp[10];
	itoa(value,temp,10);
	strcat(*str, temp);
}

uint16_t combine_bytes(uint8_t b1, uint8_t b2)
{
	//function used to concatenate two byte values and form a 16bit value
	uint16_t combined = b1 << 8 | b2;
	return combined;
}

void send_temp(uint16_t temp, char command[30])
{
	//Function used to send the PT100 temperature to the Nextion
	char temp_str[50] = "";
	char *temp_ptr = temp_str;
	//Send TP100 temperature to the Nextion
	strcpy(temp_str, command);
	add_int_to_str(temp, &temp_ptr);
	uart_send_bytes(temp_ptr);
}

void preheat_finished(){
	if(buzzer_on == true)
	{
		BUZZER_ON;
		_delay_ms(100);
		BUZZER_OFF;
		_delay_ms(100);
		BUZZER_ON;
		_delay_ms(400);
		BUZZER_OFF;
	}
}
void baketime_finished()
{
	//Buzzer chime when bake time runs out
	if(buzzer_on == true)
	{
		BUZZER_ON;
		_delay_ms(100);
		BUZZER_OFF;
		_delay_ms(100);
		BUZZER_ON;
		_delay_ms(100);
		BUZZER_OFF;
		_delay_ms(100);
		BUZZER_ON;
		_delay_ms(400);
		BUZZER_OFF;
	}
}

void run_fan()
{
	//the fan runs all the time except when the oven is stopped
	if(fan_on==true){
		FAN_RUN_ON;
		//NOTE: Fan start is not implemented anymore -> Fan is run at 100%
		//if(fan_start_mode==true){
			////set the fan start ticks to the current ticks of the timer
			//fan_start_ticks=tick_current;
			////set start mode to false
			//fan_start_mode=false;
		//}
		//The run signal is send for the entire time during operation
		//The fan needs to receive the start signal for 2sec at the beginning of startup (run 100% speed)
		//if(tick_current < fan_start_ticks + 200) 
		//{
			//FAN_START_ON;
		//}
		//else{
			//FAN_START_OFF;
		//}
	}
	else{
		//switch the fan off when the door is opened or stop is pressed
		FAN_RUN_OFF;
		//FAN_START_OFF;
		//Set fan start mode to true so the the fan can restart after the door closes
		//fan_start_mode=true;
	}
	
}

void set_temp_swellpoints(char temp){
	//this function is used to set the upper and lower bounds of the temperature
	//NOTE: The parameter char temp is used to destinguish between plate (p) and dome (d)
	
	if(temp == 'p'){
		temp_plate.max_swell_point = temp_plate.setpoint + TEMP_OVERFLOW_PLATE;
		temp_plate.min_swell_point = temp_plate.setpoint - TEMP_UNDERFLOW_PLATE;
	}
	else if(temp == 'd'){
		temp_dome.max_swell_point = temp_dome.setpoint + TEMP_OVERFLOW_DOME;
		temp_dome.min_swell_point = temp_dome.setpoint - TEMP_UNDERFLOW_DOME;
	}
}

bool remainder_test(uint16_t value, uint8_t divider ){
	//This function calculates whether there is a remainder between the value and the divider
	//if there is it returns true, otherwise it will return false
	if(value % divider != 0){
		return true;
	}else
	{
		return false;
	}
}

bool within_limits_test(uint16_t value, uint16_t lower_bound, uint16_t upper_bound){
	//This function tests whether the value is within the boundaries of the upper and lower bound
	//If so, then it returns true otherwise the function returns false
	if(value >= lower_bound && value <= upper_bound){
		return true;
	}
	else
	{
		return false;
	}
}

void reset_EEPROM_values(){
	//This function writes all first initial values to the EEPROM - Can be used if the data gets corrupted
	write_to_EEPROM_16(TEMP_PAGE_ADDR, TEMP_D_ADDR_1, TEMP_D_ADDR_2, initial_temp_d);
	_delay_ms(1);
	write_to_EEPROM_16(TEMP_PAGE_ADDR, TEMP_P_ADDR_1, TEMP_P_ADDR_2, initial_temp_p);
	write_to_EEPROM_8(REMAIN_PAGE_ADDR, ROT_SPEED_ADDR, initial_rot_spd);
	write_to_EEPROM_8(REMAIN_PAGE_ADDR, BAKE_TIME_M_ADDR, initial_bake_time_m);
	write_to_EEPROM_8(REMAIN_PAGE_ADDR, BAKE_TIME_S_ADDR, initial_bake_time_s);
	_delay_ms(1);
	write_to_EEPROM_8(REMAIN_PAGE_ADDR, BUZZER_ADDR, initial_buzzer_on);
}

//-------------------------------State Functions--------------------------------------//

void check_bake_state()
{
	if(bake_state == 1)
	{
		//RUN state
		//Start Baking Process
		//Only execute once
		//Buzzer sound
		if(buzzer_on==true)
		{
			//Pressed Start button sounds
			BUZZER_ON;
			_delay_ms(400);
			BUZZER_OFF;
		}
		//Set Plate rotation speed
		set_VSD_speed(plate_speed);
		//make sure plate reverse (left) direction is off
		VSD_REVERSE_OFF;
		//Start plate forward (right) rotation
		VSD_FORWARD_ON;
		//Set Fan on
		fan_on = true;
		//Set main contact on
		main_contact_on = true;
		
	}
	//Stop the baking process if the stop signal was sent 
	else if(bake_state == 0)
	{
		//OFF state
		//Stop baking Process
		//Only execute once
		//Buzzer sound
		if(buzzer_on==true){
			//Pressed Stop button sound
			BUZZER_ON;
			_delay_ms(200);
			BUZZER_OFF;
			_delay_ms(200);
			BUZZER_ON;
			_delay_ms(200);
			BUZZER_OFF;
		}
		//Stop plate rotation
		VSD_FORWARD_OFF;
		VSD_REVERSE_OFF;
		//Set Fan OFF
		fan_on = false;
		//set Main contact off
		main_contact_on = false;
		//Turn off repeat timeout chime
		repeat_chime = false;
			
	}
	else if(bake_state == 2){
		//Preheat state
		//set main contact on
		main_contact_on = true;
		//set fan on
		fan_on = true;
	}
	//Once executed put bake state into idle
	bake_state = 3;
}

void check_door_state()
{
	//Since PIN2 is set high with the internal pull up resistor, the pin will be set to low if the door is opened (switched triggered)
	if(!(PORTA.IN & PIN2_bm)){
		//Pin is low
		//get tick number of how long the pin was pulled low
		door_ticks = tick_current - door_ticks_ref;
		//If the pin was pulled low for 500ms then the door was opened
		if(dooropen == false && door_ticks >= 50)
		{
			uart_send_bytes("page0.door_switch.val=1");
			dooropen = true;
			//Stop plate rotation
			VSD_FORWARD_OFF;
			VSD_REVERSE_OFF;
			//Turn off repeat timeout chime
			repeat_chime = false;		
		}
	}
	else if ((PORTA.IN & PIN2_bm)){
		//Pin is high
		//resume program
		if(dooropen == true)
		{
			uart_send_bytes("page0.door_switch.val=0");
			dooropen = false;
		}
		door_ticks_ref = tick_current;
	}
}

void check_heating_elements(){
	//this function checks if the heating elements are operating properly
	//if in 20min the temp of the elements does not reach the temp ref the elements are broken
	int16_t temp_diff_dome = 0;
	int16_t temp_diff_plate = 0;
	//temp difference shouldn't be greater than 20 degrees
	temp_diff_dome = temp_plate.setpoint - PT100_Plate;
	temp_diff_plate = temp_dome.setpoint - PT100_Dome;
	//if both elements reach the set temperature (within 20 degrees) they work properly
	if(temp_diff_dome <= 20 && temp_diff_plate <= 20){
		elements_work = true;
	}
	
	if(send_error == true){
		//if the elements don't work, send error to nextion
		uart_send_bytes("page0.element_error.val=1");
	}
}

//-------------------------------ADC Functions--------------------------------------//
void init_ADC()
{
	//Setup ADCA Channel0 for PA4
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; // Single ended input on channel 0
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc; //Input Pin4 on channel 0
	//Setup ADCA Channel1 for PA5
	ADCA.CH1.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc; // Single ended input on channel 1
	ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc; //Input Pin5 on channel 1
	//Configure ADCA Registers
	ADCA.CTRLB = ADC_FREERUN_bm; // Enable free run mode
	ADCA.REFCTRL = ADC_REFSEL_INT1V_gc; // Internal reference voltage of 1V
	ADCA.PRESCALER = ADC_PRESCALER_DIV256_gc; //Set Prescaler value 256
}

void start_ADC()
{
	//Set free-running conversion  on channel0 - Pin4
	ADCA.EVCTRL = ADC_SWEEP0_bm;
	//Set free-running conversion  on channel1 - Pin5
	ADCA.EVCTRL = ADC_SWEEP1_bm;
	//Enable ADC
	ADCA.CTRLA = ADC_ENABLE_bm;
	
}

uint16_t* adc_get_mean(uint8_t counter)
{
	//Fetch adc conversion and place into buffer
	//uint32_t adc_100 = 0;
	uint8_t x;
	uint8_t chan;
	uint16_t adc_final[CHAN_NUM] = {0,0};
	static uint16_t adc_return[CHAN_NUM] = {0,0};
	uint8_t none_zero_cnt[CHAN_NUM] = {0,0};
	adc_buffer[0][counter] = ADCA_CH0RES;
	adc_buffer[1][counter] = ADCA_CH1RES;
	
	//Get Final adc Mean value form the ADC MEAN buffer
	for(chan = 0; chan < CHAN_NUM ; chan++)
	{
		for(x = 0; x < BUFFER_SIZE; x++)
		{
			//Filter out zero values for the buffer
			if(adc_buffer[chan][x] != 0){
				adc_final[chan] += adc_buffer[chan][x] - ADC_OFFSET;
				none_zero_cnt[chan] += 1;
			}
		}
	}
	
	adc_return[0] = adc_final[0] / none_zero_cnt[0];
	adc_return[1] = adc_final[1] / none_zero_cnt[1];
	
	//Return the final value
	return adc_return;
}

//-------------------------------DAC Functions--------------------------------------//
void init_DAC()
{
	DACB.CTRLB = DAC_CHSEL_SINGLE_gc; //select single channel operation on channel 1.
	DACB.CTRLC = DAC_REFSEL_INT1V_gc; //1V as conversion reference
	//DACB.CH0GAINCAL = 0x8D;
	//DACB.CH0OFFSETCAL = 0x24;
	DACB.CTRLA = DAC_CH0EN_bm; //enable DAC output 1
	DACB.CTRLA |= DAC_ENABLE_bm; // enable the DAC module itself.
	
}

//-------------------------------Timer Functions--------------------------------------//

void init_timer0()
{
	TCC0.PER = 0x004E;  //Set Timer overflow value (0x004E -> 10 msec with a 256 Prescaler)
	TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc; //Set timer Overflow interrupt 
	
	TCC0.CTRLA = TC_CLKSEL_DIV256_gc; // Set timer Prescaler 256
	
}

void init_timer1()
{
	TCC1.PER = 0x0186;  //Set Timer overflow value (0x004E -> 200 msec with a 1024 Prescaler)
	TCC1.INTCTRLA = TC_OVFINTLVL_LO_gc; //Set timer Overflow interrupt
	
	TCC1.CTRLA = TC_CLKSEL_DIV1024_gc; // Set timer Prescaler 1024
	
}

//------------------------------- UART Functions--------------------------------------//

void init_uart(uint32_t baudrate)
{
	/*       UART Setup:
	*	   - 8 bit character size
	*      - No parity
	*      - 1 stop bit
	*      - 9600 Baud */
	// Variable to control the baud rate clock generator
	double BSEL = 0;
	
	// Port Setup for UARTD0 - Pins PD2(Rx) and PD3(Tx)
  	// PC3 (TXD0) as output
	PORTD.DIRSET   = PIN3_bm;
	// PC2 (RXD0) as input
	PORTD.DIRCLR   = PIN2_bm;
	
	// Use USARTD0 and initialize buffers
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);
	
	// USARTD0, 8 Data bits, No Parity, 1 Stop bit
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);
	
	// Enable RXC interrupt
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	// Set Baud rate using default I/O clock frequency (2MHz)
	BSEL = (F_CPU / (16 * baudrate)) - 1;
	USART_Baudrate_Set(&USART, (uint8_t)BSEL , 0);
	
	// Enable both RX and TX
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);

}

uint8_t * uart_receive_bytes()
{
	// counter variable
	uint8_t i = 0;
	// Array variable to store received data in
	static uint8_t data_received[NUM_BYTES];
	//tick timer used to time data receive loop
	uint32_t tick_previous = tick_current;
	bool timeout = false;
	//receiver loop
	while (i < NUM_BYTES && timeout == false ) {
		if (USART_RXBufferData_Available(&USART_data)) {
			data_received[i] = USART_RXBuffer_GetByte(&USART_data);
			i++;
		}
		//after 30ms timeout
		if(tick_current >= tick_previous + 3)
		{
			timeout = true;
			//fill remaining bytes with zeros
			i++;
			data_received[i] = 0;
			
			//fill data received with zeros
			//uint8_t cnt;
			//for(cnt=0; cnt<NUM_BYTES;cnt++){
				//data_received[cnt] = 0;
			//}
		}
	}
	return data_received;
}


void  uart_send_bytes(char data[30]){
	uint8_t length = strlen(data);
	bool byteToBuffer;
	int i;
	
	//Wait for data register to be empty to put new data
	do{
		
	}while(!USART_IsTXDataRegisterEmpty(&USART));
	
	for(i=0;i<3;i++)
	{
		//Sending 3 clear signals (0xff) to the Nextion before transmitting instructions
		while(USART_TXBuffer_PutByte(&USART_data, 0xff)==false)
		{
			
		}
	}
	
	//Sending data instructions to the Nextion
	int cnt = 0;
	while (cnt < length) {
		byteToBuffer = USART_TXBuffer_PutByte(&USART_data, data[cnt]);
		if(byteToBuffer){
			cnt++;
		}
	}
	
	for(i=0;i<3;i++)
	{
		//Sending 3 clear signals (0xff) to the Nextion after transmitting instructions
		while(USART_TXBuffer_PutByte(&USART_data, 0xff)==false)
		{
			
		}
	}
}

//------------------------------- EventHandler Functions--------------------------------------//

void initialize_values()
{
	//This function initializes all values and sets them according to last operation
	//variables used for uart operation
	char instr[50] = "";
	char *instr_ptr = instr;
	
	uint16_t eeprom_return_val = 0;
	
	//First the saved values are fetched from the EEPROM
	//fetch initial temp dome
	eeprom_return_val =  read_from_EEPROM_16(TEMP_PAGE_ADDR, TEMP_D_ADDR_1, TEMP_D_ADDR_2);
	//Test if value falls within restrictions 
	if(remainder_test(eeprom_return_val, 10) == false && within_limits_test(eeprom_return_val, 20, 400) == true){
		initial_temp_d = eeprom_return_val;
	}
	else{
	//If not reset the EEPROM
		reset_EEPROM_values();
	}
	//fetch initial temp plate
	eeprom_return_val = read_from_EEPROM_16(TEMP_PAGE_ADDR, TEMP_P_ADDR_1, TEMP_P_ADDR_2);
	//Test if value falls within restrictions
	if(remainder_test(eeprom_return_val, 10) == false && within_limits_test(eeprom_return_val, 20, 400) == true){
		initial_temp_p = eeprom_return_val;
	}
	else{
		//If not reset the EEPROM
		reset_EEPROM_values();
	}
	//fetch plate rotation speed
	eeprom_return_val = read_from_EEPROM_8(REMAIN_PAGE_ADDR, ROT_SPEED_ADDR);
	//Test if value falls within restrictions
	if(within_limits_test(eeprom_return_val, 0, 100) == true){
		initial_rot_spd = eeprom_return_val;
	}
	else{
		//If not reset the EEPROM
		reset_EEPROM_values();
	}
	//fetch bake time (m)
	eeprom_return_val = read_from_EEPROM_8(REMAIN_PAGE_ADDR, BAKE_TIME_M_ADDR);
	//Test if value falls within restrictions
	if(within_limits_test(eeprom_return_val, 0, 30) == true){
		initial_bake_time_m = eeprom_return_val;
	}
	else{
		//If not reset the EEPROM
		reset_EEPROM_values();
	}
	//fetch bake time (s)
	eeprom_return_val = read_from_EEPROM_8(REMAIN_PAGE_ADDR, BAKE_TIME_S_ADDR);
	//Test if value falls within restrictions
	if(remainder_test(eeprom_return_val, 10) == false && within_limits_test(eeprom_return_val, 0, 10) == true){
		initial_bake_time_s = eeprom_return_val;
	}
	else{
		//If not reset the EEPROM
		reset_EEPROM_values();
	}
	//fetch buzzer on off
	 eeprom_return_val = read_from_EEPROM_8(REMAIN_PAGE_ADDR, BUZZER_ADDR);
	//Test if value falls within restrictions
	if(within_limits_test(eeprom_return_val, 0, 1) == true){
		initial_buzzer_on = eeprom_return_val;
	}
	else{
		//If not reset the EEPROM
		reset_EEPROM_values();
	}
	
	//Send initial values to NEXTION
	//Send initial Temperatures
	strcpy(instr, "page0.tempD_ref.val=");
	add_int_to_str(initial_temp_d, &instr_ptr);
	uart_send_bytes(instr_ptr);
	_delay_ms(100);
	strcpy(instr, "page0.tempP_ref.val=");
	add_int_to_str(initial_temp_p, &instr_ptr);
	uart_send_bytes(instr_ptr);
	_delay_ms(100);
	//Send initial plate rotation speed
	strcpy(instr, "page0.rotspd.val=");
	add_int_to_str(initial_rot_spd, &instr_ptr);
	uart_send_bytes(instr_ptr);
	_delay_ms(100);
	//Send baketime
	strcpy(instr, "page1.time_min.val=");
	add_int_to_str(initial_bake_time_m, &instr_ptr);
	uart_send_bytes(instr_ptr);
	_delay_ms(100);
	strcpy(instr, "page1.time_sec.val=");
	add_int_to_str(initial_bake_time_s, &instr_ptr);
	uart_send_bytes(instr_ptr);
	_delay_ms(100);
	//Send sound value
	strcpy(instr, "page1.snd.val=");
	add_int_to_str(initial_buzzer_on, &instr_ptr);
	uart_send_bytes(instr_ptr);
	_delay_ms(100);
	
	//Set ref values
	//Setting temp ref values
	temp_dome.setpoint = initial_temp_d;
	temp_plate.setpoint = initial_temp_p;
	set_temp_swellpoints('d');
	set_temp_swellpoints('p');
	//Setting rotation speed ref value
	plate_speed = initial_rot_spd;
	//setting bake time
	bake_time.minutes = initial_bake_time_m;
	bake_time.seconds = initial_bake_time_s;
	//Set buzzer
	if(initial_buzzer_on == 1){
		buzzer_on = true;
	}
	else{
		buzzer_on = false;
	}
	//Stop initialization
	uart_send_bytes("page2.init.val=1");
}

void data_received_handler(uint8_t data[NUM_BYTES]){
	//This function triggers events/functions according to data received via the UART
	uint8_t input_id = data[0];
	char instr[50] = "";
	char *instr_ptr = instr;
	
	switch (input_id)
	{
	case 1:
		//Start and Stop input
		if(data[1] == 1){
			//start signal
			bake_state = 1;
		}
		else if(data[1] == 2){
			//Preheat Start
			bake_state = 2;
		}
		else 
		{
			//stop signal
			bake_state = 0;
		}
		break;
	case 2:
		//Turn plate left input
		turn_plate_left(data[1]);
		break;
	case 3:
		//Turn plate right input
		turn_plate_right(data[1]);
		break;
	case 4:
		//Lamp ON or OFF input
		toggle_oven_lamp(data[1]);
		break;
	case 5:
		//Dome temp input
		temp_dome.setpoint = combine_bytes(data[2], data[1]);
		//Send temperature confirmation back to the Nextion
		strcpy(instr, "page0.n1.val=");
		add_int_to_str(temp_dome.setpoint, &instr_ptr);
		uart_send_bytes(instr_ptr);
		//Set swell points
		set_temp_swellpoints('d');
		//write to EEPROM
		write_to_EEPROM_16(TEMP_PAGE_ADDR, TEMP_D_ADDR_1, TEMP_D_ADDR_2, temp_dome.setpoint);
		break;
	case 6:
		//Plate temp input
		temp_plate.setpoint = combine_bytes(data[2], data[1]);
		//Send temperature confirmation back to the Nextion
		strcpy(instr, "page0.n0.val=");
		add_int_to_str(temp_plate.setpoint, &instr_ptr);
		uart_send_bytes(instr_ptr);
		//Set well points
		set_temp_swellpoints('p');
		//write to EEPROM
		write_to_EEPROM_16(TEMP_PAGE_ADDR, TEMP_P_ADDR_1, TEMP_P_ADDR_2, temp_plate.setpoint);
		break;
	case 7: 
		//Plate speed input
		plate_speed = data[1];
		//write to EEPROM
		write_to_EEPROM_8(REMAIN_PAGE_ADDR, ROT_SPEED_ADDR, plate_speed);
		break;
	case 8:
		//Bake timer timeout
		if(data[1]==0)
		{
			bake_timeout_flag = true;
		}
		else if(data[1]==1){
			//Preheat finished
			preheat_chime = true;
		}
		break;
	case 9:
		//Turn buzzer ON or OFF
		if(data[1] == 0)
		{
			buzzer_on = false;
		}
		else
		{
			buzzer_on = true;
		}
		//write to EEPROM
		write_to_EEPROM_8(REMAIN_PAGE_ADDR, BUZZER_ADDR, data[1]);
		break;
	case 10:
		//Bake_time set by user which is saved in the EEPROM
		bake_time.minutes = data[1];
		bake_time.seconds = data[2];
		//write values to EEPROM
		write_to_EEPROM_8(REMAIN_PAGE_ADDR, BAKE_TIME_M_ADDR, bake_time.minutes);
		write_to_EEPROM_8(REMAIN_PAGE_ADDR, BAKE_TIME_S_ADDR, bake_time.seconds);
		break;
	default:
		//unknown input
		break;
	}
	
}

void turn_plate_left(uint8_t input)
{
	set_VSD_speed(IDLE_SPEED);
	if(input == 1)
	{
		//Ensure plate forward direction is OFF
		VSD_FORWARD_OFF;
		//Turn VSD Reverse ON
		VSD_REVERSE_ON;
	}
	else
	{
		//Turn VSD Reverse OFF
		VSD_REVERSE_OFF;
	}
}

void turn_plate_right(uint8_t input)
{
	set_VSD_speed(IDLE_SPEED);
	if(input == 1)
	{
		//Ensure plate reverse direction is OFF
		VSD_REVERSE_OFF;
		//Turn VSD Forward ON
		VSD_FORWARD_ON;
		
	}
	else
	{
		//Turn VSD Forward OFF
		VSD_FORWARD_OFF;
	}
}

void toggle_oven_lamp(uint8_t input)
{
	//Function to toggle the Oven Lamp 
	if(input == 1)
	{
		OVEN_LAMP_ON;
	}
	else
	{
		OVEN_LAMP_OFF;
	}
}

void set_VSD_speed(uint8_t spd)
{
	//function used to set the speed of the VSD via the DAC on PB2
	float spd_percentage = 0.0;
	uint16_t speed_12bit = 0;
	//Calculate the output speed as a 12bit (0-4096) digital value
	spd_percentage = 4096 * (float)spd/MAX_SPEED;
	speed_12bit = (uint16_t)spd_percentage;
	//wait to write to channel 1
	while(!(DACB.STATUS & DAC_CH0DRE_bm)){}
	//send 12bit value to DAC Data register
	DACB.CH0DATA = speed_12bit;
}

//-----------------------------EEPROM READ/WRITE Functions -------------------------------------//

void write_to_EEPROM_16(uint8_t page_addr ,uint8_t byte1_addr, uint8_t byte2_addr, uint16_t data){
	//function that writes 16bit data to a certain address location
	uint8_t data_1 = 0;
	uint8_t data_2 = 0;
	//To prevent overflow we store the 16bit value as 2x 8bit values (the data write register for the EEPROM is only 8bits)
	if(data >= 256){
		data_1 = 255;
		data_2 = data - data_1;
	}
	else
	{
		data_1 = data;
		data_2 = 0;
	}
	//Flush EEPROM buffer
	EEPROM_FlushBuffer();
	//Write the 16bit data as 2x 8bit values to the page buffer
	EEPROM_WriteByte(page_addr, byte1_addr, data_1);
	EEPROM_WriteByte(page_addr, byte2_addr, data_2);
}

void write_to_EEPROM_8(uint8_t page_addr ,uint8_t byte1_addr, uint16_t data){
	//function that writes 8bit data to a certain address location
	//Flush EEPROM buffer
	EEPROM_FlushBuffer();
	//write 8bit value to page buffer
	EEPROM_WriteByte(page_addr, byte1_addr, data);
}

uint16_t read_from_EEPROM_16(uint8_t page_addr ,uint8_t byte1_addr, uint8_t byte2_addr){
	//Function that reads 16bit data from a certain address location
	uint8_t received_data_1 = 0;
	uint8_t received_data_2 = 0;
	received_data_1 = EEPROM_ReadByte(page_addr, byte1_addr);
	received_data_2 = EEPROM_ReadByte(page_addr, byte2_addr);
	
	//Return 16bit value
	return (received_data_1 + received_data_2);
}

uint8_t read_from_EEPROM_8(uint8_t page_addr ,uint8_t byte1_addr){
	//Function that reads 8bit data from a certain address location
	return EEPROM_ReadByte(page_addr, byte1_addr);
}


//-----------------------------ISR Functions -------------------------------------//

/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTD0_RXC_vect)
{
	if(USART_RXComplete(&USART_data)){
		rx_received = true;
	};
	
}

/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTD0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data);
}

//Timer0 Overflow interrupt service routine
ISR(TCC0_OVF_vect)
{
	//Timer interrupt called every 10msec
	tick_current += 1;
	adc_tick_count += 1;
	
	if(adc_tick_count >= 100){
		//After 1sec fetch ADC value
		fetch_adc = true;
		adc_tick_count  = 0;
	}
}

//Timer1 Overflow interrupt service routine
ISR(TCC1_OVF_vect)
{
	//Timer interrupt called every 200msec
	//This timer is used to check the heating elements after 20min and to call the timeout chime every 30 sec
	element_ticks += 1;
	
	if(element_ticks >= 6000){
		//After 20 minutes check that the heating elements are working
		if(elements_work == false){
			send_error = true;
		}
	}
	
	if(repeat_chime == true){
		tick_chime += 1;
		if(tick_chime >= 150)
		{
			bake_timeout_flag = true;
			repeat_chime =  false;
		}
		
	}
	else
	{
		tick_chime = 0;
	}
	
}

