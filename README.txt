################################################################################
/*									      */
/*		sdup751 				npet532		      */
/*		     		      CS723				      */
/*		                   Assignment 1			              */
/*									      */
################################################################################

	Files included:
	SOPCfiles ->	
	|	software->
	|	|	Assignment_One->
	|	|	|	freeRTOS->
	|	|	|	|	- rtos source
	|	|	- freq_relay_ass1.c
	| 	|	- freq_relay_ass1.h
	| 	|	- FSM.c
	|	|	- FSM.h
	|	- freq_relay_controller.sof
	|	- nios2.sopcinfo
	
	Getting Started:
 
	- Connect VGA cable to monitor (VGA Output)
	- Open NIOS II 13.0
	- Use Nios II -> 'Quartus II Programmer' to init DE-II 
	  board with .sof file
	- Import project into NIOS workspace and build Assignment_One
	- Select Run -> Run As -> Nios II Hardware

	Interfacing with frequency relay: 

		- PS/2 keyboard (enter/left/right/up/down)
		- SlideSwitches 4 down to 0 (first 5)
		- PushButton3

	Once Program Is Running: 

	- SlideSwitches4 -> SlideSwitches0 to toggle loads manually
	  (4 being highest priority)
	- RedLed On indicates load ON
	- GreenLed indicates load SHED(not toggled off manually)
	- System will automatically start in IDLE state and 
	  switch to MANAGE as needed (on monitor display)
	
	- Thresholds can ONLY be adjusted in Maintenance State

	To Enter Maintenance State:

	- Push PushButton3 
	- Use up/down and left/right arrow keys on keyboard to adjust 
	  thresholds as you wish 
	
	- To EXIT maintenance press EnterKey on keyboard

	All timing information displayed on monitor


################################################################################
