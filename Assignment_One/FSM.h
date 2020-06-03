#pragma once
static const char* STATE_STRING[3] = { "IDLE","MANAGE","MAINTENENCE" };  // USED FOR PRINTING STRINGS

typedef enum {
	IDLE,
	BAL,
	MAIN
} state;

typedef enum {
    LD_FLAG0,
    LD_FLAG1,
    MT_BTTN,
    TH_CONF
} event;

state STATE_MATRIX[3][4];
state NEXT_STATE;
state CURRENT_STATE;

void stateEval(event E);
