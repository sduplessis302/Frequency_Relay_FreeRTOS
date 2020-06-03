#include "FSM.h"

void stateEval(event E)
{

    printf("    EVALUATING STATE\n");
    NEXT_STATE = STATE_MATRIX[CURRENT_STATE][E]; // determine next state and next action from FSM table
    printf("    TRANSITIONING %s -> %s\n", STATE_STRING[CURRENT_STATE], STATE_STRING[NEXT_STATE]);
    CURRENT_STATE = NEXT_STATE;   // transition to next state

}