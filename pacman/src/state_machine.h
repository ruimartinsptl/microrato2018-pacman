//
// Created by Rui Filipe de Sousa Martins on 01/05/2018.
//

#ifndef PACMAN_STATE_MACHINE_H
#define PACMAN_STATE_MACHINE_H

// Mouse States
// TODO: INSERT YOUR CODE HERE ...
#define STATE_MOUSE_WAITTING_TO_START 1
#define STATE_MOUSE_AVOIDING_COLISION 2
#define STATE_MOUSE_WALKING 3
//    #define STATE_MOUSE_GOING_TO_CHEESE 2
#define STATE_MOUSE_ON_CHEESE 4
//    #define STATE_MOUSE_BACK_TO_HOME 4
#define STATE_MOUSE_ALL_DONE 5
#define STATE_MOUSE_ABORTED 6
#define STATE_MOUSE_TIMEOUT 7
#define STATE_MOUSE_SEARCH_BEACON 8
// TODO: INSERT YOUR CODE HERE ...


#define STATE_JOURNEY_GOING_TO_CHEESE 1
#define STATE_JOURNEY_RETURN_TO_HOME 2

#endif //PACMAN_STATE_MACHINE_H
