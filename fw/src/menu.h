#pragma once
#include "board.h"
void menu_dtr(bool on);   /* config port opened/closed (USB ISR) */
void menu_sof(void);      /* 1 ms tick (USB ISR): live page refresh */
void menu_init(void);
