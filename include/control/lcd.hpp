#ifndef _LCD_HPP_
#define _LCD_HPP_

#include "main.h"

using namespace pros::lcd;

#define NUM_ROUTINES 3;

const std::string titles[] = {"skills", "skills1", "matchAutonL", "matchAutonR"};

void clear_screen();

void register_buttons();

void on_btn0();

void on_btn1();

void on_btn2();

#endif
