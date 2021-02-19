#include "control/lcd.hpp"
#include "control/autoRoutines.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
 /*
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
*/

void clear_screen(){
  clear();
}

void register_buttons(){
  register_btn0_cb(on_btn0);
  register_btn1_cb(on_btn1);
  register_btn2_cb(on_btn2);
}

void on_btn0(){
  
}

void on_btn1(){

}

void on_btn2(){

}
