#include "main.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"


// TODO: Make AUTON selector 


// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER); //! SIGMA CONTROLLER 🤖
lemlib::ExpoDriveCurve driveCurve(5, 12, 1.132);

// Motor Groups
pros::MotorGroup left_mg({-1, -2, -3}, pros::MotorGearset::blue);    
pros::MotorGroup right_mg({5, 11, 8}, pros::MotorGearset::blue); 

// Single Motors
pros::Motor flexwheel(18);
pros::Motor chain(15);
pros::Motor ladyBrown(21);

// Solenoids
pros::adi::Pneumatics mogo('A', false);
pros::adi::Pneumatics doinker('B', false); //? IT DOINKS!!!!

// Drivetrain
lemlib::Drivetrain drivetrain(
	&left_mg, // left motor group
    &right_mg, // right motor group
    11.25, // 11.25 inch track width
	lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
    450, // drivetrain rpm is 450
    2 // horizontal drift is 2 (for now)
);


//? Random code ->

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	//? Testing LCD screen for fun
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Sigma Bots 📢");
	pros::lcd::register_btn1_cb(on_center_button);
}

void disabled() {} // NOT NEEDED!!!

void competition_initialize() {} // NOT NEEDED!

void autonomous() {} 

void opcontrol() {
	while (true) {
		int leftY = master.get_analog(ANALOG_LEFT_Y);
		int rightY = master.get_analog(ANALOG_RIGHT_Y);
		left_mg.move(leftY);
		right_mg.move(rightY);
		pros::delay(20); // Run for 20 ms then update
	}
}