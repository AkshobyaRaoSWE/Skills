#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "lemlib/api.hpp"


pros::Controller master(pros::E_CONTROLLER_MASTER); 
lemlib::ExpoDriveCurve driveCurve(3, 10, 1.019);

pros::MotorGroup left_mg({-1, -2, -3}, pros::MotorGearset::blue);    
pros::MotorGroup right_mg({13, 12, 18}, pros::MotorGearset::blue); 

pros::Motor flexwheel(11, pros::MotorGearset::green);
pros::Motor chain(14, pros::MotorGearset::blue);
pros::Motor lb(5, pros::MotorGearset::green);

pros::adi::Pneumatics mogo('A', false);
pros::adi::Pneumatics doinker('B', false); 

lemlib::Drivetrain drivetrain(
	&left_mg,
    &right_mg,
    11.25,
	lemlib::Omniwheel::NEW_325,
    450, 
    2
);

//! Make sure you test both odom sensors, imu sensor after adding the sensors
// pros::Imu imu(15); 
// pros::Distance distance_sensor(9);

//! Make sure you test both odom sensors, imu sensor after adding the sensors(test both ports to see if they are reversed)
// pros::Rotation yOdom(-16); 
// pros::Rotation ladyOdom(9); 


// lemlib::TrackingWheel vertical_tracking_wheel(&yOdom, lemlib::Omniwheel::NEW_2, -2.5); //! Measure offset

lemlib::OdomSensors sensors(
    nullptr, 
    nullptr,
    nullptr,
    nullptr,
    nullptr
);


// PID(After you tune, everything should work as expect, if not look at the printscreens to see if the robot thinks its getting to such positions, if so, then its likely a PID tuning issue, will take sometime but you have time)
// Do default, set 0, slowly adjust each factor, 
// at the end, should have smooth motions


lemlib::ControllerSettings lateral_controller(
	10, // (kP)
    0, // (kI)
    3, // (kD)
    3, // anti windup
    1, // small error range, in inches
	100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(
	2, // (kP)
	0, // (kI)
    10, // (kD)
	3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(
    drivetrain, 
    lateral_controller,
    angular_controller, 
    sensors, 
    &driveCurve,
    &driveCurve
);



void on_center_button() {}

void initialize() {
	pros::lcd::initialize();
    pros::delay(100);

	pros::delay(100);

    chassis.calibrate();
	pros::delay(500);

	pros::lcd::print(1, "Calibration Complete");

    //! Copy THis ->

	pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::delay(20);
        }
    });
    //!_________________________
}

void disabled() {} // NOT NEEDED!!!

void competition_initialize() {} // NOT NEEDED!

void autonomous() {
	chassis.setPose(0,0,0);
    chassis.turnToHeading(90,4000);
} 

void opcontrol() {
	while (true) {

        // movement
		int leftY = master.get_analog(ANALOG_LEFT_Y);
		int rightY = master.get_analog(ANALOG_RIGHT_Y);
		left_mg.move(leftY);
		right_mg.move(rightY);

		pros::delay(20); // Run for 20 ms then update

        //! Mogo Clamp
        if(master.get_digital(DIGITAL_B)){
            mogo.extend();
        } else if(master.get_digital(DIGITAL_DOWN)){
            mogo.retract();
        }

        //! Doinker
        if(master.get_digital(DIGITAL_L1)){
            doinker.set_value(true);
        } else {
            doinker.set_value(false);
        }


        //! Intake
        if(master.get_digital(DIGITAL_R1)){
            chain.move(400);
            flexwheel.move(400);
        } else if(master.get_digital(DIGITAL_R2)){
            chain.move(-400);
            flexwheel.move(-400);
        } else{
            chain.move(0);
            flexwheel.move(0);
        }
	}
}