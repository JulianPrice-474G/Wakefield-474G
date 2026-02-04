#include "main.h"

  /////
  // For installation, upgrading, documentations, and tutorials, check out our website!
  // https://ez-robotics.github.io/EZ-Template/
  /////
  //these are the global objects used for the robot
// Chassis constructor
#include "subsystems.hpp"
pros::adi::DigitalOut Matchloader('F');
pros::adi::DigitalOut DESCORE_MECH('D');
pros::adi::DigitalOut Hood2('A');
pros::adi::DigitalOut RobotUp('H');
pros::adi::DigitalOut Hood('C');
pros::adi::DigitalIn limit_switch('B');

pros::Motor intake(10); // Intake motor on port 10
pros::Motor lever(9);
bool RAISE_LOWER_BOT_STATE = true;
bool DESCORE_MECH_STATE = false;
bool MATCHLOADER_STATE = false;
bool HOOD_STATE = false;
int MaxMotorTorque = 200;
int Unjamingtime = 500;
bool shooting = false;


static pros::Task* shootTask = nullptr;
//speedMultiplier = 0.5;// Set max motor torque to 100%
ez::Drive chassis(
   // These are your drive motors, the first motor is used for sensing!
   {-17, -18, -19},     // Left Chassis Ports (negative port will reverse it!)
   {14, 15, 16},  // Right Chassis Ports (negative port will reverse it!)


   13,      // IMU Port
   3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
   360);
  // Wheel RPM = cartridge * (motor gear / wheel gear)


// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels


/**
* Runs initialization code. This occurs as soon as the program is started.
*
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds.
*/
void ez_screen_task();


void initialize() {
 // Print our branding over your terminal :D
 ez::ez_template_print();


 pros::delay(500);  // Stop the user from doing anything while legacy ports configure


 // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
 //  - change `back` to `front` if the tracking wheel is in front of the midline
 //  - ignore this if you aren't using a horizontal tracker
 // chassis.odom_tracker_back_set(&horiz_tracker);
 // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
 //  - change `left` to `right` if the tracking wheel is to the right of the centerline
 //  - ignore this if you aren't using a vertical tracker
 // chassis.odom_tracker_left_set(&vert_tracker);


 // Configure your chassis controls
 chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
 chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
 chassis.opcontrol_curve_default_set(0.0, 0.0);   // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)


 // Set the drive to your own constants from autons.cpp!
 default_constants();



 // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
 // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
 // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);


 // Autonomous Selector using LLEMU
 ez::as::auton_selector.autons_add({
        {"Solo AWP\n\nSOLO AWP", SWP},
  {"Elim CODE\n\nElim CODE RED SIDE", Elim_Code_RED_Side},
  
      {"Auto Skills\n\nAuto Skills", Auto_Skills},

    
     /*{"Test Drivetrain\n\nDrive forward 12 inches", drivetrain_test_80},
     {"Test Drivetrain\n\nDrive forward 100 inches", drivetrain_test_100},
     {"blue driver auton\n\nDrive forward 3 feet", drive_Auton_Blue},
     {"red driver auton\n\nDrive forward 3 feet", drive_Auton_Red},*/
     {"PID turn\n\nTurn test", turn_example},
     {"Drive\n\nDrive forward and come back", drive_example},
     {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
     {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
     {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
     {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
     {"Combine all 3 movements", combining_movements},
     {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
     {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
    // {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
     //{"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
     //{"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
     //{"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
     {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
 });


 // Initialize chassis and auton selector
 chassis.initialize();
 ez::as::initialize();
 master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
 pros::Task ezScreenTask(ez_screen_task);

}


/**
* Runs while the robot is in the disabled state of Field Management System or
* the VEX Competition Switch, following either autonomous or opcontrol. When
* the robot is enabled, this task will exit.
*/
void disabled() {
 // . . .
}


/**
* Runs after initialize(), and before autonomous when connected to the Field
* Management System or the VEX Competition Switch. This is intended for
* competition-specific initialization routines, such as an autonomous selector
* on the LCD.
*
* This task will exit when the robot is enabled and autonomous or opcontrol
* starts.
*/
void competition_initialize() {

 // . . .
}


/**
* Runs the user autonomous code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the autonomous
* mode. Alternatively, this function may be called in initialize or opcontrol
* for non-competition testing purposes.
*
* If the robot is disabled or communications is lost, the autonomous task
* will be stopped. Re-enabling the robot will restart the task, not re-start it
* from where it left off.
*/
void autonomous() {
 chassis.pid_targets_reset();                // Resets PID targets to 0
 chassis.drive_imu_reset();                  // Reset gyro position to 0
 chassis.drive_sensor_reset();               // Reset drive sensors to 0
 // Set the current position, you can start at a specific position with this
 chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  // Set motors to hold.  This helps autonomous consistency\


 ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}


/**
* Simplifies printing tracker values to the brain screen
*/
void screen_print_tracker(ez::tracking_wheel * tracker, std::string name, int line) {
 std::string tracker_value = "", tracker_width = "";
 // Check if the tracker exists
 if (tracker != nullptr) {
   tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
   tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
 }
 ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}


/**
* Adding new pages here will let you view them during user control or autonomous
* and will help you debug problems you're having
*/
void ez_screen_task() 
{

  while (true) {
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (!chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(3)) {
          if (!pros::competition::is_connected()) {
          pros::lcd::clear_line(1);
          pros::lcd::clear_line(2);
          pros::lcd::clear_line(3);
          pros::lcd::clear_line(4);
          pros::lcd::clear_line(5);
          pros::lcd::clear_line(6);
          pros::lcd::clear_line(7);

          // Get the temperature of all motors in Celsius
          double tempIntake = Intake.get_temperature();
          double tempLever = Lever.get_temperature();
          double tempLeftDrive = chassis.left_motors[0].get_temperature();
          double tempRightDrive = chassis.right_motors[0].get_temperature();
          
          // Format the temperatures as strings
          std::string tempText1 = "Intake: " + std::to_string(tempIntake) + "C";
          std::string tempText2 = "Lever: " + std::to_string(tempLever) + "C";
          std::string tempText3 = "LeftDrive: " + std::to_string(tempLeftDrive) + "C";
          std::string tempText4 = "RightDrive: " + std::to_string(tempRightDrive) + "C";

          // Display the temperatures on the brain LCD screen
          pros::lcd::set_text(0, tempText1);
          pros::lcd::set_text(1, tempText2);
          pros::lcd::set_text(2, tempText3);
          pros::lcd::set_text(3, tempText4);

          // Display X, Y, and Theta
      
          }
        }
      }
    }
  }
}

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

    /*if (!pros::competition::is_connected()) {
      if (!chassis.pid_tuner_enabled()) {
        if (ez::as::page_b lank_is_on(0)) {
          pros::lcd::clear_line(1);
          pros::lcd::clear_line(2);
          pros::lcd::clear_line(3);
          pros::lcd::clear_line(4);
          pros::lcd::clear_line(5);
          pros::lcd::clear_line(6);
          pros::lcd::clear_line(7);

          // Get the temperature of all motors in Celsius
          double tempIntake = Intake.get_temperature();
          double tempLever = Lever.get_temperature();
          double tempLeftDrive = chassis.left_motors[0].get_temperature();
          double tempRightDrive = chassis.right_motors[0].get_temperature();
          
          // Format the temperatures as strings
          std::string tempText1 = "Intake: " + std::to_string(tempIntake) + "C";
          std::string tempText2 = "Lever: " + std::to_string(tempLever) + "C";
          std::string tempText3 = "LeftDrive: " + std::to_string(tempLeftDrive) + "C";
          std::string tempText4 = "RightDrive: " + std::to_string(tempRightDrive) + "C";

          // Display the temperatures on the brain LCD screen
          pros::lcd::set_text(0, tempText1);
          pros::lcd::set_text(1, tempText2);
          pros::lcd::set_text(2, tempText3);
          pros::lcd::set_text(3, tempText4);

        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);
*/
/**
* Gives you some extras to run in your opcontrol:
* - run your autonomous routine in opcontrol by pressing DOWN and B
*   - to prevent this from accidentally happening at a competition, this
*     is only enabled when you're not connected to competition control.
* - gives you a GUI to change your PID values live by pressing X
*/
void ez_template_extras() {
 // Only run this when not connected to a competition switch
 if (!pros::competition::is_connected()) {
   // PID Tuner
   // - after you find values that you're happy with, you'll have to set them in auton.cpp


   // Enable / Disable PID Tuner
   //  When enabled:
   //  * use A and Y to increment / decrement the constants
   //  * use the arrow keys to navigate the constants
   if (master.get_digital_new_press(DIGITAL_X)){
     chassis.pid_tuner_toggle();}

  


   // Trigger the selected autonomous routine
   if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_LEFT)) {
    pros::lcd::print(1, "Running autonomous...");
     master.rumble(".");


     pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
     autonomous();
     chassis.drive_brake_set(preference);
   }


   // Allow PID Tuner to iterate
   chassis.pid_tuner_iterate();
 }


 // Disable PID Tuner when connected to a comp switch
 else {
   if (chassis.pid_tuner_enabled())
     chassis.pid_tuner_disable();
    
 }
}
void shootLeverTask(void* power) {
int powerValue = (int)(intptr_t)power;
shooting = true;
Hood.set_value(true);  // Hood on when lever spins
Hood2.set_value(true);

// Move UP until switch or timeout
std::uint32_t start = pros::millis();
while (!limit_switch.get_value() &&
       pros::millis() - start < 1300) {
  lever.move(-powerValue);
  intake.move(127);  // UP (flipped)
  pros::delay(10);
}

// Move DOWN briefly
pros::delay(100);
lever.move(powerValue);
intake.move(0);  // DOWN
pros::delay(1000);

// Let it fall / relax
lever.move(0);
pros::delay(300);
Hood.set_value(false);  // Hood off when lever stops
Hood2.set_value(false);

shooting = false;
  // Clean up task
  shootTask = nullptr;
}




/**
* Runs the operator control code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the operator
* control mode.
*
* If no competition control is connected, this function will run immediately
* following initialize().
*
 ez_template_extras();
    chassis.opcontrol_arcade_standard(ez::SPLIT);  
* If the robot is disabled or communications is lost, the
* operator control task will be stopped. Re-enabling the robot will restart the
* task, not resume it from where it left off.
*/
void opcontrol() {
  Hood.set_value(false);  // Hood off at start
  Hood2.set_value(false);

  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  bool reachedSpeed = false;

  bool r2_was_pressed = false;
  std::uint32_t r2_press_time = 0;

  intake.set_brake_mode(MOTOR_BRAKE_COAST);
  lever.set_brake_mode(MOTOR_BRAKE_COAST);

  while (true) {
    ez_template_extras();
int forward = master.get_analog(ANALOG_LEFT_Y);
int turn    = master.get_analog(ANALOG_RIGHT_X);

// Apply EZ joystick curves
forward = chassis.opcontrol_curve_left(forward);
turn    = chassis.opcontrol_curve_right(turn);

// Slow mode while holding R2
if (master.get_digital(DIGITAL_RIGHT)) {
  forward *= 0.5;
  turn    *= 0.5;
}

// Send to drivetrain
chassis.opcontrol_joystick_threshold_iterate(
  forward + turn,
  forward - turn
);



    // ================= ROBOT UP / DOWN (HOLD RIGHT) =================
    if (master.get_digital(DIGITAL_L2)) {
      RobotUp.set_value(false);// LOWER
      DESCORE_MECH.set_value(false);  // LOWER
    } else {
      RobotUp.set_value(true);   // RAISE
    }
  if (!shooting) {
    // ================= R1 INTAKE LOGIC =================
    if (master.get_digital(DIGITAL_R1)) {
      if (!reachedSpeed) {
        lever.move(127);
    
        if (lever.get_current_draw() <= 2000) {
          intake.move(127);
        } else {
          intake.move(127);
          reachedSpeed = true;
        }
      } else {
        intake.move(127);
        lever.move(127);

      }
    } else {
      reachedSpeed = false;
      lever.move(0);

    }

    // ================= R2 TIMING LOGIC =================
    bool r2_now = master.get_digital(DIGITAL_R2);
    std::uint32_t now = pros::millis();

    if (!master.get_digital(DIGITAL_R1)) {
      if (r2_now) {
        if (!r2_was_pressed) {
          r2_press_time = now;
        }

        std::uint32_t elapsed = now - r2_press_time;

        if (elapsed < 100) {
          intake.move(127);     // intake
        } else {
          intake.move(-127);    // outtake
        }
      } else {
        intake.move(0);
      }
    }

    r2_was_pressed = r2_now;
  }

    // ================= SOLENOIDS / MACROS =================
    if (master.get_digital_new_press(DIGITAL_B)) {
      DESCORE_MECH_STATE = !DESCORE_MECH_STATE;
      DESCORE_MECH.set_value(DESCORE_MECH_STATE);
    }

    else if (master.get_digital_new_press(DIGITAL_DOWN)) {
      MATCHLOADER_STATE = !MATCHLOADER_STATE;
      Matchloader.set_value(MATCHLOADER_STATE);
    }
    else if (master.get_digital_new_press(DIGITAL_Y)) {//Set the leber to toggle from netural and down
      HOOD_STATE = !HOOD_STATE;
      if (HOOD_STATE==true){
        Hood2.set_value(false);
        Hood.set_value(true);
      }
      else{
        Hood2.set_value(false);
        Hood.set_value(false);
   
    }
    }




if (master.get_digital_new_press(DIGITAL_L1)) {

  if (shootTask == nullptr  ) {
    // Use 70 power when bot is lowered (L2 pressed), 127 when bot is up
    int leverPower = master.get_digital(DIGITAL_L2) ? 70 : 127;
    shootTask = new pros::Task(shootLeverTask, (void*)leverPower);




}
}
 pros::delay(ez::util::DELAY_TIME);
  }