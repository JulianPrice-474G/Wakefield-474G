#pragma once

void default_constants();
void SWP();
void Elim_Code_RED_Side();
void Auto_Skills();
void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();
void lever_movement_auton(int speed);
void lever_movement_auton_slow_lever(int speed);
extern int boost_time;
extern void intake_boost_task(void* param);
extern void shootLeverTask(void*);



extern int boost_time;           // declaration
extern pros::Task intakeTask;
extern pros::Task intakeTask2;
    // if you want a global task too


