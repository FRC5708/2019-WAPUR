/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>

#include "Robot.h"

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/Spark.h>
#include <frc/XboxController.h>

ExampleSubsystem Robot::m_subsystem;
OI Robot::m_oi;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption("Default Auto", &m_defaultAuto);
  m_chooser.AddOption("My Auto", &m_myAuto);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  m_frontLeft.SetInverted(true);
  m_rearLeft.SetInverted(true);

}

double inputTransform(double input, double minPowerOutput, double inputDeadZone, bool squareInput = true) {

	double output = (fabs(input) - inputDeadZone) / (1 - inputDeadZone);
	
	if (output <= 0) return 0;

  if (squareInput) output = output*output;
  output = output*(1 - minPowerOutput) + minPowerOutput;

	if (input < 0) output = -output;
	return output;
}
translation renormalizeDeadzone(translation input, double input_threshold, double minimum_power_output){
  if((input.x*input.x + input.y*input.y) <= input_threshold*input_threshold){//Our input is within the zone of death
    return translation{0,0};
  }
  double renorm_x=input.x / (1-input_threshold); //Create linear mapping from 0 to 1 after threshold to boundary
  double renorm_y=input.y / (1-input_threshold);
  double resize_x=renorm_x * (1-minimum_power_output) + copysign(minimum_power_output,input.x); //Semi-linear transform from [0,1] => [min_power_output,1]
  double resize_y=renorm_y * (1-minimum_power_output) + copysign(minimum_power_output,input.y);
  return translation{resize_x,resize_y};
}
/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() { frc::Scheduler::GetInstance()->Run(); }

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString code to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional commands to the
 * chooser code above (like the commented example) or additional comparisons to
 * the if-else structure below with additional strings & commands.
 */
void Robot::AutonomousInit() {
  // std::string autoSelected = frc::SmartDashboard::GetString(
  //     "Auto Selector", "Default");
  // if (autoSelected == "My Auto") {
  //   m_autonomousCommand = &m_myAuto;
  // } else {
  //   m_autonomousCommand = &m_defaultAuto;
  // }

  m_autonomousCommand = m_chooser.GetSelected();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Start();
  }
}

void Robot::AutonomousPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

void Robot::TeleopPeriodic() { 
  frc::Scheduler::GetInstance()->Run();
  double joy_x=m_stick.GetX(frc::XboxController::JoystickHand::kLeftHand);
  double joy_y=m_stick.GetY(frc::XboxController::JoystickHand::kLeftHand);
  translation input{joy_x,joy_y};
  translation renormalized_input=renormalizeDeadzone(input,.25,.2); //TODO: change .1, .1
  m_robotDrive.DriveCartesian(
    renormalized_input.x,
    renormalized_input.y,
    inputTransform(m_stick.GetX(frc::XboxController::JoystickHand::kRightHand), 0.15, 0.1));
  
  std::cout << "@fl: " << m_frontLeft.Get() << std::endl;
  std::cout << "@fr: " << m_frontRight.Get() << std::endl;
  std::cout << "@rl: " << m_rearLeft.Get() << std::endl;
  std::cout << "@rr: " << m_rearRight.Get() << std::endl;

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
