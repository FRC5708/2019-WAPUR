/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/Spark.h>
#include <frc/XboxController.h>

#define leftHand frc::XboxController::JoystickHand::kLeftHand
#define rightHand frc::XboxController::JoystickHand::kRightHand

ExampleSubsystem Robot::m_subsystem;
OI Robot::m_oi;

Robot* Robot::instance;

Robot::Robot(){
	instance = this;
}

void Robot::RobotInit() {
	m_chooser.SetDefaultOption("Default Auto", &m_defaultAuto);
	m_chooser.AddOption("My Auto", &m_myAuto);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

}

double removeDeadzone(double input, double inputRangeLow, double inputRangeHigh = 1, bool squareInput = true) {

	double output = (fabs(input) - inputRangeLow) / (inputRangeHigh - inputRangeLow);
	if (output > 1) output = 1;
	
	if (output <= 0) return 0;

	if (squareInput) output = output*output;

	if (input < 0) output = -output;
	return output;
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
	//std::string autoSelected = frc::SmartDashboard::GetString(
	//     "Auto Selector", "Default");
	// if (autoSelected == "My Auto") {
	m_autonomousCommand = &m_myAuto;
	// } else {
	//   m_autonomousCommand = &m_defaultAuto;
	// }

	//m_autonomousCommand = m_chooser.GetSelected();

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

  // triggers forward/back, left stick strafe, right stick turn
  /*m_robotDrive.DriveCartesian(
    -removeDeadzone(m_stick.GetX(frc::XboxController::JoystickHand::kLeftHand), 0.15),
    removeDeadzone(m_stick.GetTriggerAxis(frc::XboxController::JoystickHand::kRightHand) 
    - m_stick.GetTriggerAxis(frc::XboxController::JoystickHand::kLeftHand), 0.15),
    removeDeadzone(m_stick.GetX(frc::XboxController::JoystickHand::kRightHand), 0.15));*/

    // left stick forward/back, right stick strafe, triggers turn
    m_robotDrive.DriveCartesian(
    -removeDeadzone(m_stick.GetX(frc::XboxController::JoystickHand::kRightHand), 0.15),
    -removeDeadzone(m_stick.GetY(frc::XboxController::JoystickHand::kLeftHand), 0.15),
    removeDeadzone(m_stick.GetTriggerAxis(frc::XboxController::JoystickHand::kRightHand) 
    - m_stick.GetTriggerAxis(frc::XboxController::JoystickHand::kLeftHand), 0.15));

		if(m_stick.GetBumper(leftHand)) m_winch.Set(1);
		else if(m_stick.GetBumper(rightHand)) m_winch.Set(-1);
		else m_winch.Set(0);
}


void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
