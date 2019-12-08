/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <frc/commands/Command.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Spark.h>
#include <frc/XboxController.h>
#include <frc/drive/MecanumDrive.h>

#include "OI.h"
#include "commands/ExampleCommand.h"
#include "commands/AutoCommand.h"
#include "subsystems/ExampleSubsystem.h"


template<class BaseMotor>
class MinOutputMotor : public BaseMotor {
public:
  double minPower = 0;

  MinOutputMotor(int channel, double minPower = 0.15): BaseMotor(channel), minPower(minPower) {};

  void Set(double val) {
    if (val == 0) BaseMotor::Set(0);
    else BaseMotor::Set(std::copysign(fabs(val) * (1 - minPower) + minPower, val));
  }
};


class Robot : public frc::TimedRobot {
 public:
  static ExampleSubsystem m_subsystem;
  static OI m_oi;
  frc::MecanumDrive m_robotDrive{m_frontLeft, m_rearLeft, m_frontRight,
                                 m_rearRight};
  
  static Robot* instance;

  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  static constexpr int kFrontLeftChannel = 0;
  static constexpr int kRearLeftChannel = 1;
  static constexpr int kFrontRightChannel = 2;
  static constexpr int kRearRightChannel = 3;
  static constexpr int winchChannel = 4;
  static constexpr int kJoystickChannel = 0;

  MinOutputMotor<frc::Spark> m_frontLeft = MinOutputMotor<frc::Spark>(kFrontLeftChannel);
  MinOutputMotor<frc::Spark> m_rearLeft = MinOutputMotor<frc::Spark>(kRearLeftChannel);
  MinOutputMotor<frc::Spark> m_frontRight = MinOutputMotor<frc::Spark>(kFrontRightChannel);
  MinOutputMotor<frc::Spark> m_rearRight = MinOutputMotor<frc::Spark>(kRearRightChannel);
  frc::Spark m_winch = frc::Spark(winchChannel);

  frc::XboxController m_stick{kJoystickChannel};
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc::Command* m_autonomousCommand = nullptr;
  ExampleCommand m_defaultAuto;
  AutoCommand m_myAuto;
  frc::SendableChooser<frc::Command*> m_chooser;
};
