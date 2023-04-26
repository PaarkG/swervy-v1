// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private XboxController driveController;
  private Swerve swerve;

  private boolean fieldRelativeDrive = true;

  @Override
  public void robotInit() {
    driveController = new XboxController(InputConstants.kDriveControllerPort);
    swerve = Swerve.getInstance();
  }

  @Override
  public void robotPeriodic() {
    swerve.updateOdometry();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if(driveController.getYButtonPressed()) {
      if(fieldRelativeDrive) {
        fieldRelativeDrive = false;
      } else {
        fieldRelativeDrive = true;
      }
    }

    if(driveController.getLeftX() != 0.0 || driveController.getLeftY() != 0.0 || driveController.getRightX() != 0.0) {
      swerve.drive(-driveController.getLeftY(), -driveController.getLeftX(), -driveController.getRightX(), fieldRelativeDrive, false);
    }

    if(driveController.getXButtonPressed()) {
      swerve.setX();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
