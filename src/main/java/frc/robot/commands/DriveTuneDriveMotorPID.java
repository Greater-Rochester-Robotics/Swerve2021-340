// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * for use with https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html
 */
public class DriveTuneDriveMotorPID extends CommandBase {
  //TODO:Create double variables for storing PID constants
  /** Creates a new DriveTuneDriveMotorPIDF. */
  public DriveTuneDriveMotorPID() {
    //TODO: Use addRequirements() here to use subsystem.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO:Push PID Constants(From Constants.java) to SmartDashboard
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //you can look at for refference https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java lines 79-94
    //TODO:Check if any SmartDashboard PID constants are different from field constants
    //TODO:Update PID constants to motorcontrollers if prior true 
    //TODO:use driveOneModule for each motor, position setting m0 to 135, m1 to -135, m2 to -45, and m3 to 45, set speed to all to joystick output times MAX_VELOCITY in Constants,mode = velocity
    //TODO:Print all module velocities
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //TODO:run the stop motors method
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
