// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

/**
 * for use with https://docs.ctre-phoenix.com/en/latest/ch16_ClosedLoop.html
 */
public class DriveTuneDriveMotorPID extends CommandBase {
  public double kP = Constants.SWERVE_DRIVE_P_VALUE;
  public double kI = Constants.SWERVE_DRIVE_I_VALUE;
  public double kD = Constants.SWERVE_DRIVE_D_VALUE;
  public double kF = Constants.SWERVE_DRIVE_F_VALUE;
  public double speed = 1;
  private double[] angle = new double[]{Math.toRadians(135), Math.toRadians(135), Math.toRadians(-45), Math.toRadians(45)};    

  /** Creates a new DriveTuneDriveMotorPIDF. */
  public DriveTuneDriveMotorPID(double speed) {
    this.speed = speed;
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO:Push PID Constants(From Constants.java) to SmartDashboard
    SmartDashboard.putNumber("driveP", kP);
    SmartDashboard.putNumber("driveI", kP);
    SmartDashboard.putNumber("driveD", kP);
    SmartDashboard.putNumber("driveF", kP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //you can look at for refference https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java lines 79-94
    //TODO:Check if any SmartDashboard PID constants are different from field constants
    double p = SmartDashboard.getNumber("driveP", 0);
    double i_I = SmartDashboard.getNumber("driveI", 0);
    double d = SmartDashboard.getNumber("driveD", 0);
    double ff = SmartDashboard.getNumber("driveF", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)||(i_I != kI)||(d != kD)||(ff != kF)) {
    
    }
    
    //TODO:Update PID constants to motorcontrollers if prior true 
    //TODO:use driveOneModule for each motor, position setting m0 to 135, m1 to -135, m2 to -45, and m3 to 45, set speed to all to joystick output times MAX_VELOCITY in Constants,mode = velocity
    for (int i=0; i<4; i++){
      RobotContainer.swerveDrive.driveOneModule(i, speed, angle[i], kDriveMode.velocity);
      SmartDashboard.putNumber("Module Velocity", RobotContainer.swerveDrive.getAllModuleVelocity()[i]);
    }
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
