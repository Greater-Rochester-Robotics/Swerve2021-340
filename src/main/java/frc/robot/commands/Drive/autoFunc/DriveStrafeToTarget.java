// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive.autoFunc;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveStrafeToTarget extends CommandBase {
  private double angle = 0;
  private int onTargetCount;
  public PIDController lateralPosController;

  /** Creates a new DriveStraffToTarget. */
  public DriveStrafeToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
    lateralPosController = new PIDController(Constants.AWAY_POS_P, Constants.AWAY_POS_I, Constants.AWAY_POS_D);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onTargetCount = 0;
    RobotContainer.limelight.setLightState(3);
    angle = RobotContainer.swerveDrive.getGyroInRad();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetPosition = 0;
    if(RobotContainer.limelight.haveTarget()){
      targetPosition = -1*(Math.tan(Math.toRadians(RobotContainer.limelight.angleToTarget()))*3.048);
    }
    
    SmartDashboard.putString("Target Position", "" + targetPosition);
    SmartDashboard.putString("Lateral Movement", "" + lateralPosController.calculate(targetPosition));
    SmartDashboard.putString("On Target?", "" + onTargetCount);
    RobotContainer.swerveDrive.driveFieldCentric(0, 
      lateralPosController.calculate(targetPosition, 0), 
      RobotContainer.swerveDrive.getRobotRotationPIDOut(angle), kDriveMode.percentOutput);
    
    if(Math.abs(angle - RobotContainer.swerveDrive.getGyroInRad()) < .03 && targetPosition < -.06){
      onTargetCount++;
    }else{
      onTargetCount = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTargetCount >= 10;
  }
}
