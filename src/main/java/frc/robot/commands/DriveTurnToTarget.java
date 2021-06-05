// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveTurnToTarget extends CommandBase {
  int onTargetCount = 0;
  /** 
   * Creates a new DriveTurnToTarget. This command 
   * turns the robot to the goal target, and is 
   * meant for use in autonomous. It ends when the 
   * robot is on target.
   * 
   */
  public DriveTurnToTarget() {
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.limelight.setLightState(3);
    onTargetCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetAngle = RobotContainer.swerveDrive.getGyroInRad();
    System.out.print("current angle: "+ targetAngle);
    if(RobotContainer.limelight.haveTarget()){
      targetAngle -= Math.toRadians(RobotContainer.limelight.angleToTarget());
    }
    
    // else{
    //   targetAngle /= (Math.PI);
    //   if(Math.floor(targetAngle)%2 != 0)
    //   {
    //     targetAngle = Math.floor(targetAngle);
    //   }else{
    //     targetAngle = Math.ceil(targetAngle);
    //   }
    //   targetAngle *= Math.PI;
    // }
    System.out.print("   hasTarget: "+ RobotContainer.limelight.haveTarget());

      System.out.println("   targetAngle: "+ targetAngle);
    RobotContainer.swerveDrive.driveFieldCentric( 0.0, 0.0,
      RobotContainer.swerveDrive.getRobotRotationPIDOut(targetAngle) * 1.5,
      kDriveMode.percentOutput
    );
    if(RobotContainer.limelight.haveTarget() && Math.abs(RobotContainer.limelight.angleToTarget()) < 1.5){
      onTargetCount++;
    }else{
      onTargetCount = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.limelight.setLightState(1);
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTargetCount >= 4;
    // return RobotContainer.limelight.haveTarget() && Math.abs(RobotContainer.limelight.angleToTarget()) < .25;
  }
}
