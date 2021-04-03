// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveGenerateVelocityGraph extends CommandBase {
  Timer time = new Timer();
  double moveSpeed = 0.0;
  /** Creates a new DriveGenerateVelocityGraph. */
  public DriveGenerateVelocityGraph() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    moveSpeed+= 0.001;
    for(int i=0 ; i<4 ; i++){
      RobotContainer.swerveDrive.driveOneModule(i, moveSpeed, 0.0, kDriveMode.percentOutput);
    }

    System.out.println(time.get() +"," + moveSpeed + "," + 
      RobotContainer.swerveDrive.getCurrentVelocity().getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    moveSpeed=0.0;
    time.stop();
    RobotContainer.swerveDrive.stopAllModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
