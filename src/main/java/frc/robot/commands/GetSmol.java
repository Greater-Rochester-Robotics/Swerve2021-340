/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SnekLoader;

public class GetSmol extends CommandBase {
  /**
   * This command causes the robot to stop all 
   * subsystems but drive, and return to the 
   * robot to its smallest state.
   * 
   * @requires SnekLoader, Harvester, Shooter
   */
  public GetSmol() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.snekLoader, RobotContainer.harvester, RobotContainer.shooter, RobotContainer.limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.harvester.raiseHarvester();
    RobotContainer.snekLoader.setState(SnekLoader.State.kOff);//turn off the snekloader
    RobotContainer.shooter.stop();
    RobotContainer.shooter.lowerHood();
    // RobotContainer.shooter.raiseHardStop();
    RobotContainer.limelight.setLightState(1);
   RobotContainer.snekLoader.setPause(false);//since the loader is off, turn off the pause

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
