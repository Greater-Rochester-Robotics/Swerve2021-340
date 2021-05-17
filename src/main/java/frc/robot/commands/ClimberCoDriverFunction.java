/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;

public class ClimberCoDriverFunction extends CommandBase {
  /**
   * Creates a new ClimberCoDriverFunction.
   */
  public ClimberCoDriverFunction() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.robotContainer.setCoDriverRumble(.5, .5);

    if(Robot.robotContainer.getCoDriverAxis(Axis.RIGHT_TRIGGER)>.3){
      RobotContainer.climber.contract();
    }else if(Robot.robotContainer.getCoDriverButton(6)){
      RobotContainer.climber.extend();
    }else{
      RobotContainer.climber.stop();
    }

    // if(Robot.robotContainer.getCoDriverAxis(Axis.LEFT_TRIGGER)>.3){
    //   RobotContainer.climber.rightArmContract();
    // }else if(Robot.robotContainer.getCoDriverButton(5)){
    //   RobotContainer.climber.rightArmExtend();
    // }else{
    //   RobotContainer.climber.rightStop();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.robotContainer.setCoDriverRumble(0, 0);
    RobotContainer.climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}