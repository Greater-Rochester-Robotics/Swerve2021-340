/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.SnekLoader;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SnekLoader.State;

public class LoadAcc extends CommandBase {
  Timer tm = new Timer();
  private boolean raiseOnEnd;

  /**
   * Creates a new LoadAcc command. This command lowers 
   * the harvester, and sets the snek to intake for 3 
   * balls. This is for the Interstellar Accuracy Challenge.
   * It will raise the harvester on the commands termination.
   * 
   * @requires SnekLoader, Harvester
   */
  public LoadAcc(){
    this(true);
  }
  
  /**
   * Creates a new LoadAcc command. This command lowers 
   * the harvester, and sets the snek to intake for 3 
   * balls. This is for the Interstellar Accuracy Challenge.
   * Harester is raised when command ends if raiseOnEnd 
   * is true.
   * 
   * @requires SnekLoader, Harvester
   * @param raiseOnEnd true if the harvester raised by command when ended
   */
  public LoadAcc(boolean raiseOnEnd) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.raiseOnEnd = raiseOnEnd;
    addRequirements(RobotContainer.snekLoader, RobotContainer.harvester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.snekLoader.setState(State.kAccFillTo3);
    RobotContainer.harvester.lowerHarvester();
    RobotContainer.snekLoader.setHarvesterJammed(false);
    tm.reset();
    tm.start();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(raiseOnEnd){
      RobotContainer.harvester.raiseHarvester();
    }
    RobotContainer.snekLoader.setState(State.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
  //  return (RobotContainer.snekLoader.getState() == State.kOff || (tm.get() > 0.25 &&RobotContainer.snekLoader.stopIntakeQ()));//|| RobotContainer.harvester.stopIntakeQ()
   return false;
  }
}
