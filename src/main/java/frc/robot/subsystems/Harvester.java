/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Harvester extends SubsystemBase {
  
  private DoubleSolenoid harvesterPneu;
  /**
   * Creates a new Intake.
   */
  public Harvester() {
    harvesterPneu = new DoubleSolenoid(Constants.HARVESTER_FWD_CHANNEL,Constants.HARVESTER_REV_CHANNEL);
  }
  public void lowerHarvester(){
    // RobotContainer.limelight.setStreamMode(2);
    harvesterPneu.set(Value.kForward);
  }

  public void raiseHarvester(){
    harvesterPneu.set(Value.kReverse);
  }

  @Override
  public void periodic() {
  }

}
