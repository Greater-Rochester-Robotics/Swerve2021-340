/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Harvester extends SubsystemBase {
  private CANSparkMax axleWheels;
  private CANEncoder axleEncoder;
  private DoubleSolenoid harvesterPneu;
  private boolean harvesterJammed = false; 

  /**
   * Creates a new Intake.
   */
  public Harvester() {
    
    axleWheels = new CANSparkMax(Constants.BALL_HANDLER_MOTOR_0, MotorType.kBrushless);
    axleWheels.setSmartCurrentLimit(45, 60);
    harvesterPneu = new DoubleSolenoid(Constants.HARVESTER_FWD_CHANNEL,Constants.HARVESTER_REV_CHANNEL);
    axleEncoder = axleWheels.getEncoder();
  }

  public void setAxleWheels(double volts) {
    axleWheels.setVoltage(volts);
  }

  public void lowerHarvester(){
    // RobotContainer.limelight.setStreamMode(2);
    harvesterPneu.set(Value.kForward);
  }

  public void raiseHarvester(){
    harvesterPneu.set(Value.kReverse);
  }

  public boolean isHarvesterJammed() {
    boolean isJammed = false;
      isJammed =
        ((axleWheels.get() != 0) && (axleEncoder.getVelocity() == 0))||
        (axleWheels.getOutputCurrent() > 40.0 && axleEncoder.getVelocity() < 1500);
    return isJammed;
  }

  @Override
  public void periodic() {
    harvesterJammed = false;
    if(this.getCurrentCommand() != null){
      SmartDashboard.putString("harvester speed", "" + axleEncoder.getVelocity());
    if (isHarvesterJammed() ) {//&& (this.getCurrentCommand().getName().equals("Load"))
      SmartDashboard.putBoolean("isHarvesterJammed", true);
      harvesterJammed = true;
    } else {
      SmartDashboard.putBoolean("isHarvesterJammed", false);
      harvesterJammed = false;
    }
  }
}
public void setHarvesterJammed(boolean jam){
  harvesterJammed = jam;
}
public boolean stopIntakeQ(){
    return harvesterJammed;
  }
}
