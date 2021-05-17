// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private static VictorSPX climberMotor;
  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new VictorSPX(Constants.CLIMBER);
    climberMotor.configVoltageCompSaturation(12.8);
    climberMotor.enableVoltageCompensation(true);

    climberMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend() {
    climberMotor.set(ControlMode.PercentOutput, Constants.CLIMBER_EXTEND_SPEED);

  }

  public void contract() {
    climberMotor.set(ControlMode.PercentOutput, Constants.CLIMBER_CONTRACT_SPEED);
  }

  public void stop() {
    climberMotor.set(ControlMode.PercentOutput, 0.0);

  }

  public void hardStop(){
    climberMotor.set(ControlMode.PercentOutput, -0.20);
  }

}
