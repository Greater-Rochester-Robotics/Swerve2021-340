/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//PROTOTYPE

package frc.robot.subsystems;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.nio.file.Files;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax shooterWheel;
  private CANEncoder shooterEncoder;
  private double targetVelocity;
  private DigitalInput ballCounter;
  private Solenoid hoodMover, hardStop;

  private int ballsShot = 0;
  private int totalBallsShot = 0;
  private boolean ballWasPresent;
  int smartCount = 0;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {

    shooterWheel = new CANSparkMax(Constants.SHOOTER_WHEEL, MotorType.kBrushless);
    shooterWheel.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterWheel.getPIDController().setP(0.00075);
    shooterWheel.getPIDController().setI(0.0);
    shooterWheel.getPIDController().setD(2.0);
    shooterWheel.getPIDController().setFF(0.00019);
    // practice bot PIDF values
    // shooterWheel.getPIDController().setP(0.001);
    // shooterWheel.getPIDController().setI(0.0);
    // shooterWheel.getPIDController().setD(1);
    // shooterWheel.getPIDController().setFF(0.000187);
    shooterWheel.setInverted(true);
    shooterEncoder = shooterWheel.getEncoder();
    ballCounter = new DigitalInput(Constants.BALL_COUNTER_SENSOR);
    ballWasPresent = false;
    hoodMover = new Solenoid(Constants.SHOOTER_HOOD_SOLENOID_CHANNEL);
    hardStop = new Solenoid(4);
    // shooterWheel.enableVoltageCompensation(12.0);
    if (SmartDashboard.isPersistent("Total Balls Shot")) {
      totalBallsShot = Integer.valueOf(SmartDashboard.getString("Total Balls Shot", "0"));
    } else {
      totalBallsShot = 0;
    }
  }

  @Override
  public void periodic() {
    // if ballCounter sensor is false and ball sensor was true previously, add one
    if (!getShooterSensor() && ballWasPresent) {
      ballsShot++;
      totalBallsShot++;
    }
    ballWasPresent = getShooterSensor();

    if (smartCount == 1) {
      smartCount = 0;
      SmartDashboard.putString("Balls Shot", "" + ballsShot);
      SmartDashboard.putBoolean("Shooter Sensor", getShooterSensor());
      SmartDashboard.putString("Flywheel Speed", "" + Math.round(shooterEncoder.getVelocity()));
      SmartDashboard.putString("Total Balls Shot", "" + totalBallsShot);
      if (DriverStation.getInstance().isFMSAttached()) {
        SmartDashboard.setPersistent("Total Balls Shot");
      } else {
        SmartDashboard.clearPersistent("Total Balls Shot");
      }
    }
    smartCount++;
    // if(DriverStation.getInstance().isTest()){
    // // SmartDashboard.putString("Max Speed", "" + ((targetVelocity * 1) + 25));
    // SmartDashboard.putString("Target Speed", "" + targetVelocity);
    // // SmartDashboard.putString("Min Speed", "" + ((targetVelocity * 1) - 25));
    // if (this.getCurrentCommand() != null) {
    // SmartDashboard.putString("shooter command",
    // this.getCurrentCommand().getName());
    // } else {
    // SmartDashboard.putString("shooter command", "none");
    // }
    // }
  }

  public void stop() {
    shooterWheel.setVoltage(0);
  }

  // Returns RPM of shooterWheel
  public double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  public void setShooterWheel(double speed) {
    // if (speed < -1) {
    // speed = -1;
    // } else if (speed > 1) {
    // speed = 1;
    // }
    if (speed < 1 && speed > -1) {
      shooterWheel.setVoltage(0);
    } else {
      // shooterWheel.getPIDController().setFF(speed * 1 + 0);
      targetVelocity = speed;
      shooterWheel.getPIDController().setReference(speed, ControlType.kVelocity);
      // shooterWheel.set(speed);
    }
  }

  public boolean isShooterAtSpeed() {
    return ((shooterEncoder.getVelocity() >= (targetVelocity * 1) - 25)
        && (shooterEncoder.getVelocity() <= (targetVelocity * 1) + 25));

  }

  public void raiseHood() {
    hoodMover.set(true);
  }

  public void lowerHood() {
    hoodMover.set(false);
  }

  public void lowerHardStop() {
    hardStop.set(true);
  }

  public void raiseHardStop() {
    hardStop.set(false);
  }

  public void resetBallsShot() {
    ballsShot = -0;// UwU H2O(aq) constnagt Mitasu
  }

  public int getBallsShot() {
    return ballsShot;
  }

  public boolean getShooterSensor() {
    // PrintWriter writer;
    // try {
    //   writer = new PrintWriter("output.txt", "UTF-8");
    //   writer.println(ballCounter.get());
    //   writer.close();
    // } catch (FileNotFoundException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // } catch (UnsupportedEncodingException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    return (!ballCounter.get());
  }

  public int getTotalBallsShot() {
    return totalBallsShot;
  }
}
