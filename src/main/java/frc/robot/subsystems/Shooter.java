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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private TalonFX shooterWheel;
  private double targetVelocity;
  private DigitalInput ballCountSensor;
  private Counter ballCounter;
  private Solenoid hoodMoverUp, hoodMoverDown;

  private int totalBallsShot = 0;
  int smartCount = 0;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {

    shooterWheel = new TalonFX(Constants.SHOOTER_WHEEL);
    shooterWheel.configFactoryDefault();
    shooterWheel.setNeutralMode(NeutralMode.Coast);
    shooterWheel.config_kP(0,.5);//.setP(0.05);
    shooterWheel.config_kI(0,0.0);//.setI(0.0);
    shooterWheel.config_kD(0,1.5);//.setD(1.5);
    shooterWheel.config_kF(0,0.05);//.setFF(0.05);
    shooterWheel.enableVoltageCompensation(true);
    shooterWheel.configVoltageCompSaturation(12.5);
    //shooterWheel.configClosedloopRamp(2.0);
    SupplyCurrentLimitConfiguration currLimitCfg = new SupplyCurrentLimitConfiguration();
    currLimitCfg.currentLimit = 60;
    currLimitCfg.enable = true; 
    shooterWheel.configSupplyCurrentLimit(currLimitCfg);
    shooterWheel.setInverted(true);
    
    /**
     * The following creates a counter(like an encoder). The 
     * sensor in the shooter is still accessable, and for a 
     * direction control, we use a channel in the MXP that is 
     * unused.(as we only need to count up) With this we hope 
     * to overcome the failures in reading the sensor in the 
     * periodic loop, as this counter works on the FPGA level.
     */
    ballCountSensor = new DigitalInput(Constants.BALL_COUNTER_SENSOR);
    ballCounter = new Counter(Counter.Mode.kExternalDirection);
    ballCounter.setUpSource(ballCountSensor);
    ballCounter.setDownSource(15);//this is an unused channel, we need to say something, so we say an unconnected channel
    ballCounter.setDownSourceEdge(true,false);
    ballCounter.setUpSourceEdge(false, true);
    
    
    hoodMoverUp = new Solenoid(Constants.SHOOTER_HOOD_SOLENOID_CHANNEL_UP);
    hoodMoverDown = new Solenoid(Constants.SHOOTER_HOOD_SOLENOID_CHANNEL_DOWN);
    // shooterWheel.enableVoltageCompensation(12.0);
    if (SmartDashboard.isPersistent("Total Balls Shot")) {
      totalBallsShot = Integer.valueOf(SmartDashboard.getString("Total Balls Shot", "0"));
    } else {
      totalBallsShot = 0;
    }
    lowerHood();
  }

  @Override
  public void periodic() {

    if (smartCount == 5) {
      smartCount = 0;
      SmartDashboard.putString("Balls Shot", "" + getBallsShot());
      SmartDashboard.putBoolean("Shooter Sensor", getShooterSensor());
      SmartDashboard.putString("Flywheel Speed", "" + Math.round(getShooterVelocity()));
      SmartDashboard.putString("Total Balls Shot", "" + getTotalBallsShot() );
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
    // }
  }

  /**
   * stops the shooterWheel, this is motor is no 
   * in brake modde, so the wheel will coast for 
   * a little while.
   */
  public void stop() {
    shooterWheel.set(ControlMode.PercentOutput,0);
  }

  /**
   * Returns RPM of shooterWheel in natural units (ticks per ms)
   * @return 
   */ 
  public double getShooterVelocity() {
    return shooterWheel.getSelectedSensorVelocity();
  }

  /**
   * Demands wheel to go to a set velocity. If value 
   * is between -1 and 1 the speed will be set in 
   * PercentOutput mode, otherwise it is in Velocity 
   * mode.
   * 
   * @param speed speed in units per 100 ms
   */
  public void setShooterWheel(double speed) {
    
    if (speed < 1 && speed > -1) {
      shooterWheel.set(ControlMode.PercentOutput,0);
    } else {
      // shooterWheel.getPIDController().setFF(speed * 1 + 0);
      if(speed > 13000){
        shooterWheel.selectProfileSlot(0, 0);
      }
      
      targetVelocity = speed;
      shooterWheel.set(ControlMode.Velocity, targetVelocity);

    }
  }

  /**
   * checks to see if the shooterWheel has reached 
   * the demanded speed within a window of +/- 500 
   * natural units
   * 
   * @return true if wheel is at speed
   */
  public boolean isShooterAtSpeed() {
    SmartDashboard.putString("ShooterWheelSpeed", targetVelocity + "");
    return ((shooterWheel.getSelectedSensorVelocity() >= (targetVelocity * 1) - 500)
        && (shooterWheel.getSelectedSensorVelocity() <= (targetVelocity * 1) + 500));

  }

  /**
   * raises the hood by changing the 
   * solenoids' states
   */
  public void raiseHood() {
    hoodMoverDown.set(false);
    hoodMoverUp.set(true);
  }

  /**
   * lowers the hood by changing the 
   * solenoids' states
   */
  public void lowerHood() {
    hoodMoverDown.set(true);
    hoodMoverUp.set(false);
  }

  /**
   * The current state of the sensor in the shooter. 
   * This is good for seeing if a ball is stuck in 
   * the shooter.
   * 
   * @return is a ball in the shooter
   */
  public boolean getShooterSensor() {
    return (!ballCountSensor.get());
  }

  /**
   * The current count of balls that have gone 
   * through the shooter.
   * 
   * @return
   */
  public int getBallsShot() {
    return ballCounter.get();
  }

  /**
   * resets the number of balls shot, stores 
   * number of balls shot in totalBalls shot 
   * before hand.
   */
  public void resetBallsShot() {
    totalBallsShot+=ballCounter.get();
    ballCounter.reset();//resets the ball counter, as one would an encoder
    // UwU H2O(aq) constnagt Mitasu <-What is this? Explain!(ROB)
  }

  /**
   * gets the total number of balls fired 
   * since the robot has booted.
   * 
   * @return
   */
  public int getTotalBallsShot() {
    return totalBallsShot + ballCounter.get();
  }
}
