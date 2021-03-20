/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * This class is for ball handling with 5 CANSparkMax objects with limit
 * switches plugged into the SparkMax
 */
public class SnekLoader extends SubsystemBase {
  // private CANSparkMax axleWheels;
  // private CANEncoder axleEncoder;
  private boolean harvesterJammed = false; 
  

  private static CANSparkMax[] handleMotors;
  private static CANDigitalInput[] handleSensors = new CANDigitalInput[5];
  private static CANEncoder[] handleEncoders = new CANEncoder[5];
  // If it is deemed necessary, uncomment all of ballsLoaded stuff
  private int ballsLoaded;
  private boolean isPaused;
  // private static boolean hadBall;
  double[] speeds = new double[5];
  int smartCount = 0;

  public enum State {
    kFillTo4, kFillTo3, kFillTo2, kFillTo1, kFillTo0, kOff, kShootBall4, kShootBall3, kShootBall2, kShootBall1,
    kShootBall0, kSpitBalls
  }

  private State state = State.kOff;

  static final double MOTOR_IN_SPEED0 = 0.5;
  static final double MOTOR_IN_SPEED1 = 0.4;
  static final double MOTOR_IN_SPEED2 = 0.5;
  static final double MOTOR_IN_SPEED3 = 0.6;
  static final double MOTOR_IN_SPEED4 = 0.35;

  public SnekLoader() {
    // axleWheels = new CANSparkMax(Constants.BALL_HANDLER_MOTOR_0, MotorType.kBrushless);
    // axleWheels.setSmartCurrentLimit(45, 60);
    // axleEncoder = axleWheels.getEncoder();

    // ballSensors = axleWheels.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    isPaused = false;
    ballsLoaded = 0;
    // hadBall = false;
    handleMotors = new CANSparkMax[] {
        
        new CANSparkMax(Constants.BALL_HANDLER_MOTOR_0, MotorType.kBrushless),//axleWheels,
        new CANSparkMax(Constants.BALL_HANDLER_MOTOR_1, MotorType.kBrushless),
        new CANSparkMax(Constants.BALL_HANDLER_MOTOR_2, MotorType.kBrushless),
        new CANSparkMax(Constants.BALL_HANDLER_MOTOR_3, MotorType.kBrushless),
        new CANSparkMax(Constants.BALL_HANDLER_MOTOR_4, MotorType.kBrushless) };

    // handleSensors
    // sets default setup for motors
    for (int i = 0; i <= 4; i++) {
      handleMotors[i].restoreFactoryDefaults();
      handleMotors[i].setIdleMode(IdleMode.kBrake);// set brake mode, so motors stop on a dime
      handleMotors[i].enableVoltageCompensation(12.0);// enable volatge compensation mode 12V
      handleMotors[i].setInverted(false);//Reverses the direction of the wheels
      // handleMotors[i].setSmartCurrentLimit(40, 40);
      handleSensors[i] = handleMotors[i].getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      handleSensors[i].enableLimitSwitch(false);// now disable limit switches, we'll turn these on later, one at a tim
      handleEncoders[i] = handleMotors[i].getEncoder();
      handleMotors[i].setInverted(true);
    }
    handleMotors[4].setInverted(false);
  }

  // public void setAxleWheels(double volts) {
  //   axleWheels.setVoltage(volts*-1); //inverted for new robot
  // }

  public boolean isHarvesterJammed() {
    boolean isJammed = false;
    //   isJammed =
    //     ((axleWheels.get() != 0) && (axleEncoder.getVelocity() == 0))||
    //     (axleWheels.getOutputCurrent() > 40.0 && axleEncoder.getVelocity() < 1500);
    return isJammed;
  }

  public void setHarvesterJammed(boolean jam){
    harvesterJammed = jam;
  }
  public boolean stopIntakeQ(){
      return harvesterJammed;
  }

  /**
   * @return the ballsLoaded
   */
  public int getBallsLoaded() {
    return ballsLoaded;
  }

  public void setPause(boolean pause){
    isPaused = pause;
  }

  public boolean getPause(){
    return isPaused;
  }

  @Override
  public void periodic() {

    
  
    // if (!hadBall && handleSensors[0].get()) {
    // ballsLoaded++;
    // }


    // This method will be called once per scheduler run
    switch (state) {
      case kSpitBalls:
        speeds = new double[] { -1.0, -.85, -.7, -.6, -.5 };
        ballsLoaded = 0;
        break;
      case kFillTo4:
        enableOneLimit(4);
        speeds = new double[] { MOTOR_IN_SPEED0, MOTOR_IN_SPEED1, MOTOR_IN_SPEED2, MOTOR_IN_SPEED3, MOTOR_IN_SPEED4 };
        if (getHandleSensor(4)) {
          state = State.kFillTo3;
        } else {
          ballsLoaded = 0;
          break;
        }
      case kFillTo3:
        speeds = new double[] { MOTOR_IN_SPEED0, MOTOR_IN_SPEED1, MOTOR_IN_SPEED2, MOTOR_IN_SPEED4, 0.0 };
        enableOneLimit(3);
        if (getHandleSensor(3)) {
          state = State.kFillTo2;
        } else {
          ballsLoaded = 1;
          break;
        }
      case kFillTo2:
        speeds = new double[] { MOTOR_IN_SPEED0, MOTOR_IN_SPEED1, MOTOR_IN_SPEED4, 0.0, 0.0 };
        enableOneLimit(2);
        if (getHandleSensor(2)) {
          state = State.kFillTo1;
        } else {
          ballsLoaded = 2;
          break;
        }
      case kFillTo1:
        speeds = new double[] { MOTOR_IN_SPEED0, MOTOR_IN_SPEED4, 0.0, 0.0, 0.0 };
        enableOneLimit(1);
        if (getHandleSensor(1)) {
          state = State.kFillTo0;
        } else {
          ballsLoaded = 3;
          break;
        }
      case kFillTo0:
        speeds = new double[] { MOTOR_IN_SPEED0, 0.0, 0.0, 0.0, 0.0 };
        enableOneLimit(0);
        ballsLoaded = -1;
        if (getHandleSensor(0)) {
          ballsLoaded = 5;
          state = State.kOff;
        } else {
          ballsLoaded = 4;
          break;
        }
  
      case kOff:
        speeds = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0 };
        enableOneLimit(-1);
        break;
  
      case kShootBall4:
        speeds = new double[] { 0.0, 0.0, 0.0, 0.0, 1.0 };
        enableOneLimit(-1);
        break;
  
      case kShootBall3:
        speeds = new double[] { 0.0, 0.0, 0.0, .9, 1.0 };
        enableOneLimit(-1);
        break;
  
      case kShootBall2:
        speeds = new double[] { 0.0, 0.0, .7, .85, 1.0 };
        enableOneLimit(-1);
        break;
  
      case kShootBall1:
        speeds = new double[] { 0.0, .5, .7, .85, 1.0 };
        enableOneLimit(-1);
        break;
      case kShootBall0:
        speeds = new double[] { .40, .65, .85, 1.0, 1.0 }; //.3,.5,.75,1.0,1.0
        enableOneLimit(-1);
        break;
      default:
          speeds = new double[] {0.0,0.0,0.0,0.0,0.0};
      }
    if(isPaused){
      speeds = new double[] {  0.0, 0.0, 0.0, 0.0 , 0.0};
    }
    setAllHandleMotors(speeds);

    //SmartDashboard Pushes
    if(smartCount == 50){
      smartCount = 0;
      if (isJammed() && getState() != State.kSpitBalls) {
        SmartDashboard.putBoolean("isJammed", true);
      } else {
        SmartDashboard.putBoolean("isJammed", false);
      }
      harvesterJammed = false;
      if(this.getCurrentCommand() != null){
        // SmartDashboard.putString("harvester speed", "" + handleEncoders[0].getVelocity());
        if (isHarvesterJammed() ) {//&& (this.getCurrentCommand().getName().equals("Load"))
          SmartDashboard.putBoolean("isHarvesterJammed", true);
          harvesterJammed = true;
        } else {
          SmartDashboard.putBoolean("isHarvesterJammed", false);
          harvesterJammed = false;
        }
      }
      SmartDashboard.putBoolean("Ball 0", handleSensors[0].get());
      SmartDashboard.putBoolean("Ball 1", handleSensors[1].get());
      SmartDashboard.putBoolean("Ball 2", handleSensors[2].get());
      SmartDashboard.putBoolean("Ball 3", handleSensors[3].get());
      SmartDashboard.putBoolean("Ball 4", handleSensors[4].get());
      SmartDashboard.putString("BallsLoaded", ""+ ballsLoaded);
    }
    smartCount++;
    // if(DriverStation.getInstance().isTest()){
    //   SmartDashboard.putString("State", state.name());
    //   if(this.getCurrentCommand() != null){
    //     SmartDashboard.putString("snek command", this.getCurrentCommand().getName());
    //   } else {
    //     SmartDashboard.putString("snek command", "none");
    //   }
    // }

  }

  /**
   * sets the state of the ballHandler periodic function
   * 
   * @param state enumeration State
   */
  public void setState(State state) {
    this.state = state;
  }

  /**
   * returns current state
   * 
   * @return current state
   */
  public State getState() {
    return state;
  }

  /**
   * returns boolean of limit switch
   * 
   * @param sensor
   * @return boolean
   */
  public boolean getHandleSensor(int sensor) {
    // PrintWriter writer;
    // try {
    //   writer = new PrintWriter("output.txt", "UTF-8");
    //   writer.println(handleSensors[sensor].get());
    //   writer.close();
    // } catch (FileNotFoundException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // } catch (UnsupportedEncodingException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    return handleSensors[sensor].get();
  }

  /**
   * sets one limit switch on while setting the rest off
   * 
   * @param sensorOn the sensor you want on, -1 if you want all to be off
   */
  private void enableOneLimit(int sensorOn) {
    for (int i = 0; i <= 4; i++) {
      handleSensors[i].enableLimitSwitch(i == sensorOn);
    }
  }

  /**
   * sets all of the handler motors to speeds
   * 
   * @param speeds an array of numbers
   */
  private void setAllHandleMotors(double[] speeds) {
    for (int i = 0; i <= 4; i++) {
        handleMotors[i].set(speeds[i]);
    }
  }

  /**
   * detects if a motor is being sent power, but isn't going
   * 
   * @return if a jam is in progress
   */
  public boolean isJammed() {
    boolean isJammed = false;
    // for (int i = 0; i <= 4; i++) {
    //   isJammed = isJammed || 
    //     ((handleMotors[i].get() != 0) && (handleEncoders[i].getVelocity() == 0))||
    //     handleMotors[i].getOutputCurrent() > 40.0;
    // }
    return isJammed;
  }
}
