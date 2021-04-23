/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoBarrelPath;
import frc.robot.commands.AutoBouncePath;
import frc.robot.commands.AutoSlalomPath;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveDrive.kDriveMode;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  static private double ENCODER_EDGES_PER_REV = 16384 / 4.;
  static private int PIDIDX = 0;
  static private int ENCODER_EPR = 16384;
  static private double GEARING = 1;
  
  private double encoderConstant = (1 / GEARING) * (1 / ENCODER_EDGES_PER_REV);

  public static RobotContainer robotContainer;

  Joystick stick;
  // WPI_TalonSRX leaderMotor;


  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  String data = "";
  
  int counter = 0;
  double startTime = 0;
  double priorAutospeed = 0;

  double[] numberArray = new double[10];
  ArrayList<Double> entries = new ArrayList<Double>();
  public Robot() {
    super(.005);
    LiveWindow.disableAllTelemetry();
  }

  public enum Sides {
    LEFT,
    RIGHT,
    FOLLOWER
  }

  // methods to create and setup motors (reduce redundancy)
  public WPI_TalonSRX setupWPI_TalonSRX(int port, Sides side, boolean inverted) {
    // create new motor and set neutral modes (if needed)
    WPI_TalonSRX motor = new WPI_TalonSRX(port);
    // setup talon
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(inverted);
    
    // setup encoder if motor isn't a follower
    if (side != Sides.FOLLOWER) {
    
      
      motor.configSelectedFeedbackSensor(
            FeedbackDevice.QuadEncoder,
            PIDIDX, 10
      );    



    switch (side) {
      // setup encoder and data collecting methods

      case LEFT:
        motor.setSensorPhase(false);
        
        leftEncoderPosition = ()
          -> motor.getSelectedSensorPosition(PIDIDX) * encoderConstant;
        leftEncoderRate = ()
          -> motor.getSelectedSensorVelocity(PIDIDX) * encoderConstant *
               10;
        

        break;
      default:
        // probably do nothing
        break;

      }
    
    }
    

    return motor;

  }

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();

    // stick = new Joystick(0);
    
    // create left motor
    // WPI_TalonSRX leftMotor = setupWPI_TalonSRX(0, Sides.LEFT, true);


    rightEncoderPosition = leftEncoderPosition;
    rightEncoderRate = leftEncoderRate;
    // leaderMotor = leftMotor;

    //
    // Configure gyro
    //

    // Note that the angle from the NavX and all implementors of WPILib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    gyroAngleRadians = () -> 0.0;

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  @Override
  public void disabledInit() {
    RobotContainer.swerveDrive.stopAllModules();
    RobotContainer.swerveDrive.setEnableLimitedOutput(true);
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    System.out.println("Robot disabled");
    // leaderMotor.set(0);
    // data processing step
    data = entries.toString();
    data = data.substring(1, data.length() - 1) + ", ";
    telemetryEntry.setString(data);
    entries.clear();
    System.out.println("Robot disabled");
    System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
    data = "";
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    // feedback for users, but not used by the control program
    SmartDashboard.putNumber("l_encoder_pos", 0.0);
    SmartDashboard.putNumber("l_encoder_rate", 0.0);
    SmartDashboard.putNumber("r_encoder_pos", 0.0);
    SmartDashboard.putNumber("r_encoder_rate", 0.0);
  }

  @Override
  public void teleopInit() {
    System.out.println("Robot in operator control mode");
  }

  @Override
  public void teleopPeriodic() {
    // leaderMotor.set(-stick.getY());
  }

  @Override
  public void autonomousInit() {
    RobotContainer.swerveDrive.setEnableLimitedOutput(false);
    RobotContainer.swerveDrive.resetAllModuleDistances();
    System.out.println("Robot in autonomous mode");
    startTime = Timer.getFPGATimestamp();
    counter = 0;
  }

  /**
  * If you wish to just use your own robot program to use with the data logging
  * program, you only need to copy/paste the logic below into your code and
  * ensure it gets called periodically in autonomous mode
  * 
  * Additionally, you need to set NetworkTables update rate to 10ms using the
  * setUpdateRate call.
  */
  @Override
  public void autonomousPeriodic() {

    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double leftPosition = 0;
    double[] allPos = RobotContainer.swerveDrive.getAllModuleDistance();
    for(int i=0;i<4;i++){
      leftPosition += allPos[i];
    }
    leftPosition /= 4 ;

    double leftRate = 0;
    double[] allRate = RobotContainer.swerveDrive.getAllModuleVelocity();
    for(int i=0; i <4; i++){
      leftRate += allRate[i];
    }
    leftRate /= 4;

    double rightPosition = 0;
    double rightRate = 0;

    double battery = RobotController.getBatteryVoltage();
    double motorVolts = battery * Math.abs(priorAutospeed);

    double leftMotorVolts = motorVolts;
    double rightMotorVolts = motorVolts;

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    // leaderMotor.set(autospeed);
    RobotContainer.swerveDrive.driveRobotCentric(autospeed, 0, 0, kDriveMode.percentOutput);

    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = 0.0;

    // Add data to a string that is uploaded to NT
    for (double num : numberArray) {
      entries.add(num);
    }
    counter++;
  }
}
