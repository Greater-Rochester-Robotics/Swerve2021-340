/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.auto.AutoBarrelPath;
import frc.robot.commands.auto.AutoBouncePath;
import frc.robot.commands.auto.AutoSlalomPath;
import frc.robot.commands.Drive.autoFunc.DriveFollowPath;
import frc.robot.commands.Drive.autoFunc.DriveStraightAtSpeed;
import frc.robot.commands.Drive.autoFunc.DriveStraightTrapProfile;
import frc.robot.commands.Drive.autoFunc.DriveStraightTrapProfile2;
import frc.robot.commands.Drive.autoFunc.DriveToPosition;
import frc.robot.commands.Drive.autoFunc.DriveTurnToAngle;
import frc.robot.commands.Drive.autoFunc.RunPath;
import frc.robot.commands.Drive.util.DriveAdjustModuleZeroPoint;
import frc.robot.commands.Drive.util.DriveAllModulesPositionOnly;
import frc.robot.commands.Drive.util.DriveFindMaxAccel;
import frc.robot.commands.Drive.util.DriveGenerateVelocityGraph;
import frc.robot.commands.Drive.util.DriveOneModule;
import frc.robot.commands.Drive.util.DriveResetAllModulePositionsToZero;
import frc.robot.commands.Drive.util.DriveVelocityPIDTune;
import frc.robot.commands.Drive.util.DriveResetGyroToZero;
import frc.robot.commands.Drive.autoFunc.DriveArc;
import frc.robot.commands.Drive.DriveFieldCentric;
import frc.robot.commands.Drive.DriveFieldCentricAdvanced;
import frc.robot.commands.Drive.DriveArcDriverControl;
import frc.robot.commands.Drive.DriveFieldCentricVelocity;
import frc.robot.commands.Drive.autoFunc.DrivePathWeaverProfile;
import frc.robot.commands.Drive.DriveRobotCentric;
import frc.robot.commands.Drive.DriveStopAllModules;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's joysticks are defined here...
  
  final Joystick driver = new Joystick(0);
  final Joystick coDriver = new Joystick(1);

  final Button driverA = new JoystickButton(driver, 1);
  final Button driverB = new JoystickButton(driver, 2);
  final Button driverX = new JoystickButton(driver, 3);
  final Button driverY = new JoystickButton(driver, 4);
  final Button driverLB = new JoystickButton(driver, 5);
  final Button driverRB = new JoystickButton(driver, 6);
  final Button driverBack = new JoystickButton(driver, 7);
  final Button driverStart = new JoystickButton(driver, 8);
  final Button driverLS = new JoystickButton(driver, 9);
  final Button driverRS = new JoystickButton(driver, 10);
  
  //The robot's subsystems are instantiated here
  public static SwerveDrive swerveDrive;
  public static Limelight limelight;

  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive = new SwerveDrive();
    limelight = new Limelight();
    swerveDrive.setDefaultCommand(new DriveFieldCentricAdvanced());
    // Configure the button bindings
    // configureButtonBindings();
    configureAutoModes();
    SmartDashboard.putData(new DriveResetAllModulePositionsToZero());
    SmartDashboard.putData(new DriveAdjustModuleZeroPoint());
    SmartDashboard.putData("Drive Module 0", new DriveOneModule(0));
    SmartDashboard.putData("Drive Module 1", new DriveOneModule(1));
    SmartDashboard.putData("Drive Module 2", new DriveOneModule(2));
    SmartDashboard.putData("Drive Module 3", new DriveOneModule(3));
    SmartDashboard.putData(new DriveStopAllModules());
    SmartDashboard.putData(new DriveAllModulesPositionOnly());
    SmartDashboard.putData(new DriveVelocityPIDTune());
    SmartDashboard.putData(new DriveStraightTrapProfile2(0, 1.0, 0, 0));
    SmartDashboard.putData(new DriveGenerateVelocityGraph());
    SmartDashboard.putData(new DrivePathWeaverProfile("TestArc"));
    // SmartDashboard.putData(new AutoBouncePath());
    // SmartDashboard.putData(new AutoSlalomPath());
    // SmartDashboard.putData(new AutoBarrelPath());
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // driverA.whenPressed(new DriveTurnToAngle(0));
    // driverB.whenPressed(new DriveTurnToAngle(Math.PI/2));
    // driverX.whenPressed(new DriveTurnToAngle(-Math.PI/2));
    // driverB.whileHeld(new DriveArcDriverControl(-.76,999));
    // driverX.whileHeld(new DriveArcDriverControl(.76,999));
    // driverA.whenPressed(new DriveArc(.3,Math.toRadians(-90),.76,Math.toRadians(-45)));
    // driverY.whenPressed(new DriveArc(.3,Math.toRadians(90),.76,Math.toRadians(90)));

    driverA.whenPressed(new DriveStraightTrapProfile(Math.toRadians(-90),0.5,0.0,0.0));
    driverB.whenPressed(new DriveArc(1.9*1.4, Math.toRadians(90), .762, Math.toRadians(90)));
    // driverY.whenPressed(new RunPath("Straight"));
    
    driverRB.whenPressed(new DriveToPosition(new Pose2d(1, 0, new Rotation2d())));
    driverLB.whenPressed(new DriveResetGyroToZero());

    driverStart.whenPressed(new DriveFieldCentricAdvanced());
    driverBack.whenPressed(new DriveRobotCentric());
  }
  
  private void configureAutoModes() {
    
    autoChooser.setDefaultOption("Wait 1 sec(do nothing)", new WaitCommand(1));

    autoChooser.addOption("Barrel Racing 64", new AutoBarrelPath());

    autoChooser.addOption("Bouncy Path", new AutoBouncePath());

    autoChooser.addOption("Slalom Path", new AutoSlalomPath());

    SmartDashboard.putData(RobotContainer.autoChooser);

  }

  public enum Axis {
    LEFT_X(0), LEFT_Y(1), LEFT_TRIGGER(2), RIGHT_TRIGGER(3), RIGHT_X(4), RIGHT_Y(5);

    private int axis;

    private Axis(int axis) {
      this.axis = axis;
    }

    public int getAxisNumber() {
      return axis;
    }
  }

  /**
   * A method to return the value of a joystick axis, which
   * runs from -1.0 to 1.0, with a .1 dead zone(a 0 value 
   * returned if the joystick value is between -.1 and .1)
   * @param axis
   * @return value of the joystick, from -1.0 to 1.0 where 0.0 is centered
   */
  public double getDriverAxis(Axis axis) {
    return (driver.getRawAxis(axis.getAxisNumber()) < -.1 || driver.getRawAxis(axis.getAxisNumber()) > .1)
        ? driver.getRawAxis(axis.getAxisNumber())
        : 0;
  }

  /**
   * Returns the int position of the DPad/POVhat based
   * on the following table:
   *    input    |return
   * not pressed |  -1
   *     up      |   0
   *   up right  |  45
   *    right    |  90
   *  down right | 135
   *    down     | 180
   *  down left  | 225
   *    left     | 270
   *   up left   | 315
   * @return
   */
  public int getDriverDPad() {
    return (driver.getPOV());
  }


}
