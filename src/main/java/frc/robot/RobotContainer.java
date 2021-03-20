/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.DriveAdjustModuleZeroPoint;
import frc.robot.commands.DriveAllModulesPositionOnly;
import frc.robot.commands.DriveFieldCentric;
import frc.robot.commands.DriveFieldCentricAdvanced;
import frc.robot.commands.DriveFieldCentricArc;
import frc.robot.commands.DriveFieldCentricVelocity;
import frc.robot.commands.DriveOneModule;
import frc.robot.commands.DriveResetAllModulePositionsToZero;
import frc.robot.commands.DriveRobotCentric;
import frc.robot.commands.DriveStopAllModules;
import frc.robot.commands.DriveStraightAtSpeed;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.DriveTurnToAngle;
import frc.robot.commands.RunPath;
import frc.robot.commands.DriveResetGyroToZero;

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


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive = new SwerveDrive();
    limelight = new Limelight();
    swerveDrive.setDefaultCommand(new DriveFieldCentricAdvanced());
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putData(new DriveResetAllModulePositionsToZero());
    SmartDashboard.putData(new DriveAdjustModuleZeroPoint());
    SmartDashboard.putData("Drive Module 0", new DriveOneModule(0));
    SmartDashboard.putData("Drive Module 1", new DriveOneModule(1));
    SmartDashboard.putData("Drive Module 2", new DriveOneModule(2));
    SmartDashboard.putData("Drive Module 3", new DriveOneModule(3));
    SmartDashboard.putData(new DriveStopAllModules());
    SmartDashboard.putData(new DriveAllModulesPositionOnly());
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
    driverB.whenPressed(new DriveFieldCentricArc(-1));
    driverX.whenPressed(new DriveFieldCentricArc(1));

    driverY.whenPressed(new RunPath("Straight"));
    
    driverRB.whenPressed(new DriveToPosition(new Pose2d(1, 0, new Rotation2d())));
    driverLB.whenPressed(new DriveResetGyroToZero());

    driverStart.whenPressed(new DriveFieldCentricAdvanced());
    driverBack.whenPressed(new DriveRobotCentric());
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
