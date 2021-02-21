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
import frc.robot.commands.DriveAdjustModuleZeroPoint;
import frc.robot.commands.DriveOneModule;
import frc.robot.commands.DriveResetAllModulePositionsToZero;
import frc.robot.commands.DriveRobotCentric;
import frc.robot.commands.DriveStopAllModules;
import frc.robot.commands.DriveStraightAtSpeed;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
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
  

  public static SwerveDrive swerveDrive;

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive = new SwerveDrive();
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverA.whenPressed(new DriveOneModule(0));
    driverB.whenPressed(new DriveOneModule(1));
    driverX.whenPressed(new DriveOneModule(2));
    driverY.whenPressed(new DriveOneModule(3));
    
    driverA.whenReleased(new DriveStopAllModules());
    driverB.whenReleased(new DriveStopAllModules());
    driverX.whenReleased(new DriveStopAllModules());
    driverY.whenReleased(new DriveStopAllModules());
    
    driverLB.whenPressed(new DriveAdjustModuleZeroPoint());
    driverRB.whenPressed(new DriveResetAllModulePositionsToZero());

    driverStart.whenPressed(new DriveStraightAtSpeed(0.5, Math.toRadians(45), SwerveDrive.kDriveMode.percentOutput));
    driverStart.whenReleased(new DriveStopAllModules());
    driverBack.whenPressed(new DriveRobotCentric());
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
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
   * 
   * @param axis
   * @return
   */
  public double getDriverAxis(Axis axis) {
    return (driver.getRawAxis(axis.getAxisNumber()) < -.1 || driver.getRawAxis(axis.getAxisNumber()) > .1)
        ? driver.getRawAxis(axis.getAxisNumber())
        : 0;
  }

  public int getDriverDPad() {
    return (driver.getPOV());
  }
}
