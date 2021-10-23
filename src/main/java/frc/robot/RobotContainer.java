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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import frc.robot.commands.auto.AutoColorWheelStealThenShoot;
import frc.robot.commands.auto.AutoForwardThenWallShot;
import frc.robot.commands.auto.AutoShootAndMove;
import frc.robot.commands.auto.AutoShootThen3TrenchThenShoot;
import frc.robot.commands.auto.AutoTrench2BallShoot3TrenchShoot;
import frc.robot.commands.Drive.autoFunc.DriveFollowPath;
import frc.robot.commands.Drive.autoFunc.DriveStraightAtSpeed;
import frc.robot.commands.Drive.autoFunc.DriveStraightTrapProfile;
import frc.robot.commands.Drive.autoFunc.DriveStraightTrapProfile2;
import frc.robot.commands.Drive.autoFunc.DriveStraightenAllModules;
import frc.robot.commands.Drive.autoFunc.DriveToPosition;
import frc.robot.commands.Drive.autoFunc.DriveTurnToAngle;
import frc.robot.commands.Drive.autoFunc.DriveTurnToTarget;
import frc.robot.commands.Drive.autoFunc.RunPath;
import frc.robot.commands.Drive.autoFunc.DriveArc;
import frc.robot.commands.Drive.util.DriveAdjustModuleZeroPoint;
import frc.robot.commands.Drive.util.DriveAllModulesPositionOnly;
import frc.robot.commands.Drive.util.DriveFindMaxAccel;
import frc.robot.commands.Drive.util.DriveGenerateVelocityGraph;
import frc.robot.commands.Drive.util.DriveOneModule;
import frc.robot.commands.Drive.util.DriveResetAllModulePositionsToZero;
import frc.robot.commands.Drive.util.DriveVelocityPIDTune;
import frc.robot.commands.Drive.util.DriveResetGyroToZero;
import frc.robot.commands.Drive.DriveFieldCentric;
import frc.robot.commands.Drive.DriveFieldCentricAdvanced;
import frc.robot.commands.Drive.DriveArcDriverControl;
import frc.robot.commands.Drive.DriveFieldCentricVelocity;
import frc.robot.commands.Drive.DriveOnTargetWithLimeLight;
import frc.robot.commands.Drive.autoFunc.DrivePathWeaverProfile;
import frc.robot.commands.Drive.DriveRobotCentric;
import frc.robot.commands.Drive.DriveStopAllModules;
import frc.robot.commands.Shooter.ShootOneBallDemo;
import frc.robot.commands.Shooter.FullWallShot;
import frc.robot.commands.Shooter.PrepHoodShot;
import frc.robot.commands.Shooter.PrepWallShot;
import frc.robot.commands.Shooter.ProgTBallWithHintOfLime;
import frc.robot.commands.Shooter.ResetBallsShot;
import frc.robot.commands.Shooter.SpinUpShooterWheel;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.SnekLoader.Load;
import frc.robot.commands.SnekLoader.Regurgitate;
import frc.robot.commands.ClimberCoDriverFunction;
import frc.robot.commands.GetSmol;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SnekLoader;

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
  // final Button driverDUp = new DPad(driver, DPad.Direction.UP);//Commented out because it is used in DriveFeildCentricAdvanced
  final Button driverDDown = new DPad(driver, DPad.Direction.DOWN);
  final Button driverDLeft = new DPad(driver, DPad.Direction.LEFT);
  // final Button driverDRight = new DPad(driver, DPad.Direction.RIGHT);//Commented out because it is used in DriveFeildCentricAdvanced
  
  final Button coDriverA = new JoystickButton(coDriver, 1);
  final Button coDriverB = new JoystickButton(coDriver, 2);
  final Button coDriverX = new JoystickButton(coDriver, 3);
  final Button coDriverY = new JoystickButton(coDriver, 4);
  final Button coDriverLB = new JoystickButton(coDriver, 5);
  final Button coDriverRB = new JoystickButton(coDriver, 6);
  final Button coDriverBack = new JoystickButton(coDriver, 7);
  final Button coDriverStart = new JoystickButton(coDriver, 8);
  final Button coDriverLS = new JoystickButton(coDriver, 9);
  final Button coDriverRS = new JoystickButton(coDriver, 10);
  final Button coDriverDDown = new DPad(coDriver, DPad.Direction.DOWN);
  
  //The robot's subsystems are instantiated here
  public static SwerveDrive swerveDrive;
  public static Climber climber;
  public static Limelight limelight;
  public static SnekLoader snekLoader;
  public static Harvester harvester;
  public static Shooter shooter;
  
  //The sendable choser form autonomous is constructed here
  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    //create(construct) subsystems
    shooter = new Shooter();
    harvester = new Harvester();
    snekLoader = new SnekLoader();
    limelight = new Limelight();
    limelight.setStreamMode(0);
    limelight.setLightState(1);
    climber = new Climber();
    swerveDrive = new SwerveDrive();
    // SmartDashboard.putData("Harvester", snekLoader);
    swerveDrive.setDefaultCommand(new DriveFieldCentricAdvanced());

    // Configure the button bindings
    configureButtonBindings();

    //Add all autos to the auto selector
    configureAutoModes();

    //add some commands to dashboard for testing/configuring
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
    SmartDashboard.putData(new DrivePathWeaverProfile("Straight"));//,Math.PI/2));
    SmartDashboard.putData(new DriveTurnToTarget());
    SmartDashboard.putData(new ResetBallsShot());
    SmartDashboard.putData(new DriveStraightenAllModules());
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
    
    //==========  DRIVER  ==========
    driverA.whenPressed(new Load());
    // driverA.whenPressed(new LoadAcc());
    driverA.whenReleased(new GetSmol());
    driverB.whileHeld(new Regurgitate());
    // driverX.whenPressed(new SmartLimeShot());
    driverX.whenPressed(new ProgTBallWithHintOfLime(0.1));
    driverX.whenReleased(new GetSmol());
    driverY.whenPressed(new FullWallShot());
    driverY.whenReleased(new GetSmol());
    
    driverLB.whenPressed(new DriveResetGyroToZero());
    driverRB.whileHeld(new DriveOnTargetWithLimeLight());
    
    driverBack.whenPressed(new DriveRobotCentric());
    driverStart.whenPressed(new DriveFieldCentricAdvanced());



    //========== CODRIVER ==========
    coDriverA.whenReleased(new GetSmol());
    coDriverB.whenPressed(new SpinUpShooterWheel());
    coDriverX.whenPressed(new PrepHoodShot().withTimeout(1.5));
    coDriverY.whenPressed(new PrepWallShot().withTimeout(1.5));
    coDriverBack.whenPressed(new StopShoot());
    coDriverDDown.toggleWhenPressed(new ClimberCoDriverFunction());
    
  }
  
  /**
   * Define all autonomous modes here to have them 
   * appear in the autonomous select drop down menu.
   */
  private void configureAutoModes() {
    
    autoChooser.setDefaultOption("Wait 1 sec(do nothing)", new WaitCommand(1));

    // autoChooser.addOption("Barrel Racing 64", new AutoBarrelPath());

    // autoChooser.addOption("Bouncy Path", new AutoBouncePath());

    // autoChooser.addOption("Slalom Path", new AutoSlalomPath());

    autoChooser.addOption("Shoot and Move", new AutoShootAndMove());

    autoChooser.addOption("Forward Then WallShot", new AutoForwardThenWallShot());
    
    autoChooser.addOption("Color Wheel Steal", new AutoColorWheelStealThenShoot());

    autoChooser.addOption("Shoot Then 3 Trench Run", new AutoShootThen3TrenchThenShoot());

    // autoChooser.addOption("Trench Run, Shoot, More Trench", new AutoTrench2BallShoot3TrenchShoot());

    SmartDashboard.putData(RobotContainer.autoChooser);

  }

  /**
   * Defines the axis of the Xbox gamepad.
   */
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
   * A method to return the value of a driver joystick axis,
   * which runs from -1.0 to 1.0, with a .1 dead zone(a 0 
   * value returned if the joystick value is between -.1 and 
   * .1)
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

  /**
   * A method to return the value of a codriver joystick axis,
   * which runs from -1.0 to 1.0, with a .1 dead zone(a 0 
   * value returned if the joystick value is between -.1 and 
   * .1) 
   * @param axis
   * @return
   */
  public double getCoDriverAxis(Axis axis) {
    return (coDriver.getRawAxis(axis.getAxisNumber()) < -.1 || coDriver.getRawAxis(axis.getAxisNumber()) > .1)
        ? coDriver.getRawAxis(axis.getAxisNumber())
        : 0;
  }

  /**
   * Accessor method to set codriver rumble function
   * 
   * @param leftRumble
   * @param rightRumble
   */
  public void setCoDriverRumble(double leftRumble, double rightRumble) {
    coDriver.setRumble(RumbleType.kLeftRumble, leftRumble);
    coDriver.setRumble(RumbleType.kRightRumble, rightRumble);
  }

  /**
   * accessor to get the true/false of the buttonNum 
   * on the coDriver control
   * @param buttonNum
   * @return the value of the button
   */
  public boolean getCoDriverButton(int buttonNum) {
    return coDriver.getRawButton(buttonNum);
  }

}
