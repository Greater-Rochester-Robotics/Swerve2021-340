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
import frc.robot.commands.AutoBarrelPath;
import frc.robot.commands.AutoBouncePath;
import frc.robot.commands.AutoSlalomPath;
import frc.robot.commands.DriveAdjustModuleZeroPoint;
import frc.robot.commands.DriveAllModulesPositionOnly;
import frc.robot.commands.DriveArc;
import frc.robot.commands.DriveFieldCentric;
import frc.robot.commands.DriveFieldCentricAdvanced;
import frc.robot.commands.DriveArcDriverControl;
import frc.robot.commands.DriveFieldCentricVelocity;
import frc.robot.commands.DriveFindMaxAccel;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveGenerateVelocityGraph;
import frc.robot.commands.DriveOnTargetWithLimeLight;
import frc.robot.commands.DriveOneModule;
import frc.robot.commands.DrivePathWeaverProfile;
import frc.robot.commands.DriveResetAllModulePositionsToZero;
import frc.robot.commands.DriveRobotCentric;
import frc.robot.commands.DriveStopAllModules;
import frc.robot.commands.DriveStraightAtSpeed;
import frc.robot.commands.DriveStraightTrapProfile;
import frc.robot.commands.DriveStraightTrapProfile2;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.DriveTurnToAngle;
import frc.robot.commands.DriveVelocityPIDTune;
import frc.robot.commands.RunPath;
import frc.robot.commands.DriveResetGyroToZero;
import frc.robot.commands.GetSmol;
import frc.robot.commands.Harvester.PickHarvesterUp;
import frc.robot.commands.Harvester.SetHarvesterDown;
import frc.robot.commands.Shooter.DriveAimAndPrepHood;
import frc.robot.commands.Shooter.FastBallWithHintOfLime;
import frc.robot.commands.Shooter.FullSendsWall;
import frc.robot.commands.Shooter.PrepHoodShot;
import frc.robot.commands.Shooter.PrepWallShot;
import frc.robot.commands.Shooter.ShootWithLimelight;
import frc.robot.commands.Shooter.SmartLimeShot;
import frc.robot.commands.Shooter.SpinUpShooterWheel;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Shooter.WallShot;
import frc.robot.commands.SnekLoader.Load;
import frc.robot.commands.SnekLoader.LoadAcc;
import frc.robot.commands.SnekLoader.Regurgitate;
import frc.robot.commands.SnekLoader.StopSnek;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Limelight;
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
  final Button driverDUp = new DPad(driver, DPad.Direction.UP);
  final Button driverDDown = new DPad(driver, DPad.Direction.DOWN);
  final Button driverDLeft = new DPad(driver, DPad.Direction.LEFT);
  final Button driverDRight = new DPad(driver, DPad.Direction.RIGHT);
  
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
  
  //The robot's subsystems are instantiated here
  public static SwerveDrive swerveDrive;
  public static Limelight limelight;
  public static SnekLoader snekLoader;
  public static Harvester harvester;
  public static Shooter shooter;
  

  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    //create subsystems
    shooter = new Shooter();
    harvester = new Harvester();
    snekLoader = new SnekLoader();
    limelight = new Limelight();
    limelight.setStreamMode(0);
    limelight.setLightState(1);
    swerveDrive = new SwerveDrive();
    SmartDashboard.putData("Harvester", snekLoader);
    swerveDrive.setDefaultCommand(new DriveFieldCentricAdvanced());

    // Configure the button bindings
    configureButtonBindings();

    //Add all autos to the auto selector
    configureAutoModes();

    //add some cmmands to dashboard for testing
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
    // driverX.whenPressed(new DriveArcDriverControl(.822,Math.toRadians(270)));
    // driverB.whenPressed(new DriveArcDriverControl(-.822, Math.toRadians(360)));
    // driverA.whenPressed(new SetHarvesterDown());
    // driverY.whenPressed(new PickHarvesterUp());
    
    //driverA.whenPressed(new Load());
    driverA.whenPressed(new LoadAcc());
    driverA.whenReleased(new GetSmol());
    // driverA.whenPressed(new SetHarvesterDown());
    // driverA.whenReleased(new PickHarvesterUp());
    driverB.whileHeld(new Regurgitate());
    //driverX.whenPressed(new SmartLimeShot());
    driverX.whenPressed(new FastBallWithHintOfLime());
    driverX.whenReleased(new GetSmol());
    driverRB.whileHeld(new DriveAimAndPrepHood());
    driverY.whenPressed(new WallShot());
    driverY.whenReleased(new GetSmol());
    
    // driverDDown.whenPressed(new PrepHoodShot());
    
    driverLB.whenPressed(new DriveResetGyroToZero());

    driverStart.whenPressed(new DriveFieldCentricAdvanced());
    driverBack.whenPressed(new DriveRobotCentric());

    //========== CODRIVER ==========
    coDriverA.whenReleased(new GetSmol());
    coDriverB.whenPressed(new SpinUpShooterWheel());
    coDriverX.whenPressed(new PrepWallShot().withTimeout(1.5));
    coDriverY.whenPressed(new PrepHoodShot().withTimeout(1.5));
    coDriverBack.whenPressed(new StopShoot());
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
