// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive.autoFunc;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwervePath;
import frc.robot.subsystems.SwervePathFollowing;
import frc.robot.subsystems.SwerveDrive.kDriveMode;

public class DriveFollowPath extends CommandBase {
    Timer timer;
    SwervePath path;
    SwervePathFollowing pathController;
    double lastTime;
    boolean ignoreRotation;

    public DriveFollowPath(String pathname, boolean ignoreRotation) {
      addRequirements(RobotContainer.swerveDrive);
      this.timer = new Timer();
      this.path = SwervePath.fromJson(pathname);

      PIDController posController = new PIDController(Constants.LATERAL_POS_P, Constants.LATERAL_POS_I, Constants.LATERAL_POS_D);
      PIDController headingController = new PIDController(0.0, 0.0, 0.0);
      ProfiledPIDController rotationController = new ProfiledPIDController(Constants.ROBOT_SPIN_P, Constants.ROBOT_SPIN_I, Constants.ROBOT_SPIN_D,
              new TrapezoidProfile.Constraints(0.6, 0.3));
      this.pathController = new SwervePathFollowing(posController, headingController, rotationController);
      this.ignoreRotation = ignoreRotation;
    }

    public DriveFollowPath(String pathname) {
      this(pathname, false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        SwervePath.States initialState = path.getInitialState();
        RobotContainer.swerveDrive.setCurrentPos(new Pose2d(RobotContainer.swerveDrive.getCurrentPose().getTranslation(), initialState.getRotation()));
        pathController.reset(RobotContainer.swerveDrive.getCurrentPose());
        lastTime = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double time = timer.get();
        SwervePath.States desiredState = path.sample(time);

        if(ignoreRotation) desiredState.rotation = new Rotation2d(0);

        double[] targetSpeeds = pathController.calculate(RobotContainer.swerveDrive.getCurrentPose(),
            RobotContainer.swerveDrive.getCurrentVelocity(), desiredState, time - lastTime, timer.hasElapsed(0.1));
        RobotContainer.swerveDrive.driveFieldCentric(targetSpeeds[0], targetSpeeds[1], targetSpeeds[2], kDriveMode.percentOutput);

        lastTime = time;

        // Position Graph
        SmartDashboard.putNumber("PIDTarget", desiredState.getPos());
        SmartDashboard.putNumber("PIDActual", pathController.getTotalDistance());


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println(timer.get());
        timer.stop();
        RobotContainer.swerveDrive.stopAllModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(path.getRuntime());
    }
}