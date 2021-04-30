package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.SwervePath;

public class SwervePathFollowing {
    private final PIDController posErrorController;
    private final PIDController headingErrorController;
    private final ProfiledPIDController rotController;

    private Translation2d lastPosition;
    private double totalDistance = 0 ;
    private Rotation2d currentHeading;

    /**
     * Construct a SwervePathController
     *
     * @param posErrorController     PIDController for the robot's position
     * @param headingController PIDController for the robot's heading
     * @param rotationController     ProfiledPIDController for the robot's rotation
     */
    public SwervePathFollowing(PIDController posErrorController, PIDController headingController, ProfiledPIDController rotationController) {
        this.posErrorController = posErrorController;
        this.headingErrorController = headingController;
        this.headingErrorController.enableContinuousInput(-Math.PI, Math.PI);
        this.rotController = rotationController;
        this.rotController.enableContinuousInput(-Math.PI, Math.PI);
        this.lastPosition = new Translation2d();
        this.currentHeading = new Rotation2d(0);
    }

    public SwervePathFollowing(PIDController posErrorController, PIDController headingController) {
        this.posErrorController = posErrorController;
        this.headingErrorController = headingController;
        this.headingErrorController.enableContinuousInput(-Math.PI, Math.PI);
        this.rotController = null;
        this.lastPosition = new Translation2d();
        this.currentHeading = new Rotation2d(0);
    }

    public double getPosError(){
        return posErrorController.getPositionError();
    }

    /**
     * Reset the state of the path controller
     *
     * @param currentPose The current pose of the robot
     */
    public void reset(Pose2d currentPose) {
        this.posErrorController.reset();
        this.headingErrorController.reset();
        if(rotController != null) this.rotController.reset(currentPose.getRotation().getDegrees());
        this.lastPosition = currentPose.getTranslation();
        this.totalDistance = 0;
        this.currentHeading = new Rotation2d(0);
    }

    public double getTotalDistance(){
        return this.totalDistance;
    }

    public Rotation2d getCurrentHeading(){
        return this.currentHeading;
    }

    /**
     * Calculate the robot's speeds to match the path
     *
     * @param currentPose     Current pose of the robot
     * @param goalState       Goal state of the robot
     * @return The calculated speeds and rotation
     */
    public double[] calculate(Pose2d currentPose, Pose2d velocity, SwervePath.States goalState, double deltaTime, boolean doHeading) {
        Translation2d currentPos = currentPose.getTranslation();
        Rotation2d currentRotation = currentPose.getRotation();

        totalDistance += lastPosition.getDistance(currentPos);
        this.currentHeading = velocity.getRotation();

        double vel = goalState.getVelocity();
        Rotation2d heading = goalState.getHeading();
        double rotSpeed = (rotController != null) ? rotController.calculate(currentRotation.getRadians(), goalState.getRotation().getRadians()) : 0;

        vel += posErrorController.calculate(totalDistance, goalState.getPos());
        if(doHeading) {
            heading = heading.plus(new Rotation2d(headingErrorController.calculate(this.currentHeading.getRadians(), goalState.getHeading().getRadians())));
        }

        double xVelocity = vel * heading.getCos();
        double yVelocity = vel * heading.getSin();

        this.lastPosition = currentPos;

        return new double[]{xVelocity, yVelocity,rotSpeed};
    }
}