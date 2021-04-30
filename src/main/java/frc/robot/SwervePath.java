package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Custom version of the wpilib trajectory class that is
 * made to work better with PathPlanner and swerve drive
 */
public class SwervePath {
    public static final double TIME_STEP = 0.01;
    private ArrayList<States> states;
    private static Trajectory robotPath;

    /**
     * Construct a new Swerve Path
     */
    public SwervePath() {
        this.states = new ArrayList<>();
    }

    /**
     * Get all of the states in the path
     *
     * @return An arraylist of all path states
     */
    public ArrayList<States> getStates() {
        return this.states;
    }

    /**
     * Get the number of states in the path
     *
     * @return The number of states
     */
    public int numStates() {
        return this.states.size();
    }

    /**
     * Get the total runtime of a path
     *
     * @return Total runtime in seconds
     */
    public double getRuntime() {
        return states.get(states.size() - 1).time;
    }

    /**
     * Get the initial state in the path
     *
     * @return First state in the path
     */
    public States getInitialState() {
        return this.states.get(0);
    }

    /**
     * Create a SwervePath object from a JSON file
     * Expected format is xPos, yPos, velocity, acceleration, heading (direction robot is moving), rotation
     *
     * @param filename The path file to load
     * @return The SwervePath object
     */
    public static SwervePath fromJson(String fileName) {
        SwervePath outputTrajectory = new SwervePath();

        String trajectoryJSON = "output/output/" + fileName + ".wpilib.json";
        robotPath = new Trajectory();
        try {
            Path filePath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            robotPath = TrajectoryUtil.fromPathweaverJson(filePath);
            Pose2d prevPos =  robotPath.getInitialPose();
            double prevDist = 0;
            for(State state: robotPath.getStates()){
                double pos = state.poseMeters.minus(prevPos).getTranslation().getNorm() + prevDist;//(state.poseMeters.getTranslation().getNorm());
                double vel = (state.velocityMetersPerSecond);
                double acc = (state.accelerationMetersPerSecondSq);
                Rotation2d heading = state.poseMeters.getRotation();
                double rotation = 0.0;
                double time = state.timeSeconds;
                outputTrajectory.states.add(new States(pos, heading, vel, acc, new Rotation2d(rotation), time));
                prevDist = pos;
                prevPos = state.poseMeters;
            }
        } catch (Exception err) {
            DriverStation.reportError("Unable to open trajectory file: " + trajectoryJSON, err.getStackTrace());
        }
        return outputTrajectory;
    }

    private static double lerp(double startVal, double endVal, double t) {
        return startVal + (endVal - startVal) * t;
    }

    private static Translation2d lerp(Translation2d startVal, Translation2d endVal, double t) {
        return startVal.plus((endVal.minus(startVal)).times(t));
    }

    private static Rotation2d lerp(Rotation2d startVal, Rotation2d endVal, double t) {
        return startVal.plus((endVal.minus(startVal)).times(t));
    }

    /**
     * Sample the path at a given point in time
     *
     * @param time The elapsed time to sample
     * @return State of the path at the given time
     */
    public States sample(double time) {
        if (time <= states.get(0).time) {
            return states.get(0);
        }

        if (time >= getRuntime()) {
            return states.get(states.size() - 1);
        }

        int low = 1;
        int high = states.size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (states.get(mid).time < time) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }

        States sample = states.get(low);
        States prevSample = states.get(low - 1);

        if (Math.abs(sample.time - prevSample.time) < 1E-5) {
            return sample;
        }

        return prevSample.interpolate(sample, (time - prevSample.time) / (sample.time - prevSample.time));
    }

    public static class States {
        private final double pos;
        private final Rotation2d heading;
        private final double velocity;
        private final double acceleration;
        public Rotation2d rotation;
        private final double time;

        /**
         * Construct a State
         *
         * @param pos          the robot's distance of travel on the path
         * @param heading      Rotation2d representing the direction of robot motion
         * @param velocity     Velocity of the robot
         * @param acceleration Acceleration of the robot
         * @param rotation     Rotation of the robot
         * @param time         Time this state represents
         */
        public States(double pos, Rotation2d heading, double velocity, double acceleration, Rotation2d rotation, double time) {
            this.pos = pos;
            this.heading = heading;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.rotation = rotation;
            this.time = time;
        }


        public double getPos() {
            return this.pos;
        }

        public double getVelocity() {
            return this.velocity;
        }

        public Rotation2d getRotation() {
            return this.rotation;
        }

        public Rotation2d getHeading(){
            return this.heading;
        }

        public double getTime() {
            return this.time;
        }

        States interpolate(States endVal, double t) {
            double newT = lerp(time, endVal.time, t);
            double deltaT = newT - time;

            if (deltaT < 0) {
                return endVal.interpolate(this, 1 - t);
            }

            double newV = velocity + (acceleration * deltaT);
            double newS = (velocity * deltaT) + (0.5 * acceleration * Math.pow(deltaT, 2));

            double interpolationFrac = newS / (pos - endVal.pos);

            return new States(
                    lerp(pos, endVal.pos, interpolationFrac),
                    lerp(heading, endVal.heading, interpolationFrac),
                    newV,
                    acceleration,
                    lerp(rotation, endVal.rotation, interpolationFrac),
                    newT
            );
        }
    }
}