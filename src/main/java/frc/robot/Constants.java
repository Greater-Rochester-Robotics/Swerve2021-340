/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /* Swerve conversion factors */
    public static final double RAD_TO_ENC_CONV_FACTOR = 10.1859;//10.17985; //the radian to enc factor
    public static final double DRIVE_ENC_TO_METERS_FACTOR = 0.00002226;//TODO:find this ratio from mechanical specs
    public static final double PI_OVER_TWO = Math.PI/2;
    public static final double THREE_PI_OVER_TWO = 3*PI_OVER_TWO;
    public static final double TWO_PI = 2*Math.PI;

    /*Swerve dimension factors, distances from center of mass(units must be in meters!)*/
    public static final double X_POSITIVE_DISTANCE_FROM_CENTER = .294;//this is to robot center
    public static final double Y_POSITIVE_DISTANCE_FROM_CENTER = .294;
    public static final double X_NEGATIVE_DISTANCE_FROM_CENTER = -.294;//THIS NUMBER MUST BE NEGATIVE!!!!!
    public static final double Y_NEGATIVE_DISTANCE_FROM_CENTER = -.294;//THIS NUMBER MUST BE NEGATIVE!!!!!
    //These are computed from above values
    public static final double DISTANCE_TO_MODULE_0 = Math.sqrt(
        (X_POSITIVE_DISTANCE_FROM_CENTER*X_POSITIVE_DISTANCE_FROM_CENTER) +
        (Y_POSITIVE_DISTANCE_FROM_CENTER*Y_POSITIVE_DISTANCE_FROM_CENTER) );
    public static final double DISTANCE_TO_MODULE_1 = Math.sqrt(
        (X_NEGATIVE_DISTANCE_FROM_CENTER*X_NEGATIVE_DISTANCE_FROM_CENTER) +
        (Y_POSITIVE_DISTANCE_FROM_CENTER*Y_POSITIVE_DISTANCE_FROM_CENTER) );
    public static final double DISTANCE_TO_MODULE_2 = Math.sqrt(
        (X_NEGATIVE_DISTANCE_FROM_CENTER*X_NEGATIVE_DISTANCE_FROM_CENTER) +
        (Y_NEGATIVE_DISTANCE_FROM_CENTER*Y_NEGATIVE_DISTANCE_FROM_CENTER) );
    public static final double DISTANCE_TO_MODULE_3 = Math.sqrt(
        (X_POSITIVE_DISTANCE_FROM_CENTER*X_POSITIVE_DISTANCE_FROM_CENTER) +
        (Y_NEGATIVE_DISTANCE_FROM_CENTER*Y_NEGATIVE_DISTANCE_FROM_CENTER) );
    public static final double[][] MODULE_VECTORS = new double[][]{
            {X_POSITIVE_DISTANCE_FROM_CENTER , Y_POSITIVE_DISTANCE_FROM_CENTER},
            {X_NEGATIVE_DISTANCE_FROM_CENTER , Y_POSITIVE_DISTANCE_FROM_CENTER},
            {X_NEGATIVE_DISTANCE_FROM_CENTER , Y_NEGATIVE_DISTANCE_FROM_CENTER},
            {X_POSITIVE_DISTANCE_FROM_CENTER , Y_NEGATIVE_DISTANCE_FROM_CENTER}
        };
    //this is an array of unit vectors(a vector of length one)
    public static final double[][] MODULE_UNIT_VECTORS = new double[][]{
            {X_POSITIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_0 , Y_POSITIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_0},
            {X_NEGATIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_1 , Y_POSITIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_1},
            {X_NEGATIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_2 , Y_NEGATIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_2},
            {X_POSITIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_3 , Y_NEGATIVE_DISTANCE_FROM_CENTER/DISTANCE_TO_MODULE_3}
        };
    
    /* Swerve Drive Constants */
    public static final double MINIMUM_DRIVE_SPEED = 0.1;//the slowest the wheels can turn, in m/s
    public static final double MINIMUM_DRIVE_DUTY_CYCLE = 0.05;//the slowest the wheels can turn, in duty cycle
    public static final double MAXIMUM_VELOCITY = 4.5;
    public static final double MAXIMUM_ACCELERATION = 1.0;
    public static final double MAX_ROBOT_ROT_VELOCITY = MAXIMUM_VELOCITY/DISTANCE_TO_MODULE_0;
    public static final double MAXIMUM_VOLTAGE = 12.0; 
    public static final double SWERVE_DRIVE_P_VALUE = 1000; //0.035;
    public static final double SWERVE_DRIVE_I_VALUE = 0.0;
    public static final double SWERVE_DRIVE_D_VALUE = 25;
    public static final double SWERVE_DRIVE_F_VALUE = 1023/(MAXIMUM_VELOCITY/DRIVE_ENC_TO_METERS_FACTOR);

    /* Swerve Module Rotation constants */
    public static final double SWERVE_ROT_P_VALUE = .105;//if sluggish, increase P value
    public static final double SWERVE_ROT_I_VALUE = 0.0;
    public static final double SWERVE_ROT_D_VALUE = 2.5;
    public static final double SWERVE_ROT_I_ZONE_VALUE = 0.0;
    public static final double SWERVE_ROT_NONARB_FF_VALUE = 0.0;//.0001;//Not arbitrary, this is multiplied by setpoint, must be 0 in position PID
    public static final double SWERVE_ROT_ARB_FF_VOLTAGE = 1.1;
    public static final double SWERVE_ROT_PID_VOLTAGE_MINIMUM = -12.0;
    public static final double SWERVE_ROT_PID_VOLTAGE_MAXIMUM = 12.0;
    public static final double SWERVE_MODULE_TOLERANCE = 0.034;//this is in radians
    
    /* Robot Motion PID controller constants */
    public static final double ROBOT_SPIN_P = .55;
    public static final double ROBOT_SPIN_I = 0;
    public static final double ROBOT_SPIN_D = 0;

    /* Robot lateral and away pos PID controller constants */
    public static final double LATERAL_POS_P = 1;
    public static final double LATERAL_POS_I = 0;
    public static final double LATERAL_POS_D = 0;

    public static final double AWAY_POS_P = 1;
    public static final double AWAY_POS_I = 0;
    public static final double AWAY_POS_D = 0;

    /* Robot lateral speed PID controller & feed forward constants */
    public static final double LATERAL_SPEED_P = .55;
    public static final double LATERAL_SPEED_I = 0;
    public static final double LATERAL_SPEED_D = 0;
    public static final double LATERAL_FEEDFORWARD_STATIC = 0.05;
    public static final double LATERAL_FEEDFORWARD_VELOCITY = 0.95/4.5;

    /* Robot away speed PID controller & feed forward constants */
    public static final double AWAY_SPEED_P = .55;
    public static final double AWAY_SPEED_I = 0;
    public static final double AWAY_SPEED_D = 0;
    public static final double AWAY_FEEDFORWARD_STATIC = 0.05;
    public static final double AWAY_FEEDFORWARD_VELOCITY = 0.95/4.5;

    /* Driver scaling constants to slow robot */
    public static final double DRIVER_SPEED_SCALE_LATERAL = 0.8;
    public static final double DRIVER_ROTATIONAL_SCALE = 0.6;

    //SparkMAX motor controllers
    public static final int FRONT_LEFT_ROTATE_MOTOR = 41;//module 0
    public static final int REAR_LEFT_ROTATE_MOTOR = 44;//module 1
    public static final int REAR_RIGHT_ROTATE_MOTOR = 47;//module 2
    public static final int FRONT_RIGHT_ROTATE_MOTOR = 50;//module 3


    //CTRE motor and sensors
    public static final int FRONT_LEFT_MOVE_MOTOR = 40;//module 0
    public static final int FRONT_LEFT_ROTATE_SENSOR = 42;//module 0
    public static final int REAR_LEFT_MOVE_MOTOR = 43;//module 1
    public static final int REAR_LEFT_ROTATE_SENSOR = 45;//module 1
    public static final int REAR_RIGHT_MOVE_MOTOR = 46;//module 2
    public static final int REAR_RIGHT_ROTATE_SENSOR = 48;//module 2
    public static final int FRONT_RIGHT_MOVE_MOTOR = 49;//module 3
    public static final int FRONT_RIGHT_ROTATE_SENSOR = 51;//module 3 

    /* Spark MAXes */
	public static final int SHOOTER_WHEEL = 35; //launching wheel for the prototype
	public static final int BALL_HANDLER_MOTOR_0 = 30;
	public static final int BALL_HANDLER_MOTOR_1 = 31;
	public static final int BALL_HANDLER_MOTOR_2 = 32;
	public static final int BALL_HANDLER_MOTOR_3 = 33;
	public static final int BALL_HANDLER_MOTOR_4 = 34;
    
    
    /* Solenoids/LED PCM */
	public static final int HARVESTER_FWD_CHANNEL = 7;
	public static final int HARVESTER_REV_CHANNEL = 1;
	public static final int SHOOTER_HOOD_SOLENOID_CHANNEL = 2;
	public static final int CLIMBER_BRAKE_CHANNEL = 6;//No brake
    public static final int SECONDARY_PCM_ID = 11;//does not exist

    /* Shooter Speeds */
	public static final int INITIATION_SHOT_RPM = 13500; //Put real value here for safe keeping, rpm: 3425
	public static final int WALL_SHOT_RPM = 12750; // Old value 2550 MAX SPEED 21777
	public static final double RPM_MUL_FACTOR = 1.0; // 1.35 is too high
	public static final int RPM_ADD_FACTOR = 0; 

    /* Sensors */
    public static final int BALL_COUNTER_SENSOR = 4;

    
}
