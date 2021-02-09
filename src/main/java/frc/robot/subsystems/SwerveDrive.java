/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.analog.adis16448.frc.ADIS16448_IMU.IMUAxis;
/**
 * This is the subsystem that governs the four swerve module objects.
 *  In this class, the positive x-axis is toward the front of the robot,
 *  and the positive y-axis is toward the left side. All angles are
 *  measured from the positive x-axis and positive angles are couter
 *  clockwise from that axis. The modules are numbered, starting at 
 *  module0 for the left front module continuing counter clockwise.
 *  This makes the rear left module1, the rear right module2, and the
 *  front right module3.
 */
public class SwerveDrive extends SubsystemBase {
  private static SwerveModule swerveModules[];
  private static SwerveModule frontLeft, rearLeft, rearRight, frontRight;
  public ADIS16448_IMU imu;
  
  /**
   * This enumeration clarifies the numbering of the swerve module for new users.
   * frontLeft  | 0
   * rearLeft   | 1
   * rearRight  | 2
   * frontRight | 3
   */
  public enum kSwerveModule{
    frontLeft(0) , rearLeft(1) , rearRight(2) , frontRight(3);
    public int num;
    private kSwerveModule(int number){
      num = number;
    }
    public int getNumber() {
			return num;
		}
  }
  
  /**
   * Creates a new SwerveDrive.
   */
  public SwerveDrive() {
    //adds CANCoder address as third param, already named as rotation sensor in constants
    // Constructs the swerve modules 
    frontLeft = new SwerveModule(Constants.FRONT_LEFT_MOVE_MOTOR, Constants.FRONT_LEFT_ROTATE_MOTOR, Constants.FRONT_LEFT_ROTATE_SENSOR);
    rearLeft = new SwerveModule(Constants.REAR_LEFT_MOVE_MOTOR, Constants.REAR_LEFT_ROTATE_MOTOR, Constants.REAR_LEFT_ROTATE_SENSOR);
    rearRight = new SwerveModule(Constants.REAR_RIGHT_MOVE_MOTOR, Constants.REAR_RIGHT_ROTATE_MOTOR, Constants.REAR_RIGHT_ROTATE_SENSOR);
    frontRight = new SwerveModule(Constants.FRONT_RIGHT_MOVE_MOTOR, Constants.FRONT_RIGHT_ROTATE_MOTOR, Constants.FRONT_RIGHT_ROTATE_SENSOR);
    
     //This may seem repetitive, but it makes clear which module is which.
    
    swerveModules = new SwerveModule[]{
      frontLeft,
      rearLeft,
      rearRight,
      frontRight
    };
    
    // Constructs IMU object
    imu = new ADIS16448_IMU();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveRobotCentric(double forwardSpeed, double strafeSpeed, double rotSpeed){
    double[] targetMoveVector = { forwardSpeed , strafeSpeed };//the direction we want the robot to move

    //create a 2d array for the goal output of each module(in vector component form)
    double[][] targetModuleVectors = new double[2][4];
    //create a vector for each module, one at a time
    for(int i=0 ; i<4 ; i++){
      //compute the x-component of the vector by adding the targetVector to the cross product with rotspeed
      targetModuleVectors[0][i] =
        targetMoveVector[0] - (rotSpeed*Constants.MODULE_UNIT_VECTORS[1][i] );
      
      //compute the y-component of the vector by adding the targetVector to the cross product with rotspeed
      targetModuleVectors[1][i] = 
        targetMoveVector[1] + (rotSpeed*Constants.MODULE_UNIT_VECTORS[0][i] );//TODO: check if this sign is right
    }

    //generates angles for each module
    double[] targetModuleAngles = new double[4];
    for(int i=0 ; i<4 ; i++){
      targetModuleAngles[i] = Math.atan2( targetModuleVectors[1][i] , targetModuleVectors[0][i] );
    }

    // assign the angles to the swerve drive modules (use a for loop and setPosInRad())
    for (int i=0; i<4; i++){
      swerveModules[i].setPosInRad(targetModuleAngles[i]); 
    }

    double[] curAngles = new double[4]; // pull the current angles of the modules(use a for loop and getPosInRad())
    for (int i=0; i<4; i++){
      curAngles[i] = swerveModules[i].getPosInRad();
    }
    

    double[] targetMotorSpeeds = new double[4];
    //create a variable so we can find the maxSpeed
    double maxSpeed = 0.0;
    
    for(int i=0 ; i<4 ; i++){
      //find the length(power) for each direction vector
      targetMotorSpeeds[i] = Math.sqrt(
        (targetModuleVectors[0][i]*targetModuleVectors[0][i]) +
        (targetModuleVectors[1][i]*targetModuleVectors[1][i])   );
      //find and store the largest speed(all speeds are positive)
      if(maxSpeed < targetMotorSpeeds[i]){
        maxSpeed = targetMotorSpeeds[i];
      }
    }

    //normalize all speeds, by dividing by the largest, if largest is greater than 1
    if(maxSpeed > 1){
      for(int i=0 ; i<4 ; i++){
        targetMotorSpeeds[i] = targetMotorSpeeds[i]/maxSpeed;
      }
    }

    //Reduce power to motors until they align with the target angle(might remove later)
    for(int i=0 ; i<4 ; i++){
      targetMotorSpeeds[i] = targetMotorSpeeds[i]*Math.cos(targetModuleAngles[i]-curAngles[i]);
    }

    
    //assign output to each module(use a for loop with targetMotorSpeeds[])
    for (int i=0; i<4; i++){
      swerveModules[i].setDriveMotor(targetMotorSpeeds[i]);
    }

  }

  /**
   * 
   * @param awaySpeed
   * @param lateralSpeed
   * @param rotSpeed
   */
  public void driveFieldCentric(double awaySpeed, double lateralSpeed, double rotSpeed){
    double robotForwardSpeed = (Math.cos(this.getGyroInRad())*awaySpeed) + (Math.sin(this.getGyroInRad()) * lateralSpeed);
    double robotStrafeSpeed = (Math.cos(this.getGyroInRad())*lateralSpeed) + (Math.sin(this.getGyroInRad()) * awaySpeed);
    this.driveRobotCentric( robotForwardSpeed , robotStrafeSpeed , rotSpeed );
  }

  /**
   * This function is meant to drive one module at a time for testing purposes.
   * @param moduleNumber
   * @param moveSpeed
   * @param rotatePos
   */
  public void driveOneModule(int moduleNumber,double moveSpeed, double rotatePos){
    //test that moduleNumber is between 0-3, return if not(return;)
    if (moduleNumber > 3 && moduleNumber < 0){
      return;
    }
    
    //write code to drive one module in a testing form
    swerveModules[moduleNumber].setPosInRad(rotatePos);
    swerveModules[moduleNumber].setDriveMotor(moveSpeed);
  }

  /**
   * A function that allows the user to reset the gyro
   */
  public void resetGyro(){
    //Resets the gyro(zero it)
    imu.reset();
     
    
  }

  /**
   * this polls the onboard gyro, which, when the robot boots, assumes and angle of zero
   * @return the angle of the robot in radians
   */
  public double getGyroInRad(){
  return imu.getAngle() * Math.PI/180 ;
   //Pull and return gyro in radians
    


  }

  /**
   * this polls the onboard gyro, which, when the robot boots, assumes and angle of zero
   * @return the angle of the robot in degrees
   */
  public double getGyroInDeg(){
    return imu.getAngle();//Pull gyro in radians and convert to degrees
    
  }

  /**
   * Lets all the modules spin freely.
   */
  public void stopAllModules(){
    for (int i=0; i<4; i++){
      swerveModules[i].stopAll();
    }
  }

  public double[] getAllModuleAngles(){
    double[] moduleAngles = new double[4];
    for(int i=0; i<4; i++){
      moduleAngles[i]=swerveModules[i].getPosInDeg();
    }
    return moduleAngles;
  }

  public void resetAllModules(){
    
  }

}
