/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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
  private Pose2d currentPosition = new Pose2d(new Translation2d(),new Rotation2d());
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
   * This enumeration is for the change between, velocity mode and dutyCycle mode
   * percentOutput | 0
   * velocity      | 1
   */
  public enum kDriveMode{
    percentOutput(0) , velocity(1);
    public int num;
    private kDriveMode(int number){
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
  /**
   * This method is run once per scheduler run. It works by 
   * adding the change of each module to update the currentPosition
   */
  public void periodic() {
    //create an array to store the distance travelled by the robot since the last time this was called
    double[] deltaPosition = new double[]{0.0,0.0};
    for (int i=0; i<4; i++){
      //pull the distance travelled by a module(robot centric)
      double[] deltaPerMod = swerveModules[i].periodic();
      //add the distance travelled in the x by this module to the others(with respect to the robot)
      deltaPosition[0]+= deltaPerMod[0];
      //add the distance travelled in the y by this module to the others(with respect to the robot)
      deltaPosition[1]+= deltaPerMod[1];
    }
    //the prior array is based around the robot's x and y and not the feild's

    //The following pulls the current rotational orientation of the robot(Rotation2d)
    // Rotation2d currentRot = this.getGyroRotation2d();
    // // The following updates the currentPostion object
    // currentPosition = new Pose2d(
    //   currentPosition.getX() + (deltaPosition[0]*currentRot.getCos()) + (deltaPosition[1]*currentRot.getSin()),
    //   currentPosition.getY() + (deltaPosition[1]*currentRot.getCos()) + (deltaPosition[0]*currentRot.getSin()),
    //   currentRot);
  }

  /**
   * Drives the robot based on speeds from the robot's orientation.
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not moving in that direction
   * @param forwardSpeed the movement forward and backward
   * @param strafeSpeed the movement side to side
   * @param rotSpeed the speed of rotation
   */
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
    
    //create an empty array tto put output speeds in
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
    //the following creates an effective deadzone
    }else if(maxSpeed < Constants.MINIMUM_DRIVE_DUTY_CYCLE){
      //if the maxSpeed is below the minimum movement speed, don't let the modules turn.
      return;
    }

    // assign the angles to the swerve drive modules
    for (int i=0; i<4; i++){
      swerveModules[i].setPosInRad(targetModuleAngles[i]); 
    }

    double[] curAngles = new double[4]; // pull the current angles of the modules
    for (int i=0; i<4; i++){
      curAngles[i] = swerveModules[i].getPosInRad();
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
   * Drive the robot so that all directions are independent of the robots orientation (rotation)
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not moving in that direction
   * @param awaySpeed from field centric, aka a fix direction, away from or toward the driver, a speed
   * @param lateralSpeed from field centric, aka a fix direction regardless of robot rotation, a speed 
   * @param rotSpeed rotational speed of the robot
   */
  public void driveFieldCentric(double awaySpeed, double lateralSpeed, double rotSpeed){
    //pull the current oreintation of the robot(based on gyro)
    Rotation2d gyro = this.getGyroRotation2d();
    double robotForwardSpeed = (gyro.getCos()*awaySpeed) + (gyro.getSin() * lateralSpeed);
    double robotStrafeSpeed = (gyro.getCos()*lateralSpeed) + (gyro.getSin() * awaySpeed);
    this.driveRobotCentric( robotForwardSpeed , robotStrafeSpeed , rotSpeed);
  }

  /**
   * This function is meant to drive one module at a time for testing purposes.
   * @param moduleNumber which of the four modules(0-3) we are using
   * @param moveSpeed move speed -1.0 to 1.0, where 0.0 is stopped
   * @param rotatePos a positon between -PI and PI where we want the module to be
   */
  public void driveOneModule(int moduleNumber,double moveSpeed, double rotatePos){
    //test that moduleNumber is between 0-3, return if not(return;)
    if (moduleNumber > 3 && moduleNumber < 0){
      System.out.println("Module " + moduleNumber + " is out of bounds.");
      return;
    }else if(rotatePos<-Math.PI || rotatePos > Math.PI){
      System.out.println("Input angle out of range.");
      return;
    }
    
    //write code to drive one module in a testing form
    swerveModules[moduleNumber].setPosInRad(rotatePos);
    swerveModules[moduleNumber].setDriveMotor(moveSpeed);
  }

  /**
   * This method generates wheel directions for driving along an arc
   * @param arcRadius the distance from the center of the robot to the point we wish to move around
   * @param arcAngleInRad the angle from the front of the robot to the point we wish to move around
   * @return an array of angles and scaled normalizations for speed management
   */
  public double[][] generateArcAngles(double arcRadius, double arcAngleInRad){
    //create a vector out of the arcAngle and length, this is from the
    // center of the robot to the center of the circle we are following
    double[] arcVector = new double[]{
      arcRadius*Math.cos(arcAngleInRad), arcRadius*Math.sin(arcAngleInRad)};

    //create an array to store our vectors from module to circle center
    double[][] mathVector = new double[2][4];

    //creates an output array for angles and normalizing factors for each module
    double[][] outputArray = new double[2][4];

    //will want to know the longest vector, the furtherest away a module is from center circle
    double maxLength = 0.0;

    //for every module, find the vector from the module to the arc's center
    for(int i=0; i<4; i++){
      //add the module position vector to the arc vector
      mathVector[0][i] = arcVector[0]-Constants.MODULE_VECTORS[0][i];
      mathVector[1][i] = arcVector[1]-Constants.MODULE_VECTORS[1][i];

      //find the length of the vector from this module to the arc center
      outputArray[1][i] = Math.sqrt( (mathVector[0][i]*mathVector[0][i])
        + (mathVector[0][i]*mathVector[0][i]) );
      
      //find the longest vector, from module to circle center
      if(outputArray[1][i] > maxLength){
        maxLength = outputArray[1][i];
      }

      //turn the vector from this module to the arc center to a unit vector
      mathVector[0][i]/=outputArray[1][i];
      mathVector[1][i]/=outputArray[1][i];

      //in order to drive an arc, we need angles that are rotated 90 degrees, hence x for y and -y for x
      outputArray[0][i] = Math.atan2(mathVector[0][i],-1*mathVector[1][i]);
    }

    //normalize the max length of the Vectors, this will let us scale drive speed.
    for(int i=0; i<4; i++){
      outputArray[1][i]/=maxLength;
    }

    return outputArray;
  }

  /**
   * A method to drive an arc with the robot
   * @param speed speed for robot to move, either -1.0 to 1.0 for percentVoltage, or in m/s in Velocity 
   * @param arcArray an array generated by generateArcAngles()
   * @param mode whether to use percentVoltage(aka duty cycle) or velocity
   */
  public void driveArc(double speed,double[][] arcArray,kDriveMode mode){

    // restrict speed to between -1.0 and 1.0, if in PercentOutput 
    if (mode.equals(kDriveMode.percentOutput)){
      if (speed < -1.0){
        speed = -1.0;
      }
      else if (speed > 1.0){
        speed = 1.0;
      }
      //return if speed below Constants.Minimum_speed in kPercentOutput
      else if (Math.abs(speed) < Constants.MINIMUM_DRIVE_DUTY_CYCLE){
        System.out.println("Drive speed too slow for robot to move! " + speed);
        return;
      }
    }
    // TODO: Sanitize for mode velocity in the future
    //set each module to the angle in arcArray(the first value)
    for (int i=0; i<4; i++){
      swerveModules[i].setPosInRad(arcArray[0][i]); 
    }

    double[] curAngles = new double[4]; // pull the current angles of the modules
    for (int i=0; i<4; i++){
      curAngles[i] = swerveModules[i].getPosInRad();
    }

    //set the speed of each motor to the speed multiplied by the second value of arcArray
    //Reduce power to motors until they align with the target angle(might remove later)
    for(int i=0 ; i<4 ; i++){
      swerveModules[i].setDriveMotor(speed*Math.cos(arcArray[0][i]-curAngles[i])*arcArray[1][i]);
    }
  }

  /**
   * This is a function to drive the robot straight in a set direction
   * @param speed the speed the robot should drive, -1.0 to 1.0 in percentOutput, 
   * @param angle the angle, robot centric, the robot should drive in, inRadians, from -pi to pi
   * @param mode whether to use percentVoltage(aka duty cycle) or velocity
   */
  public void driveStraight(double speed,double angle,kDriveMode mode){
    //Sanitize inputs, both angle and speed, speed must be sanitized based on mode
    
    if (mode.equals(kDriveMode.percentOutput)){
      if (speed < -1.0){
        speed = -1.0;
      }
      else if (speed > 1.0){
        speed = 1.0;
      }
      else if (Math.abs(speed) < Constants.MINIMUM_DRIVE_DUTY_CYCLE){
        System.out.println("Drive speed too slow for robot to move! " + speed);
        return;
      }
    if (mode.equals(kDriveMode.percentOutput)){
      if (angle < -Math.PI){
        angle = -Math.PI;
      }
      else if (angle > Math.PI){
        angle = Math.PI;
      }
    }
    // TODO: Sanitize for mode velocity in the future
    //set all modules to angle
    for (int i=0; i<4; i++){
      swerveModules[i].setPosInRad(angle); 
    }
    // pull the current angles of the modules
    double[] curAngles = new double[4]; 
    for (int i=0; i<4; i++){
      curAngles[i] = swerveModules[i].getPosInRad();
    }

    //
    double[] targetMotorSpeeds = new double[4];

    //Reduce power to motors until they align with the target angle(might remove later)
    for(int i=0 ; i<4 ; i++){
      targetMotorSpeeds[i] = speed*Math.cos(angle-curAngles[i]);
    }
    //set all modules to targetMotorSpeeds, use setDriveMotor if in percentOUtput, and setDriveSpeed if in velocity
    if (mode.equals(kDriveMode.percentOutput)){
      for (int i=0; i<4; i++){
        swerveModules[i].setDriveMotor(targetMotorSpeeds[i]);
      }
    }
    else if (mode.equals(kDriveMode.velocity)){
      for (int i=0; i<4; i++){
        swerveModules[i].setDriveSpeed(targetMotorSpeeds[i]);
      }
    }
  }
}

  /**
   * Stops all module motion, then lets all the modules spin freely.
   */
  public void stopAllModules(){
    for (int i=0; i<4; i++){
      swerveModules[i].stopAll();
    }
  }

  /**
   * A function that allows the user to reset the gyro
   */
  public void resetGyro(){
    //Resets the gyro(zero it)
    imu.reset();
  }

  /**
   * This calls the drive Gyro and returns the Rotation2d object.
   * This object contains angle in radians, as well as the sin 
   * and cos of that angle. This is an object that represents the
   * rotation of the robot.
   * @return a Rotation2d object
   */
  public Rotation2d getGyroRotation2d(){
    //return a newly constructed Rotation2d object, it takes the angle in radians as a constructor arguement
    return imu.getRotation2d();
    //TODO:(with robot) check that counterclockwise rotation is positive
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in radians
   */
  public double getGyroInRad(){
    return Math.toRadians(imu.getAngle());
    // Pull and return gyro in radians
    //TODO:(with robot) check that counterclockwise rotation is positive
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in degrees
   */
  public double getGyroInDeg(){
    return imu.getAngle();//Pull gyro in radians and convert to degrees
    //TODO:(with robot) check that counterclockwise rotation is positive
  }

  /**
   * this will only work if periodic function is running in the subsystem
   * @return the position of the robot with respect to the field, as a Pose2d object
   */
  public Pose2d getCurrentPose(){
    return this.currentPosition;
  }

  public double[] getAllAbsModuleAngles(){
    double[] moduleAngles = new double[4];
    for(int i=0; i<4; i++){
      moduleAngles[i]=swerveModules[i].getAbsPosInDeg();
    }
    return moduleAngles;
  }

  /**
   * a method to print all module positions for testing purposes
   */
  public void printAllModuleAngles(){
    //Use a for loop to and print() all modules' angles(degrees) on one line, make sure to newline "\n" at the end  
    System.out.print("Angle = ");
  
    for(int i=0; i<4; i++){
      System.out.print(swerveModules[i].getAbsPosInDeg()+"\t");
    }
    System.out.print("\n");
  }
  

  public void resetAllModules(){
    for (int i=0; i<4; i++){
      swerveModules[i].resetPositionArray();
    }
  }

}
