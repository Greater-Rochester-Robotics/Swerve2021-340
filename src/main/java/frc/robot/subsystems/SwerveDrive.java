/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

import frc.robot.Constants;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.analog.adis16470.frc.ADIS16470_IMU.ADIS16470CalibrationTime;
import com.analog.adis16470.frc.ADIS16470_IMU.IMUAxis;

import edu.wpi.first.wpilibj.SPI;

/**
 * This is the subsystem that governs the four swerve module objects.
 *  In this class, the positive x-axis is toward the front of the robot,
 *  and the positive y-axis is toward the left side. All angles are
 *  measured from the positive x-axis and positive angles are counter
 *  clockwise from that axis. The modules are numbered, starting at 
 *  module0 for the left front module continuing counter clockwise.
 *  This makes the rear left module1, the rear right module2, and the
 *  front right module3.
 */
public class SwerveDrive extends SubsystemBase {
  private static SwerveModule swerveModules[];
  private static SwerveModule frontLeft, rearLeft, rearRight, frontRight;
  private Pose2d currentPosition = new Pose2d(new Translation2d(),new Rotation2d());
  private Pose2d currentVelocity = new Pose2d(new Translation2d(), new Rotation2d());
  public ADIS16470_IMU imu;
  public PIDController robotSpinController;

  public PIDController lateralSpeedPIDController;
  public SimpleMotorFeedforward lateralSpeedFeedforward;

  public PIDController awaySpeedPIDController;
  public SimpleMotorFeedforward awaySpeedFeedforward;

  public PIDController lateralPosPidController;
  public PIDController awayPosPidController;

  public Boolean isMinOutLimited = true;  
  public boolean isOdometryRunning = true;
  

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
    imu = new ADIS16470_IMU(IMUAxis.kY, SPI.Port.kOnboardCS0, ADIS16470CalibrationTime._4s);

    //construct the wpilib PIDcontroller for rotation.
    robotSpinController = new PIDController(Constants.ROBOT_SPIN_P, Constants.ROBOT_SPIN_I, Constants.ROBOT_SPIN_D);

    lateralSpeedPIDController = new PIDController(Constants.LATERAL_SPEED_P, Constants.LATERAL_SPEED_I, Constants.LATERAL_SPEED_D);
    lateralSpeedFeedforward = new SimpleMotorFeedforward(Constants.LATERAL_FEEDFORWARD_STATIC, Constants.LATERAL_FEEDFORWARD_VELOCITY);

    awaySpeedPIDController = new PIDController(Constants.AWAY_SPEED_P, Constants.AWAY_SPEED_I, Constants.AWAY_SPEED_D);
    awaySpeedFeedforward = new SimpleMotorFeedforward(Constants.AWAY_FEEDFORWARD_STATIC, Constants.AWAY_FEEDFORWARD_VELOCITY, Constants.AWAY_FEEDFORWARD_ACCELERATION);

    lateralPosPidController = new PIDController(Constants.LATERAL_POS_P, Constants.LATERAL_POS_I, Constants.LATERAL_SPEED_D);
    awayPosPidController = new PIDController(Constants.AWAY_POS_P, Constants.AWAY_POS_I, Constants.AWAY_POS_D);

    
  }

  @Override
  /**
   * This method is run once per scheduler run. It works by 
   * adding the change in position of each module to update
   * the currentPosition
   */
  public void periodic() {
    
    SmartDashboard.putNumber("Gyro", getGyroInDeg());

    //create an array to store the distance travelled by the robot since the last time this was called
    double[] deltaPosition = new double[]{0.0,0.0};
    double[] currentVelocities = new double[] {0.0,0.0};

    //in order to save process time, this allows the odometry in module to be turned off
    int countTo = isOdometryRunning?4:0;

    //velo array
    for (int i=0; i<countTo; i++){
      //pull the distance travelled by a module(robot centric)
      double[] deltaPerMod = swerveModules[i].periodic();
      //add the distance travelled in the x by this module to the others(with respect to the robot)
      deltaPosition[0]+= deltaPerMod[0];
      //add the distance travelled in the y by this module to the others(with respect to the robot)
      deltaPosition[1]+= deltaPerMod[1];
      // add velocities of all modules
      currentVelocities[0]+= deltaPerMod[2];
      currentVelocities[1] += deltaPerMod[3];
    }

    //the prior array is based around the robot's x and y and not the field's

    // Divide by 4 because all swerve modules were added together
    deltaPosition[0] /= 4;
    deltaPosition[1] /= 4;

    currentVelocities[0] /=4;
    currentVelocities[1] /= 4;

    // //The following pulls the current rotational orientation of the robot(Rotation2d)
    Rotation2d currentRot = this.getGyroRotation2d();
    // The following updates the currentPosition & currentVelocities object to be relative to the field
    currentPosition = new Pose2d(
      currentPosition.getX() + (deltaPosition[0]*currentRot.getCos()) + (deltaPosition[1]*currentRot.getSin()),
      currentPosition.getY() + (deltaPosition[1]*currentRot.getCos()) - (deltaPosition[0]*currentRot.getSin()),
      currentRot);

    Translation2d currVelocityTrans = new Translation2d(
      currentVelocities[0]*currentRot.getCos() + currentVelocities[1]*currentRot.getSin(),
      currentVelocities[1]*currentRot.getCos() - currentVelocities[0]*currentRot.getSin());
    Rotation2d currVelocityRot = new Rotation2d( Math.atan2(
      currentVelocities[1]*currentRot.getCos() - currentVelocities[0]*currentRot.getSin(),
      currentVelocities[0]*currentRot.getCos() + currentVelocities[1]*currentRot.getSin()));
    currentVelocity = new Pose2d(currVelocityTrans,currVelocityRot);
    
    SmartDashboard.putNumber("Current position X", currentPosition.getX());
    SmartDashboard.putNumber("Current position Y", currentPosition.getY());
    SmartDashboard.putNumber("Current velocity X", currentVelocity.getX());
    SmartDashboard.putNumber("Current velocity Y", currentVelocity.getY());
  }

  /**
   * Drives the robot based on speeds from the robot's orientation.
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not 
   * moving for percentVoltage mode and between the Max Velocity 
   * and -Max Velocity with 0 not moving in Velocity mode
   * @param forwardSpeed the movement forward and backward
   * @param strafeSpeed the movement side to side
   * @param rotSpeed the speed of rotation
   * @param mode the mode of either percentOutput or velocity
   */
  public void driveRobotCentric(double forwardSpeed, double strafeSpeed, double rotSpeed, kDriveMode mode){
    boolean isVelocityMode = kDriveMode.velocity == mode;
    double[] targetMoveVector = { forwardSpeed , strafeSpeed };//the direction we want the robot to move

    //create a 2d array for the goal output of each module(in vector component form)
    double[][] targetModuleVectors = new double[4][2];

    //create a vector for each module, one at a time
    for(int i=0 ; i<4 ; i++){
      //compute the x-component of the vector by adding the targetVector to the cross product with rotspeed
      targetModuleVectors[i][0] =
        targetMoveVector[0] - 
        (rotSpeed*(isVelocityMode?Constants.MODULE_VECTORS[i][1]:Constants.MODULE_UNIT_VECTORS[i][1]));
      
      //compute the y-component of the vector by adding the targetVector to the cross product with rotspeed
      targetModuleVectors[i][1] = 
        targetMoveVector[1] + 
        (rotSpeed*(isVelocityMode?Constants.MODULE_VECTORS[i][0]:Constants.MODULE_UNIT_VECTORS[i][0]));
    }

    //generates angles for each module
    double[] targetModuleAngles = new double[4];
    for(int i=0 ; i<4 ; i++){
      targetModuleAngles[i] = Math.atan2( targetModuleVectors[i][1] , targetModuleVectors[i][0] );
    }
    
    //create an empty array to put output speeds in
    double[] targetMotorSpeeds = new double[4];
    //create a variable so we can find the maxSpeed
    double maxSpeed = 0.0;
    
    for(int i=0 ; i<4 ; i++){
      //find the length(power) for each direction vector
      targetMotorSpeeds[i] = Math.sqrt(
        (targetModuleVectors[i][0]*targetModuleVectors[i][0]) +
        (targetModuleVectors[i][1]*targetModuleVectors[i][1])   );
      //find and store the largest speed(all speeds are positive)
      if(maxSpeed < targetMotorSpeeds[i]){
        maxSpeed = targetMotorSpeeds[i];
      }
    }

    //normalize all speeds, by dividing by the largest, if largest is greater than 1
    if(maxSpeed > (isVelocityMode?Constants.MAXIMUM_VELOCITY:1) ){
      for(int i=0 ; i<4 ; i++){
        targetMotorSpeeds[i] = targetMotorSpeeds[i]/maxSpeed;
      }
    //the following creates an effective deadzone
    }

    //change the following to a simple if, invert the logic, and within, place the next for loop
    if(isMinOutLimited && maxSpeed < (isVelocityMode?Constants.MINIMUM_DRIVE_SPEED:Constants.MINIMUM_DRIVE_DUTY_CYCLE)){
      //if the maxSpeed is below the minimum movement speed, stop all the motors.
      for(int i=0 ; i<4 ; i++){
        swerveModules[i].setDriveMotor(0.0);
      }
      //if the maxSpeed is below the minimum movement speed, end this method, without turning modules
      return;
    }

    // assign the angles to the swerve drive modules
    for (int i=0; i<4; i++){
      swerveModules[i].setPosInRad(targetModuleAngles[i]); 
    }

    // pull the current angles of the modules(do this now, to allow module to invert)
    double[] curAngles = new double[4]; 
    for (int i=0; i<4; i++){
      curAngles[i] = swerveModules[i].getPosInRad();
    }

    //Reduce power to motors until they align with the target angle(might remove later)
    for(int i=0 ; i<4 ; i++){
      targetMotorSpeeds[i] = targetMotorSpeeds[i]*Math.cos(targetModuleAngles[i]-curAngles[i]);
    }


    //assign output to each module(uses a for loop with targetMotorSpeeds[])
    if(isVelocityMode){
      for (int i=0; i<4; i++){
        swerveModules[i].setDriveSpeed(targetMotorSpeeds[i]);
      }
    }else{
      for (int i=0; i<4; i++){
        swerveModules[i].setDriveMotor(targetMotorSpeeds[i]);
      }
    }

  }

  /**
   * Drive the robot so that all directions are independent of the robots orientation (rotation)
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not moving in that direction
   * 
   * @param awaySpeed from field centric, aka a fix direction,
   *                  away from or toward the driver, a speed
   *                  valued between -1.0 and 1.0, where 1.0
   *                  is to away from the driver 
   * @param lateralSpeed from field centric, aka a fix direction
   *                     regardless of robot rotation, a speed
   *                     valued between -1.0 and 1.0, where 1.0
   *                     is to the left 
   * @param rotSpeed rotational speed of the robot
   *                 -1.0 to 1.0 where 0.0 is not rotating
   * @param mode the mode of either percentOutput or velocity
   */
  public void driveFieldCentric(double awaySpeed, double lateralSpeed, double rotSpeed, kDriveMode mode){
    //pull the current orientation of the robot(based on gyro)
    Rotation2d gyro = this.getGyroRotation2d();
    double robotForwardSpeed = (gyro.getCos()*awaySpeed) + (gyro.getSin() * lateralSpeed);
    double robotStrafeSpeed = (gyro.getCos()*lateralSpeed) - (gyro.getSin() * awaySpeed);
    this.driveRobotCentric( robotForwardSpeed , robotStrafeSpeed , rotSpeed, mode);
  }

  /**
   * This function is meant to drive one module at a time for testing purposes.
   * @param moduleNumber which of the four modules(0-3) we are using
   * @param moveSpeed move speed -1.0 to 1.0, where 0.0 is stopped
   * @param rotatePos a position between -PI and PI where we want the module to be
   * @param kDriveMode changes between velocity mode and dutyCycle mode
   */
  public void driveOneModule(int moduleNumber,double moveSpeed, double rotatePos, kDriveMode mode){
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
    if (mode == kDriveMode.percentOutput){ 
      swerveModules[moduleNumber].setDriveMotor(moveSpeed);
    }
    else {
      swerveModules[moduleNumber].setDriveSpeed(moveSpeed);
    }
    
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
    double[][] mathVector = new double[4][2];

    //creates an output array for angles and normalizing factors for each module
    double[][] outputArray = new double[4][2];

    //will want to know the longest vector, the furthest away a module is from center circle
    double maxLength = 0.0;

    //for every module, find the vector from the module to the arc's center
    for(int i=0; i<4; i++){
      //add the module position vector to the arc vector
      mathVector[i][0] = arcVector[0]-Constants.MODULE_VECTORS[i][0];
      mathVector[i][1] = arcVector[1]-Constants.MODULE_VECTORS[i][1];

      //find the length of the vector from this module to the arc center
      outputArray[i][1] = Math.sqrt( (mathVector[i][0]*mathVector[i][0])
        + (mathVector[i][0]*mathVector[i][0]) );
      
      //find the longest vector, from module to circle center
      if(outputArray[i][1] > maxLength){
        maxLength = outputArray[i][1];
      }

      //turn the vector from this module to the arc center to a unit vector
      mathVector[i][0]/=outputArray[i][1];
      mathVector[i][1]/=outputArray[i][1];

      //in order to drive an arc, we need angles that are rotated 90 degrees, hence x for y and -y for x
      outputArray[i][0] = Math.atan2(mathVector[i][0],-1*mathVector[i][1]);
    }

    //normalize the max length of the Vectors, this will let us scale drive speed.
    for(int i=0; i<4; i++){
      outputArray[i][1]/=maxLength;
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
    else {
      if (speed < -Constants.MAXIMUM_VELOCITY){  
        speed = -Constants.MAXIMUM_VELOCITY;
      }
      else if (speed > Constants.MAXIMUM_VELOCITY){
        speed = Constants.MAXIMUM_VELOCITY;
      }
      //return if speed below Constants.Minimum_speed in velocity
      else if (Math.abs(speed) < Constants.MINIMUM_DRIVE_SPEED){
        System.out.println("Drive speed too slow for robot to move! " + speed);
        return;
      }
    }
    //set each module to the angle in arcArray(the first value)
    for (int i=0; i<4; i++){
      swerveModules[i].setPosInRad(arcArray[i][0]); 
    }

    double[] curAngles = new double[4]; // pull the current angles of the modules
    for (int i=0; i<4; i++){
      curAngles[i] = swerveModules[i].getPosInRad();
    }

    //set the speed of each motor to the speed multiplied by the second value of arcArray
    //Reduce power to motors until they align with the target angle(might remove later)
    for(int i=0 ; i<4 ; i++){
      if (mode == kDriveMode.percentOutput){ 
        swerveModules[i].setDriveMotor(speed*Math.cos(arcArray[i][0]-curAngles[i])*arcArray[i][1]);
      }
      else {
        swerveModules[i].setDriveSpeed(speed*Math.cos(arcArray[i][0]-curAngles[i])*arcArray[i][1]);
      }
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
    }
    else {
      if (speed < -Constants.MAXIMUM_VELOCITY){  
        speed = -Constants.MAXIMUM_VELOCITY;
      }
      else if (speed > Constants.MAXIMUM_VELOCITY){
        speed = Constants.MAXIMUM_VELOCITY;
      }
      //return if speed below Constants.Minimum_speed in velocity
      else if (Math.abs(speed) < Constants.MINIMUM_DRIVE_SPEED){
        System.out.println("Drive speed too slow for robot to move! " + speed);
        return;
      }
    }

    if (angle < -Math.PI){
      angle = -Math.PI;
    }
    else if (angle > Math.PI){
      angle = Math.PI;
    }

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
    //set all modules to targetMotorSpeeds, use setDriveMotor if in percentOutput, and setDriveSpeed if in velocity
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

  /**
   * Stops all module motion, then lets all the modules spin freely.
   */
  public void stopAllModules(){
    for (int i=0; i<4; i++){
      swerveModules[i].stopAll();
    }
  }

  /**
   * A function that allows the user to reset the gyro, this 
   * makes the current orientation of the robot 0 degrees on 
   * the gyro.
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
    //return a newly constructed Rotation2d object, it takes the angle in radians as a constructor argument
    return new Rotation2d(getGyroInRad());
    //note that counterclockwise rotation is positive
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in radians
   */
  public double getGyroInRad(){
    return Math.toRadians(getGyroInDeg());// Pull the gyro in degrees, convert and return in radians
    //note that counterclockwise rotation is positive
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in degrees
   */
  public double getGyroInDeg(){
    return imu.getAngle();//getYawAxis();//*-1;//Pull gyro in degrees
    //note counterclockwise rotation is positive
  }

  /**
   * this will only work if periodic function is running in the subsystem
   * @return the position of the robot with respect to the field, as a Pose2d object
   */
  public Pose2d getCurrentPose(){
    return this.currentPosition;
  }

  public Pose2d getCurrentVelocity(){
    return this.currentVelocity;
  }

  public double[] getAllAbsModuleAngles(){
    double[] moduleAngles = new double[4];
    for(int i=0; i<4; i++){
      moduleAngles[i]=swerveModules[i].getAbsPosInDeg();
    }
    return moduleAngles;
  }

  public double[] getAllModuleRelEnc(){
    double[] moduleRelEnc = new double[4];
    for(int i=0; i<4; i++){
      moduleRelEnc[i]=swerveModules[i].getRelEncCount();
    }
    return moduleRelEnc;
  }

  /**
   * 
   * @return
   */
  public double[] getAllModuleDistance(){
    double[] moduleDistances = new double[4];
    for(int i=0; i<4; i++){
      moduleDistances[i]=swerveModules[i].getDriveDistance();
    }
    return moduleDistances;
  }

  /**
   *  Gets all the drive velocities.
   * 
   * @return An array of velocities.
   */
  public double[] getAllModuleVelocity(){
    double[] moduleVelocities = new double[4];
    for(int i=0; i<4; i++){
      moduleVelocities[i]=swerveModules[i].getDriveVelocity();
    }
    return moduleVelocities;
  }

  /**
   * a method to print all module positions for testing purposes
   */
  public void printAllModuleAngles(){
    //Use a for loop to and print() all modules' angles(degrees) on one line  
    System.out.print("Angle = ");
  
    for(int i=0; i<4; i++){
      System.out.print(swerveModules[i].getAbsPosInDeg()+"\t");
    }
    //make sure to newline "\n" at the end
    System.out.print("\n");
  }
  
  /**
   * Method for taking the current position of all modules,
   * and making that position the absolute zero of each 
   * modules position respectively.
   */
  public void zeroAllModulePosSensors(){
    //a for loop so cycle through all modules
    for (int i=0; i<4; i++){
      //call the zero position method
      swerveModules[i].zeroAbsPositionSensor();
    }
  }

  public void setEnableLimitedOutput(boolean value){
    isMinOutLimited = value;
  }
  
  /**  
   * method to configure all modules DriveMotor PIDF
   * these are the PIDF on the TalonFX
   */
  public void setDrivePIDF(double P, double I, double D, double F){
    for (int i=0; i<4; i++){
      swerveModules[i].setDriveMotorPIDF(P, I, D, F);
    }
  }

  /**
   * a function to allow odometry to be turned off and on
   * @param isOdometryRunning
   */
  public void setOdometryActive(boolean isOdometryRunning){
    this.isOdometryRunning = isOdometryRunning;
  }

  /**
   * sets x, y to 0; angle remains the same
   */
  public void resetCurrentPos(){
    currentPosition = new Pose2d(0, 0, getGyroRotation2d());
  }

  public void setCurrentPos(Pose2d target) {
    currentPosition = target;
  }

  /**
   * 
   * @param target an angle in radians
   * @return a value to give the rotational input, -1.0 to 1.0
   */
  public double getRobotRotationPIDOut(double target){
    double currentGyroPos = getGyroInRad();
    //Why is this back? well if we end up switching to a NavX, we'll need it
    // double posDiff =  currentGyroPos - target;
    // if ( posDiff > Math.PI) {
    //   // the distance the other way around the circle
    //   target = currentGyroPos + (Constants.TWO_PI - (posDiff));
    // }
    // else if (posDiff < -Math.PI){
    //   //if the distance to the goal is small enough, stop rotation and return
    //   target = currentGyroPos - (Constants.TWO_PI + (posDiff));
    // }
    return robotSpinController.calculate(currentGyroPos, target);
  }

  public double getLateralSpeedPIDFFOut (double targetVel, double targetAccel){
    return lateralSpeedPIDController.calculate(currentVelocity.getY() , targetVel) + lateralSpeedFeedforward.calculate(currentVelocity.getY(), targetAccel);
  }

  public double getAwaySpeedPIDFFOut (double targetVel, double targetAccel){
    return awaySpeedPIDController.calculate(currentVelocity.getX(), targetVel) + awaySpeedFeedforward.calculate(targetVel, targetAccel);
  }

  public double getAwayPositionPIDOut (double targetPos){
    return awayPosPidController.calculate(currentPosition.getX(), targetPos);
  }
  
  public double getLateralPositionPIDOut (double targetPos){
    return awayPosPidController.calculate(currentPosition.getY(), targetPos);
  }
}
