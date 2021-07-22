/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PCM_LED;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Axis;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.TreeMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Limelight extends SubsystemBase {
  PCM_LED led;
  /**
   * Creates a new Limelight. A subsystem that handles 
   * limelight interface and controls the extra LED 
   * from the PCM.
   */
  public Limelight() {
    led = new PCM_LED(0, 5);
    setLightState(1);//turn the Light off on construction
    //y = -0.104167 for crosshair positioning
  }

  public void periodic(){
    SmartDashboard.putString("Distance", ""+ getDistance());
  }

  public void setStreamMode(int Stream){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(Stream);
  }

  public void setPipeline(int Pipeline){
	  NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(Pipeline);
  }
  
  public void setCammode(int Cammode){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(Cammode);
  }

  /**
   * Set the LED light on or off, this 
   * includes the LED from the PCM
   * @param LightState 1 for off, 3 for On
   */
  public void setLightState(int LightState){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(LightState);  //controls if limelight is on or not // 3 is on, 1 is off
    led.set(3 == LightState);
  }

  /**
   * True if the limelight sees a target
   * @return
   */
  public boolean haveTarget(){
    return (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1); //returns true if it detects a target
  }

  /**
   * horizontal angle to the target.
   * (+ is left of target, - is right of target)
   * @return
   */
  public double angleToTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); //returns the angle offset (+ is left of target, - is right of target)
  }

  public double verticalAngleToTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); //returns the vertical angle offset
  }

  /**
   * Get the speed needed for the distance 
   * that the robot is from the target.
   * 
   * @return speed in units per 100ms
   */
  public static int calcHoodRPM(){
    double rpm;
    double distance = getDistance();
    rpm = calcSpeed(distance/12);
      
  
    rpm *= Constants.RPM_MUL_FACTOR;
    rpm += Constants.RPM_ADD_FACTOR; 
    SmartDashboard.putString("LimeLight pre-min RPM", rpm + "");
    if (rpm > 0 ){
      rpm = Math.min(rpm, 20000); 
    }
    // double rpm = 0.0;
    SmartDashboard.putString("LimeLight RPM", rpm + "");
    return (int) rpm;
  }

  /**
   * Calculate the speed based on a distance in feet.
   * numbers greater than 22.5 feet and numbers less 
   * than 4.5 feet will just very large values as this 
   * is not in the shooting range.
   * 
   * @param distance distance read from the target in feet.
   */ 
  public static double calcSpeed(double distance){
    TreeMap<Double,Double> lookUp;
    //adding .5 to account for the distance between the front wall and 3 point hole
    distance += .5;
    if(distance < 5 || distance > 23){
      return 16000;
    } else if(distance < 8){
      lookUp = Constants.SHOOTER_HOOD_DOWN_LOOKUP_TABLE;
    }
    else{
      lookUp = Constants.SHOOTER_LOOKUP_TABLE;
    }
    double lowSpeed;
    double highSpeed;
    try{
      lowSpeed = lookUp.get(Math.floor(distance));
      highSpeed =  lookUp.get(Math.ceil(distance));
    }
    catch(Exception ex){
      return 17500;
    }
    double decimal = distance % 1;
    return ((highSpeed - lowSpeed) * decimal) + lowSpeed;
  }

  /**
   * return the distance from the target as 
   * currently read by the limelight. this 
   * distance is in inches.
   * 
   * @return distance in inches(a double)
   */
  public static double getDistance(){
    //all distance values are in inches
    double cameraHeight =  25.5;    //not final value
    double targetHeight = 98; //final value = 98
    double cameraAngle = 18;    //changeable, need to recallibrate
    double distance = ((targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle + RobotContainer.limelight.verticalAngleToTarget())))-8.5; // Returns distance to target, 12.5 is distance camera is from front? of robot
    // System.out.println("Math = " + Math.tan( Math.toRadians(cameraAngle + RobotContainer.limelight.verticalAngleToTarget()) ) +"   distance = " + Distance);
    // System.out.println(Distance);
    return distance;
    //d = (h2-h1) / tan(a1+a2)
    //h2 = height of target above floor
    //h1 = height of camera above floorh 
    //a1 = angle of camera (angle from line parallel to floor)
    //a2 = angle to target (vertical)
    //d = distance to target
  }



}
