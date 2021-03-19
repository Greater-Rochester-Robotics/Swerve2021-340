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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;

public class Limelight extends SubsystemBase {
  PCM_LED led;
  /**
   * Creates a new Limelight.
   */
  public Limelight() {
    led = new PCM_LED(0, 5);
    setLightState(1);
    //y = -0.104167 for crosshair positioning
  }

  public void periodic(){
    SmartDashboard.putString("Distance", ""+ getDistance());
    // if(DriverStation.getInstance().isTest()){
    //   SmartDashboard.putString("AngleToTarget", ""+angleToTarget());
    // }
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

  public void setLightState(int LightState){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(LightState);  //controls if limelight is on or not // 3 is on, 1 is off
    led.set(3 == LightState);
  }

  public boolean haveTarget(){
    return (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1); //returns true if it detects a target
  }

  public double angleToTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); //returns the angle offset (+ is left of target, - is right of target)
  }

  public double verticalAngleToTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); //returns the vertical angle offset
  }

  public static int calcHoodRPM(){
    double cameraHeight =  25.5;    //not final value
    double targetHeight = 98; //final value = 98
    double cameraAngle = 18;    //changeable 32.1
    double rpm;
    double distance = ((targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle + RobotContainer.limelight.verticalAngleToTarget())))-12.5;
    //Comp Bot Equation
    //Base Equation: -.000000008x^6 + .000007x^5 - .0024x^4 + .03631x^3 - 18.414x^2 - 1034.6x + 119076
    rpm = 0; // TODO: Translafe the above equation into code where x = distance
    rpm *= Constants.RPM_MUL_FACTOR;
    rpm += Constants.RPM_ADD_FACTOR; 
    if (rpm > 0 ){
      rpm = Math.min(rpm, 20000); 
    }
    // double rpm = 0.0;
    SmartDashboard.putString("LimeLight RPM", rpm + "");
    return (int) rpm;
  }

  public double getDistance(){
    //TODO: find actual values for these, and implement/test them
    //all distance values are in inches
    double cameraHeight =  24.56;    //not final value
    double targetHeight = 98.5; //final value = 98
    double cameraAngle = 18;    //changeable, need to recallibrate
    double distance = ((targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle + RobotContainer.limelight.verticalAngleToTarget())))-12.5; // Returns distance to target, 12.5 is distance camera is from front? of robot
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
