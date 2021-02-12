/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANAnalog;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

import frc.robot.Constants;

/**
 * This is the class containing both motor controllers and 
 * all functions needed to run one swerve module.
 */
public class SwerveModule{
    private TalonFX driveMotor;
    private CANSparkMax rotationMotor;
    // The rotateSensor is an absolute position sensor ranging from -180 to 180,
    // We'll use it to tell where we are and where we want to be, but due to the dicontinuity
    // from -180 to 180, it can't be used for motion. We use the rotationEncoder for that.
    private CANCoder rotateSensor;
    // The rotationEncoder is a built in relative position sensor on the sparkMax motor.
    // It can't tell us our angle (like the rotateSensor), but it does not have the same discontinuity issue.
    // So when we know how far we want to move, we can use the rotationEncoder to track our progress as we move.
    private CANEncoder rotationEncoder;
    // private CANAnalog rotationSensor;//switch to the CANCoder requires not doing this
    private CANPIDController rotatePID;
    private boolean isInverted = false;//this is for a future function
    //these are for the periodic call thread
    private volatile double currentAngle = 0.0;
    private volatile double prevAngle = 0.0;
    private volatile double currentPosition = 0.0;
    private volatile double prevPosition = 0.0;
    private volatile double[] positionArray = new double[]{0.0,0.0};

    /**
     * Creates a new SwerveModule object
     * 
     * @param driveMotorID The CAN ID of the SparkMax connected to the drive motor(expecting NEO)
     * @param rotationMotorID The CAN ID of the SparkMax connected to the module rotation motor(expecting NEO 550)
     * @param canCoderID The CAN ID of the rotation sensor 
     */  
    public SwerveModule(int driveMotorID,int rotationMotorID,int canCoderID){
        driveMotor = new TalonFX(driveMotorID);
        driveMotor.configSelectedFeedbackCoefficient(Constants.DRIVE_ENC_TO_METERS_FACTOR);
        //Use configSelectedFeedbackCoefficient(), to scale the driveMotor to real distance, DRIVE_ENC_TO_METERS_FACTOR
        
        rotationMotor = new CANSparkMax(rotationMotorID , MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();//reset the motor controller, wipe old stuff
        
        rotationMotor.setIdleMode(IdleMode.kBrake);//set rotationMotor brake mode, so motors stop on a dime
        rotationMotor.enableVoltageCompensation(12);//enable voltage compensation mode 12V for the rotation motor
        rotationMotor.setSmartCurrentLimit(40);//Set smartCurrentLimit for the rotationMotor maybe 40A?
        driveMotor.setInverted(true);//Set motor inverted(set to true)

        rotateSensor = new CANCoder(canCoderID);
        rotateSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        // rotationSensor = rotationMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);//switch to the CANCoder requires not doing this
        // rotationSensor.setPositionConversionFactor(Constants.VOLTAGE_TO_RAD_CONV_FACTOR);//switch to the CANCoder requires not doing this

        rotationEncoder = rotationMotor.getEncoder();
        rotatePID = rotationMotor.getPIDController();
        rotatePID.setFeedbackDevice(rotationEncoder);

        //set the PID values for the Encoder controlled rotation
        rotatePID.setP(Constants.SWERVE_ROT_P_VALUE);
        rotatePID.setI(Constants.SWERVE_ROT_I_VALUE);
        rotatePID.setD(Constants.SWERVE_ROT_D_VALUE);
        rotatePID.setIZone(Constants.SWERVE_ROT_I_ZONE_VALUE);
        rotatePID.setFF(Constants.SWERVE_ROT_FF_VALUE);
        
        rotatePID.setOutputRange(Constants.SWERVE_ROT_PID_VOLTAGE_MINIMUM,Constants.SWERVE_ROT_PID_VOLTAGE_MAXIMUM);//use setOutput on the rotatePID(this will make sure we don't stall the motor, or give too much power)
    }

    /**
     * This method is used to pull and compute the currentAngle
     *  so the sensor is called once, additionally position of
     *  the module is computed here.
     */
    protected synchronized void periodicThread(){
        //this.currentDegAngle = rotateSensor.getAbsolutePosition();
        //this.currentAngle = Math.toRadians(this.currentDegAngle);
        this.currentAngle = Math.toRadians(rotateSensor.getAbsolutePosition());
        //average the angle between this cycle and the previous
        double averAngle = (this.currentAngle + this.prevAngle)/2;
        //pull the distance travelled by the driveMotor
        this.currentPosition = driveMotor.getSensorCollection().getIntegratedSensorPosition();
        //find the distance travelled since the previous cycle
        double deltaPos = this.currentPosition - this.prevPosition;
        
        //add the distance travelled, in each the X and Y, too the total distance for this module
        this.positionArray[0]+= Math.cos(averAngle)*deltaPos;
        this.positionArray[1]+= Math.sin(averAngle)*deltaPos;

        //store the current distance and angle for the next cycle
        this.prevAngle = this.currentAngle;
        this.prevPosition = this.currentPosition;
    }

    /**
     * This method returns the array of distance traveled by the module in X and Y
     * @return distance in X(0) and Y(1), units based on 
     */
    public double[] getModulePosition(){
        return this.positionArray;
    }
    
    /**
     * Resets the positional array to [0.0, 0.0]
     */
    public void resetPositionArray(){
        this.resetPositionArray(new double[]{0.0,0.0});
    }

    /**
     * Resets the positional array to the input posArray
     * @param posArray a two value array where X is the first value and Y is the second
     */
    public void resetPositionArray(double[] posArray){
        this.positionArray = posArray;
    }

    /**
     * Set the speed of the drive motor
     * 
     * @param value a number between -1.0 and 1.0, where 0.0 is not moving
     */
    public void setDriveMotor(double value){
        driveMotor.set(TalonFXControlMode.PercentOutput,value*(isInverted?-1:0));
    }

    /**
     * @return the distance the drive wheel has traveled
     */
    public double getDriveDistance(){
        // return this.currentPosition;//if we use the periodic call thread/method use this instead(might change to volatile)
        return driveMotor.getSensorCollection().getIntegratedSensorPosition();
    }

    /**
     * 
     * @return speed of the drive wheel
     */
    public double getDriveVelocity(){
        return driveMotor.getSensorCollection().getIntegratedSensorVelocity();
    }

    /**
     * A method to set the position of the drive encoder to zero
     * essentially resetting it.
     */
    public void resetDriveMotorEncoder(){
        
        driveMotor.setSelectedSensorPosition(0.0);//this code sets the motor speed to 0.0
    }

    
    /**
     * The CANCoder has a mechanical zero point, this is hard to move,
     *  so this methood is used to set the offset of the CANCoder so
     *  we can dictate the zero position.
     * 
     * @param value a number between -180 and 180, where 0 is straight ahead
     */
    public void setRotateSensor(double value){
        rotateSensor.configMagnetOffset(value, 0);
    }

    /**
     * @return the position of the module in degrees, should limit from -180 to 180
     */
    public double getPosInDeg(){ 
        return rotateSensor.getAbsolutePosition();
    }

    /**
     * this is a function meant for testing by getting the count from
     *  the rotational encoder which is internal to the NEO550.
     * 
     * @return the encoder count(no units, naturally just the count)
     */
    public double getEncCount(){
        return rotationEncoder.getPosition();
    }

    /**
     * If this is too resource intensive, switch to a periodic call,
     *  and replace with a poll of said variable
     * 
     * @return the position of the module in radians, should limit from -PI to PI
     */
    public double getPosInRad(){
        // return this.currentPosition;//if we use the periodic call thread/method use this instead
        return getPosInDeg()*Constants.DEG_TO_RAD_CONV_FACTOR;//(isInverted?0:Math.PI));
        //TODO:Above has to be checked, if the sensor is positive clockwise, fix(Need Robot)
        // double currentAngleRad = Math.toRadians(getPosInDeg());
        // //the following is a single line return, used with invertable drive
        // return isInverted?
        //     ((currentAngleRad <= 0.0)?currentAngleRad-180:-180+currentAngleRad):
        //     (currentAngleRad);
        // //the following is an if statement set used with invertable drive
        // if(isInverted){
        //     if(currentAngle <= Math.PI){
        //         return currentAngle;
        //     }else{
        //         return Constants.TWO_PI-currentAngle;
        //     }
        // }else{
        //     return rotationSensor.getPosition() - Math.PI;
        // }
    }

    /**
     * set the setpoint for the module rotation
     * 
     * @param targetPos a value between -PI and PI, PI is counter-clockwise, 0.0 is forward 
     */
    public void setPosInRad(double targetPos){
        double posDiff = targetPos - getPosInRad();
        double absDiff = Math.abs(posDiff);

        //if the distance is more than a half circle,we going the wrong way, fix
        if(absDiff > Math.PI){
            //the distance the other way around the circle
            posDiff = posDiff - (Constants.TWO_PI*Math.signum(posDiff));
        }

        // //This is for inverting the motor if target angle is 90-270 degrees away, invert 
        // //To fix going the wrong way around the circle
        // if(absDiff >= Constants.THREE_PI_OVER_TWO){
        //     //the distance the other way around the circle
        //     posDiff = posDiff - (Constants.TWO_PI*Math.signum(posDiff));
        // //if between 90 and 270 invert the motor
        // }else if(absDiff < Constants.THREE_PI_OVER_TWO && absDiff > Constants.PI_OVER_TWO){
        //     //switch the motor inversion
        //     isInverted = !isInverted;
        //     //Since inverted, recompute everything
        //     posDiff = targetPos - getPosInRad();
        //     absDiff = Math.abs(posDiff);
        //     if(absDiff > Constants.THREE_PI_OVER_TWO){
        //         //the distance the other way around the circle
        //         posDiff = posDiff - (Constants.TWO_PI*Math.signum(posDiff));
        //     }
        // }
     
        //Convert the shortest distance to encoder value(use convertion factor) 
        double targetEncDistance = posDiff*Constants.RAD_TO_ENC_CONV_FACTOR;
        //add the encoder distance to the current encoder count
        double outputEncValue = targetEncDistance + rotationEncoder.getPosition();
        
        //Set the setpoint using setReference on the PIDController
        rotatePID.setReference(outputEncValue, ControlType.kPosition);
    }

    /**
     * This method is used to stop the module completely. The
     *  drive motor is switched to percent voltage and and 
     *  output of 0.0 percent volts. The rotation motor's 
     *  PIDController is set to DutyCyclevoltage control mode,
     *  and output of 0.0% output
     */
    public void stopAll(){
        driveMotor.set(TalonFXControlMode.PercentOutput,0.0);
        rotatePID.setReference(0.0,ControlType.kDutyCycle);
    }
}
