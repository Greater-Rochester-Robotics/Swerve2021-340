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

import edu.wpi.first.wpilibj.geometry.Rotation2d;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANAnalog;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

import frc.robot.Constants;

/**
 * This is the class containing both motor controllers, the CANCodder and all
 * functions needed to run one swerve module. It has a periodic method that must
 * be called periodically for it to work.
 */
public class SwerveModule {
    private TalonFX driveMotor;
    private CANSparkMax rotationMotor;
    // The rotateAbsSensor is an absolute position sensor ranging from -180 to 180,
    // We'll use it to tell where we are and where we want to be, but due to the
    // dicontinuity
    // from -180 to 180, it can't be used for motion. We use the rotateRelEncoder
    // for that.
    private CANCoder rotateAbsSensor;
    // The rotateRelEncoder is a built in relative position sensor on the sparkMax
    // motor.
    // It can't tell us our angle (like the rotateAbsSensor), but it does not have
    // the same discontinuity issue.
    // So when we know how far we want to move, we can use the rotateRelEncoder to
    // track our progress as we move.
    private CANEncoder rotateRelEncoder;
    // private CANAnalog rotationSensor;//switch to the CANCoder requires not doing
    // this
    private CANPIDController rotatePID;
    private boolean isInverted = false;// this is for a future function
    // these are for the periodic call thread
    private double currentDegAngle = 0.0;
    private volatile double prevAngle = 0.0;
    private volatile double currentPosition = 0.0;
    private volatile double prevPosition = 0.0;
    private Rotation2d currentRotPos = new Rotation2d();
    private volatile double[] positionArray = new double[] { 0.0, 0.0 };

    /**
     * Creates a new SwerveModule object
     * 
     * @param driveMotorID    The CAN ID of the SparkMax connected to the drive
     *                        motor(expecting NEO)
     * @param rotationMotorID The CAN ID of the SparkMax connected to the module
     *                        rotation motor(expecting NEO 550)
     * @param canCoderID      The CAN ID of the rotation sensor
     */
    public SwerveModule(int driveMotorID, int rotationMotorID, int canCoderID) {
        driveMotor = new TalonFX(driveMotorID);
        driveMotor.configSelectedFeedbackCoefficient(Constants.DRIVE_ENC_TO_METERS_FACTOR);
        // Use configSelectedFeedbackCoefficient(), to scale the driveMotor to real
        // distance, DRIVE_ENC_TO_METERS_FACTOR
         driveMotor.setInverted(false);// Set motor inverted(set to true)
        // TODO: setup the PID on the TalonFX for velocity control

        rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();// reset the motor controller, wipe old stuff

        rotationMotor.setIdleMode(IdleMode.kBrake);// set rotationMotor brake mode, so motors stop on a dime
        rotationMotor.enableVoltageCompensation(12);// enable voltage compensation mode 12V for the rotation motor
        rotationMotor.setSmartCurrentLimit(40);// Set smartCurrentLimit for the rotationMotor maybe 40A?
        rotationMotor.setInverted(true);

        rotateAbsSensor = new CANCoder(canCoderID);
        rotateAbsSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        // rotationSensor =
        // rotationMotor.getAnalog(CANAnalog.AnalogMode.kAbsolute);//switch to the
        // CANCoder requires not doing this
        // rotationSensor.setPositionConversionFactor(Constants.VOLTAGE_TO_RAD_CONV_FACTOR);//switch
        // to the CANCoder requires not doing this

        rotateRelEncoder = rotationMotor.getEncoder();
        rotatePID = rotationMotor.getPIDController();
        rotatePID.setFeedbackDevice(rotateRelEncoder);

        // set the PID values for the Encoder controlled rotation
        rotatePID.setP(Constants.SWERVE_ROT_P_VALUE);
        rotatePID.setI(Constants.SWERVE_ROT_I_VALUE);
        rotatePID.setD(Constants.SWERVE_ROT_D_VALUE);
        rotatePID.setIZone(Constants.SWERVE_ROT_I_ZONE_VALUE);
        rotatePID.setFF(Constants.SWERVE_ROT_FF_VALUE);
        
        // use setOutput on the rotatePID(this will make sure we don't stall the motor, or give too much power)
        rotatePID.setOutputRange(Constants.SWERVE_ROT_PID_VOLTAGE_MINIMUM, Constants.SWERVE_ROT_PID_VOLTAGE_MAXIMUM);
    }

    /**
     * This method is used to pull and compute the currentAngle so the sensor is
     * called once, additionally position of the module is computed here.
     */
    public double[] periodic() {
        // pull the current positon from the absolute rotation sesnors
        this.currentDegAngle = rotateAbsSensor.getAbsolutePosition();
        this.currentRotPos = new Rotation2d(Math.toRadians(this.currentDegAngle));
        // this.currentAngle = Math.toRadians(rotateAbsSensor.getAbsolutePosition());

        // average the angle between this cycle and the previous
        double averAngle = (this.currentRotPos.getRadians() + this.prevAngle) / 2;
        // double averAngle = (this.currentAngle + this.prevAngle)/2;

        // pull the distance travelled by the driveMotor
        this.currentPosition = driveMotor.getSensorCollection().getIntegratedSensorPosition();
        // find the distance travelled since the previous cycle
        double deltaPos = this.currentPosition - this.prevPosition;

        // add the distance travelled, in each the X and Y, to the total distance for
        // this module
        double[] deltaPositionArray = new double[] { deltaPos * this.currentRotPos.getCos(),
                deltaPos * this.currentRotPos.getSin() };

        // store the current distance and angle for the next cycle
        this.prevAngle = this.currentRotPos.getRadians();
        this.prevPosition = this.currentPosition;
        return deltaPositionArray;
    }

    /**
     * This method returns the array of distance traveled by the module in X and Y
     * 
     * @return distance in X(0) and Y(1), units based on
     */
    public double[] getModulePosition() {
        return this.positionArray;
    }

    /**
     * Resets the positional array to [0.0, 0.0]
     */
    public void resetPosition() {
        
        
        setRotateAbsSensor(this.rotateAbsSensor.configGetMagnetOffset()-getAbsPosInDeg());
        
    }

    

    /**
     * Set the speed of the drive motor in percent duty cycle
     * 
     * @param dutyCycle a number between -1.0 and 1.0, where 0.0 is not moving, as
     *                  percent duty cycle
     */
    public void setDriveMotor(double dutyCycle) {
        driveMotor.set(TalonFXControlMode.PercentOutput, dutyCycle * (isInverted ? -1 : 1));
    }

    /**
     * Set the speed of the drive motor in meter per second, this relies on the
     * PIDController built into the TalonFX.
     * 
     * @param speed a speed in meters per second
     */
    public void setDriveSpeed(double speed) {
        driveMotor.set(TalonFXControlMode.Velocity, speed * (isInverted ? -1 : 1));
    }

    /**
     * @return the distance the drive wheel has traveled
     */
    public double getDriveDistance() {
        // return this.currentPosition;//if we use the periodic call thread/method use
        // this instead(might change to volatile)
        return driveMotor.getSensorCollection().getIntegratedSensorPosition();
    }

    /**
     * 
     * @return speed of the drive wheel
     */
    public double getDriveVelocity() {
        return driveMotor.getSensorCollection().getIntegratedSensorVelocity();
    }

    /**
     * A method to set the position of the drive encoder to zero essentially
     * resetting it.
     */
    public void resetDriveMotorEncoder() {
        driveMotor.setSelectedSensorPosition(0.0);// this code sets the Drive position to 0.0
    }

    /**
     * The CANCoder has a mechanical zero point, this is hard to move, so this
     * methood is used to set the offset of the CANCoder so we can dictate the zero
     * position.
     * 
     * @param value a number between -180 and 180, where 0 is straight ahead
     */
    public void setRotateAbsSensor(double value) {
        rotateAbsSensor.configMagnetOffset(value, 0);
    }

    /**
     * @return the position of the module in degrees, should limit from -180 to 180
     */
    public double getAbsPosInDeg() {
        return rotateAbsSensor.getAbsolutePosition();
        // TODO:Above has to be checked, if the sensor is positive clockwise, fix(Need
        // Robot)
    }

    /**
     * this is a function meant for testing by getting the count from the rotational
     * encoder which is internal to the NEO550.
     * 
     * @return the encoder count(no units, naturally just the count)
     */
    public double getRelEncCount() {
        return rotateRelEncoder.getPosition();
    }

    /**
     * If this is too resource intensive, switch to a periodic call, and replace
     * with a poll of said variable
     * 
     * @return the position of the module in radians, should limit from -PI to PI
     */
    public double getPosInRad() {
        // return this.currentRotPos.getRadians();//if we use the periodic call
        // thread/method use this instead
        return Math.toRadians(getAbsPosInDeg());// (isInverted?0:Math.PI));
        // double currentAngleRad = this.currentRotPos.getRadians();
        // //the following is a single line return, used with invertable drive
        // return isInverted?
        // ((currentAngleRad <= 0.0)?currentAngleRad-180:-180+currentAngleRad):
        // (currentAngleRad);
        // //the following is an if statement set used with invertable drive
        // if(isInverted){
        // if(currentAngle <= Math.PI){
        // return currentAngle;
        // }else{
        // return Constants.TWO_PI-currentAngle;
        // }
        // }else{
        // return rotationSensor.getPosition() - Math.PI;
        // }
    }

    /**
     * set the setpoint for the module rotation
     * 
     * @param targetPos a value between -PI and PI, PI is counter-clockwise, 0.0 is
     *                  forward
     */
    public void setPosInRad(double targetPos) {
        double posDiff = targetPos - getPosInRad();
        double absDiff = Math.abs(posDiff);
        // System.out.println("targetPos = " + targetPos);
        // if the distance is more than a half circle,we going the wrong way, fix
        if (absDiff > Math.PI) {
            // the distance the other way around the circle
            posDiff = posDiff - (Constants.TWO_PI * Math.signum(posDiff));
        }
        else if (absDiff < Constants.SWERVE_MODULE_TOLERANCE){
            rotatePID.setReference(0.0, ControlType.kDutyCycle);
            return;
        }
        // //This is for inverting the motor if target angle is 90-270 degrees away,
        // invert
        // //To fix going the wrong way around the circle, distance is larger than 270
        // if(absDiff >= Constants.THREE_PI_OVER_TWO){
        // //the distance the other way around the circle
        // posDiff = posDiff - (Constants.TWO_PI*Math.signum(posDiff));
        // //if between 90 and 270 invert the motor
        // }else if(absDiff < Constants.THREE_PI_OVER_TWO && absDiff >
        // Constants.PI_OVER_TWO){
        // //switch the motor inversion
        // isInverted = !isInverted;
        // //Since inverted, recompute everything
        // posDiff = targetPos - getPosInRad();
        // absDiff = Math.abs(posDiff);
        // if(absDiff > Constants.THREE_PI_OVER_TWO){
        // //the distance the other way around the circle
        // posDiff = posDiff - (Constants.TWO_PI*Math.signum(posDiff));
        // }
        // }

        // Convert the shortest distance to encoder value(use convertion factor)
        double targetEncDistance = posDiff * Constants.RAD_TO_ENC_CONV_FACTOR;
        // add the encoder distance to the current encoder count
        double outputEncValue = targetEncDistance + rotateRelEncoder.getPosition();

        // System.out.println("Target Encoder Distance, targetPos, outputEncValue" + targetEncDistance + " " + targetPos + " " + outputEncValue);
        // Set the setpoint using setReference on the PIDController
        rotatePID.setReference(outputEncValue, ControlType.kPosition);
    }
    public void driveRotateMotor(double speed) {
        this.rotationMotor.set(speed);
    }
    /**
     * This method is used to stop the module completely. The drive motor is
     * switched to percent voltage and and output of 0.0 percent volts. The rotation
     * motor's PIDController is set to DutyCyclevoltage control mode, and output of
     * 0.0% output
     */
    public void stopAll() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        rotatePID.setReference(0.0, ControlType.kDutyCycle);
    }

    
}
