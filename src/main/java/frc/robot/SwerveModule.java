package frc.robot;

import java.util.concurrent.ThreadPoolExecutor.AbortPolicy;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwervePIDConstants;


//util class, daniel sabalakov, creates WCP SwerveXS Module
public class SwerveModule extends SubsystemBase{
    public static final double kWheelRadius = 0.0379; //convert m
    public static final int kEncoderResolution = 4096;
    //gear reduction real????? idk but I didn't use it yet
    public static final double kGearReduction = 0.5;
    //IMPORTANT: value of each encoder tick converted to radians.
    public static final double encoderToRadians = (2*Math.PI)/(kEncoderResolution);
    //max speed 1 m/s, max angular momentum speed is 2pi rads/sec (360deg/sec). 
    public static final double kMaxSpeed = 1.0; // 2 meters per second should be fine to work with 
    public static final double kMaxAngularSpeed = 2*Math.PI; //1 rotation per second

    //declare objects: 
    //Drive motor: powerController, powerConfigurer, powerEncoder, powerPIDController
    //Steering controller: steeringController
    public SparkMax powerController;
    public SparkMaxConfig powerConfigurer;
    public WPI_TalonSRX steeringController;
    public RelativeEncoder powerEncoder;
    public SparkClosedLoopController powerPIDController;
    // public Translation2d moduleLocation;

    //i dont believe feed forward is ever used
    private SimpleMotorFeedforward feedforward;

    /**
     * @param powerID needs powerID (configured in rev hardware client)
     * @param steeringID needs steerinID (configured in tunerX or phoenixtuner)
     * @param powerInvert invert the power wheel?
     * @param absoluteChange absoluteChange is used to set the "0" point: it reads the absolute value of the encoder
     */
    public SwerveModule(int powerID, int steeringID,boolean powerInvert, double absoluteChange){
        /*
         * PowerController:
         * -> SparkMax (set power)
         * -> SparkMaxConfig (configure stuff such as PID and current limits)
         * -> SparkMaxEncoder (get position of power)
         */

        //create objects
        powerController = new SparkMax(powerID,MotorType.kBrushless);
        powerConfigurer = new SparkMaxConfig();
        
        //set inverted and idlemode
        powerConfigurer.inverted(powerInvert);
        powerConfigurer.idleMode(IdleMode.kBrake);

        //position & velocity conversion factors
        powerConfigurer.encoder.positionConversionFactor(2*Math.PI*kWheelRadius/(42*kGearReduction));
        powerConfigurer.encoder.velocityConversionFactor(2*Math.PI*kWheelRadius/(42*kGearReduction*60));

        //current limit !important
        powerConfigurer.smartCurrentLimit(40);
        //configure PID 
        powerConfigurer.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        powerConfigurer.closedLoop.pid(0.2, 0, 0.002);
        
        //get pid controller 
        powerPIDController = powerController.getClosedLoopController();
        
        //configure the motor !important
        powerController.configure(powerConfigurer, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        
        //power Encoder
        powerEncoder = powerController.getEncoder();
        powerEncoder.setPosition(0);
        
        //feed forward controller
        //CHANGE FOR EACH ROBOT
        //not used in 2025
        feedforward = new SimpleMotorFeedforward(Constants.KsKvKaConstants.ks,Constants.KsKvKaConstants.kv,Constants.KsKvKaConstants.ka);

        /*
         * WPI_TalonSRX
         * -> WPI_TalonSRX is used for all motor control
         */

        //create object for steering controller
        steeringController = new WPI_TalonSRX(steeringID);
        steeringController.configFactoryDefault();
        //sets the configured sensor to various predetermined constants
        steeringController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        //ensure sensor is positive when output is positive
        steeringController.setSensorPhase(SwervePIDConstants.kSensorPhase);
        //sets the steeringController to inverted if the motor tells it to do so.
        steeringController.setInverted(SwervePIDConstants.kMotorInvert);
        //set peak and nominal outputs
        steeringController.configNominalOutputForward(0,SwervePIDConstants.kTimeoutMs);
        steeringController.configNominalOutputReverse(0,SwervePIDConstants.kTimeoutMs);
        steeringController.configPeakOutputForward(1,SwervePIDConstants.kTimeoutMs);
        steeringController.configPeakOutputReverse(-1,SwervePIDConstants.kTimeoutMs);
        //configures position closed loop in slot0
        steeringController.config_kF(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kF,SwervePIDConstants.kTimeoutMs);
        steeringController.config_kP(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kP,SwervePIDConstants.kTimeoutMs);
        steeringController.config_kI(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kI,SwervePIDConstants.kTimeoutMs);
        steeringController.config_kD(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kD,SwervePIDConstants.kTimeoutMs);

        // steeringController.setDistancePerPulse();
        steeringController.configSelectedFeedbackCoefficient(1/kEncoderResolution,0,0);
        
        //sets the relative sensor to match absolute
        int absolutePosition = steeringController.getSensorCollection().getPulseWidthPosition() % 4096; // Mask out to 12-bit value
        steeringController.setSelectedSensorPosition(absolutePosition - absoluteChange, SwervePIDConstants.kPIDLoopIdx, SwervePIDConstants.kTimeoutMs);
        
        //used when aligning wheels manually
        System.out.println(powerID + " " + absolutePosition);


        
    }

    //set gains for powerController. 
    public void setVelocityGains(double kp, double ki, double kd, double ks, double kv, double ka){
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        powerConfigurer.closedLoop.pid(kp,ki,kd);
        powerController.configure(powerConfigurer,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //azimuth is used to set the direction. debug method
    /**
     * 
     * @param azimuth -> Rotation2d used for direction of wheel
     */
    public void setAzimuth(Rotation2d azimuth) {
        steeringController.set(ControlMode.Position,azimuth.getDegrees());
    }

    /**
     * sets PID of steering controller
     * @param kp
     * @param ki
     * @param kd
     */
    public void setAzimuthGains(double kp, double ki, double kd){
        steeringController.config_kP(SwervePIDConstants.kPIDLoopIdx,kp,SwervePIDConstants.kTimeoutMs);
        steeringController.config_kI(SwervePIDConstants.kPIDLoopIdx,ki,SwervePIDConstants.kTimeoutMs);
        steeringController.config_kD(SwervePIDConstants.kPIDLoopIdx,kd,SwervePIDConstants.kTimeoutMs);
        
    }
    /**
     * resets encoder position of the steering motor to 0
     */
    public void resetSteerPosition(){
        steeringController.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
    }
    /**
     *sets voltage of the DRIVE motor
     * @param volts -> target volts
     */
    public void setDriveVolts(double volts){
        powerController.setVoltage(volts);
    }
    /**
     * returns volts
     * @return volts on the output
     */
    public double getDriveVolts(){
        return powerController.getAppliedOutput() * powerController.getBusVoltage();
    }
    /**
     * gets current
     * @return current of the power controller
     */
    public double getDriveCurrent(){
        return powerController.getAppliedOutput() * powerController.getBusVoltage();
    }

    /**
     * ARGUABLY THE MOST IMPORTANT METHOD
     * sets targeted state of the swerve module wheels
     * @param desiredState SwerveModuleState calculated in different class
     * @param isOpenLoop if open loop
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        //gets optimized state
        desiredState = CTREModuleState.optimize(desiredState, getAngle());
        //if is open loop then set percent output, else set velocity
        if (isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / kMaxSpeed;
            powerController.set(percentOutput);
        }
        else {
            double velocity = desiredState.speedMetersPerSecond;
            powerPIDController.setReference(velocity, ControlType.kVelocity,ClosedLoopSlot.kSlot0,feedforward.calculate(velocity));
        }
        steeringController.set(ControlMode.Position, desiredState.angle.getDegrees() * 4096 / 360);
    }
    /**
     * gets module state (velocity, angle)
     * @return state of module
     */
    public SwerveModuleState getState(){
        double velocity;
        Rotation2d azmimuth;
        velocity = powerEncoder.getVelocity();
        azmimuth = Rotation2d.fromRadians(steeringController.getSelectedSensorPosition() * encoderToRadians);
        return new SwerveModuleState(velocity, azmimuth);
    }
    
    //gets angle
    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees((steeringController.getSelectedSensorPosition() * 360.0 / 4096.0));
    }
    //get turning heading
    public double getTurningHeading() {
        double heading = steeringController.getSelectedSensorPosition() * encoderToRadians;
        heading %= 2 * Math.PI;
        return heading;
    }
    //gets position
    public SwerveModulePosition getPosition(){
        double position;
        Rotation2d azmimuth;
        position = powerEncoder.getPosition();
        azmimuth = Rotation2d.fromRadians(steeringController.getSelectedSensorPosition() * encoderToRadians);
        return new SwerveModulePosition(position, azmimuth);
    }
    //turns module
    public void turnModule(double speed){
        steeringController.set(speed);
    }

}
