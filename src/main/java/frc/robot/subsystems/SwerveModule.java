package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwervePIDConstants;

public class SwerveModule extends SubsystemBase{
    public static final double kWheelRadius = 0.0379; //convert LATER, in m
    public static final int kEncoderResolution = 4096;
    //change later
    public static final double kGearReduction = 0.5;
    public static final double encoderToRadians = (2*Math.PI)/(kEncoderResolution);
    public static final double kMaxSpeed = 1.0; // 3 meters per second should be fine to work with 
    public static final double kMaxAngularSpeed = 2*Math.PI; // 1/2 rotation per second

    public SparkMax powerController;
    public SparkMaxConfig powerConfigurer;
    public WPI_TalonSRX steeringController;
    public RelativeEncoder powerEncoder;
    public SparkClosedLoopController powerPIDController;

    


    public SwerveModule(int powerID, int steeringID,boolean powerInvert){
        powerController = new SparkMax(powerID,MotorType.kBrushless);
        powerConfigurer = new SparkMaxConfig();

        powerConfigurer.inverted(powerInvert);
        powerConfigurer.idleMode(IdleMode.kBrake);

        powerConfigurer.encoder.positionConversionFactor(2*Math.PI*kWheelRadius/(42*kGearReduction));
        powerConfigurer.encoder.velocityConversionFactor(2*Math.PI*kWheelRadius/(42*kGearReduction*60));

        powerConfigurer.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        powerConfigurer.closedLoop.pid(0.2, 0, 0.002);
        // powerController.setInverted(powerInvert);
        powerPIDController = powerController.getClosedLoopController();
        
        powerController.configure(powerConfigurer, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        
        //power Encoder
        powerEncoder = powerController.getEncoder();
        powerEncoder.setPosition(0);
        

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
        steeringController.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);


        
    }
    public void resetSteerPosition(){
        steeringController.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
    }

    public SwerveModulePosition getPosition(){
        double distanceMeters = powerEncoder.getPosition();
        double angleRadians = steeringController.getSelectedSensorPosition() * encoderToRadians % (2*Math.PI);
        return new SwerveModulePosition(
            distanceMeters,
            new Rotation2d(angleRadians)
        );
    }
    
    public void setDesiredState(SwerveModuleState desiredState){
        double currentAngleRadians = steeringController.getSelectedSensorPosition() * encoderToRadians;
        Rotation2d currentRotation = new Rotation2d(currentAngleRadians);
        SmartDashboard.putString("GoofyState", desiredState.angle.toString());
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState,currentRotation);
        // optimizedState.optimize(desiredState,currentRotation);
        
        SmartDashboard.putNumber("DesiredState", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Optimized: ", optimizedState.angle.getDegrees());

        // double angleError = optimizedState.angle.minus(currentRotation).getRadians();
        double scaledSpeed = optimizedState.speedMetersPerSecond * optimizedState.angle.minus(currentRotation).getCos();

        double driveOutput = Math.max(-1, Math.min(1, scaledSpeed / kMaxSpeed)); // Clamp between -1 and 1
    
        powerController.set(driveOutput);
        
        double targetAngleRadians = optimizedState.angle.getRadians();
        double targetEncoderPosition = targetAngleRadians / encoderToRadians;
        // if (Math.abs(angleError) > 0.01) {
        //     steeringController.set(ControlMode.Position, targetEncoderPosition);
        // } else {
        //     steeringController.set(ControlMode.PercentOutput, 0); // Stop if error is small
        // }
        steeringController.set(ControlMode.Position, targetEncoderPosition);
    }
    public SwerveModuleState getState(){
        double speedMPS = powerEncoder.getVelocity();
        double angleRadians = steeringController.getSelectedSensorPosition() * encoderToRadians;
        return new SwerveModuleState(speedMPS,new Rotation2d(angleRadians));

    }

}
