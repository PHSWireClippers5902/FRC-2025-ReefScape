package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.SwervePIDConstants;
import frc.robot.Robot;

public class SystemIdentificationSubsystem extends SubsystemBase{
    public final SparkMax m_fl = new SparkMax(Constants.SwerveCANConstants.kFrontLeftDrivingCanId, MotorType.kBrushless);
    public final SparkMax m_fr = new SparkMax(Constants.SwerveCANConstants.kFrontRightDrivingCanId, MotorType.kBrushless);
    public final SparkMax m_bl = new SparkMax(Constants.SwerveCANConstants.kRearLeftDrivingCanId, MotorType.kBrushless);
    public final SparkMax m_br = new SparkMax(Constants.SwerveCANConstants.kRearRightDrivingCanId, MotorType.kBrushless);


    public final WPI_TalonSRX m_controlFL = new WPI_TalonSRX(Constants.SwerveCANConstants.kFrontLeftTurningCanId);
    public final WPI_TalonSRX m_controlFR = new WPI_TalonSRX(Constants.SwerveCANConstants.kFrontRightTurningCanId);
    public final WPI_TalonSRX m_controlBL = new WPI_TalonSRX(Constants.SwerveCANConstants.kRearLeftTurningCanId);
    public final WPI_TalonSRX m_controlBR = new WPI_TalonSRX(Constants.SwerveCANConstants.kRearRightTurningCanId);


    //0:fl 1:fr 2: bl 3: br
    public SparkMaxConfig[] configs = new SparkMaxConfig[4];

    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
    public XboxController testXbox;

    public final SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism(
            voltage -> {
                m_fl.setVoltage(voltage);
                m_fr.setVoltage(voltage);
                m_bl.setVoltage(voltage);
                m_br.setVoltage(voltage);
            },
            log -> {
                log.motor("drive-front-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_fl.get() * RobotController.getBatteryVoltage(),Volts))
                        .linearPosition(m_distance.mut_replace(m_fl.getEncoder().getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(m_fl.getEncoder().getVelocity(), MetersPerSecond));
                log.motor("drive-front-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        m_fr.get() * RobotController.getBatteryVoltage(),Volts))
                    .linearPosition(m_distance.mut_replace(m_fr.getEncoder().getPosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(m_fr.getEncoder().getVelocity(), MetersPerSecond));
                log.motor("drive-back-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        m_bl.get() * RobotController.getBatteryVoltage(),Volts))
                    .linearPosition(m_distance.mut_replace(m_bl.getEncoder().getPosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(m_bl.getEncoder().getVelocity(), MetersPerSecond));
                log.motor("drive-back-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        m_br.get() * RobotController.getBatteryVoltage(),Volts))
                    .linearPosition(m_distance.mut_replace(m_br.getEncoder().getPosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(m_br.getEncoder().getVelocity(), MetersPerSecond));
        
                    
            }, 
            this));;

    public SystemIdentificationSubsystem(XboxController m_xbox){
        
        m_controlFL.configFactoryDefault();
        //sets the configured sensor to various predetermined constants
        m_controlFL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        //ensure sensor is positive when output is positive
        m_controlFL.setSensorPhase(SwervePIDConstants.kSensorPhase);
        //sets the m_controlFL to inverted if the motor tells it to do so.
        m_controlFL.setInverted(SwervePIDConstants.kMotorInvert);
        //set peak and nominal outputs
        m_controlFL.configNominalOutputForward(0,SwervePIDConstants.kTimeoutMs);
        m_controlFL.configNominalOutputReverse(0,SwervePIDConstants.kTimeoutMs);
        m_controlFL.configPeakOutputForward(1,SwervePIDConstants.kTimeoutMs);
        m_controlFL.configPeakOutputReverse(-1,SwervePIDConstants.kTimeoutMs);
        //configures position closed loop in slot0
        m_controlFL.config_kF(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kF,SwervePIDConstants.kTimeoutMs);
        m_controlFL.config_kP(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kP,SwervePIDConstants.kTimeoutMs);
        m_controlFL.config_kI(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kI,SwervePIDConstants.kTimeoutMs);
        m_controlFL.config_kD(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kD,SwervePIDConstants.kTimeoutMs);

        // m_controlFL.setDistancePerPulse();
        m_controlFL.configSelectedFeedbackCoefficient(1/4096,0,0);
        
        //sets the relative sensor to match absolute
        m_controlFL.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);


        m_controlFR.configFactoryDefault();
        //sets the configured sensor to various predetermined constants
        m_controlFR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        //ensure sensor is positive when output is positive
        m_controlFR.setSensorPhase(SwervePIDConstants.kSensorPhase);
        //sets the m_controlFR to inverted if the motor tells it to do so.
        m_controlFR.setInverted(SwervePIDConstants.kMotorInvert);
        //set peak and nominal outputs
        m_controlFR.configNominalOutputForward(0,SwervePIDConstants.kTimeoutMs);
        m_controlFR.configNominalOutputReverse(0,SwervePIDConstants.kTimeoutMs);
        m_controlFR.configPeakOutputForward(1,SwervePIDConstants.kTimeoutMs);
        m_controlFR.configPeakOutputReverse(-1,SwervePIDConstants.kTimeoutMs);
        //configures position closed loop in slot0
        m_controlFR.config_kF(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kF,SwervePIDConstants.kTimeoutMs);
        m_controlFR.config_kP(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kP,SwervePIDConstants.kTimeoutMs);
        m_controlFR.config_kI(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kI,SwervePIDConstants.kTimeoutMs);
        m_controlFR.config_kD(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kD,SwervePIDConstants.kTimeoutMs);

        // m_controlFR.setDistancePerPulse();
        m_controlFR.configSelectedFeedbackCoefficient(1/4096,0,0);
        
        //sets the relative sensor to match absolute
        m_controlFR.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);


        m_controlBL.configFactoryDefault();
        //sets the configured sensor to various predetermined constants
        m_controlBL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        //ensure sensor is positive when output is positive
        m_controlBL.setSensorPhase(SwervePIDConstants.kSensorPhase);
        //sets the m_controlBL to inverted if the motor tells it to do so.
        m_controlBL.setInverted(SwervePIDConstants.kMotorInvert);
        //set peak and nominal outputs
        m_controlBL.configNominalOutputForward(0,SwervePIDConstants.kTimeoutMs);
        m_controlBL.configNominalOutputReverse(0,SwervePIDConstants.kTimeoutMs);
        m_controlBL.configPeakOutputForward(1,SwervePIDConstants.kTimeoutMs);
        m_controlBL.configPeakOutputReverse(-1,SwervePIDConstants.kTimeoutMs);
        //configures position closed loop in slot0
        m_controlBL.config_kF(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kF,SwervePIDConstants.kTimeoutMs);
        m_controlBL.config_kP(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kP,SwervePIDConstants.kTimeoutMs);
        m_controlBL.config_kI(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kI,SwervePIDConstants.kTimeoutMs);
        m_controlBL.config_kD(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kD,SwervePIDConstants.kTimeoutMs);

        // m_controlBL.setDistancePerPulse();
        m_controlBL.configSelectedFeedbackCoefficient(1/4096,0,0);
        
        //sets the relative sensor to match absolute
        m_controlBL.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);


        m_controlBR.configFactoryDefault();
        //sets the configured sensor to various predetermined constants
        m_controlBR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        //ensure sensor is positive when output is positive
        m_controlBR.setSensorPhase(SwervePIDConstants.kSensorPhase);
        //sets the m_controlBR to inverted if the motor tells it to do so.
        m_controlBR.setInverted(SwervePIDConstants.kMotorInvert);
        //set peak and nominal outputs
        m_controlBR.configNominalOutputForward(0,SwervePIDConstants.kTimeoutMs);
        m_controlBR.configNominalOutputReverse(0,SwervePIDConstants.kTimeoutMs);
        m_controlBR.configPeakOutputForward(1,SwervePIDConstants.kTimeoutMs);
        m_controlBR.configPeakOutputReverse(-1,SwervePIDConstants.kTimeoutMs);
        //configures position closed loop in slot0
        m_controlBR.config_kF(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kF,SwervePIDConstants.kTimeoutMs);
        m_controlBR.config_kP(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kP,SwervePIDConstants.kTimeoutMs);
        m_controlBR.config_kI(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kI,SwervePIDConstants.kTimeoutMs);
        m_controlBR.config_kD(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kD,SwervePIDConstants.kTimeoutMs);

        // m_controlBR.setDistancePerPulse();
        m_controlBR.configSelectedFeedbackCoefficient(1/4096,0,0);
        
        //sets the relative sensor to match absolute
        m_controlBR.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);


        testXbox = m_xbox;


        for (int i = 0; i < configs.length; i++){
            configs[i] = new SparkMaxConfig();
            if (i % 2 == 1){

                configs[i].inverted(true);
            }
            else {
                configs[i].inverted(false);
            }
        }
        m_fl.configure(configs[0],ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_fr.configure(configs[1],ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_bl.configure(configs[2],ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_br.configure(configs[3],ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        
    }

    public Command sysIDQuasistatic(SysIdRoutine.Direction direction){
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return routine.dynamic(direction);
    }
    @Override
    public void periodic() {
        //     m_driverController
    //     .a()
    //     .and(m_driverController.rightBumper())
    //     .whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController
    //     .b()
    //     .and(m_driverController.rightBumper())
    //     .whileTrue(m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController
    //     .x()
    //     .and(m_driverController.rightBumper())
    //     .whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController
    //     .y()
    //     .and(m_driverController.rightBumper())
        boolean rightbump =  testXbox.getRightBumperButton();
        m_controlFL.set(ControlMode.Position, 0);
        m_controlFR.set(ControlMode.Position, 0);
        m_controlBL.set(ControlMode.Position, 0);
        m_controlBR.set(ControlMode.Position, 0);
    //     .whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        if (Robot.isReal()){
            if (testXbox.getAButton() && rightbump){
                CommandScheduler.getInstance().schedule(sysIDQuasistatic(SysIdRoutine.Direction.kForward));
            }
            else if (testXbox.getBButton() && rightbump){
                CommandScheduler.getInstance().schedule(sysIDQuasistatic(SysIdRoutine.Direction.kReverse));
            }
            else if (testXbox.getXButton() && rightbump){
                CommandScheduler.getInstance().schedule(sysIdDynamic(SysIdRoutine.Direction.kForward));
            }
            else if (testXbox.getYButton() && rightbump){
                CommandScheduler.getInstance().schedule(sysIdDynamic(SysIdRoutine.Direction.kReverse));
            }   
            else {
                CommandScheduler.getInstance().schedule();
            }
        }   
        
    }


}
