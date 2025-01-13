package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class SystemIdentificationSubsystem extends SubsystemBase{
    public final SparkMax m_fl = new SparkMax(Constants.SwerveMotorConstants.powerFLID, MotorType.kBrushless);
    public final SparkMax m_fr = new SparkMax(Constants.SwerveMotorConstants.powerFRID, MotorType.kBrushless);
    public final SparkMax m_bl = new SparkMax(Constants.SwerveMotorConstants.powerBLID, MotorType.kBrushless);
    public final SparkMax m_br = new SparkMax(Constants.SwerveMotorConstants.powerBRID, MotorType.kBrushless);
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
        testXbox = m_xbox;


        for (int i = 0; i < configs.length; i++){
            configs[i] = new SparkMaxConfig();
            if (i % 2 == 0){

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
    //     .whileTrue(m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        if (testXbox.getAButton() && testXbox.getRightBumperButton()){
            sysIDQuasistatic(SysIdRoutine.Direction.kForward);
        }
        else if (testXbox.getBButton() && testXbox.getRightBumperButton()){
            sysIDQuasistatic(SysIdRoutine.Direction.kReverse);
        }
        else if (testXbox.getXButton() && testXbox.getRightBumperButton()){
            sysIdDynamic(SysIdRoutine.Direction.kForward);
        }
        else if (testXbox.getYButton() && testXbox.getRightBumperButton()){
            sysIdDynamic(SysIdRoutine.Direction.kReverse);
        }   
    }


}
