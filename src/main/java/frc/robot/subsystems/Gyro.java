package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro extends SubsystemBase{
    public ADXRS450_Gyro m_gyro;
    public double originalPosition;

    public Gyro(){
        m_gyro = new ADXRS450_Gyro();
        m_gyro.calibrate();
        reset();
    }
    public Rotation2d getAng(){ 
        return new Rotation2d(Units.degreesToRadians(-m_gyro.getAngle()));
        // return m_gyro.getRotation2d();  
    }
    public Rotation2d getRotation(){
        return new Rotation2d(Units.degreesToRadians(-m_gyro.getAngle()));
        // return m_gyro.getRotation2d();
    }
    public void reset(){
        // m_gyro.zeroYaw();
        m_gyro.reset();
    }

}
