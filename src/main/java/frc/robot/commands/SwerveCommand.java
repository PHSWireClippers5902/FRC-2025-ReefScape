package frc.robot.commands;

// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class SwerveCommand extends Command {
    // public Joystick myJoystick;
    public XboxController myXbox;
    public Swerve mySwerve;
    private static final double kDeadband = 0.3;
    public double daDirection = 1;
    private static final double kZedDeadBand = 0.3;
    private static final double kMaxSpeed = Swerve.kMaxSpeed;
    private static final double kMaxAngularSpeed = Swerve.kMaxAngularSpeed;
    double xSpeed,ySpeed,rot;
    public SwerveCommand(XboxController xbox, Swerve m_s){
        myXbox = xbox;
        mySwerve = m_s;
        addRequirements(mySwerve);
    }
    @Override
    public void execute() {
        

        boolean fieldRelative = false;
        // if (myJoystick.getRawButton(0)){

        // }
        //getleftbumper()
        if (false){
            xSpeed = -daDirection*applyDeadband(myXbox.getLeftY()) * kMaxSpeed*0.7;
            // double ySpeed = 0,rot = 0;
            // double rot = 0;
            ySpeed = daDirection*applyDeadband(myXbox.getLeftX()) * kMaxSpeed*0.7;
            rot = -applyZedDeadband(myXbox.getRightX()) * kMaxAngularSpeed*0.2;
        }
        else {
            xSpeed = -daDirection*applyDeadband(myXbox.getLeftY()) * kMaxSpeed*0.2;
            // double ySpeed = 0,rot = 0;
            // double rot = 0;
            ySpeed = -daDirection*applyDeadband(myXbox.getLeftX()) * kMaxSpeed*0.2;
            rot = -applyZedDeadband(myXbox.getRightX()) * kMaxAngularSpeed*0.6;
        }
        // if (myXbox.getRightBumper()){
        //     daDirection = -1;
        // }
        // else {
        //     daDirection = 1;
        // }
        // if (myXbox.getYButton()){
        //     mySwerve.resetAll();
        // }
        // if (myXbox.getAButton()){
        //     mySwerve.myGyro.m_gyro.reset();
        // }
        
        SmartDashboard.putNumber("Front Left Position: " ,mySwerve.m_frontLeft.steeringController.getSelectedSensorPosition(0)*360/4096);
        SmartDashboard.putNumber("Front Right Position: " ,mySwerve.m_frontRight.steeringController.getSelectedSensorPosition(0)*360/4096);
        SmartDashboard.putNumber("Back Left Position: " ,mySwerve.m_backLeft.steeringController.getSelectedSensorPosition(0)*360/4096);
        SmartDashboard.putNumber("Back Right Position: " , mySwerve.m_backRight.steeringController.getSelectedSensorPosition(0)*360/4096);
        
        mySwerve.drive(xSpeed,ySpeed,rot,fieldRelative,0.02);
        // mySwerve.m_frontLeft.steeringController.set(0.1);
        // mySwerve.m_frontRight.steeringController.set(0.1);
        // mySwerve.m_backLeft.steeringController.set(0.1);
        // mySwerve.m_backRight.steeringController.set(0.1);

    }
    private double applyDeadband(double value) {
        return Math.abs(value) > kDeadband ? value : 0.0;
    }
    private double applyZedDeadband(double value){
        if (Math.abs(value) > kZedDeadBand){
            value = (Math.abs(value) - kZedDeadBand)*value/Math.abs(value);
            return value;
        }
        else {
            return 0;
        }
    }

}
