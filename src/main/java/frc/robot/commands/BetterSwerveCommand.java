package frc.robot.commands;

// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class BetterSwerveCommand extends Command {
    // public Joystick myJoystick;
    public XboxController myXbox;
    public Swerve mySwerve;
    private static final double kDeadband = 0.3;
    public double daDirection = 1;
    private static final double kZedDeadBand = 0.3;
    private static final double kMaxSpeed = Swerve.kMaxSpeed;
    private static final double kMaxAngularSpeed = Swerve.kMaxAngularSpeed;
    double xSpeed,ySpeed,rot;
    public BetterSwerveCommand(XboxController xbox, Swerve m_s){
        myXbox = xbox;
        mySwerve = m_s;
        addRequirements(mySwerve);
    }
    @Override
    public void execute() {
        

        boolean fieldRelative = true;
        // if (myJoystick.getRawButton(0)){

        // }

        if (myXbox.getLeftBumper()){
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
            ySpeed = daDirection*applyDeadband(myXbox.getLeftX()) * kMaxSpeed*0.2;
            rot = -applyZedDeadband(myXbox.getRightX()) * kMaxAngularSpeed*0.6;
        }
        if (myXbox.getRightBumper()){
            daDirection = -1;
        }
        else {
            daDirection = 1;
        }
        if (myXbox.getYButton()){
            mySwerve.resetAll();
        }
        if (myXbox.getAButton()){
            mySwerve.myGyro.m_gyro.reset();
        }
        mySwerve.drive(xSpeed,ySpeed,rot,fieldRelative,0.02);

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
