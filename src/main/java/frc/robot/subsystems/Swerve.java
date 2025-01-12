package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import frc.robot.Constants.SwerveCANConstants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase{
    public static final double kMaxSpeed = 3.0; // 3 meters per second should be fine to work with 
    public static final double kMaxAngularSpeed = 0.5*Math.PI; // 1/2 rotation per second

    public Gyro myGyro = new Gyro();

    public Translation2d m_frontLeftLocation = new Translation2d(-0.573,0.573);
    public Translation2d m_frontRightLocation = new Translation2d(-0.573,-0.573);
    public Translation2d m_backLeftLocation = new Translation2d(0.573,0.573);
    public Translation2d m_backRightLocation = new Translation2d(0.573,-0.573);

    public SwerveModule m_frontLeft = new SwerveModule(SwerveCANConstants.kFrontLeftDrivingCanId,SwerveCANConstants.kFrontLeftTurningCanId,false);
    public SwerveModule m_frontRight = new SwerveModule(SwerveCANConstants.kFrontRightDrivingCanId,SwerveCANConstants.kFrontRightTurningCanId,true);
    public SwerveModule m_backLeft = new SwerveModule(SwerveCANConstants.kRearLeftDrivingCanId,SwerveCANConstants.kRearLeftTurningCanId,false);
    public SwerveModule m_backRight = new SwerveModule(SwerveCANConstants.kRearRightDrivingCanId,SwerveCANConstants.kRearRightTurningCanId,true);
    public SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                                                m_frontLeftLocation,
                                                m_frontRightLocation,
                                                m_backLeftLocation,
                                                m_backRightLocation
                                            );
    
        

        
        
    // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    //                                 m_kinematics,
    //                                 myGyro.getAng(),
    //                                 new SwerveModulePosition[] {
    //                                     m_frontLeft.getPosition(),
    //                                     m_frontRight.getPosition(),
    //                                     m_backLeft.getPosition(),
    //                                     m_backRight.getPosition()
    //                                 } 
    //                             );

    public Swerve() {
        myGyro.reset();

        // RobotConfig config;
        // try {
            
        // config = RobotConfig.fromGUISettings();
        // }
        // catch (Exception e){
        //     System.out.println("Bad output lmao");
        // }
        
        // AutoBuilder.configure(
        //     this::getPose, // Robot pose supplier
        //     this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //     this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //     (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        //     new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        //             new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        //             new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        //     ),
        //     config, // The robot configuration
        //     () -> {
        //       // Boolean supplier that controls when the path will be mirrored for the red alliance
        //       // This will flip the path being followed to the red side of the field.
        //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                
        //       var alliance = DriverStation.getAlliance();
        //       if (alliance.isPresent()) {
        //         return alliance.get() == DriverStation.Alliance.Red;
        //       }
        //       return false;
        //     },
        //     this // Reference to this subsystem to set requirements
        // );

        // myGyro.originalPosition = myGyro.m_gyro.getRoll()+180;
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return new ChassisSpeeds();
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates,kMaxSpeed
        );
        m_frontLeft.setDesiredState(swerveModuleStates[0], true);
        m_frontRight.setDesiredState(swerveModuleStates[1], true);
        m_backLeft.setDesiredState(swerveModuleStates[2], true);
        m_backRight.setDesiredState(swerveModuleStates[3], true);
        SmartDashboard.putNumber("FL Distance: ", m_frontLeft.steeringController.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("FL Speed: "  , m_frontLeft.steeringController.getSelectedSensorVelocity(0));
    }
    public void drive(double xSpeed,double ySpeed, double rot, boolean fieldRelative, double periodSeconds){
        ChassisSpeeds chassisSpeed = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,rot,Rotation2d.fromDegrees(myGyro.m_gyro.getAngle()))
                                                    : new ChassisSpeeds(xSpeed,ySpeed,rot);
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates,kMaxSpeed
        );
        m_frontLeft.setDesiredState(swerveModuleStates[0], true);
        m_frontRight.setDesiredState(swerveModuleStates[1], true);
        m_backLeft.setDesiredState(swerveModuleStates[2], true);
        m_backRight.setDesiredState(swerveModuleStates[3], true);
        SmartDashboard.putNumber("FL Distance: ", m_frontLeft.steeringController.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("FL Speed: "  , m_frontLeft.steeringController.getSelectedSensorVelocity(0));
        // SmartDashboard.putRaw("Translation2d: ", myGyro.m_gyro.getRotation2d());
        // SmartDashboard.putNumber("Yaw: ",myGyro.m_gyro.getYaw());
        // SmartDashboard.putNumber("Pitch: ",myGyro.m_gyro.getPitch());
        // SmartDashboard.putNumber("Roll: ",myGyro.getRollFunction());

        // SmartDashboard.putData("Module States: ", );
        
    }
    public Pose2d getPose(){
        return new Pose2d(new Translation2d(), myGyro.getRotation());

    }
    public void resetPose(Pose2d pose){
        
    }
    public void resetAll(){
        m_frontLeft.resetSteerPosition();
        m_frontRight.resetSteerPosition();
        m_backLeft.resetSteerPosition();
        m_backRight.resetSteerPosition();
    }
    
    

}
