package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.Constants;
import frc.robot.Constants.SwerveCANConstants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase{
    public static final double kMaxSpeed = 3.0; // 3 meters per second should be fine to work with 
    public static final double kMaxAngularSpeed = 0.5*Math.PI; // 1/2 rotation per second

    private final SwerveModule[] swerveModules;
    private SwerveModulePosition[] currentModulePositions = new SwerveModulePosition[4];
    private SwerveModuleState[] currentModuleStates = new SwerveModuleState[4];

    public Gyro myGyro = new Gyro();

    public Field2d ourField = new Field2d();
    private Field2d sternCam, bowCam;

    private int poseErrorCounter, debugFlags;

    // private final SwerveDrivePoseEstimator odometry;

    private Pose2d currentPose, currentTelePose;

    private ChassisSpeeds currentRobotVelocity, currentFieldVelocity, currentTeleFieldVelocity;

    private Alliance alliance = null;

    public boolean isAuto;

    // private final SysIdRoutine driveCharacterizer, angleCharacterizer;

    public Translation2d m_frontLeftLocation = new Translation2d(0.573,0.573);
    public Translation2d m_frontRightLocation = new Translation2d(0.573,-0.573);
    public Translation2d m_backLeftLocation = new Translation2d(-0.573,0.573);
    public Translation2d m_backRightLocation = new Translation2d(-0.573,-0.573);

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

        this.swerveModules = new SwerveModule[] {
            m_frontLeft,
            m_frontRight,
            m_backLeft,
            m_backRight
        };
        for (int i = 0; i < swerveModules.length; i++){
            currentModulePositions[i] = swerveModules[i].getPosition();
        }

        // odometry = new SwerveDrivePoseEstimator(m_kinematics, myGyro.getAng(), currentModulePositions, new Pose2d());


        currentPose = new Pose2d();
        currentTelePose = new Pose2d();
        
        currentRobotVelocity = new ChassisSpeeds();
        currentFieldVelocity = new ChassisSpeeds();
        currentTeleFieldVelocity = new ChassisSpeeds();

        sternCam = new Field2d();
        bowCam = new Field2d();

        // driveCharacterizer = new SysIdRoutine(
        // new SysIdRoutine.Config(),
        // new SysIdRoutine.Mechanism(
        //     (Voltage volts) -> {
        //     for (SwerveModule module : this.swerveModules) {
        //         module.setAzimuth(new Rotation2d());
        //         module.setDriveVolts(volts.in(Volts));
        //     }
        //     },
        //     log -> {
        //     log.motor("driveLinear")
        //     .voltage(Volts.of(avgDriveVolts()))
        //     .linearPosition(Meters.of(odometry.getEstimatedPosition().getX()))
        //     .linearVelocity(MetersPerSecond.of(getRobotVelocity().vxMetersPerSecond));
        //     },
        //     this));
        
        // angleCharacterizer = new SysIdRoutine(
        // new SysIdRoutine.Config(),
        // new SysIdRoutine.Mechanism(
        //     (Voltage volts) -> {
        //     for (SwerveModule module : this.swerveModules) {
        //         module.setAzimuth(module.getModuleLocation().getAngle().plus(Rotation2d.fromDegrees(90)));
        //         module.setDriveVolts(volts.in(Volts));
        //     }
        //     },
        //     log -> {
        //     log.motor("driveAngular")
        //     .voltage(Volts.of(avgDriveVolts()))
        //     .angularPosition(Radians.of(currentPose.getRotation().getRadians()))
        //     .angularVelocity(RadiansPerSecond.of(getRobotVelocity().omegaRadiansPerSecond));
        //     },
        //     this));



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
        // SmartDashboard.putNumber("FL Distance: ", m_frontLeft.steeringController.getSelectedSensorPosition(0));
        // SmartDashboard.putNumber("FL Speed: "  , m_frontLeft.steeringController.getSelectedSensorVelocity(0));
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
    public ChassisSpeeds getRobotVelocity() {
        return currentRobotVelocity;
    }
    private double avgDriveVolts() {
        double sum = 0;
        for (SwerveModule module : swerveModules) {
          sum += module.getDriveVolts();
        }

        return (sum / 4);
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
