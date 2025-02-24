package frc.robot;

public final class Constants{
    public static final class ArmHookConstants{
        //CHANGE BEFORE USAGE
        public static final MotorInitialize arm = new MotorInitialize(
            10,
            new Gains(0.01,0,0.8,0,0,1),
            new EncoderLimits(-66.9,0)
        );
        public static final MotorInitialize wrist = new MotorInitialize(
            12,
            new Gains(0.4,0,0,0,0,1),
            new EncoderLimits(-50, 50)
        );
        public static final MotorInitialize intake = new MotorInitialize(
            13,
            new Gains(0.005,0,0,0,0,1),
            new EncoderLimits(0, 0)
        );
        public static final MotorInitialize extendo = new MotorInitialize(
            11,
            new Gains(0.02,0,0.3,0,0,1),
            new EncoderLimits(-11.97,5.59)
        );
        public static final MotorInitialize lift = new MotorInitialize(
            15, 
            new Gains(0.02,0,0.3,0,0,1), 
            new EncoderLimits(0,0)
        );
        public static final double intakePercentIn = 0;
        public static final double intakePercentOut = 0;
        public static final double[] stageOne = {0,0,0};
        public static final double[] stageTwo = {0,0,0};
        public static final double[] stageThree = {0,0,0};
        public static final double[] stageFour = {0,0,0};
        public static final double[] amp = {0,0,0};
        public static final double[] intakePosUp = {-5.166,25.904,-12.785873};
        public static final double[] intakePosDown = {0,0,0};
    
    }
    public static final class SwervePIDConstants{
        public static final int kSlotIdx = 0;

        public static final int kPIDLoopIdx = 0;

        public static final int kTimeoutMs = 0;

        public static boolean kSensorPhase = true;

        public static boolean kMotorInvert = false;

        public static final Gains kGains = new Gains(0.3, 0.0000, 0.5, 0.0, 0, 1.0);

        // public static final double drivekP = ;
        // public static final double drivekI = ;

    }
    public static final class Translations{
        public static final double xPos = 0;
        public static final double yPos = 0;
    }
    public static final class KsKvKaConstants{
        public static final double ks = 0.02348;
        public static final double kv = 0;
        public static final double ka = 0.12066;
    }
    // public static final class SwerveMotorConstants{
    //     public static final int powerFLID = 1;
    //     public static final int controlFLID = 1;

    //     public static final int powerFRID = 2;
    //     public static final int controlFRID = 2;

    //     public static final int powerBLID = 3;
    //     public static final int controlBLID = 3;

    //     public static final int powerBRID = 4;
    //     public static final int controlBRID = 4;

    // }
    public static final class SwerveCANConstants{
        public static final int kFrontLeftDrivingCanId = 25;
        public static final int kRearLeftDrivingCanId = 24;
        public static final int kFrontRightDrivingCanId = 23;
        public static final int kRearRightDrivingCanId = 22;

        public static final int kFrontLeftTurningCanId = 1;
        public static final int kRearLeftTurningCanId = 2;
        public static final int kFrontRightTurningCanId = 3;
        public static final int kRearRightTurningCanId = 4;
    }
    
    
}