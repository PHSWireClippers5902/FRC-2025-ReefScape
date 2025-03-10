package frc.robot;

public final class Constants{
    public static final class ArmHookConstants{
        //CHANGE BEFORE USAGE
        public static final MotorInitialize arm = new MotorInitialize(
            10,
            new Gains(0.024,0,0.8,0,0,0.5),
            new EncoderLimits(-42,1),
            false,
            new Gains(0.008,0,0.8,0,0,1)
        );
        public static final MotorInitialize wrist = new MotorInitialize(
            12,
            new Gains(0.02,0,0,0,0,1),
            new EncoderLimits(-80, 80),
            false
        );
        public static final MotorInitialize intake = new MotorInitialize(
            13,
            new Gains(0.5,0,0,0,0,1),
            new EncoderLimits(0, 0),
            false
        );
        public static final MotorInitialize extendo = new MotorInitialize(
            11,
            new Gains(0.15,0,0.3,0,0,1),
            new EncoderLimits(-17.2,0.01),
            false
        );
        public static final MotorInitialize lift = new MotorInitialize(
            15, 
            new Gains(0.02,0,0.3,0,0,1), 
            new EncoderLimits(0,0),
            true
        );
        public static final double intakePercentIn = 0;
        public static final double intakePercentOut = 0;
        public static final double[] stageFour = {-30,0,-9.45};
        public static final double[] stageThree = {-23.428,0,0};
        public static final double[] stageTwo = {-17,0,0};
        public static final double[] intakePosDown = {-7.07,0,-10.833};
        public static final double[] intakePosUp = {-15.7,0,0};
    
    }
    public static final class ArmExtendoK{
        public static double arms = 0.03;
        public static double extendos = 0.06;
    }
    public static final class SwervePIDConstants{
        public static final int kSlotIdx = 0;

        public static final int kPIDLoopIdx = 0;

        public static final int kTimeoutMs = 0;

        public static boolean kSensorPhase = true;

        public static boolean kMotorInvert = true;

        public static final Gains kGains = new Gains(0.5, 0.0000, 1, 0.0, 0, 1.0);

        // public static final double drivekP = ;
        // public static final double drivekI = ;

    }
    public static final class Translations{
        // public static final double xPos = 0;
        // public static final double yPos = 0;
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
    public static final class AbsoluteChange{
        public static final int FrontLeftChange = 742;
        public static final int FrontRightChange = 314;
        public static final int BackLeftChange = 839;
        public static final int BackRightChange = 3675;
    }
    public static final class SwerveCANConstants{
        public static final int kFrontLeftDrivingCanId = 25;
        public static final int kRearLeftDrivingCanId = 24;
        public static final int kFrontRightDrivingCanId = 23;
        public static final int kRearRightDrivingCanId = 22;

        public static final int kFrontLeftTurningCanId = 1;
        public static final int kRearLeftTurningCanId  = 3;
        public static final int kFrontRightTurningCanId = 2;
        public static final int kRearRightTurningCanId = 4;
    }
    
    
}