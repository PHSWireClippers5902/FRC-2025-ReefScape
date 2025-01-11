package frc.robot;

public final class Constants{
    public static final class SwervePIDConstants{
        public static final int kSlotIdx = 0;

        public static final int kPIDLoopIdx = 0;

        public static final int kTimeoutMs = 0;

        public static boolean kSensorPhase = true;

        public static boolean kMotorInvert = false;

        public static final Gains kGains = new Gains(0.6, 0.0000, 0.0, 0.0, 0, 1.0);

        // public static final double drivekP = ;
        // public static final double drivekI = ;

    }
    public static final class SwerveMotorConstants{
        public static final int powerFLID = 1;
        public static final int controlFLID = 1;

        public static final int powerFRID = 2;
        public static final int controlFRID = 2;

        public static final int powerBLID = 3;
        public static final int controlBLID = 3;

        public static final int powerBRID = 4;
        public static final int controlBRID = 4;

    }
    public static final class SwerveCANConstants{
        public static final int kFrontLeftDrivingCanId = 5;
        public static final int kRearLeftDrivingCanId = 7;
        public static final int kFrontRightDrivingCanId = 6;
        public static final int kRearRightDrivingCanId = 8;

        public static final int kFrontLeftTurningCanId = 1;
        public static final int kRearLeftTurningCanId = 3;
        public static final int kFrontRightTurningCanId = 2;
        public static final int kRearRightTurningCanId = 4;
    }
    
    
}