package frc.robot;
public class MotorInitialize {
    public int CANConstant;
    public Gains MotorGAINS;
    public EncoderLimits MotorLIMITS;
    public boolean MotorInverted;
    public Gains OptionalMotorGAINS;
    public MotorInitialize(int constant, Gains gains, EncoderLimits limits, boolean inverted){
        CANConstant = constant;
        MotorGAINS = gains;
        MotorLIMITS = limits;
        MotorInverted = inverted;
    }
    public MotorInitialize(int constant, Gains gains, EncoderLimits limits, boolean inverted, Gains optionalGains){
        CANConstant = constant;
        MotorGAINS = gains;
        MotorLIMITS = limits;
        MotorInverted = inverted;
        OptionalMotorGAINS = optionalGains;
    }
    
}
