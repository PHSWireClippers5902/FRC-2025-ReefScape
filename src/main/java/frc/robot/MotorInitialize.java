package frc.robot;
public class MotorInitialize {
    public int CANConstant;
    public Gains MotorGAINS;
    public EncoderLimits MotorLIMITS;
    public boolean MotorInverted;
    public MotorInitialize(int constant, Gains gains, EncoderLimits limits, boolean inverted){
        CANConstant = constant;
        MotorGAINS = gains;
        MotorLIMITS = limits;
        MotorInverted = inverted;
    }
}
