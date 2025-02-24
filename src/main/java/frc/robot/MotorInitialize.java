package frc.robot;
public class MotorInitialize {
    public int CANConstant;
    public Gains MotorGAINS;
    public EncoderLimits MotorLIMITS;
    public MotorInitialize(int constant, Gains gains, EncoderLimits limits){
        CANConstant = constant;
        MotorGAINS = gains;
        MotorLIMITS = limits;
    }
}
