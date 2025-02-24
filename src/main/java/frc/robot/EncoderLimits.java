package frc.robot;



public class EncoderLimits {
    public double leftLimit;
    public double rightLimit;
    public EncoderLimits(double leftMostLimit, double rightMostLimit){
        leftLimit = leftMostLimit;
        rightLimit = rightMostLimit;
    }
}
