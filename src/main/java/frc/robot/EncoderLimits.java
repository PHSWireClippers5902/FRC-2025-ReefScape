/*5902 WireClippers 2025
 * By Daniel Sabalakov
 * Helper class to for storage of left and right encoder limits. Does not implement.
 */

package frc.robot;
public class EncoderLimits {
    public double leftLimit;
    public double rightLimit;
    /**
     * Maps inputs to instance variables
     * @param leftMostLimit
     * @param rightMostLimit
     */
    public EncoderLimits(double leftMostLimit, double rightMostLimit){
        leftLimit = leftMostLimit;
        rightLimit = rightMostLimit;
    }
}
