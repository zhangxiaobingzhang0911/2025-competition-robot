package org.frcteam6941.drivers;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.AllianceFlipUtil;

// This class implements a Gyro interface using the Pigeon2 sensor from CTRE Phoenix6
public class Pigeon2Gyro implements Gyro {
    // Actual pigeon object
    public final Pigeon2 mGyro;

    // Configurations for the gyro sensor
    private boolean inverted = false;//AllianceFlipUtil.shouldFlip();//false;
    private Rotation2d yawAdjustmentAngle = new Rotation2d(0);
    private Rotation2d rollAdjustmentAngle = new Rotation2d();
    private Rotation2d pitchAdjustmentAngle = new Rotation2d();

    // Constructor to initialize Pigeon2 with a specific port
    public Pigeon2Gyro(int port) {
        mGyro = new Pigeon2(port, "");
    }

    // Constructor to initialize Pigeon2 with a specific port and CAN bus
    public Pigeon2Gyro(int port, String canbus) {
        mGyro = new Pigeon2(port, canbus);
    }

    // Returns the adjusted yaw angle as a Rotation2d object
    @Override
    public Rotation2d getYaw() {
        Rotation2d angle = getUnadjustedYaw().minus(AllianceFlipUtil.shouldFlip() ? yawAdjustmentAngle.unaryMinus() : yawAdjustmentAngle);
//        if (inverted) {
//            System.out.println(angle.unaryMinus());
//            return angle.minus(Rotation2d.fromDegrees(180));
//        }
        return angle;
    }

    /**
     * Sets the yaw register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    @Override
    public void setYaw(double angleDeg) {
        //yawAdjustmentAngle = getUnadjustedYaw().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
        yawAdjustmentAngle = Rotation2d.fromDegrees(yawAdjustmentAngle.getDegrees() + angleDeg);
    }

    // Returns the adjusted roll angle as a Rotation2d object
    @Override
    public Rotation2d getRoll() {
        return getUnadjustedRoll().minus(rollAdjustmentAngle);
    }

    /**
     * Sets the roll register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    @Override
    public void setRoll(double angleDeg) {
        rollAdjustmentAngle = getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    // Returns the adjusted pitch angle as a Rotation2d object
    @Override
    public Rotation2d getPitch() {
        return getUnadjustedPitch().minus(pitchAdjustmentAngle);
    }

    /**
     * Sets the pitch register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    @Override
    public void setPitch(double angleDeg) {
        pitchAdjustmentAngle = getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    // Sets the inversion state of the gyro sensor
    public void setInverted(boolean inv) {
        inverted = inv;
    }

    // Returns the unadjusted yaw angle from the Pigeon2 sensor as a Rotation2d object
    public Rotation2d getUnadjustedYaw() {
        return Rotation2d.fromDegrees(mGyro.getYaw().getValueAsDouble());
    }

    // Returns the unadjusted pitch angle from the Pigeon2 sensor as a Rotation2d object
    public Rotation2d getUnadjustedPitch() {
        return Rotation2d.fromDegrees(mGyro.getPitch().getValueAsDouble());
    }

    // Returns the unadjusted roll angle from the Pigeon2 sensor as a Rotation2d object
    public Rotation2d getUnadjustedRoll() {
        return Rotation2d.fromDegrees(mGyro.getRoll().getValueAsDouble());
    }

    // Returns the yaw angular velocity from the Pigeon2 sensor in degrees per second
    public double getYawAngularVelocity() {
        return mGyro.getAngularVelocityYWorld().getValueAsDouble();
    }

    // Returns the raw sensor values (X, Y, Z angular velocities) as a double array
    @Override
    public double[] getRaw() {
        double[] xyz_dps = new double[]{0.0, 0.0, 0.0};
        // mGyro.getRawGyro(xyz_dps);
        return xyz_dps;
    }
}