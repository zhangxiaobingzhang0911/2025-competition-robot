package org.frcteam6941.drivers;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro {
    Rotation2d getYaw();

    void setYaw(double angle);

    Rotation2d getPitch();

    void setPitch(double angle);

    Rotation2d getRoll();

    void setRoll(double angle);

    double[] getRaw();
}
