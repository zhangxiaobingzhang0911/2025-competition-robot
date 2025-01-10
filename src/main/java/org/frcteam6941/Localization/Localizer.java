package org.frcteam6941.Localization;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface Localizer {
    Pose2d getLatestPose();
    Pose2d getPoseAtTime(double time);
    Pose2d getPredictedPose(double lookahead);
    Pose2d getCoarseFieldPose(double time);

    Pose2d getMeasuredVelocity();
    Pose2d getMeasuredAcceleration();
    Pose2d getPredictedVelocity();
    Pose2d getSmoothedVelocity();
    Pose2d getSmoothedPredictedVelocity();
    Pose2d getSmoothedAccleration();

    double getDistanceDriven();

    void addMeasurement(double time, Pose2d measuredDelta, Pose2d stdDeviation);
    void addMeasurement(double time, Pose2d measuredDelta, Matrix<N3, N1> stdDeviation);
}
