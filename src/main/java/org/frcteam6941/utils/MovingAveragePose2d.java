package org.frcteam6941.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.concurrent.CopyOnWriteArrayList;

/**
 * Helper class for storing and calculating a moving average of the Pose2d class
 */
public class MovingAveragePose2d {
    private final int maxSize;
    CopyOnWriteArrayList<Pose2d> poses = new CopyOnWriteArrayList<>();

    public MovingAveragePose2d(int maxSize) {
        this.maxSize = maxSize;
    }

    public synchronized void add(Pose2d pose) {
        // Adds a new pose to the list and removes the oldest pose if the list exceeds the maximum size
        poses.add(pose);
        if (poses.size() > maxSize) {
            poses.remove(0);
        }
    }

    public synchronized Pose2d getAverage() {
        // Calculates the average of all stored poses and returns it as a new Pose2d
        double x = 0.0, y = 0.0, t = 0.0;

        for (Pose2d pose : poses) {
            x += pose.getX();
            y += pose.getY();
            t += pose.getRotation().getDegrees();
        }

        double size = getSize();
        return new Pose2d(x / size, y / size, Rotation2d.fromDegrees(t / size));
    }

    public int getSize() {
        // Returns the current number of poses stored in the list
        return poses.size();
    }

    public boolean isUnderMaxSize() {
        // Checks if the number of stored poses is less than the maximum size
        return getSize() < maxSize;
    }

    public void clear() {
        // Clears all stored poses from the list
        poses.clear();
    }

}