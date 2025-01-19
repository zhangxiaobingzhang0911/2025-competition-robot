// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.apriltagvision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
    // Updates the inputs of the AprilTag vision system
    default void updateInputs(AprilTagVisionIOInputs inputs) {
    }

    // Class representing the input data for the AprilTag vision system
    class AprilTagVisionIOInputs implements LoggableInputs {
        // Array of timestamps corresponding to each frame
        public double[] timestamps = new double[]{};
        // 2D array of frames captured by the vision system
        public double[][] frames = new double[][]{};
        // A single demo frame for visualization or testing purposes
        public double[] demoFrame = new double[]{};
        // Frames per second of the vision system
        public long fps = 0;

        // Logs the input data to the provided LogTable
        @Override
        public void toLog(LogTable table) {
            table.put("Timestamps", timestamps);
            table.put("FrameCount", frames.length);
            for (int i = 0; i < frames.length; i++) {
                table.put("Frame/" + i, frames[i]);
            }
            table.put("DemoFrame", demoFrame);
            table.put("Fps", fps);
        }

        // Restores the input data from the provided LogTable
        @Override
        public void fromLog(LogTable table) {
            timestamps = table.get("Timestamps", new double[]{0.0});
            int frameCount = table.get("FrameCount", 0);
            frames = new double[frameCount][];
            for (int i = 0; i < frameCount; i++) {
                frames[i] = table.get("Frame/" + i, new double[]{});
            }
            demoFrame = table.get("DemoFrame", new double[]{});
            fps = table.get("Fps", 0);
        }
    }
}