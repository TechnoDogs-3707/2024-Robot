// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.localizer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class VisionPose {
    public final Pose3d pose;
    public final double timestampSeconds;
    public final Matrix<N3, N1> stddevs;

    public VisionPose(Pose3d pose, double time, Matrix<N3, N1> stddevs) {
        this.pose = pose;
        this.timestampSeconds = time;
        this.stddevs = stddevs;
    }
}
