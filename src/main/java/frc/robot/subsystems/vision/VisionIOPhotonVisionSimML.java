// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Notifier;

import java.util.function.Supplier;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSimML extends VisionIOPhotonVision {
    private static VisionSystemSim visionSim;
    private static Notifier simThread;

    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonVisionSimML(
            String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("ml");
            visionSim.addVisionTargets(
                    new VisionTargetSim[] {
                        new VisionTargetSim(
                                new Pose3d(5, 5, 0, new Rotation3d(0, 0, 0)),
                                new TargetModel(.5, 1)), // creates the target in simulation
                    });
            simThread = new Notifier(() -> {
                visionSim.update(poseSupplier.get());
            });
            simThread.setName("VisionSimThreadML");
            simThread.startPeriodic(0.02);
        }

        // Add sim camera

        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        super.updateInputs(inputs);
    }
}
