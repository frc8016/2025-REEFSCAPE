package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("PhotonCamera");

    public PhotonVision() {
        // camera.subsystemDriverMode(true, driverMode);
        PortForwarder.add(5800, "photonvision.local", 5800);
    }

}
