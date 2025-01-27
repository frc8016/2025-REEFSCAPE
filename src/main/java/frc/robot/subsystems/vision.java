package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class vision  extends SubsystemBase{
  
  private final PhotonCamera camera;

  public vision(String Opi_camera) {// this code should hopefully print out the targets photon vison sees 
          camera = new PhotonCamera(Opi_camera);
          var result = camera.getLatestResult();
          List<PhotonTrackedTarget> targets = result.getTargets();
          System.out.println(targets);
  

  }
  
   
  }
 



  
  


