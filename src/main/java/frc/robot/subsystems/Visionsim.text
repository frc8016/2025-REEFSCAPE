package frc.robot.subsystems;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Visionsim extends SubsystemBase {
    VisionSystemSim visionSimulation = new VisionSystemSim("photonvision");
    double camPith = Units.degreesToRadians(10);
    double camHeightOffGround =0.8;
    Transform3d cameratrans = new Transform3d(
        new Translation3d(0.0, 0, camHeightOffGround), new Rotation3d(0, camPith, 0));
        


        
    public Visionsim(){
    }
    @Override
    public void periodic(){
    
    }
}
