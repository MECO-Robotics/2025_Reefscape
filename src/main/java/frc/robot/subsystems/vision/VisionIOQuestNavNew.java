package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

public class VisionIOQuestNavNew implements VisionIO {
  public record QuestNavData(
      Pose3d pose,
      double batteryPercent,
      double timestamp,
      float[] translation,
      float[] rotation) {}

  private QuestNav questNav = new QuestNav();

  private final Transform3d robotToCamera;

  private final VisionIO absoluteVisionIO;
  private final VisionIOInputsAutoLogged absoluteInputs = new VisionIOInputsAutoLogged();

  public VisionIOQuestNavNew(Transform3d robotToCamera, VisionIO absoluteVisionIO) {
    // Initialize the camera to robot transform
    this.robotToCamera = robotToCamera;
    this.absoluteVisionIO = absoluteVisionIO;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    QuestNavData[] questNavData;

    absoluteVisionIO.updateInputs(absoluteInputs);
    Logger.processInputs("QuestNav/absolute", absoluteInputs);
  }

  private boolean connected() {
    return questNav.isConnected();
  }

  private double getBatteryPercent() {
    return questNav.getBatteryPercent().orElse(0);
  }

  // private QuestNavData[] getQuestNavData() {
  //   questNav.commandPeriodic();

  //   PoseFrame[] newFrame = questNav.getAllUnreadPoseFrames();
  //   double battery = getBatteryPercent();
  //   QuestNavData[] data;

  //   for (PoseFrame frame : newFrame){
  //     data[frame]
  //   }

  // }
}
