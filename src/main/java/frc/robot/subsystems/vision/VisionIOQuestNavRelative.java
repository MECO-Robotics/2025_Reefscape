package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedFloatArray;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.vision.VisionIOQuestNav.QuestNavData;
import org.littletonrobotics.junction.Logger;

public class VisionIOQuestNavRelative implements VisionIO {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest
  // HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion =
      nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent =
      nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  // Local heading helper variables
  private float yaw_offset = 0.0f;
  private Pose2d resetPosition = new Pose2d();

  private final Transform3d robotToCamera;

  public VisionIOQuestNavRelative(Transform3d robotToCamera) {
    // Initialize the camera to robot transform
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    QuestNavData[] questNavData = getQuestNavData();

    inputs.connected = connected();
    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    inputs.poseObservations = new PoseObservation[questNavData.length];

    for (int i = 0; i < questNavData.length; i++) {
      inputs.poseObservations[i] =
          new PoseObservation(
              questNavData[i].timestamp(),
              new Pose3d(
                  questNavData[i].pose().getTranslation(), questNavData[i].pose().getRotation()),
              0.0,
              -1,
              0.0,
              PoseObservationType.QUESTNAV);
    }

    Logger.recordOutput("QuestNav/battery", getBatteryPercent());

    cleanUpQuestNavMessages();
  }

  private QuestNavData[] getQuestNavData() {
    TimestampedDouble[] timestamps = questTimestamp.readQueue();
    TimestampedFloatArray[] positions = questPosition.readQueue();
    TimestampedFloatArray[] angles = questEulerAngles.readQueue();
    // TimestampedDouble[] battery = questBatteryPercent.readQueue();

    double battery = getBatteryPercent();

    int length = Math.min(timestamps.length, Math.min(positions.length, angles.length));

    QuestNavData[] data = new QuestNavData[length];

    for (int i = 0; i < length; i++) {
      data[i] =
          new QuestNavData(
              getQuestNavPose(positions[i].value, angles[i].value).plus(robotToCamera.inverse()),
              battery,
              timestamps[i].timestamp,
              positions[i].value,
              angles[i].value);
    }

    return data;
  }

  private Translation3d getQuestNavTranslation(float[] position) {
    return new Translation3d(position[2], -position[0], position[1]);
  }

  // Gets the Rotation of the Quest.
  public Rotation3d getQuestNavRotation(float[] angles) {
    return new Rotation3d(
        Units.degreesToRadians(-angles[2]),
        Units.degreesToRadians(angles[0]),
        Units.degreesToRadians(-angles[1]));
  }

  private Pose3d getQuestNavPose(float[] position, float[] angles) {
    var oculousPositionCompensated = getQuestNavTranslation(position); // 6.5
    return new Pose3d(oculousPositionCompensated, getQuestNavRotation(angles));
  }

  // Gets the Quest's measured position.
  public Pose2d getPose() {
    return new Pose2d(
        getQuestNavPose().minus(resetPosition).getTranslation(),
        Rotation2d.fromDegrees(getOculusYaw()));
  }

  // Gets the battery percent of the Quest.
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  // Returns if the Quest is connected.
  public boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Gets the Quaternion of the Quest.
  public Quaternion getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
  }

  // Gets the Quests's timestamp.
  public double timestamp() {
    return questTimestamp.get();
  }

  // Zero the relativerobot heading
  public void zeroHeading() {
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
  }

  // Zero the absolute 3D position of the robot (similar to long-pressing the
  // quest logo)
  public void zeroPosition() {
    resetPosition = getPose();
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  // Clean up questnav subroutine messages after processing on the headset
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  // Get the yaw Euler angle of the headset
  private float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1] - yaw_offset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return ret;
  }

  private Translation2d getQuestNavTranslation() {
    float[] questnavPosition = questPosition.get();
    return new Translation2d(questnavPosition[2], -questnavPosition[0]);
  }

  private Pose2d getQuestNavPose() {
    var oculousPositionCompensated =
        getQuestNavTranslation().minus(new Translation2d(0, 0.1651)); // 6.5
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
  }
}
