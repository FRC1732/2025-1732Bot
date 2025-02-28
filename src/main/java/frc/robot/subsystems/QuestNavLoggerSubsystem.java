// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.QuestNav;

public class QuestNavLoggerSubsystem extends SubsystemBase {
  private NetworkTableInstance nt4Instance;
  private NetworkTable nt4Table;
  private IntegerSubscriber questMiso;
  private IntegerSubscriber questMosi;

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp;
  private FloatArraySubscriber questPosition;
  private FloatArraySubscriber questQuaternion;
  private FloatArraySubscriber questEulerAngles;
  private DoubleSubscriber questBatteryPercent;

  private QuestNav questNav;

  /** Creates a new QuestNavLoggerSubsystem. */
  public QuestNavLoggerSubsystem(QuestNav questNav) {
    this.questNav = questNav;
    nt4Instance = NetworkTableInstance.getDefault();
    nt4Table = nt4Instance.getTable("questnav");
    questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    questMosi = nt4Table.getIntegerTopic("mosi").subscribe(0);

    questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
    questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[] { 0.0f, 0.0f, 0.0f });
    questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] { 0.0f, 0.0f, 0.0f, 0.0f });
    questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] { 0.0f, 0.0f, 0.0f });
    questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);
  }

  @Override
  public void periodic() {
    doLogging();

  }

  private void doLogging() {
    Logger.recordOutput("QuestNav" + "/questMiso", questMiso.get());
    Logger.recordOutput("QuestNav" + "/TquestMosi", questMosi.get());
    Logger.recordOutput("QuestNav" + "/questTimestamp", questTimestamp.get());
    Logger.recordOutput("QuestNav" + "/questPosition", questPosition.get());
    Logger.recordOutput("QuestNav" + "/questQuaternion", questQuaternion.get());
    Logger.recordOutput("QuestNav" + "/questEulerAngles", questEulerAngles.get());
    Logger.recordOutput("QuestNav" + "/questBatteryPercent", questBatteryPercent.get());

    Logger.recordOutput("QuestNav" + "/getAverageRobotPose", questNav.getAverageRobotPose());
    Logger.recordOutput("QuestNav" + "/isConnected", questNav.isConnected());
    Logger.recordOutput("QuestNav" + "/getQuestPose", questNav.getQuestPose());
    Logger.recordOutput("QuestNav" + "/getRobotPose", questNav.getRobotPose());

  }
}
