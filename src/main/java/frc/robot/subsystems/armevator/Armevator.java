// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.armevator;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Armevator extends SubsystemBase {
  /** Creates a new Armevator. */
  private HashMap<ArmevatorPose, Double> poseAngleMap;
  private HashMap<ArmevatorPose, Double> poseCarriageHeightMap;
  private HashMap<ArmevatorPose, Double> poseAlgaeAngleMap;

  private double armAngleSetpoint;
  private double carriageHeightSetpoint;
  private double algaeAngleSetpoint;

  private int limitSwitchCounter;
  private boolean elevatorPIDOverride;

  private ProfiledPIDController armPID;
  private ProfiledPIDController elevatorPID;
  private ElevatorFeedforward elevatorHeightFeedforward;

  private SparkMax elevatorRightMotor;
  private SparkMax elevatorLeftMotor;

  private SparkLimitSwitch elevatorLimitSwitch;
  private RelativeEncoder elevatorRelativeEncoder;

  public Armevator() {
    // setup poses
    poseAngleMap = new HashMap<>();

    poseAngleMap.put(ArmevatorPose.STARTING, 96.9);
    poseAngleMap.put(ArmevatorPose.CLIMB, -110.0);
    poseAngleMap.put(ArmevatorPose.CORAL_HP_LOAD, -125.0);
    poseAngleMap.put(ArmevatorPose.CORAL_L4_SCORE, 55.0);
    poseAngleMap.put(ArmevatorPose.CORAL_L3_SCORE, 45.0);
    poseAngleMap.put(ArmevatorPose.CORAL_L2_SCORE, 72.0);
    poseAngleMap.put(ArmevatorPose.CORAL_L1_SCORE, 80.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_INTAKE, 90.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_HANDOFF, 90.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_NET_SCORE, -55.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_L3_PLUCK, 25.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_L3_DROP, 5.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_L2_PLUCK, 25.0);
    poseAngleMap.put(ArmevatorPose.ALGAE_L2_DROP, 5.0);

    poseCarriageHeightMap = new HashMap<>(); // in inches
    poseCarriageHeightMap.put(ArmevatorPose.STARTING, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.CLIMB, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.CORAL_HP_LOAD, 3.25);
    poseCarriageHeightMap.put(ArmevatorPose.CORAL_L4_SCORE, 32.0);
    poseCarriageHeightMap.put(ArmevatorPose.CORAL_L3_SCORE, 4.5);
    poseCarriageHeightMap.put(ArmevatorPose.CORAL_L2_SCORE, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.CORAL_L1_SCORE, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_INTAKE, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_HANDOFF, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_NET_SCORE, 32.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_L3_PLUCK, 15.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_L3_DROP, 20.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_L2_PLUCK, 0.0);
    poseCarriageHeightMap.put(ArmevatorPose.ALGAE_L2_DROP, 4.5);

    poseAlgaeAngleMap = new HashMap<>();
    poseAlgaeAngleMap.put(ArmevatorPose.STARTING, 0.0);
    poseAlgaeAngleMap.put(ArmevatorPose.CLIMB, 35.0);
    poseAlgaeAngleMap.put(ArmevatorPose.CORAL_HP_LOAD, 5.0);
    poseAlgaeAngleMap.put(ArmevatorPose.CORAL_L4_SCORE, 5.0);
    poseAlgaeAngleMap.put(ArmevatorPose.CORAL_L3_SCORE, 5.0);
    poseAlgaeAngleMap.put(ArmevatorPose.CORAL_L2_SCORE, 5.0);
    poseAlgaeAngleMap.put(ArmevatorPose.CORAL_L1_SCORE, 5.0);
    poseAlgaeAngleMap.put(ArmevatorPose.ALGAE_INTAKE, 55.0);
    poseAlgaeAngleMap.put(ArmevatorPose.ALGAE_HANDOFF, 10.0);
    poseAlgaeAngleMap.put(ArmevatorPose.ALGAE_NET_SCORE, 5.0);
    poseAlgaeAngleMap.put(ArmevatorPose.ALGAE_L3_PLUCK, 5.0);
    poseAlgaeAngleMap.put(ArmevatorPose.ALGAE_L3_DROP, 15.0);
    poseAlgaeAngleMap.put(ArmevatorPose.ALGAE_L2_PLUCK, 5.0);
    poseAlgaeAngleMap.put(ArmevatorPose.ALGAE_L2_DROP, 10.0);

    // setup motors
    elevatorRightMotor = new SparkMax(ArmevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    elevatorLeftMotor = new SparkMax(ArmevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig rightConfig = new SparkMaxConfig();

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.follow(elevatorRightMotor, true);

    elevatorRightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorLeftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    elevatorLimitSwitch = elevatorRightMotor.getForwardLimitSwitch();
    elevatorRelativeEncoder = elevatorRightMotor.getEncoder();

    elevatorHeightFeedforward = new ElevatorFeedforward(
        ArmevatorConstants.ELEVATOR_HEIGHT_KS,
        ArmevatorConstants.ELEVATOR_HEIGHT_KG,
        ArmevatorConstants.ELEVATOR_HEIGHT_KV,
        ArmevatorConstants.ELEVATOR_HEIGHT_KA);

    elevatorPID = new ProfiledPIDController(
        ArmevatorConstants.ELEVATOR_KP,
        ArmevatorConstants.ELEVATOR_KI,
        ArmevatorConstants.ELEVATOR_KD,
        new TrapezoidProfile.Constraints(
            ArmevatorConstants.ELEVATOR_MAX_VELOCITY,
            ArmevatorConstants.ELEVATOR_MAX_ACCELERATION),
        ArmevatorConstants.ELEVATOR_PERIOD_SEC);

  }

  public void setPose(ArmevatorPose pose) {
    elevatorPID.setGoal(poseCarriageHeightMap.get(pose));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (elevatorLimitSwitch.isPressed()) {
      limitSwitchCounter++;
    } else {
      limitSwitchCounter = 0;
    }

    // turn off elevator when limit switch is pressed, leave it off if goal isn't
    // changed
    if (elevatorPID.getGoal().position != 0) {
      elevatorPIDOverride = false;
    } else if (limitSwitchCounter > 10) {
      elevatorPIDOverride = true;
    }

    if (elevatorPIDOverride) {
      elevatorRightMotor.stopMotor();
      elevatorRelativeEncoder.setPosition(0);
    } else {
      elevatorRightMotor.set(
          elevatorPID.calculate(elevatorRelativeEncoder.getPosition())
              + elevatorHeightFeedforward.calculate(elevatorRelativeEncoder.getVelocity()));
    }
  }

  public void doLogging() {
    Logger.recordOutput("Elevator Position", elevatorRelativeEncoder.getPosition());
    Logger.recordOutput("Elevator Velocity", elevatorRelativeEncoder.getVelocity());
    Logger.recordOutput("Elevator Goal", elevatorPID.getGoal().position);
  }
}
