// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake_subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.armevator.ArmevatorConstants;
import frc.robot.subsystems.armevator.ArmevatorPose;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private HashMap<ArmevatorPose, Double> poseAlgaeAngleMap;
  private double algaeAngleSetpoint;

  private ProfiledPIDController intakePID;
  private ArmFeedforward intakeFeedforward;

  private TalonFX rollerMotor;
  private TalonFX tiltMotor;

  public Intake() {
    rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
    tiltMotor = new TalonFX(IntakeConstants.TILT_MOTOR_ID);

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

    intakeFeedforward =
        new ArmFeedforward(
            ArmevatorConstants.ARM_HEIGHT_KS,
            ArmevatorConstants.ARM_HEIGHT_KG,
            ArmevatorConstants.ARM_HEIGHT_KV,
            ArmevatorConstants.ARM_HEIGHT_KA);

    intakePID =
        new ProfiledPIDController(
            ArmevatorConstants.ARM_KP,
            ArmevatorConstants.ARM_KI,
            ArmevatorConstants.ARM_KD,
            new TrapezoidProfile.Constraints(
                ArmevatorConstants.ARM_MAX_VELOCITY, ArmevatorConstants.ARM_MAX_ACCELERATION),
            ArmevatorConstants.ARM_PERIOD_SEC);

    setupNT();
  }

  public boolean isAtGoal() {
    return intakePID.atGoal();
  }

  public void runIntake() {
    rollerMotor.set(IntakeConstants.ROLLER_INTAKE_SPEED);
  }

  public void ejectIntake() {
    rollerMotor.set(IntakeConstants.ROLLER_EJECT_SPEED);
  }

  public void stopIntake() {
    rollerMotor.set(0);
  }

  public double getTiltPosition() {
    return tiltMotor.getPosition().getValueAsDouble();
  }

  public double getTiltVelocity() {
    return tiltMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    tiltMotor.set(
        MathUtil.clamp(intakePID.calculate(getTiltPosition()), -0.5, 0.5)
            + intakeFeedforward.calculate(
                Math.toRadians(getTiltPosition() + IntakeConstants.TILT_COG_OFFSET),
                getTiltVelocity()));

    doLogging();
  }

  public void setPose(ArmevatorPose pose) {
    intakePID.setGoal(poseAlgaeAngleMap.get(pose));
    algaeAngleSetpoint = poseAlgaeAngleMap.get(pose);
  }

  private void doLogging() {
    Logger.recordOutput(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Position", getTiltPosition());
    Logger.recordOutput(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Velocity", getTiltVelocity());
    Logger.recordOutput(
        IntakeConstants.SUBSYSTEM_NAME + "/Tilt Goal", intakePID.getGoal().position);
  }

  private void setupNT() {
    SmartDashboard.putData(IntakeConstants.SUBSYSTEM_NAME + "/Tilt PID", intakePID);

    SmartDashboard.putNumber(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Position", getTiltPosition());
    SmartDashboard.putNumber(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Velocity", getTiltVelocity());
    SmartDashboard.putNumber(
        IntakeConstants.SUBSYSTEM_NAME + "/Tilt Goal", intakePID.getGoal().position);
  }
}
