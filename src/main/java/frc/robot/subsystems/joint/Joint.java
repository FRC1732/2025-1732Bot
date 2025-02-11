// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.joint;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Joint extends SubsystemBase {
  /** Creates a new Joint. */
  private final SparkMax jointMotor;

  private ShuffleboardTab jointTab;
  private double setpoint;
  private RelativeEncoder jointEncoder;
  private AbsoluteEncoder jointAbsoluteEncoder;
  private ProfiledPIDController jointPID;
  private ArmFeedforward jointFeedforward;

  // private GenericEntry goalEntry;
  // private GenericEntry jointP, jointI, jointD;
  private double prevJointP, prevJointI, prevJointD;

  public Joint() {
    jointMotor = new SparkMax(JointConstants.JOINT_MOTOR_CAN_ID, SparkMax.MotorType.kBrushless);
    // jointAbsoluteEncoder = new DutyCycleEncoder(8, 1.0, JointConstants.JOINT_ABSOLUTE_OFFSET);
    jointAbsoluteEncoder = jointMotor.getAbsoluteEncoder();
    SparkMaxConfig config = new SparkMaxConfig();

    prevJointP = JointConstants.JOINT_KP;
    prevJointI = JointConstants.JOINT_KI;
    prevJointD = JointConstants.JOINT_KD;

    config.idleMode(IdleMode.kBrake);
    config
        .encoder
        .positionConversionFactor(JointConstants.JOINT_DEGREES_PER_ROTATION)
        .velocityConversionFactor(JointConstants.JOINT_RPM_TO_DEGREES_PER_SECOND);

    // config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1, 2,
    // 3);

    jointMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    jointMotor.stopMotor();
    jointEncoder = jointMotor.getEncoder();
    jointEncoder.setPosition(angleModulusDegZero360(JointConstants.JOINT_START_SETPOINT));

    jointPID =
        new ProfiledPIDController(
            JointConstants.JOINT_KP,
            JointConstants.JOINT_KI,
            JointConstants.JOINT_KD,
            new TrapezoidProfile.Constraints(
                JointConstants.JOINT_MAX_VELOCITY, JointConstants.JOINT_MAX_ACCELERATION),
            JointConstants.JOINT_PID_PERIOD_SEC);
    jointPID.setTolerance(JointConstants.JOINT_GOAL_TOLERANCE_DEGREES);
    jointPID.reset(jointEncoder.getPosition());
    setJointPosition(JointConstants.JOINT_START_SETPOINT);

    jointFeedforward =
        new ArmFeedforward(
            JointConstants.JOINT_KS,
            JointConstants.JOINT_KG,
            JointConstants.JOINT_KV,
            JointConstants.JOINT_KA);

    setupShuffleboard();
  }

  public void setJointPose(JointPosition jointPosition) {
    switch (jointPosition) {
      case START_POSITION:
        setJointPosition(JointConstants.JOINT_START_SETPOINT);
        break;

      case CORAL_STATION:
        setJointPosition(JointConstants.JOINT_CORAL_SETPOINT);
        break;

      case LEVEL_1:
        setJointPosition(JointConstants.JOINT_LV1_SETPOINT);
        break;

      case LEVEL_2:
        setJointPosition(JointConstants.JOINT_LV2_SETPOINT);
        break;

      case LEVEL_3:
        setJointPosition(JointConstants.JOINT_LV3_SETPOINT);
        break;

      case LEVEL_4:
        setJointPosition(JointConstants.JOINT_LV4_SETPOINT);
        break;

      default:
        break;
    }
  }

  public void setJointPosition(double setpoint) {
    this.setpoint = setpoint;
    jointPID.setGoal(setpoint);
  }

  public double getJointPosition() {
    return setpoint;
  }

  public boolean isAtGoal() {
    return jointPID.atGoal();
  }

  private double getAbsolutePosition() {
    return ((jointAbsoluteEncoder.getPosition() + 0.85) % 1.0) * -360.0 / 2.4 + 198.0;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      jointPID.reset(jointEncoder.getPosition());
    }

    if (prevJointP != jointPID.getP()
        || prevJointI != jointPID.getI()
        || prevJointD != jointPID.getD()) {
      prevJointP = jointPID.getP();
      prevJointI = jointPID.getI();
      prevJointD = jointPID.getD();
      System.out.println("New PID values: ");
      System.out.println("P: " + prevJointP);
      System.out.println("I: " + prevJointI);
      System.out.println("D: " + prevJointD);
    }

    jointMotor.set(
        MathUtil.clamp(jointPID.calculate(jointEncoder.getPosition()), -0.5, 0.5)
            + jointFeedforward.calculate(
                Math.toRadians(jointEncoder.getPosition() + JointConstants.JOINT_COG_OFFSET),
                jointEncoder.getVelocity()));

    updateIO();
  }

  private void updateIO() {
    Logger.recordOutput(JointConstants.SUBSYSTEM_NAME + "/Angle", jointEncoder.getPosition());
    Logger.recordOutput(
        JointConstants.SUBSYSTEM_NAME + "/AngleVelocity", jointEncoder.getVelocity());
    Logger.recordOutput(
        JointConstants.SUBSYSTEM_NAME + "/AbsoluteAngle", jointEncoder.getPosition());
    Logger.recordOutput(JointConstants.SUBSYSTEM_NAME + "/AngleGoal", jointPID.getGoal().position);
    Logger.recordOutput(JointConstants.SUBSYSTEM_NAME + "/AngleSetpoint", this.setpoint);
    Logger.recordOutput(
        JointConstants.SUBSYSTEM_NAME + "/AngleFeedforward",
        jointFeedforward.calculate(
            Math.toRadians(jointEncoder.getPosition() + JointConstants.JOINT_COG_OFFSET),
            jointEncoder.getVelocity()));
    // Logger.recordOutput(
    //     JointConstants.SUBSYSTEM_NAME + "/AbsoluteIsConnected",
    // jointAbsoluteEncoder.isConnected());
    Logger.recordOutput(
        JointConstants.SUBSYSTEM_NAME + "/AbsoluteEncoderDegrees", getAbsolutePosition());
  }

  public void resetToAbsoluteEncoder() {
    // if (jointAbsoluteEncoder.isConnected()) {
    jointEncoder.setPosition(angleModulusDegZero360(getAbsolutePosition()));
    // }
  }

  public void runJointForward() {
    jointMotor.set(JointConstants.JOINT_FORWARD_SPEED);
  }

  public void runJointBackward() {
    jointMotor.set(JointConstants.JOINT_REVERSE_SPEED);
  }

  public void stopJoint() {
    jointMotor.stopMotor();
  }

  public double getSetPoint() {
    return setpoint;
  }

  public void setupShuffleboard() {
    jointTab = Shuffleboard.getTab("Joint Tab");
    jointTab.addDouble("Setpoint", () -> getSetPoint());
    jointTab.addDouble("Encoder Position", () -> jointEncoder.getPosition());
    jointTab.addDouble("Absolute Position", () -> jointAbsoluteEncoder.getPosition());
    jointTab.addDouble("Absolute Position (Degrees)", () -> getAbsolutePosition());

    jointTab.add("PID Controllor", this.jointPID);
    //    jointTab.add("Absolute Encoder", jointAbsoluteEncoder);

    // jointP = jointTab.add("Shooter P", JointConstants.JOINT_KP).getEntry();
    // jointI = jointTab.add("Shooter I", JointConstants.JOINT_KI).getEntry();
    // jointD = jointTab.add("Shooter D", JointConstants.JOINT_KD).getEntry();

    /*
     * goalEntry =
     * jointTab
     * .add("Goal", 0)
     * .withWidget(BuiltInWidgets.kNumberSlider)
     * .withProperties(
     * Map.of(
     * "min",
     * JointConstants.MIN_JOINT_DEGREES,
     * "max",
     * JointConstants.MAX_JOINT_DEGREES))
     * .getEntry();
     */
  }

  private double angleModulusDegZero360(double angleDeg) {
    return Math.toDegrees(MathUtil.inputModulus(Math.toRadians(angleDeg), 0, 2 * Math.PI));
  }
}
