package frc.robot.subsystems.intake_subsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.armevator.ArmevatorPose;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private HashMap<ArmevatorPose, Double> intakeMap;
  private double algaeAngleSetpoint;

  private PIDController intakePID;
  private ArmFeedforward intakeFeedforward;

  private RelativeEncoder tiltEncoder;

  private TalonFX rollerMotor;
  private TalonFX intakeMotor;

  private ArmevatorPose pose;

  private double targetSetpoint;
  private GenericEntry tiltSpeed;

  private NetworkTableInstance table = NetworkTableInstance.getDefault();

  private NetworkTable networkTable = table.getTable("IntakeConstants");

  private GenericEntry subscriberIntakeGoalTolerance =
      networkTable.getTopic("intakeGoalTolerance").getGenericEntry();
  private GenericEntry subscriberIntakeSetpoint =
      networkTable.getTopic("intakeSetpoint").getGenericEntry();
  private GenericEntry subscriberIntakeKG = networkTable.getTopic("intakeKG").getGenericEntry();

  public Intake() {
    subscriberIntakeGoalTolerance.setDouble(IntakeConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
    subscriberIntakeSetpoint.setDouble(-9.0);
    subscriberIntakeKG.setDouble(IntakeConstants.INTAKE_KG);

    rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
    intakeMotor = new TalonFX(IntakeConstants.TILT_MOTOR_ID);

    intakeMap = new HashMap<>();
    intakeMap.put(ArmevatorPose.STARTING, -9.0);
    intakeMap.put(ArmevatorPose.CLIMB, 15.0);
    intakeMap.put(ArmevatorPose.CORAL_L4_STAGE, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_HP_LOAD, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_L4_SCORE, -9.0);
    intakeMap.put(ArmevatorPose.CORAL_L3_SCORE, -9.0);
    intakeMap.put(ArmevatorPose.CORAL_L2_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_L1_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.CORAL_POST_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.ALGAE_INTAKE, 62.0);
    intakeMap.put(ArmevatorPose.ALGAE_PRE_HANDOFF, 20.0);
    intakeMap.put(ArmevatorPose.ALGAE_HANDOFF, 0.0);
    intakeMap.put(ArmevatorPose.ALGAE_POST_HANDOFF, 10.0);
    intakeMap.put(ArmevatorPose.ALGAE_NET_SCORE, 5.0);
    intakeMap.put(ArmevatorPose.ALGAE_NET_STAGE, 10.0);
    intakeMap.put(ArmevatorPose.ALGAE_L3_PLUCK, 0.0);
    intakeMap.put(ArmevatorPose.ALGAE_L3_DROP, 15.0);
    intakeMap.put(ArmevatorPose.ALGAE_L2_PLUCK, 0.0);
    intakeMap.put(ArmevatorPose.ALGAE_L2_DROP, 10.0);
    intakeMap.put(ArmevatorPose.ALGAE_L2_PLUCK, 5.0);
    intakeMap.put(ArmevatorPose.ALGAE_PRE_PLUCK_L2, -9.0);
    intakeMap.put(ArmevatorPose.ALGAE_PRE_PLUCK_L3, -9.0);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    intakeConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    intakeConfig.Voltage.withPeakForwardVoltage(12.0);
    intakeConfig.Voltage.withPeakReverseVoltage(-12.0);
    intakeConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    intakeConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    intakeConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(
        IntakeConstants.MAX_ANGLE_DEGREES / IntakeConstants.INTAKE_DEGREES_PER_ROTATION);
    intakeConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(
        IntakeConstants.MIN_ANGLE_DEGREES / IntakeConstants.INTAKE_DEGREES_PER_ROTATION);
    intakeConfig.CurrentLimits.withStatorCurrentLimit(40.0);

    intakeMotor.getConfigurator().apply(intakeConfig);

    intakeMotor.setPosition(
        intakeMap.get(ArmevatorPose.STARTING) / IntakeConstants.INTAKE_DEGREES_PER_ROTATION);

    intakeFeedforward =
        new ArmFeedforward(
            IntakeConstants.INTAKE_KS,
            IntakeConstants.INTAKE_KG,
            IntakeConstants.INTAKE_KV,
            IntakeConstants.INTAKE_KA);

    intakePID =
        new PIDController(
            IntakeConstants.INTAKE_KP, IntakeConstants.INTAKE_KI, IntakeConstants.INTAKE_KD);
    intakePID.setTolerance(IntakeConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
    intakePID.reset();

    intakeMotor.stopMotor();

    setupNT();
  }

  public boolean isAtGoal() {
    return intakePID.atSetpoint();
  }

  public void runIntake() {
    rollerMotor.set(IntakeConstants.ROLLER_INTAKE_SPEED);
  }

  public void ejectIntake() {
    rollerMotor.set(IntakeConstants.ROLLER_EJECT_SPEED);
  }

  public void tiltForward() {
    intakeMotor.set(tiltSpeed.getDouble(IntakeConstants.INTAKE_TILT_SPEED));
  }

  public void tiltBackwards() {
    intakeMotor.set(-tiltSpeed.getDouble(IntakeConstants.INTAKE_TILT_SPEED));
  }

  public void stopTilt() {
    intakeMotor.set(0);
  }

  public void stopIntake() {
    rollerMotor.set(0);
  }

  public double getTiltPosition() {
    return intakeMotor.getPosition().getValueAsDouble();
  }

  public double getTiltVelocity() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  public void doConstantChecks() {
    double setGoalTolerance =
        subscriberIntakeGoalTolerance.getDouble(IntakeConstants.ANGLE_GOAL_TOLERANCE_DEGREES);
    if (intakePID.getPositionTolerance() != setGoalTolerance) {
      intakePID.setTolerance(setGoalTolerance);
      System.out.println("Updated intake degree tolerance: " + setGoalTolerance);
    }

    double setIntakeKG = subscriberIntakeKG.getDouble(IntakeConstants.INTAKE_KG);
    if (intakeFeedforward.getKg() != setIntakeKG) {
      intakeFeedforward =
          new ArmFeedforward(
              IntakeConstants.INTAKE_KS,
              setIntakeKG,
              IntakeConstants.INTAKE_KV,
              IntakeConstants.INTAKE_KA);

      System.out.println("Updated intake KG: " + setIntakeKG);
    }
    double getNewSetpoint = subscriberIntakeSetpoint.getDouble(-9.0);
    if (targetSetpoint != getNewSetpoint) {
      targetSetpoint = getNewSetpoint;
    }
  }

  @Override
  public void periodic() {
    // doConstantChecks();

    if (DriverStation.isDisabled()) {
      intakePID.reset();
    }

    double output = intakePID.calculate(getAngle(), targetSetpoint);
    intakeMotor.set(
        output
            + intakeFeedforward.calculate(
                MathUtil.angleModulus(Math.toRadians(getAngle() + 90.0)), getVelocity()));

    doLogging();
  }

  public double getAngle() {
    return new Rotation2d(intakeMotor.getPosition().getValue()).getRotations()
        * IntakeConstants.INTAKE_DEGREES_PER_ROTATION;
  }

  public double getVelocity() {
    return intakeMotor.getVelocity().getValueAsDouble()
        * IntakeConstants.INTAKE_RPM_TO_DEGREES_PER_SECOND
        * 60;
  }

  public void setTargetPose(ArmevatorPose pose) {
    this.pose = pose;
    intakePID.reset();
    targetSetpoint = intakeMap.get(pose);
  }

  public ArmevatorPose getPose() {
    return pose;
  }

  private void doLogging() {
    Logger.recordOutput(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Position", getTiltPosition());
    Logger.recordOutput(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Velocity", getTiltVelocity());
    Logger.recordOutput(IntakeConstants.SUBSYSTEM_NAME + "/Tilt Goal", targetSetpoint);
  }

  private void setupNT() {

    ShuffleboardTab tab = Shuffleboard.getTab(IntakeConstants.SUBSYSTEM_NAME);

    tab.addDouble("Tilt Position", this::getAngle);
    tab.addDouble("Tilt Velocity", this::getVelocity);
    // tab.addDouble("Tilt Setpoint", () -> targetSetpoint);
    tiltSpeed = tab.add("Tilt Speed Set", IntakeConstants.INTAKE_TILT_SPEED).getEntry();

    tab.add("Tilt PID", intakePID);
  }
}
