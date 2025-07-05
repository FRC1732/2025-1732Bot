// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.DriveToPoseSlew;
import frc.robot.commands.DynamicCommand;
// import frc.lib.team3061.leds.LEDs;
import frc.robot.commands.clawcommands.ClawBackwards;
import frc.robot.commands.clawcommands.IntakeCoral;
import frc.robot.field.Field2d;
import frc.robot.field.FieldObject;
import frc.robot.field.Region2d;
import frc.robot.generated.TunerConstants;
import frc.robot.limelightVision.ApriltagVision.VisionApriltagConstants.Pipelines;
import frc.robot.limelightVision.ApriltagVision.VisionApriltagSubsystem;
import frc.robot.limelightVision.LimelightHelpers;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.L1Scorer.L1Scorer;
import frc.robot.subsystems.L1Scorer.L1ScorerPose;
import frc.robot.subsystems.QuestNavLoggerSubsystem;
import frc.robot.subsystems.armevator.Armevator;
import frc.robot.subsystems.armevator.ArmevatorPose;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.climber_subsystem.Climber;
import frc.robot.subsystems.intake_subsystem.Intake;
import frc.robot.subsystems.rgb.ScoringLevel;
import frc.robot.subsystems.rgb.ScoringPosition;
import frc.robot.subsystems.rgb.StatusRgb;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};

  private Claw claw;
  private StatusRgb statusRgb;
  private VisionApriltagSubsystem visionApriltagSubsystem;
  private QuestNav questNav = new QuestNav();
  private QuestNavLoggerSubsystem questNavLoggerSubsystem = new QuestNavLoggerSubsystem(questNav);
  private Armevator armevator;
  private Intake intake;
  private Climber climber;
  private L1Scorer l1Scorer;

  private static final double NET_SCORE_LOCATION_X = 7.55;
  private static final double FAR_NET_SCORE_LOCATION_X = (8.774 - NET_SCORE_LOCATION_X) + 8.774;
  private static final double NET_SCORE_MIN_Y = 3.5;
  private static final double NET_SCORE_MAX_Y = 8.0;

  private static final Pose2d APRILTAG_POSE_F =
      new Pose2d(3.6576, 4.0208, Rotation2d.fromDegrees(180));
  private static final Pose2d APRILTAG_POSE_BL =
      new Pose2d(4.9047, 4.7404, Rotation2d.fromDegrees(60));
  private static final Pose2d APRILTAG_POSE_BR =
      new Pose2d(4.9047, 3.3012, Rotation2d.fromDegrees(-60));
  private static final Pose2d APRILTAG_POSE_B =
      new Pose2d(5.321046, 4.0208, Rotation2d.fromDegrees(0));

  private Alliance lastAlliance = Alliance.Blue; // Field2d.getInstance().getAlliance();

  public final CommandSwerveDrivetrain drivetrain =
      TunerConstants.createDrivetrain((pose) -> questNav.resetPose(pose));

  private Command fourPieceRight;

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(1.5)
          .in(RadiansPerSecond); // 1.5 rotations per second max angular velocity
  private double MaxSlowSpeed = 0.25 * MaxSpeed; // 25% of max speed
  private double MaxSlowAngularRate = 0.25 * MaxAngularRate; // 25% of max angular rate
  private boolean isSlowMode = false;
  private boolean isFullAuto = true;
  private boolean preferNetRightSide = false;
  private boolean isPlucking = false;
  private boolean isVisionEnabled = true;
  private boolean isPluckTargetHigh = false;
  private boolean isRunningPath = false;
  private boolean isL1ModeEnabled = false;

  private boolean driveSlowlyDirectionAlert =
      false; // do drive slowly cyan flash when bot is stopped

  private BooleanSupplier slowModeSupplier = () -> isSlowMode;
  private BooleanSupplier preferNetRightSideSupplier = () -> preferNetRightSide;
  private BooleanSupplier isFullAutoSupplier = () -> isFullAuto;
  private BooleanSupplier isPluckingSupplier = () -> isPlucking;
  private BooleanSupplier isVisionEnabledSupplier = () -> isVisionEnabled;
  private BooleanSupplier isPluckTargetHighSupplier = () -> isPluckTargetHigh;

  private boolean adjustingRight = false;

  public enum AprilTagStatus {
    REEF_TARGET_IN_RANGE,
    REEF_TARGET_OUTSIDE_RANGE,
    NO_TARGET,
  }

  private AprilTagStatus apriltagStatus = AprilTagStatus.NO_TARGET;
  private Supplier<AprilTagStatus> apriltagStatusSupplier = () -> apriltagStatus;

  private Pose2d currentPathPose = new Pose2d();

  private ArmevatorPose currentScoringLevel = ArmevatorPose.CORAL_L4_SCORE;
  private Supplier<ArmevatorPose> currentScoringLevelSupplier = () -> currentScoringLevel;
  private ShuffleboardTab tab;
  private final Telemetry telemetryLogger = new Telemetry(MaxSpeed);

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.01)
          .withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband to raw input
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.ApplyRobotSpeeds driveWithSpeedsRequest =
      new SwerveRequest.ApplyRobotSpeeds();

  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngleRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(MaxSpeed * 0.01)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to
  // ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #1", 20.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #2", 10.0);
  private Alert pathFileMissingAlert =
      new Alert("Could not find the specified path file.", AlertType.kError);
  private static final String LAYOUT_FILE_MISSING =
      "Could not find the specified AprilTags layout file";
  private Alert layoutFileMissingAlert = new Alert(LAYOUT_FILE_MISSING, AlertType.kError);
  private Alert tuningAlert = new Alert("Tuning mode enabled", AlertType.kInfo);

  private boolean preferHpLeftSide = false;

  StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault().getStructTopic("robotPose", Pose2d.struct).publish();
  StructPublisher<Pose2d> questPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("questPose", Pose2d.struct).publish();

  DoublePublisher pathfindErrorNetwork =
      NetworkTableInstance.getDefault().getDoubleTopic("PathfindError").publish();

  PathConstraints hpPathConstraints = new PathConstraints(4.5, 3.2, 8.42, 12.8876585);
  PathConstraints pluckPathConstraints = new PathConstraints(4.5, 3.2, 8.0, 10.0);
  PathConstraints scorePathConstraints = new PathConstraints(4.5, 3.2, 8.0, 10.0);

  PathPlannerPath pathF1;
  PathPlannerPath pathF2;
  PathPlannerPath pathFL1;
  PathPlannerPath pathFL2;
  PathPlannerPath pathFR1;
  PathPlannerPath pathFR2;
  PathPlannerPath pathBL1;
  PathPlannerPath pathBL2;
  PathPlannerPath pathBR1;
  PathPlannerPath pathBR2;
  PathPlannerPath pathB1;
  PathPlannerPath pathB2;
  PathPlannerPath pathLeftHP;
  PathPlannerPath pathRightHP;

  PathPlannerPath pathFAlgae;
  PathPlannerPath pathFLAlgae;
  PathPlannerPath pathFRAlgae;
  PathPlannerPath pathBLAlgae;
  PathPlannerPath pathBRAlgae;
  PathPlannerPath pathBAlgae;

  PathPlannerPath pathR2HP;

  PathPlannerPath pathL1F1;
  PathPlannerPath pathL1F2;
  PathPlannerPath pathL1FL1;
  PathPlannerPath pathL1FL2;
  PathPlannerPath pathL1FR1;
  PathPlannerPath pathL1FR2;
  PathPlannerPath pathL1BL1;
  PathPlannerPath pathL1BL2;
  PathPlannerPath pathL1BR1;
  PathPlannerPath pathL1BR2;
  PathPlannerPath pathL1B1;
  PathPlannerPath pathL1B2;

  private ScoringPathOption scoringPathOption = ScoringPathOption.PATH_F1;

  public enum ScoringPathOption {
    PATH_F1,
    PATH_F2,
    PATH_FL1,
    PATH_FL2,
    PATH_FR1,
    PATH_FR2,
    PATH_BL1,
    PATH_BL2,
    PATH_BR1,
    PATH_BR2,
    PATH_B1,
    PATH_B2
  }

  Map<ScoringPathOption, Command> scoringPathMap = new HashMap<>(12);
  Map<ScoringPathOption, Command> simpleScoringPathMap = new HashMap<>(12);
  Map<ScoringPathOption, Command> simplePluckScoringMap = new HashMap<>(12);
  Map<ScoringPathOption, Command> pluckAlgaePathMap = new HashMap<>(12);
  Map<ScoringPathOption, Rotation2d> scoringAngleMap = new HashMap<>(12);
  Map<ScoringPathOption, Command> scoringPathL1Map = new HashMap<>(12);

  private Field2d field2d;

  /**
   * Create the container for the robot. Contains subsystems, operator interface (OI) devices, and
   * commands.
   */
  public RobotContainer() {
    try {
      pathF1 = PathPlannerPath.fromPathFile("F1");
      pathF2 = PathPlannerPath.fromPathFile("F2");
      pathFL1 = PathPlannerPath.fromPathFile("FL1");
      pathFL2 = PathPlannerPath.fromPathFile("FL2");
      pathFR1 = PathPlannerPath.fromPathFile("FR1");
      pathFR2 = PathPlannerPath.fromPathFile("FR2");
      pathBL1 = PathPlannerPath.fromPathFile("BL1");
      pathBL2 = PathPlannerPath.fromPathFile("BL2");
      pathBR1 = PathPlannerPath.fromPathFile("BR1");
      pathBR2 = PathPlannerPath.fromPathFile("BR2");
      pathB1 = PathPlannerPath.fromPathFile("B1");
      pathB2 = PathPlannerPath.fromPathFile("B2");
      pathLeftHP = PathPlannerPath.fromPathFile("LeftHP");
      pathRightHP = PathPlannerPath.fromPathFile("RightHP");

      pathFAlgae = PathPlannerPath.fromPathFile("F Algae");
      pathFLAlgae = PathPlannerPath.fromPathFile("FL Algae");
      pathFRAlgae = PathPlannerPath.fromPathFile("FR Algae");
      pathBLAlgae = PathPlannerPath.fromPathFile("BL Algae");
      pathBRAlgae = PathPlannerPath.fromPathFile("BR Algae");
      pathBAlgae = PathPlannerPath.fromPathFile("B Algae");

      pathR2HP = PathPlannerPath.fromPathFile("R2-HP");

      /* L1 scroing paths */
      pathL1F1 = PathPlannerPath.fromPathFile("L1 F1");
      pathL1F2 = PathPlannerPath.fromPathFile("L1 F2");
      pathL1FL1 = PathPlannerPath.fromPathFile("L1 FL1");
      pathL1FL2 = PathPlannerPath.fromPathFile("L1 FL2");
      pathL1FR1 = PathPlannerPath.fromPathFile("L1 FR1");
      pathL1FR2 = PathPlannerPath.fromPathFile("L1 FR2");
      pathL1BL1 = PathPlannerPath.fromPathFile("L1 BL1");
      pathL1BL2 = PathPlannerPath.fromPathFile("L1 BL2");
      pathL1BR1 = PathPlannerPath.fromPathFile("L1 BR1");
      pathL1BR2 = PathPlannerPath.fromPathFile("L1 BR2");
      pathL1B1 = PathPlannerPath.fromPathFile("L1 B1");
      pathL1B2 = PathPlannerPath.fromPathFile("L1 B2");
    } catch (Exception e) {
      System.out.println(e.getMessage());
    }

    setupScoringPathMap();

    defineSubsystems();

    // disable all telemetry in the LiveWindow to reduce the processing during each
    // iteration
    LiveWindow.disableAllTelemetry();

    constructField();

    updateOI();

    configureAutoCommands();

    // Alert when tuning
    if (Constants.TUNING_MODE) {
      this.tuningAlert.set(true);
    }

    pathfindErrorNetwork.set(0.0);
  }

  private void defineSubsystems() {
    claw = new Claw();
    armevator = new Armevator();
    statusRgb =
        new StatusRgb(
            armevator,
            () -> false,
            this::getCurrentPathfindError,
            isFullAutoSupplier,
            apriltagStatusSupplier);
    intake = new Intake();
    climber = new Climber();
    l1Scorer = new L1Scorer();

    visionApriltagSubsystem =
        new VisionApriltagSubsystem(() -> drivetrain.getPose().getRotation().getDegrees());
  }

  /**
   * Creates the field from the defined regions and transition points from one region to its
   * neighbor. The field is used to generate paths.
   */
  private void constructField() {
    field2d = Field2d.getInstance();
    field2d.setRegions(new Region2d[] {});

    SmartDashboard.putData("Field", field2d);
  }

  /**
   * This method scans for any changes to the connected operator interface (e.g., joysticks). If
   * anything changed, it creates a new OI object and binds all of the buttons to commands.
   */
  public void updateOI() {
    OperatorInterface prevOI = oi;
    oi = OISelector.getOperatorInterface();
    if (oi == prevOI) {
      return;
    }
    System.out.println(oi.getClass());

    // clear the list of composed commands since we are about to rebind them to
    // potentially new
    // triggers
    CommandScheduler.getInstance().clearComposedCommands();
    configureButtonBindings();
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    configureDrivetrainCommands();

    configureSubsystemCommands();

    configureVisionCommands();

    new PrintOperatorPanelTests(
        oi); // this is for verifying operator panel buttons. not for competition
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    NamedCommands.registerCommand(
        "intakeCoral",
        Commands.sequence(
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_HP_LOAD)),
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_HP_LOAD)),
            new IntakeCoral(claw, statusRgb)));
    NamedCommands.registerCommand("brakeCoral", new IntakeCoral(claw, statusRgb));
    NamedCommands.registerCommand("ejectCoral", new ClawBackwards(claw));
    NamedCommands.registerCommand(
        "setPoseL4",
        Commands.sequence(
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L4_SCORE)),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_L4_SCORE))));
    NamedCommands.registerCommand(
        "setPoseL3",
        Commands.sequence(
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L4_SCORE)),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_L3_SCORE))));
    NamedCommands.registerCommand(
        "setPosePrePluckHigh",
        Commands.sequence(
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L4_SCORE)),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_PRE_PLUCK_L3))));
    NamedCommands.registerCommand(
        "setPosePrePluckLow",
        Commands.sequence(
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L4_SCORE)),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_PRE_PLUCK_L2))));
    NamedCommands.registerCommand(
        "setPoseL4Wait",
        Commands.sequence(
            Commands.waitUntil(armevator::isMaxHeight),
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L4_SCORE)),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_L4_SCORE))));
    NamedCommands.registerCommand(
        "setPoseStage",
        Commands.sequence(
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L4_STAGE)),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_L4_STAGE))));
    NamedCommands.registerCommand(
        "setPoseFlip",
        armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_L4_FLIP)));
    NamedCommands.registerCommand(
        "setPipelineLocalization",
        new InstantCommand(() -> visionApriltagSubsystem.setPipeline(Pipelines.LOCALIZATION)));
    NamedCommands.registerCommand(
        "setPipelineCenter",
        new InstantCommand(() -> visionApriltagSubsystem.setPipeline(Pipelines.TRACKING_CENTER)));
    NamedCommands.registerCommand(
        "setPipelineLeft",
        new InstantCommand(
            () ->
                visionApriltagSubsystem.setPipeline(
                    isAutoFlipped().getAsBoolean()
                        ? Pipelines.TRACKING_RIGHT
                        : Pipelines.TRACKING_LEFT)));
    NamedCommands.registerCommand(
        "setPipelineRight",
        new InstantCommand(
            () ->
                visionApriltagSubsystem.setPipeline(
                    isAutoFlipped().getAsBoolean()
                        ? Pipelines.TRACKING_LEFT
                        : Pipelines.TRACKING_RIGHT)));
    NamedCommands.registerCommand(
        "localizeRobot",
        new ConditionalCommand(
            Commands.deadline(
                Commands.sequence(
                    new WaitCommand(0.4),
                    Commands.runOnce(
                        () -> {
                          System.out.println("curPose: " + drivetrain.getPose().toString());
                          Pose2d visionPose =
                              inferPoseFromTarget(
                                  isAutoFlipped().getAsBoolean()
                                      ? APRILTAG_POSE_BR
                                      : APRILTAG_POSE_BL,
                                  visionApriltagSubsystem.getTX());
                          drivetrain.resetPose(visionPose);
                          questNav.resetPose(visionPose);
                          System.out.println("visionPose: " + visionPose.toString());
                        })),
                drivetrain.run(
                    () ->
                        driveSlowlyDirection(
                            isAutoFlipped().getAsBoolean()
                                ? scoringAngleMap.get(scoringPathOption.PATH_BR1)
                                : scoringAngleMap.get(scoringPathOption.PATH_BL1)))),
            new InstantCommand(),
            () -> true)); // visionApriltagSubsystem.hasReefTarget()));
    NamedCommands.registerCommand(
        "localizeRobotB",
        new ConditionalCommand(
            Commands.deadline(
                Commands.sequence(
                    new WaitCommand(0.4),
                    Commands.runOnce(
                        () -> {
                          System.out.println("curPose: " + drivetrain.getPose().toString());
                          Pose2d visionPose =
                              inferPoseFromTarget(APRILTAG_POSE_B, visionApriltagSubsystem.getTX());
                          drivetrain.resetPose(visionPose);
                          questNav.resetPose(visionPose);
                          System.out.println("visionPose: " + visionPose.toString());
                        })),
                drivetrain.run(
                    () -> driveSlowlyDirection(scoringAngleMap.get(scoringPathOption.PATH_B1)))),
            new InstantCommand(),
            () -> true)); // visionApriltagSubsystem.hasReefTarget()));
    NamedCommands.registerCommand(
        "driveHpSlowly",
        drivetrain.run(
            () ->
                driveSlowlyDirection(
                    isAutoFlipped().getAsBoolean()
                        ? Rotation2d.fromDegrees(-120)
                        : Rotation2d.fromDegrees(120))));
    NamedCommands.registerCommand(
        "driveFlSlowly",
        drivetrain.run(
            () ->
                driveSlowlyDirection(
                    isAutoFlipped().getAsBoolean()
                        ? scoringAngleMap.get(scoringPathOption.PATH_FR1)
                        : scoringAngleMap.get(scoringPathOption.PATH_FL1))));
    NamedCommands.registerCommand(
        "driveBlSlowly",
        drivetrain.run(
            () ->
                driveSlowlyDirection(
                    isAutoFlipped().getAsBoolean()
                        ? scoringAngleMap.get(scoringPathOption.PATH_BR1)
                        : scoringAngleMap.get(scoringPathOption.PATH_BL1))));
    NamedCommands.registerCommand(
        "driveBrSlowly",
        drivetrain.run(
            () ->
                driveSlowlyDirection(
                    isAutoFlipped().getAsBoolean()
                        ? scoringAngleMap.get(scoringPathOption.PATH_BL1)
                        : scoringAngleMap.get(scoringPathOption.PATH_BR1))));
    NamedCommands.registerCommand(
        "driveFSlowly",
        drivetrain.run(() -> driveSlowlyDirection(scoringAngleMap.get(scoringPathOption.PATH_F1))));
    NamedCommands.registerCommand(
        "driveBSlowly",
        drivetrain.run(() -> driveSlowlyDirection(scoringAngleMap.get(scoringPathOption.PATH_B1))));
    NamedCommands.registerCommand(
        "adjustBlSlowly",
        getAdjustSlowlyCommand(
            () ->
                isAutoFlipped().getAsBoolean()
                    ? scoringAngleMap.get(scoringPathOption.PATH_BR1)
                    : scoringAngleMap.get(scoringPathOption.PATH_BL1),
            () -> !isAutoFlipped().getAsBoolean()));
    NamedCommands.registerCommand(
        "adjustBrSlowly",
        getAdjustSlowlyCommand(
            () ->
                !isAutoFlipped().getAsBoolean()
                    ? scoringAngleMap.get(scoringPathOption.PATH_BR1)
                    : scoringAngleMap.get(scoringPathOption.PATH_BL1),
            () -> isAutoFlipped().getAsBoolean()));
    NamedCommands.registerCommand(
        "adjustBl1Slowly",
        getAdjustSlowlyCommand(
            () ->
                isAutoFlipped().getAsBoolean()
                    ? scoringAngleMap.get(scoringPathOption.PATH_BR1)
                    : scoringAngleMap.get(scoringPathOption.PATH_BL1),
            () -> isAutoFlipped().getAsBoolean()));
    NamedCommands.registerCommand(
        "adjustFl1Slowly",
        getAdjustSlowlyCommand(
            () ->
                isAutoFlipped().getAsBoolean()
                    ? scoringAngleMap.get(scoringPathOption.PATH_FR1)
                    : scoringAngleMap.get(scoringPathOption.PATH_FL1),
            isAutoFlipped()));
    NamedCommands.registerCommand(
        "adjustFl2Slowly",
        getAdjustSlowlyCommand(
            () ->
                isAutoFlipped().getAsBoolean()
                    ? scoringAngleMap.get(scoringPathOption.PATH_FR1)
                    : scoringAngleMap.get(scoringPathOption.PATH_FL1),
            () -> !isAutoFlipped().getAsBoolean()));
    NamedCommands.registerCommand(
        "adjustFSlowly",
        getAdjustSlowlyCommand(
            () -> scoringAngleMap.get(scoringPathOption.PATH_F1),
            () -> !isAutoFlipped().getAsBoolean()));
    NamedCommands.registerCommand(
        "adjustBSlowly",
        getAdjustSlowlyCommand(
            () -> scoringAngleMap.get(scoringPathOption.PATH_B1),
            () -> isAutoFlipped().getAsBoolean()));
    NamedCommands.registerCommand(
        "adjustPluckSlowly",
        getAutoPluckAdjustSlowlyCommand(
            () -> scoringAngleMap.get(scoringPathOption.PATH_B1),
            () -> isAutoFlipped().getAsBoolean()));
    NamedCommands.registerCommand(
        "adjustPluckSlowlyBL",
        getPluckAdjustSlowlyCommand(
            () -> scoringAngleMap.get(scoringPathOption.PATH_BL1),
            () -> isAutoFlipped().getAsBoolean()));
    NamedCommands.registerCommand(
        "adjustPluckQuicklyBL",
        getAutoPluckAdjustSlowlyCommand(
            () -> scoringAngleMap.get(scoringPathOption.PATH_BL1),
            () -> isAutoFlipped().getAsBoolean()));
    NamedCommands.registerCommand(
        "adjustPluckSlowlyBR",
        getPluckAdjustSlowlyCommand(
            () -> scoringAngleMap.get(scoringPathOption.PATH_BR1),
            () -> isAutoFlipped().getAsBoolean()));
    NamedCommands.registerCommand(
        "pluckAlgaeHigh",
        Commands.sequence(
            claw.runOnce(() -> claw.intakeAlgaePluck()),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_L3_PLUCK)),
            Commands.waitSeconds(0.85),
            claw.runOnce(() -> claw.intakeAlgae()),
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_NET_STAGE)),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_NET_STAGE))));
    NamedCommands.registerCommand(
        "pluckAlgaeHighEnd",
        Commands.sequence(
            claw.runOnce(() -> claw.intakeAlgaePluck()),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_L3_PLUCK)),
            Commands.waitSeconds(0.85),
            claw.runOnce(() -> claw.intakeAlgae()),
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_NET_STAGE))));
    NamedCommands.registerCommand(
        "pluckAlgaeLow",
        Commands.sequence(
            claw.runOnce(() -> claw.intakeAlgaePluck()),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_L2_PLUCK)),
            Commands.waitSeconds(0.85),
            claw.runOnce(() -> claw.intakeAlgae()),
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_NET_STAGE)),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_NET_STAGE))));
    NamedCommands.registerCommand(
        "shootNet",
        Commands.sequence(
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_NET_SCORE)),
            Commands.waitUntil(armevator::isAtNetReleaseAngle),
            claw.runOnce(() -> claw.ejectAlgae()),
            Commands.waitSeconds(0.2),
            claw.runOnce(() -> claw.stopClaw()),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_HP_LOAD))));
    NamedCommands.registerCommand(
        "shootNetFar",
        Commands.sequence(
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_NET_SCORE)),
            Commands.waitUntil(armevator::isAtNetReleaseAngleFar),
            claw.runOnce(() -> claw.ejectAlgae()),
            Commands.waitSeconds(0.2),
            claw.runOnce(() -> claw.stopClaw()),
            armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_HP_LOAD))));

    // Event Markers
    new EventTrigger("Marker").onTrue(Commands.print("reached event marker"));
    new EventTrigger("ZoneMarker").onTrue(Commands.print("entered zone"));
    new EventTrigger("ZoneMarker").onFalse(Commands.print("left zone"));

    // build auto path commands

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    Command startPoint = new PathPlannerAuto("Start Point");
    autoChooser.addOption("Start Point", startPoint);

    Command fourPiece = new PathPlannerAuto("4 piece");
    autoChooser.addOption("4 piece left", fourPiece);

    Command centerAlgaeFar = new PathPlannerAuto("Center Algae Far");
    autoChooser.addOption("Center Algae Far", centerAlgaeFar);

    Command centerCoral = new PathPlannerAuto("Center Coral");
    autoChooser.addOption("Center Coral", centerCoral);

    fourPieceRight = new PathPlannerAuto("4 piece", true);
    autoChooser.addOption("4 piece right", fourPieceRight);

    // Command startPoint =
    // Commands.runOnce(
    // () -> {
    // try {
    // drivetrain.resetPose(
    // PathPlannerPath.fromPathFile("Start Point").getStartingDifferentialPose());
    // } catch (Exception e) {
    // pathFileMissingAlert.setText("Could not find the specified path file: Start
    // Point");
    // pathFileMissingAlert.set(true);
    // }
    // },
    // drivetrain);
    // autoChooser.addOption("Start Point", startPoint);

    /************
     * Drive Velocity Tuning ************
     *
     * useful for tuning the drive velocity PID controller
     *
     */
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.deadline(
                Commands.waitSeconds(1.0),
                drivetrain.run(
                    () ->
                        drivetrain.setControl(
                            driveWithSpeedsRequest
                                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                                .withSpeeds(new ChassisSpeeds(2.0, 0, 0))))),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                drivetrain.run(
                    () ->
                        drivetrain.setControl(
                            driveWithSpeedsRequest
                                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                                .withSpeeds(new ChassisSpeeds(-0.5, 0, 0))))),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                drivetrain.run(
                    () ->
                        drivetrain.setControl(
                            driveWithSpeedsRequest
                                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                                .withSpeeds(new ChassisSpeeds(1.0, 0, 0))))),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                drivetrain.run(
                    () ->
                        drivetrain.setControl(
                            driveWithSpeedsRequest
                                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                                .withSpeeds(new ChassisSpeeds(3.0, 0, 0))))),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                drivetrain.run(
                    () ->
                        drivetrain.setControl(
                            driveWithSpeedsRequest
                                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                                .withSpeeds(new ChassisSpeeds(1.0, 0, 0))))),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                drivetrain.run(
                    () ->
                        drivetrain.setControl(
                            driveWithSpeedsRequest
                                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                                .withSpeeds(new ChassisSpeeds(-1.0, 0, 0))))),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                drivetrain.run(
                    () ->
                        drivetrain.setControl(
                            driveWithSpeedsRequest
                                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                                .withSpeeds(new ChassisSpeeds(-3.0, 0, 0))))),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                drivetrain.run(
                    () ->
                        drivetrain.setControl(
                            driveWithSpeedsRequest
                                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                                .withSpeeds(new ChassisSpeeds(-1.0, 0, 0)))))));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  private void driveFacingAngle(double xVelocity, double yVelocity, Rotation2d targetDirection) {
    drivetrain.setControl(
        driveFacingAngleRequest
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withTargetDirection(targetDirection));
  }

  private void driveSlowlyDirection(Rotation2d targetDirection) {
    drivetrain.setControl(
        driveRequest
            .withVelocityX(0.3 * Math.cos(targetDirection.getRadians()))
            .withVelocityY(0.3 * Math.sin(targetDirection.getRadians()))
            .withRotationalRate(0.0));

    driveSlowlyDirectionAlert = true;
  }

  private void driveSlowlyDirectionLocked(
      Rotation2d targetDirection, Rotation2d rotationDireciton) {
    drivetrain.setControl(
        driveFacingAngleRequest
            .withVelocityX(0.3 * Math.cos(targetDirection.getRadians()))
            .withVelocityY(0.3 * Math.sin(targetDirection.getRadians()))
            .withTargetDirection(rotationDireciton));

    driveSlowlyDirectionAlert = true;
  }

  private void driveAutoSlowlyDirectionLocked(
      Rotation2d targetDirection, Rotation2d rotationDireciton) {
    drivetrain.setControl(
        driveFacingAngleRequest
            .withVelocityX(0.6 * Math.cos(targetDirection.getRadians()))
            .withVelocityY(0.6 * Math.sin(targetDirection.getRadians()))
            .withTargetDirection(rotationDireciton));

    driveSlowlyDirectionAlert = true;
  }

  private Command getAdjustSlowlyCommand(
      Supplier<Rotation2d> targetDirectionSupplier, BooleanSupplier isTargetRight) {

    // Custom command that ends based on vision target conditions.
    Command timeoutCommand =
        new Command() {
          private double startTime;
          private double lastTargetTime;
          private boolean hasSeenTarget;
          private double timeoutSeconds;

          @Override
          public void initialize() {
            startTime = Timer.getFPGATimestamp();
            lastTargetTime = startTime;
            hasSeenTarget = false;
            timeoutSeconds = 0;
          }

          @Override
          public void execute() {
            if (visionApriltagSubsystem.hasReefTarget()) {
              hasSeenTarget = true;
              lastTargetTime = Timer.getFPGATimestamp();
              timeoutSeconds =
                  Math.max(Math.abs(visionApriltagSubsystem.getTX()) - 2.5, 0.0) * 0.09;
            }
          }

          @Override
          public boolean isFinished() {
            double currentTime = Timer.getFPGATimestamp();

            // Condition 1: Never saw a target and 0.5 seconds elapsed
            if (!hasSeenTarget && (currentTime - startTime) > 0.5) {
              return true;
            }

            // Condition 2: Time since last target visible exceeds calculated timeout
            if (hasSeenTarget && (currentTime - lastTargetTime) >= timeoutSeconds) {
              return true;
            }

            return false;
          }
        };

    return Commands.deadline(
        timeoutCommand,
        Commands.sequence(
            new InstantCommand(() -> adjustingRight = isTargetRight.getAsBoolean()),
            drivetrain.run(
                () -> {
                  if (visionApriltagSubsystem.hasReefTarget()) {
                    adjustingRight = visionApriltagSubsystem.getTX() > 0;
                  }
                  driveSlowlyDirectionLocked(
                      targetDirectionSupplier
                          .get()
                          .plus(Rotation2d.kCW_90deg.times(adjustingRight ? 1 : -1)),
                      targetDirectionSupplier.get());
                })));
  }

  private Command getPluckAdjustSlowlyCommand(
      Supplier<Rotation2d> targetDirectionSupplier, BooleanSupplier isTargetRight) {

    // Custom command that ends based on vision target conditions.
    Command timeoutCommand =
        new Command() {
          private double startTime;
          private double lastTargetTime;
          private boolean hasSeenTarget;
          private double timeoutSeconds;

          @Override
          public void initialize() {
            startTime = Timer.getFPGATimestamp();
            lastTargetTime = startTime;
            hasSeenTarget = false;
            timeoutSeconds = 0;
          }

          @Override
          public void execute() {
            if (visionApriltagSubsystem.hasReefTarget()) {
              hasSeenTarget = true;
              lastTargetTime = Timer.getFPGATimestamp();
              timeoutSeconds =
                  Math.max(Math.abs(visionApriltagSubsystem.getTX()) - 1.0, 0.0) * 0.09;
            }
          }

          @Override
          public boolean isFinished() {
            double currentTime = Timer.getFPGATimestamp();

            // Condition 1: Never saw a target and 0.5 seconds elapsed
            if (!hasSeenTarget && (currentTime - startTime) > 0.5) {
              return true;
            }

            // Condition 2: Time since last target visible exceeds calculated timeout
            if (hasSeenTarget && (currentTime - lastTargetTime) >= timeoutSeconds) {
              return true;
            }

            return false;
          }
        };

    return Commands.deadline(
        timeoutCommand,
        Commands.sequence(
            new InstantCommand(() -> adjustingRight = isTargetRight.getAsBoolean()),
            drivetrain.run(
                () -> {
                  if (visionApriltagSubsystem.hasReefTarget()) {
                    adjustingRight = visionApriltagSubsystem.getTX() > 0;
                  }
                  driveSlowlyDirectionLocked(
                      targetDirectionSupplier
                          .get()
                          .plus(Rotation2d.kCW_90deg.times(adjustingRight ? 1 : -1)),
                      targetDirectionSupplier.get());
                })));
  }

  private Command getAutoPluckAdjustSlowlyCommand(
      Supplier<Rotation2d> targetDirectionSupplier, BooleanSupplier isTargetRight) {

    // Custom command that ends based on vision target conditions.
    Command timeoutCommand =
        new Command() {
          private double startTime;
          private double lastTargetTime;
          private boolean hasSeenTarget;
          private double timeoutSeconds;

          @Override
          public void initialize() {
            startTime = Timer.getFPGATimestamp();
            lastTargetTime = startTime;
            hasSeenTarget = false;
            timeoutSeconds = 0;
          }

          @Override
          public void execute() {
            if (visionApriltagSubsystem.hasReefTarget()) {
              hasSeenTarget = true;
              lastTargetTime = Timer.getFPGATimestamp();
              timeoutSeconds =
                  Math.max(Math.abs(visionApriltagSubsystem.getTX()) - 3.5, 0.0) * 0.045;
            }
          }

          @Override
          public boolean isFinished() {
            double currentTime = Timer.getFPGATimestamp();

            // Condition 1: Never saw a target and 0.5 seconds elapsed
            if (!hasSeenTarget && (currentTime - startTime) > 0.25) {
              return true;
            }

            // Condition 2: Time since last target visible exceeds calculated timeout
            if (hasSeenTarget && (currentTime - lastTargetTime) >= timeoutSeconds) {
              return true;
            }

            return false;
          }
        };

    return Commands.deadline(
        timeoutCommand,
        Commands.sequence(
            new InstantCommand(() -> adjustingRight = isTargetRight.getAsBoolean()),
            drivetrain.run(
                () -> {
                  if (visionApriltagSubsystem.hasReefTarget()) {
                    adjustingRight = visionApriltagSubsystem.getTX() > 0;
                  }
                  driveAutoSlowlyDirectionLocked(
                      targetDirectionSupplier
                          .get()
                          .plus(Rotation2d.kCW_90deg.times(adjustingRight ? 1 : -1)),
                      targetDirectionSupplier.get());
                })));
  }

  private BooleanSupplier isAutoFlipped() {
    return () -> getAutonomousCommand() == fourPieceRight;
  }

  private void configureDrivetrainCommands() {
    /*
     * Set up the default command for the drivetrain. The joysticks' values map to
     * percentage of the
     * maximum velocities. The velocities may be specified from either the robot's
     * frame of
     * reference or the field's frame of reference.
     * Robot-centric: +x is forward, +y is left, +theta is CCW
     * Field-centric: origin is back-right (blue), 0deg is forward, +x is forward,
     * +y is left,
     * +theta is CCW direction.
     * ___________
     * | | | ^
     * (0,0).____|____| y, x-> 0->
     */
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                driveRequest
                    .withVelocityX(
                        -oi.getTranslateX()
                            * (slowModeSupplier.getAsBoolean()
                                ? MaxSlowSpeed
                                : MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -oi.getTranslateY()
                            * (slowModeSupplier.getAsBoolean()
                                ? MaxSlowSpeed
                                : MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(
                        -oi.getRotate()
                            * (slowModeSupplier.getAsBoolean()
                                ? MaxSlowAngularRate
                                : MaxAngularRate)) // Drive counterclockwise with negative X
            // (left)
            ));

    driveFacingAngleRequest.HeadingController.setPID(7, 0, 0);

    oi.operatorAlignForClimb()
        .whileTrue(
            drivetrain.run(
                () ->
                    driveFacingAngle(
                        -oi.getTranslateX()
                            * (slowModeSupplier.getAsBoolean() ? MaxSlowSpeed : MaxSpeed),
                        -oi.getTranslateY()
                            * (slowModeSupplier.getAsBoolean() ? MaxSlowSpeed : MaxSpeed),
                        Rotation2d.fromDegrees(-90))));

    oi.operatorAlignForClimbDefendingSide()
        .whileTrue(
            drivetrain.run(
                () ->
                    driveFacingAngle(
                        -oi.getTranslateX()
                            * (slowModeSupplier.getAsBoolean() ? MaxSlowSpeed : MaxSpeed),
                        -oi.getTranslateY()
                            * (slowModeSupplier.getAsBoolean() ? MaxSlowSpeed : MaxSpeed),
                        Rotation2d.fromDegrees(90))));

    // slow-mode toggle
    oi.slowModeSwitch().onTrue(Commands.runOnce(() -> isSlowMode = true));
    oi.slowModeSwitch().onFalse(Commands.runOnce(() -> isSlowMode = false));

    // reset gyro to 0 degrees
    oi.resetGyroButton()
        .onTrue(
            Commands.sequence(
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_POST_HANDOFF)),
                new WaitCommand(0.15),
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_L3_SCORE)),
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L3_SCORE)),
                new WaitCommand(0.75),
                Commands.runOnce(
                    () -> {
                      drivetrain.resetPose(new Pose2d(3.203, 4.190, new Rotation2d(0)));
                      questNav.resetPose(new Pose2d(3.203, 4.190, new Rotation2d(0)));
                      visionApriltagSubsystem.setPipeline(Pipelines.LOCALIZATION);
                    }),
                new ConditionalCommand(
                    Commands.deadline(
                        Commands.sequence(
                            new WaitCommand(0.75),
                            Commands.runOnce(
                                () -> {
                                  Pose2d visionPose =
                                      visionApriltagSubsystem.hasReefTarget()
                                          ? inferPoseFromTarget(
                                              APRILTAG_POSE_F, visionApriltagSubsystem.getTX())
                                          : new Pose2d(3.203, 4.190, new Rotation2d(0));
                                  drivetrain.resetPose(visionPose);
                                  questNav.resetPose(visionPose);
                                })),
                        drivetrain.run(() -> driveSlowlyDirection(Rotation2d.fromDegrees(0)))),
                    new InstantCommand(),
                    () -> true)));

    oi.resetGyroF1Button()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drivetrain.resetPose(new Pose2d(3.203, 4.190, new Rotation2d(0)));
                  questNav.resetPose(new Pose2d(3.203, 4.190, new Rotation2d(0)));
                }));

    oi.operatorResetGyroButton()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drivetrain.resetPose(new Pose2d(3.203, 4.190, new Rotation2d(0)));
                  questNav.resetPose(new Pose2d(3.203, 4.190, new Rotation2d(0)));
                }));

    // x-stance
    // oi.xStanceButton().whileTrue(drivetrain.applyRequest(() -> brakeRequest));

    // oi.getSysIdDynamicForward().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // oi.getSysIdDynamicReverse().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // oi.getSysIdQuasistaticForward().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // oi.getSysIdQuasistaticReverse().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    oi.testAutoButton()
        .whileTrue(
            Commands.sequence(
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_HP_LOAD)),
                AutoBuilder.followPath(pathR2HP),
                drivetrain.run(() -> driveSlowlyDirection(Rotation2d.fromDegrees(125.0)))));

    // drivetrain.registerTelemetry(telemetryLogger::telemeterize);
  }

  private void configureSubsystemCommands() {

    // full-auto toggle
    oi.operatorFullAutoPlacementSwitch().onTrue(Commands.runOnce(() -> isFullAuto = true));
    oi.operatorFullAutoPlacementSwitch().onFalse(Commands.runOnce(() -> isFullAuto = false));

    // oi.fullAutoModeButton().onTrue(Commands.runOnce(() -> isFullAuto = true));
    // oi.manualModeButton().onTrue(Commands.runOnce(() -> isFullAuto = false));

    oi.operatorVisionIsEnabledSwitch().onTrue(Commands.runOnce(() -> isVisionEnabled = true));
    oi.operatorVisionIsEnabledSwitch().onFalse(Commands.runOnce(() -> isVisionEnabled = false));

    oi.operatorAlgaePluckHeightSwitch().onTrue(Commands.runOnce(() -> isPluckTargetHigh = true));
    oi.operatorAlgaePluckHeightSwitch().onFalse(Commands.runOnce(() -> isPluckTargetHigh = false));

    oi.operatorNetSideSwitch().onTrue(Commands.runOnce(() -> preferNetRightSide = true));
    oi.operatorNetSideSwitch().onFalse(Commands.runOnce(() -> preferNetRightSide = false));

    //////////////////
    // Coral Commands
    //////////////////

    oi.ejectCoralButton().whileTrue(new ClawBackwards(claw));

    oi.operatorEjectCoral().whileTrue(new ClawBackwards(claw));

    oi.scoreCoralButton()
        .whileTrue(
            new ConditionalCommand(
                /* L1 Score */
                l1Scorer.runOnce(() -> l1Scorer.ejectIntake()),
                Commands.sequence(
                    new InstantCommand(
                        () -> visionApriltagSubsystem.setPipeline(getScoringTargetPipeline())),
                    intake.runOnce(() -> intake.setTargetPose(currentScoringLevelSupplier.get())),
                    new ConditionalCommand(
                        // Full Auto
                        Commands.parallel(
                            // Raise Piece to scoring level
                            Commands.sequence(
                                new InstantCommand(() -> isRunningPath = true),
                                new WaitUntilCommand(this::isRobotCloseToScoringPosition),
                                armevator.runOnce(
                                    () ->
                                        armevator.setTargetPose(
                                            currentScoringLevelSupplier.get()))),
                            // Drive to scoring location
                            Commands.sequence(
                                new ConditionalCommand(
                                    // Pathfind
                                    getScoringPathCommand(),
                                    // Drive directly to pose
                                    new InstantCommand(),
                                    this::isFarEnoughForPathfinding),
                                Commands.sequence(
                                    new DriveToPoseSlew(
                                        drivetrain,
                                        () -> getPathStartingPose(scoringPathOption),
                                        driveFacingAngleRequest), //
                                    // new WaitCommand(0.2),
                                    getSimpleScoringPathCommand()),
                                new ConditionalCommand(
                                    new InstantCommand(),
                                    getAdjustSlowlyCommand(
                                        () -> scoringAngleMap.get(scoringPathOption),
                                        () -> getScoringAprilTagRight()),
                                    () -> currentScoringLevel == ArmevatorPose.CORAL_L2_SCORE),
                                Commands.parallel(
                                    new ConditionalCommand(
                                        Commands.sequence(
                                            Commands.waitSeconds(0.1),
                                            Commands.runOnce(
                                                () ->
                                                    armevator.setTargetPose(
                                                        ArmevatorPose.CORAL_L4_FLIP))),
                                        new InstantCommand(),
                                        () -> currentScoringLevel == ArmevatorPose.CORAL_L4_SCORE),
                                    Commands.sequence(
                                        new WaitCommand(0.075),
                                        new ClawBackwards(claw, () -> currentScoringLevel)),
                                    drivetrain.run(
                                        () ->
                                            driveSlowlyDirection(
                                                scoringAngleMap.get(scoringPathOption)))))),
                        // Manual
                        Commands.parallel(
                            Commands.sequence(
                                new WaitCommand(0.5),
                                armevator.runOnce(
                                    () ->
                                        armevator.setTargetPose(
                                            currentScoringLevelSupplier.get()))),
                            drivetrain.run(
                                () ->
                                    driveFacingAngle(
                                        -oi.getTranslateX() * MaxSpeed,
                                        -oi.getTranslateY() * MaxSpeed,
                                        scoringAngleMap.get(scoringPathOption)))),
                        isFullAutoSupplier)),
                this::isL1Mode));

    oi.scoreCoralButton()
        .onFalse(
            new ConditionalCommand(
                    Commands.sequence(
                        /* L1 */
                        l1Scorer.runOnce(() -> l1Scorer.setL1Pose(L1ScorerPose.Hold))),
                    new ConditionalCommand(
                        new InstantCommand(),
                        armevator
                            .runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_POST_SCORE))
                            .asProxy(),
                        () -> isPlucking),
                    this::isL1Mode)
                .alongWith(new InstantCommand(() -> isRunningPath = false)));

    oi.intakeCoralRight()
        .whileTrue(
            Commands.deadline(
                Commands.sequence(
                    intake
                        .runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L1_SCORE))
                        .asProxy(),
                    armevator
                        .runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_HP_LOAD))
                        .asProxy(),
                    new IntakeCoral(claw, statusRgb)),
                Commands.runOnce(() -> preferHpLeftSide = shouldIntakeLeftSide()),
                drivetrain
                    .run(
                        () ->
                            driveFacingAngle(
                                -oi.getTranslateX() * MaxSpeed,
                                -oi.getTranslateY() * MaxSpeed,
                                Rotation2d.fromDegrees(55)))
                    .asProxy()));

    oi.intakeCoralLeft()
        .whileTrue(
            Commands.deadline(
                Commands.sequence(
                    intake
                        .runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L1_SCORE))
                        .asProxy(),
                    armevator
                        .runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_HP_LOAD))
                        .asProxy(),
                    new IntakeCoral(claw, statusRgb)),
                Commands.runOnce(() -> preferHpLeftSide = shouldIntakeLeftSide()),
                drivetrain
                    .run(
                        () ->
                            driveFacingAngle(
                                -oi.getTranslateX() * MaxSpeed,
                                -oi.getTranslateY() * MaxSpeed,
                                Rotation2d.fromDegrees(-55)))
                    .asProxy()));

    /*
        oi.intakeCoralButton()
            .whileTrue(
                Commands.deadline(
                    Commands.sequence(
                        new InstantCommand(() -> isRunningPath = true),
                        intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L1_SCORE)),
                        armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_HP_LOAD)),
                        new IntakeCoral(claw, statusRgb)),
                    Commands.sequence(
                        new ConditionalCommand(
                            Commands.sequence(
                                AutoBuilder.pathfindThenFollowPath(pathLeftHP, hpPathConstraints),
                                drivetrain.run(
                                    () -> driveSlowlyDirection(Rotation2d.fromDegrees(125.0)))),
                            Commands.sequence(
                                AutoBuilder.pathfindThenFollowPath(pathRightHP, hpPathConstraints),
                                drivetrain.run(
                                    () -> driveSlowlyDirection(Rotation2d.fromDegrees(-125.0)))),
                            this::shouldIntakeLeftSide))));
    */

    oi.intakeCoralButton()
        .whileTrue(
            new ConditionalCommand(
                Commands.sequence(
                    l1Scorer.runOnce(() -> l1Scorer.ejectIntake()),
                    l1Scorer.runOnce(() -> l1Scorer.setL1Pose(L1ScorerPose.Intake)),
                    new WaitUntilCommand(l1Scorer::isAtPosition),
                    l1Scorer.runOnce(() -> l1Scorer.runIntake())),
                Commands.deadline(
                    Commands.sequence(
                        new InstantCommand(() -> isRunningPath = true),
                        intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L1_SCORE)),
                        armevator.runOnce(
                            () -> armevator.setTargetPose(ArmevatorPose.CORAL_HP_LOAD)),
                        new IntakeCoral(claw, statusRgb)),
                    Commands.sequence(
                        new ConditionalCommand(
                            Commands.sequence(
                                AutoBuilder.pathfindThenFollowPath(pathLeftHP, hpPathConstraints),
                                drivetrain.run(
                                    () -> driveSlowlyDirection(Rotation2d.fromDegrees(125.0)))),
                            Commands.sequence(
                                AutoBuilder.pathfindThenFollowPath(pathRightHP, hpPathConstraints),
                                drivetrain.run(
                                    () -> driveSlowlyDirection(Rotation2d.fromDegrees(-125.0)))),
                            this::shouldIntakeLeftSide))),
                this::isL1Mode));

    oi.intakeCoralButton()
        .whileFalse(
            Commands.sequence(
                Commands.runOnce(() -> isRunningPath = false),
                new ConditionalCommand(
                    Commands.sequence(
                        l1Scorer.runOnce(() -> l1Scorer.runIntakeHoldSpeed()),
                        l1Scorer.runOnce(() -> l1Scorer.setL1Pose(L1ScorerPose.Score)),
                        new WaitUntilCommand(l1Scorer::isAtPosition),
                        l1Scorer.runOnce(() -> l1Scorer.stopIntake())),
                    new InstantCommand(),
                    this::isL1Mode)));

    oi.operatorF1()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_F1),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.F1))));

    oi.operatorF2()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_F2),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.F2))));

    oi.operatorFL1()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_FL1),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.FL1))));

    oi.operatorFL2()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_FL2),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.FL2))));

    oi.operatorFR1()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_FR1),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.FR1))));

    oi.operatorFR2()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_FR2),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.FR2))));

    oi.operatorBL1()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_BL1),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.BL1))));

    oi.operatorBL2()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_BL2),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.BL2))));

    oi.operatorBR1()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_BR1),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.BR1))));

    oi.operatorBR2()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_BR2),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.BR2))));

    oi.operatorB1()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_B1),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.B1))));

    oi.operatorB2()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> scoringPathOption = ScoringPathOption.PATH_B2),
                Commands.runOnce(() -> statusRgb.setScoringPosition(ScoringPosition.B2))));

    oi.operatorL1()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> isL1ModeEnabled = true),
                Commands.runOnce(() -> currentScoringLevel = ArmevatorPose.CORAL_L1_SCORE),
                Commands.runOnce(() -> statusRgb.setScoringLevel(ScoringLevel.LEVEL_1)),
                Commands.runOnce(() -> armevator.updateScoringLevel(currentScoringLevel))));
    oi.operatorL2()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> isL1ModeEnabled = false),
                Commands.runOnce(() -> currentScoringLevel = ArmevatorPose.CORAL_L2_SCORE),
                Commands.runOnce(() -> statusRgb.setScoringLevel(ScoringLevel.LEVEL_2)),
                Commands.runOnce(() -> armevator.updateScoringLevel(currentScoringLevel))));
    oi.operatorL3()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> isL1ModeEnabled = false),
                Commands.runOnce(() -> currentScoringLevel = ArmevatorPose.CORAL_L3_SCORE),
                Commands.runOnce(() -> statusRgb.setScoringLevel(ScoringLevel.LEVEL_3)),
                Commands.runOnce(() -> armevator.updateScoringLevel(currentScoringLevel))));
    oi.operatorL4()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> isL1ModeEnabled = false),
                Commands.runOnce(() -> currentScoringLevel = ArmevatorPose.CORAL_L4_SCORE),
                Commands.runOnce(() -> statusRgb.setScoringLevel(ScoringLevel.LEVEL_4)),
                Commands.runOnce(() -> armevator.updateScoringLevel(currentScoringLevel))));

    //////////////////
    // Algae Commands
    //////////////////

    /*Command bangBang =
        Commands.sequence(
            intake.runOnce(() -> intake.runIntake()),
            intake.runOnce(() -> intake.tiltForward()),
            new WaitUntilCommand(() -> intake.getAngle() >= 60),
            intake.runOnce(() -> intake.stopTilt()));

    Command unBangBang =
        Commands.sequence(
            intake.runOnce(() -> intake.tiltBackwards()),
            new WaitUntilCommand(() -> intake.getAngle() <= 6),
            intake.runOnce(() -> intake.stopTilt()),
            intake.runOnce(() -> intake.stopIntake()));
    */

    oi.intakeAlgaeButton()
        .whileTrue(
            Commands.sequence(
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_INTAKE)),
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_INTAKE)),
                Commands.parallel(
                    intake.run(() -> intake.runIntake()), claw.run(() -> claw.intakeAlgae()))));
    oi.intakeAlgaeButton()
        .onFalse(
            Commands.sequence(
                intake.runOnce(() -> intake.stopIntake()),
                claw.runOnce(() -> claw.stopClaw()),
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_PRE_HANDOFF)),
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_POST_HANDOFF)),
                Commands.waitSeconds(0.65),
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_HANDOFF)),
                Commands.waitSeconds(0.2),
                intake.runOnce(() -> intake.runIntake()),
                claw.runOnce(() -> claw.intakeAlgae()),
                Commands.waitSeconds(0.5),
                intake.runOnce(() -> intake.stopIntake()),
                claw.runOnce(() -> claw.brakeAlgae()),
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_POST_HANDOFF))));

    oi.ejectAlgaeButton()
        .whileTrue(
            Commands.sequence(
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_HANDOFF)),
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_POST_HANDOFF)),
                Commands.parallel(
                    intake.run(() -> intake.ejectIntake()), claw.run(() -> claw.ejectAlgae()))));
    oi.ejectAlgaeButton()
        .onFalse(
            Commands.sequence(
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CORAL_L1_SCORE)),
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_L1_SCORE)),
                Commands.parallel(
                    intake.run(() -> intake.stopIntake()), claw.run(() -> claw.stopClaw()))));

    oi.pullInCoralButton()
        .whileTrue(
            Commands.parallel(
                intake.run(() -> intake.ejectIntake()), claw.run(() -> claw.ejectAlgae())));
    oi.pullInCoralButton()
        .onFalse(
            Commands.sequence(
                intake.runOnce(() -> intake.stopIntake()), claw.runOnce(() -> claw.stopClaw())));

    oi.operatorEjectAlgae()
        .whileTrue(
            Commands.parallel(
                intake.run(() -> intake.ejectIntake()), claw.run(() -> claw.ejectAlgae())));
    oi.operatorEjectAlgae()
        .onFalse(
            Commands.sequence(
                intake.runOnce(() -> intake.stopIntake()), claw.runOnce(() -> claw.stopClaw())));

    Command nonAutoPluck =
        Commands.sequence(
            Commands.runOnce(() -> isPlucking = true),
            new PrintCommand("Non-Auto Command Started"),
            claw.runOnce(() -> claw.intakeAlgaePluck()),
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_PRE_PLUCK_L2)),
            armevator.runOnce(
                () -> {
                  ArmevatorPose setPose =
                      isPluckTargetHighSupplier.getAsBoolean()
                          ? ArmevatorPose.ALGAE_L3_PLUCK
                          : ArmevatorPose.ALGAE_L2_PLUCK;

                  if (isFullAutoSupplier.getAsBoolean()) {
                    setPose = inferPluckArmevatorPose(false);
                  }
                  armevator.setTargetPose(setPose);
                }));

    Command autoPluckCommand =
        Commands.sequence(
                new InstantCommand(
                    () -> visionApriltagSubsystem.setPipeline(Pipelines.TRACKING_CENTER)),
                armevator.runOnce(() -> armevator.setTargetPose(inferPluckArmevatorPose(true))),
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_POST_HANDOFF)),
                new InstantCommand(() -> isRunningPath = true),
                Commands.deadline(
                    new ConditionalCommand(
                        getPluckPathCommand(),
                        Commands.sequence(
                            new DriveToPoseSlew(
                                drivetrain,
                                () -> getPluckPathStartingPose(scoringPathOption),
                                driveFacingAngleRequest),
                            getSimplePluckPathCommand()),
                        this::isFarEnoughForPathfindingPluck),
                    new InstantCommand(() -> isRunningPath = false),
                    Commands.sequence(
                        new WaitCommand(0.25),
                        intake.runOnce(
                            () -> intake.setTargetPose(ArmevatorPose.ALGAE_PRE_PLUCK_L2)))),
                Commands.deadline(
                    Commands.waitSeconds(0.25),
                    drivetrain.run(
                        () -> driveSlowlyDirection(scoringAngleMap.get(scoringPathOption)))),
                getPluckAdjustSlowlyCommand(
                    () -> scoringAngleMap.get(scoringPathOption), () -> true),
                Commands.parallel(
                    nonAutoPluck,
                    drivetrain.run(
                        () -> driveSlowlyDirection(scoringAngleMap.get(scoringPathOption)))))
            .finallyDo(() -> isRunningPath = false);

    oi.pluckAlgaeButton()
        .whileTrue(
            new ConditionalCommand(
                autoPluckCommand,
                Commands.deadline(
                    Commands.sequence(
                            Commands.runOnce(() -> isPlucking = true),
                            new PrintCommand("Non-Auto Command Started"),
                            claw.runOnce(() -> claw.intakeAlgaePluck()),
                            intake.runOnce(
                                () -> intake.setTargetPose(ArmevatorPose.ALGAE_PRE_PLUCK_L2)),
                            armevator.runOnce(
                                () -> {
                                  ArmevatorPose setPose =
                                      isPluckTargetHighSupplier.getAsBoolean()
                                          ? ArmevatorPose.ALGAE_L3_PLUCK
                                          : ArmevatorPose.ALGAE_L2_PLUCK;

                                  armevator.setTargetPose(setPose);
                                }),
                            drivetrain.run(
                                () ->
                                    driveFacingAngle(
                                        -oi.getTranslateX() * MaxSpeed,
                                        -oi.getTranslateY() * MaxSpeed,
                                        scoringAngleMap.get(scoringPathOption))))
                        .asProxy()),
                isFullAutoSupplier));

    oi.pluckAlgaeButton()
        .onFalse(
            Commands.sequence(
                Commands.runOnce(() -> isPlucking = false),
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_POST_HANDOFF)),
                Commands.waitSeconds(0.15),
                armevator.runOnce(
                    () -> {
                      if (isFullAutoSupplier.getAsBoolean()) {
                        armevator.setTargetPose(ArmevatorPose.ALGAE_POST_HANDOFF);
                      } else {
                        armevator.setTargetPose(
                            isPluckTargetHighSupplier.getAsBoolean()
                                ? ArmevatorPose.ALGAE_L3_PLUCK
                                : ArmevatorPose.ALGAE_L2_PLUCK);
                      }
                    }),
                claw.run(() -> claw.brakeAlgae())));

    oi.aimAtNetButton()
        .whileTrue(
            Commands.sequence(
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_NET_STAGE)),
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_NET_STAGE)),
                claw.runOnce(() -> claw.intakeAlgae()),
                new ConditionalCommand(
                    new ConditionalCommand(
                        Commands.sequence(
                            new ConditionalCommand(
                                getDynamicFarNetPathCommand(),
                                new InstantCommand(),
                                this::isFarEnoughFromFarNetForPathfinding),
                            new DriveToPoseSlew(
                                drivetrain,
                                () ->
                                    new Pose2d(
                                        FAR_NET_SCORE_LOCATION_X,
                                        drivetrain.getPose().getY(),
                                        preferNetRightSideSupplier.getAsBoolean()
                                            ? Rotation2d.fromDegrees(45 - 8)
                                            : Rotation2d.fromDegrees(-45 + 8)),
                                driveFacingAngleRequest)),
                        Commands.waitSeconds(1.0),
                        isFullAutoSupplier),
                    new ConditionalCommand(
                        Commands.sequence(
                            new ConditionalCommand(
                                getDynamicNetPathCommand(),
                                new InstantCommand(),
                                this::isFarEnoughFromNetForPathfinding),
                            new DriveToPoseSlew(
                                drivetrain,
                                () ->
                                    new Pose2d(
                                        NET_SCORE_LOCATION_X,
                                        drivetrain.getPose().getY(),
                                        preferNetRightSideSupplier.getAsBoolean()
                                            ? Rotation2d.fromDegrees(135 + 8)
                                            : Rotation2d.fromDegrees(-135 - 8)),
                                driveFacingAngleRequest)),
                        Commands.waitSeconds(1.0),
                        isFullAutoSupplier),
                    this::isOnFarSideOfField),
                Commands.deadline(
                    Commands.sequence(
                        claw.runOnce(() -> claw.brakeAlgae()),
                        // score net command
                        armevator.runOnce(
                            () -> armevator.setTargetPose(ArmevatorPose.ALGAE_NET_SCORE)),
                        Commands.waitUntil(armevator::isAtNetReleaseAngle),
                        claw.runOnce(() -> claw.ejectAlgae()),
                        Commands.waitSeconds(0.3),
                        claw.runOnce(() -> claw.stopClaw()),
                        armevator.runOnce(
                            () -> armevator.setTargetPose(ArmevatorPose.ALGAE_POST_HANDOFF))),
                    drivetrain.run(
                        () ->
                            driveFacingAngle(
                                0,
                                0,
                                preferNetRightSideSupplier.getAsBoolean()
                                    ? isOnFarSideOfField()
                                        ? Rotation2d.fromDegrees(45 - 8)
                                        : Rotation2d.fromDegrees(135 + 8)
                                    : isOnFarSideOfField()
                                        ? Rotation2d.fromDegrees(-45 + 8)
                                        : Rotation2d.fromDegrees(-135 - 8))))));
    oi.aimAtNetButton()
        .onFalse(
            Commands.sequence(
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_POST_HANDOFF)),
                claw.runOnce(() -> claw.stopClaw())));

    ////////////////////
    // Climber Commands
    ////////////////////

    oi.operatorExtendClimber()
        .whileTrue(
            Commands.sequence(
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CLIMB)),
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CLIMB)),
                climber.runOnce(() -> climber.disengageWindmill()),
                climber.run(() -> climber.extendClimber())));
    oi.operatorExtendClimber().onFalse(climber.runOnce(() -> climber.stopClimber()));

    oi.operatorRetractClimber()
        .whileTrue(
            Commands.sequence(
                climber.runOnce(() -> climber.engageWindmill()),
                new WaitCommand(0.1),
                climber.run(() -> climber.retractClimber())));
    oi.operatorRetractClimber().onFalse(climber.runOnce(() -> climber.brakeClimber()));

    oi.retractClimberSlowlySwitch().whileTrue(climber.runOnce(() -> climber.brakeClimber()));
    oi.retractClimberSlowlySwitch().onFalse(climber.runOnce(() -> climber.stopClimber()));
  }

  private boolean isL1Mode() {
    return isL1ModeEnabled;
  }

  private void configureVisionCommands() {
    // enable/disable vision
    /*
     * oi.getVisionIsEnabledSwitch()
     * .onTrue(
     * Commands.runOnce(() -> vision.enable(true))
     * .ignoringDisable(true)
     * .withName("enable vision"));
     * oi.getVisionIsEnabledSwitch()
     * .onFalse(
     * Commands.runOnce(() -> vision.enable(false), vision)
     * .ignoringDisable(true)
     * .withName("disable vision"));
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Check if the alliance color has changed; if so, update the vision subsystem and Field2d
   * singleton.
   */
  public void checkAllianceColor() {
    lastAlliance = Alliance.Blue;
    Field2d.getInstance().updateAlliance(this.lastAlliance);

    // Optional<Alliance> alliance = DriverStation.getAlliance();
    // if (alliance.isPresent() && alliance.get() != lastAlliance) {
    // this.lastAlliance = alliance.get();
    // Field2d.getInstance().updateAlliance(this.lastAlliance);
    // }
  }

  public void periodic() {
    // add robot-wide periodic code here
    questNav.cleanUpQuestNavMessages();
    posePublisher.set(drivetrain.getPose());
    questNav.updateAverageRobotPose();
    updateVisionPose();
    questPosePublisher.set(questNav.getRobotPose());

    // new field pose updates (overlaps above code)
    field2d.setPose(FieldObject.ROBOT_POSE, drivetrain.getPose());
    field2d.setPose(FieldObject.QUEST_POSE, questNav.getRobotPose());

    Pose2d llPose2d = extractLimelightPose();
    if (llPose2d != null) {
      field2d.setPose(FieldObject.LIMELIGHT_POSE, llPose2d);
    }

    double botSpeed =
        Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond)
            + Math.abs(drivetrain.getState().Speeds.vyMetersPerSecond);

    // TODO, 0.05 is a guess on minimum speed, this needs testing
    if (botSpeed < 0.05 && driveSlowlyDirectionAlert) {
      driveSlowlyDirectionAlert = false;
      statusRgb.driveSlowlyTrigger();
    }

    if (visionApriltagSubsystem.hasReefTarget()) {
      if (Math.abs(visionApriltagSubsystem.getTX()) > 1.0) {
        apriltagStatus = AprilTagStatus.REEF_TARGET_OUTSIDE_RANGE;
      } else {
        apriltagStatus = AprilTagStatus.REEF_TARGET_IN_RANGE;
      }
    } else {
      apriltagStatus = AprilTagStatus.NO_TARGET;
    }
  }

  private Pose2d extractLimelightPose() {
    LimelightHelpers.PoseEstimate limelightMeasurement = visionApriltagSubsystem.getPoseEstimate();

    if (limelightMeasurement != null) {
      if (limelightMeasurement.tagCount >= 2
          || (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist < 1.25)) {

        return limelightMeasurement.pose;
      }
    }
    return null;
  }

  public void disablePeriodic() {
    armevator.resetToAbsoluteEncoder();
    this.checkAllianceColor();
  }

  public void autonomousInit() {
    // add robot-wide code here that will be executed when autonomous starts
  }

  public void teleopInit() {
    // check if the alliance color has changed based on the FMS data; if the robot
    // power cycled
    // during a match, this would be the first opportunity to check the alliance
    // color based on FMS
    // data.
    this.checkAllianceColor();
  }

  private boolean shouldIntakeLeftSide() {
    if (oi.getTranslateY() < -0.02) {
      return true;
    }
    if (oi.getTranslateY() > 0.02) {
      return false;
    }
    return drivetrain.getPose().getY() > 4.0; // half the field width in meters
  }

  private boolean isRobotFacingTargetAngle(Rotation2d target) {
    double difference = Math.abs(drivetrain.getPose().getRotation().minus(target).getDegrees());
    return difference <= 3.0;
  }

  private boolean isRobotInNetScoringPosition() {
    return drivetrain.getPose().getX() >= 6.0;
  }

  private boolean isRobotCloseToNet() {
    return drivetrain.getPose().getX() >= 7.0; // 7.5 is hitting
  }

  private boolean isFarEnoughFromNetForPathfinding() {
    return getDistanceFromNet() > 2.0;
  }

  private boolean isFarEnoughFromFarNetForPathfinding() {
    return getDistanceFromFarNet() > 2.0;
  }

  private boolean isOnFarSideOfField() {
    return drivetrain.getPose().getX() > 9.0;
  }

  private boolean isFarEnoughForPathfinding() {
    return getDistanceFromTarget() > 2.0;
  }

  private boolean isFarEnoughForPathfindingPluck() {
    return getPluckDistanceFromTarget() > 2.0;
  }

  private boolean isRobotCloseToScoringPosition() {
    return getDistanceFromTarget() < 0.4;
  }

  private Pipelines getScoringTargetPipeline() {
    if (scoringPathOption == ScoringPathOption.PATH_B2
        || scoringPathOption == ScoringPathOption.PATH_BL2
        || scoringPathOption == ScoringPathOption.PATH_BR1
        || scoringPathOption == ScoringPathOption.PATH_FL2
        || scoringPathOption == ScoringPathOption.PATH_FR1
        || scoringPathOption == ScoringPathOption.PATH_F1) {
      return Pipelines.TRACKING_LEFT;
    } else {
      return Pipelines.TRACKING_RIGHT;
    }
  }

  private boolean getScoringAprilTagRight() {
    if (scoringPathOption == ScoringPathOption.PATH_B2
        || scoringPathOption == ScoringPathOption.PATH_BL2
        || scoringPathOption == ScoringPathOption.PATH_BR1
        || scoringPathOption == ScoringPathOption.PATH_FL2
        || scoringPathOption == ScoringPathOption.PATH_FR1
        || scoringPathOption == ScoringPathOption.PATH_F1) {
      return true;
    } else {
      return false;
    }
  }

  private double getDistanceFromTarget() {
    Pose2d targetPose = getPathStartingPose(scoringPathOption);
    Pose2d currentPose = drivetrain.getPose();
    return targetPose.getTranslation().getDistance(currentPose.getTranslation());
  }

  private double getPluckDistanceFromTarget() {
    Pose2d targetPose = getPluckPathStartingPose(scoringPathOption);
    Pose2d currentPose = drivetrain.getPose();
    return targetPose.getTranslation().getDistance(currentPose.getTranslation());
  }

  private double getDistanceFromNet() {
    Pose2d targetPose =
        new Pose2d(NET_SCORE_LOCATION_X, drivetrain.getPose().getY(), Rotation2d.fromDegrees(135));
    Pose2d currentPose = drivetrain.getPose();
    return targetPose.getTranslation().getDistance(currentPose.getTranslation());
  }

  private double getDistanceFromFarNet() {
    Pose2d targetPose =
        new Pose2d(
            FAR_NET_SCORE_LOCATION_X, drivetrain.getPose().getY(), Rotation2d.fromDegrees(135));
    Pose2d currentPose = drivetrain.getPose();
    return targetPose.getTranslation().getDistance(currentPose.getTranslation());
  }

  private ArmevatorPose inferPluckArmevatorPose(boolean getPrePluck) {
    switch (scoringPathOption) {
      case PATH_F1, PATH_F2, PATH_BR1, PATH_BR2, PATH_BL1, PATH_BL2 -> {
        if (getPrePluck) {
          return ArmevatorPose.ALGAE_PRE_PLUCK_L3;
        }
        return ArmevatorPose.ALGAE_L3_PLUCK;
      }

      default -> {
        if (getPrePluck) {
          return ArmevatorPose.ALGAE_PRE_PLUCK_L2;
        }
        return ArmevatorPose.ALGAE_L2_PLUCK;
      }
    }
  }

  private Command getPluckPathCommand() {
    return new SelectCommand<>(pluckAlgaePathMap, () -> scoringPathOption);
  }

  private Command getScoringPathCommand() {
    return new SelectCommand<>(scoringPathMap, () -> scoringPathOption);
  }

  private Command getDynamicNetPathCommand() {
    return new DynamicCommand(
        () -> {
          Pose2d currentPose = drivetrain.getPose();
          double currentY = currentPose.getY();
          if (currentY < NET_SCORE_MIN_Y || currentY > NET_SCORE_MAX_Y) {
            return new InstantCommand();
          }
          Pose2d targetPose =
              new Pose2d(
                  NET_SCORE_LOCATION_X,
                  currentY,
                  preferNetRightSideSupplier.getAsBoolean()
                      ? Rotation2d.fromDegrees(135 + 8)
                      : Rotation2d.fromDegrees(-135 - 8));
          return AutoBuilder.pathfindToPose(targetPose, hpPathConstraints);
        });
  }

  private Command getDynamicFarNetPathCommand() {
    return new DynamicCommand(
        () -> {
          Pose2d currentPose = drivetrain.getPose();
          double currentY = currentPose.getY();
          if (currentY < NET_SCORE_MIN_Y || currentY > NET_SCORE_MAX_Y) {
            return new InstantCommand();
          }
          Pose2d targetPose =
              new Pose2d(
                  FAR_NET_SCORE_LOCATION_X,
                  currentY,
                  preferNetRightSideSupplier.getAsBoolean()
                      ? Rotation2d.fromDegrees(45 - 8)
                      : Rotation2d.fromDegrees(-45 + 8));
          return AutoBuilder.pathfindToPose(targetPose, hpPathConstraints);
        });
  }

  private Command getSimpleScoringPathCommand() {
    return new SelectCommand<>(simpleScoringPathMap, () -> scoringPathOption);
  }

  private Command getSimplePluckPathCommand() {
    return new SelectCommand<>(simplePluckScoringMap, () -> scoringPathOption);
  }

  private Pose2d getPathStartingPose(ScoringPathOption scoringPathOption) {
    return getCurrentScoringPath(scoringPathOption).getStartingHolonomicPose().get();
  }

  private Pose2d getPathStartingPose(PathPlannerPath path) {
    return path.getStartingHolonomicPose().get();
  }

  private Pose2d getPluckPathStartingPose(ScoringPathOption scoringPathOption) {
    return getCurrentScoringPluckPath(scoringPathOption).getStartingHolonomicPose().get();
  }

  private PathPlannerPath getCurrentScoringPath(ScoringPathOption scoringPathOption) {
    switch (scoringPathOption) {
      case PATH_F1:
        return pathF1;
      case PATH_F2:
        return pathF2;
      case PATH_FL1:
        return pathFL1;
      case PATH_FL2:
        return pathFL2;
      case PATH_FR1:
        return pathFR1;
      case PATH_FR2:
        return pathFR2;
      case PATH_BL1:
        return pathBL1;
      case PATH_BL2:
        return pathBL2;
      case PATH_BR1:
        return pathBR1;
      case PATH_BR2:
        return pathBR2;
      case PATH_B1:
        return pathB1;
      case PATH_B2:
        return pathB2;
    }
    return pathF1;
  }

  private PathPlannerPath getCurrentScoringPluckPath(ScoringPathOption scoringPathOption) {
    switch (scoringPathOption) {
      case PATH_F1, PATH_F2 -> {
        return pathFAlgae;
      }
      case PATH_FL1, PATH_FL2 -> {
        return pathFLAlgae;
      }
      case PATH_FR1, PATH_FR2 -> {
        return pathFRAlgae;
      }
      case PATH_BL1, PATH_BL2 -> {
        return pathBLAlgae;
      }
      case PATH_BR1, PATH_BR2 -> {
        return pathBRAlgae;
      }
      case PATH_B1, PATH_B2 -> {
        return pathBAlgae;
      }
    }
    return pathFAlgae;
  }

  // run on init

  private void setupScoringPathMap() {
    scoringPathMap.put(
        ScoringPathOption.PATH_F1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathF1), scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_F2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathF2), scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_FL1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathFL1), scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_FL2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathFL2), scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_FR1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathFR1), scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_FR2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathFR2), scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_BL1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathBL1), scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_BL2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathBL2), scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_BR1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathBR1), scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_BR2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathBR2), scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_B1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathB1), scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_B2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathB2), scorePathConstraints));

    scoringPathL1Map.put(
        ScoringPathOption.PATH_F1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1F1), scorePathConstraints));
    scoringPathL1Map.put(
        ScoringPathOption.PATH_F2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1F2), scorePathConstraints));
    scoringPathL1Map.put(
        ScoringPathOption.PATH_FL1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1FL1), scorePathConstraints));
    scoringPathL1Map.put(
        ScoringPathOption.PATH_FL2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1FL2), scorePathConstraints));
    scoringPathL1Map.put(
        ScoringPathOption.PATH_FR1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1FR1), scorePathConstraints));
    scoringPathL1Map.put(
        ScoringPathOption.PATH_FR2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1FR2), scorePathConstraints));
    scoringPathL1Map.put(
        ScoringPathOption.PATH_BL1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1BL1), scorePathConstraints));
    scoringPathL1Map.put(
        ScoringPathOption.PATH_BL2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1BL2), scorePathConstraints));
    scoringPathL1Map.put(
        ScoringPathOption.PATH_BR1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1BR1), scorePathConstraints));
    scoringPathL1Map.put(
        ScoringPathOption.PATH_BR2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1BR2), scorePathConstraints));
    scoringPathL1Map.put(
        ScoringPathOption.PATH_B1,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1B1), scorePathConstraints));
    scoringPathL1Map.put(
        ScoringPathOption.PATH_B2,
        AutoBuilder.pathfindToPose(getPathStartingPose(pathL1B2), scorePathConstraints));

    simpleScoringPathMap.put(ScoringPathOption.PATH_F1, AutoBuilder.followPath(pathF1));
    simpleScoringPathMap.put(ScoringPathOption.PATH_F2, AutoBuilder.followPath(pathF2));
    simpleScoringPathMap.put(ScoringPathOption.PATH_FL1, AutoBuilder.followPath(pathFL1));
    simpleScoringPathMap.put(ScoringPathOption.PATH_FL2, AutoBuilder.followPath(pathFL2));
    simpleScoringPathMap.put(ScoringPathOption.PATH_FR1, AutoBuilder.followPath(pathFR1));
    simpleScoringPathMap.put(ScoringPathOption.PATH_FR2, AutoBuilder.followPath(pathFR2));
    simpleScoringPathMap.put(ScoringPathOption.PATH_BL1, AutoBuilder.followPath(pathBL1));
    simpleScoringPathMap.put(ScoringPathOption.PATH_BL2, AutoBuilder.followPath(pathBL2));
    simpleScoringPathMap.put(ScoringPathOption.PATH_BR1, AutoBuilder.followPath(pathBR1));
    simpleScoringPathMap.put(ScoringPathOption.PATH_BR2, AutoBuilder.followPath(pathBR2));
    simpleScoringPathMap.put(ScoringPathOption.PATH_B1, AutoBuilder.followPath(pathB1));
    simpleScoringPathMap.put(ScoringPathOption.PATH_B2, AutoBuilder.followPath(pathB2));

    simplePluckScoringMap.put(ScoringPathOption.PATH_F1, AutoBuilder.followPath(pathFAlgae));
    simplePluckScoringMap.put(ScoringPathOption.PATH_F2, AutoBuilder.followPath(pathFAlgae));
    simplePluckScoringMap.put(ScoringPathOption.PATH_FL1, AutoBuilder.followPath(pathFLAlgae));
    simplePluckScoringMap.put(ScoringPathOption.PATH_FL2, AutoBuilder.followPath(pathFLAlgae));
    simplePluckScoringMap.put(ScoringPathOption.PATH_FR1, AutoBuilder.followPath(pathFRAlgae));
    simplePluckScoringMap.put(ScoringPathOption.PATH_FR2, AutoBuilder.followPath(pathFRAlgae));
    simplePluckScoringMap.put(ScoringPathOption.PATH_BL1, AutoBuilder.followPath(pathBLAlgae));
    simplePluckScoringMap.put(ScoringPathOption.PATH_BL2, AutoBuilder.followPath(pathBLAlgae));
    simplePluckScoringMap.put(ScoringPathOption.PATH_BR1, AutoBuilder.followPath(pathBRAlgae));
    simplePluckScoringMap.put(ScoringPathOption.PATH_BR2, AutoBuilder.followPath(pathBRAlgae));
    simplePluckScoringMap.put(ScoringPathOption.PATH_B1, AutoBuilder.followPath(pathBAlgae));
    simplePluckScoringMap.put(ScoringPathOption.PATH_B2, AutoBuilder.followPath(pathBAlgae));

    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_F1,
        AutoBuilder.pathfindThenFollowPath(pathFAlgae, pluckPathConstraints));
    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_F2,
        AutoBuilder.pathfindThenFollowPath(pathFAlgae, pluckPathConstraints));
    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_FL1,
        AutoBuilder.pathfindThenFollowPath(pathFLAlgae, pluckPathConstraints));
    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_FL2,
        AutoBuilder.pathfindThenFollowPath(pathFLAlgae, pluckPathConstraints));
    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_FR1,
        AutoBuilder.pathfindThenFollowPath(pathFRAlgae, pluckPathConstraints));
    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_FR2,
        AutoBuilder.pathfindThenFollowPath(pathFRAlgae, pluckPathConstraints));
    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_BL1,
        AutoBuilder.pathfindThenFollowPath(pathBLAlgae, pluckPathConstraints));
    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_BL2,
        AutoBuilder.pathfindThenFollowPath(pathBLAlgae, pluckPathConstraints));
    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_BR1,
        AutoBuilder.pathfindThenFollowPath(pathBRAlgae, pluckPathConstraints));
    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_BR2,
        AutoBuilder.pathfindThenFollowPath(pathBRAlgae, pluckPathConstraints));
    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_B1,
        AutoBuilder.pathfindThenFollowPath(pathBAlgae, pluckPathConstraints));
    pluckAlgaePathMap.put(
        ScoringPathOption.PATH_B2,
        AutoBuilder.pathfindThenFollowPath(pathBAlgae, pluckPathConstraints));

    scoringAngleMap.put(ScoringPathOption.PATH_F1, Rotation2d.fromDegrees(0.0));
    scoringAngleMap.put(ScoringPathOption.PATH_F2, Rotation2d.fromDegrees(0.0));
    scoringAngleMap.put(ScoringPathOption.PATH_FL1, Rotation2d.fromDegrees(-60.0));
    scoringAngleMap.put(ScoringPathOption.PATH_FL2, Rotation2d.fromDegrees(-60.0));
    scoringAngleMap.put(ScoringPathOption.PATH_FR1, Rotation2d.fromDegrees(60.0));
    scoringAngleMap.put(ScoringPathOption.PATH_FR2, Rotation2d.fromDegrees(60.0));
    scoringAngleMap.put(ScoringPathOption.PATH_BL1, Rotation2d.fromDegrees(-120.0));
    scoringAngleMap.put(ScoringPathOption.PATH_BL2, Rotation2d.fromDegrees(-120.0));
    scoringAngleMap.put(ScoringPathOption.PATH_BR1, Rotation2d.fromDegrees(120.0));
    scoringAngleMap.put(ScoringPathOption.PATH_BR2, Rotation2d.fromDegrees(120.0));
    scoringAngleMap.put(ScoringPathOption.PATH_B1, Rotation2d.fromDegrees(180.0));
    scoringAngleMap.put(ScoringPathOption.PATH_B2, Rotation2d.fromDegrees(180.0));
  }

  public Pose2d inferPoseFromTarget(Pose2d targetPose, double txDegrees) {
    // Convert the tx angle from degrees to radians.
    double txRadians = Math.toRadians(txDegrees);

    // The camera is mounted so its optical axis is opposite to the target’s facing.
    // Therefore, the camera's forward direction is targetPose's rotation plus 180°.
    Rotation2d cameraDirection = targetPose.getRotation().rotateBy(new Rotation2d(Math.PI));

    // "Ideal" camera position if the target were centered (tx == 0):
    // 0.64115 m away from the target along the camera's forward direction.
    Translation2d idealCameraTranslation =
        targetPose
            .getTranslation()
            .minus(
                new Translation2d(cameraDirection.getCos(), cameraDirection.getSin())
                    .times(0.65615));

    // The lateral offset (in meters) caused by an off-center target:
    double lateralOffset = 0.65615 * Math.tan(txRadians);

    // Compute the camera's right vector by rotating the forward vector -90°.
    Rotation2d cameraRight = cameraDirection.rotateBy(Rotation2d.fromDegrees(-90));

    // The actual camera position is shifted from the ideal position.
    // A positive tx (target appears to the right) implies the camera is offset to
    // the left.
    Translation2d actualCameraTranslation =
        idealCameraTranslation.minus(
            new Translation2d(cameraRight.getCos(), cameraRight.getSin()).times(lateralOffset));

    // The robot center is 0.19115 m forward from the camera (along the same forward
    // direction).
    Translation2d robotTranslation =
        actualCameraTranslation.plus(
            new Translation2d(cameraDirection.getCos(), cameraDirection.getSin()).times(0.20615));
    System.out.println("target pose x " + targetPose.getX());
    System.out.println("target pose y " + targetPose.getY());
    System.out.println("txDegrees " + txDegrees);
    System.out.println("lateralOffset " + lateralOffset);
    System.out.println("actualcamerax " + actualCameraTranslation.getX());
    // The robot is assumed to have the same heading as the camera.
    return new Pose2d(robotTranslation, cameraDirection);
  }

  public void updateVisionPose() {
    if (questNav.isConnected() && isVisionEnabled) {
      drivetrain.addVisionMeasurement(
          questNav.getAverageRobotPose(), VecBuilder.fill(0.0, 0.0, 0.0));
      return;
    }

    // LimelightHelpers.PoseEstimate limelightMeasurement =
    // visionApriltagSubsystem.getPoseEstimate();
    // if (limelightMeasurement != null && (limelightMeasurement.tagCount >= 2
    // || (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist <
    // 1.25))) {
    // drivetrain.addVisionMeasurement(
    // limelightMeasurement.pose,
    // limelightMeasurement.timestampSeconds,
    // VecBuilder.fill(.6, .6, 9999999));
    // }
  }

  public void setCurrentPathPose(Pose2d pose) {
    currentPathPose = pose;
  }

  private int debugPublishCounter = 0;

  private int getCurrentPathfindError() {
    if (!isRunningPath) {
      return -1;
    }
    double calc =
        drivetrain.getPose().getTranslation().getDistance(currentPathPose.getTranslation());

    if (debugPublishCounter++ > 10) {
      // Since this for debug, not every cycle
      pathfindErrorNetwork.set(calc);
      debugPublishCounter = 0;
    }

    int calculatedMode =
        Math.min(10, (int) (calc)) + 10; // TODO: unsure what values this will give, adjust later
    return calculatedMode;
  }
}
