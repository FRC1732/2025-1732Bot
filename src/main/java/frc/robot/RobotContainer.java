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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.DriveToPose;
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
import java.util.function.IntSupplier;
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

  private static final double NET_SCORE_LOCATION_X = 7.4;
  private static final double NET_SCORE_MIN_Y = 4.7;
  private static final double NET_SCORE_MAX_Y = 6.27;

  private Alliance lastAlliance = Alliance.Blue; // Field2d.getInstance().getAlliance();

  public final CommandSwerveDrivetrain drivetrain =
      TunerConstants.createDrivetrain((pose) -> questNav.resetPose(pose));

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(1.5)
          .in(RadiansPerSecond); // 1.5 rotations per second max angular velocity
  private double MaxSlowSpeed = 0.25 * MaxSpeed; // 25% of max speed
  private double MaxSlowAngularRate = 0.25 * MaxAngularRate; // 25% of max angular rate
  private boolean isSlowMode = false;
  private boolean isFullAuto = true;
  private boolean preferNetRightSide = true;
  private boolean isPlucking = false;
  private boolean isVisionEnabled = true;
  private boolean isPluckTargetHigh = false;
  private ClimberPathOption climbingPathOption = ClimberPathOption.PATH_LEFT;

  private BooleanSupplier slowModeSupplier = () -> isSlowMode;
  private BooleanSupplier preferNetRightSideSupplier = () -> preferNetRightSide;
  private BooleanSupplier isFullAutoSupplier = () -> isFullAuto;
  private BooleanSupplier isPluckingSupplier = () -> isPlucking;
  private BooleanSupplier isVisionEnabledSupplier = () -> isVisionEnabled;
  private BooleanSupplier isPluckTargetHighSupplier = () -> isPluckTargetHigh;
  private ArmevatorPose currentScoringLevel = ArmevatorPose.CORAL_L4_SCORE;
  private Supplier<ArmevatorPose> currentScoringLevelSupplier = () -> currentScoringLevel;
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

  PathConstraints hpPathConstraints = new PathConstraints(4.5, 3.2, 8.42, 12.8876585);
  PathConstraints pluckPathConstraints = new PathConstraints(4.5, 3.2, 8.0, 10.0);
  PathConstraints scorePathConstraints = new PathConstraints(4.5, 3.2, 8.0, 10.0);
  PathConstraints climberPathConstraints = new PathConstraints(2, 1.25, 8.0, 10.0);

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

  PathPlannerPath pathRightClimber;
  PathPlannerPath pathLeftClimber;
  PathPlannerPath pathMiddleClimber;

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

  public enum ClimberPathOption {
    PATH_LEFT,
    PATH_RIGHT,
    PATH_MIDDLE,
  }

  Map<ScoringPathOption, Command> scoringPathMap = new HashMap<>(12);
  Map<ScoringPathOption, Command> simpleScoringPathMap = new HashMap<>(12);
  Map<ScoringPathOption, Command> simplePluckScoringMap = new HashMap<>(12);
  Map<ClimberPathOption, Command> simpleClimberScoringMap =  new HashMap<>(3);
  Map<ScoringPathOption, Command> pluckAlgaePathMap = new HashMap<>(12);
  Map<ClimberPathOption, Command> climberPathMap = new HashMap<>(3);
  Map<ScoringPathOption, Rotation2d> scoringAngleMap = new HashMap<>(12);
  Map<Integer, Command> NetPathMap = new HashMap<>(12);

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

      pathRightClimber = PathPlannerPath.fromPathFile("Climber Right");
      pathLeftClimber = PathPlannerPath.fromPathFile("Climber Left");
      pathMiddleClimber = PathPlannerPath.fromPathFile("Climber Middle");

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
  }

  private void defineSubsystems() {
    claw = new Claw();
    armevator = new Armevator();
    statusRgb = new StatusRgb(armevator);
    intake = new Intake();
    climber = new Climber();

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

    Command fourPieceRight = new PathPlannerAuto("4 piece", true);
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
  }

  private Command getAdjustSlowlyCommand(Supplier<Rotation2d> targetDirectionSupplier) {
    return new ConditionalCommand(
        Commands.deadline(
            new WaitUntilCommand(
                () ->
                    !visionApriltagSubsystem.hasReefTarget()
                        || Math.abs(visionApriltagSubsystem.getTX()) < 3.0),
            drivetrain.run(
                () ->
                    driveSlowlyDirection(
                        targetDirectionSupplier
                            .get()
                            .plus(
                                Rotation2d.kCW_90deg.times(
                                    Math.signum(visionApriltagSubsystem.getTX())))))),
        new InstantCommand(),
        visionApriltagSubsystem::hasReefTarget);
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
     *      ___________
     *      |    |    | ^
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

    // slow-mode toggle
    oi.slowModeSwitch().onTrue(Commands.runOnce(() -> isSlowMode = true));
    oi.slowModeSwitch().onFalse(Commands.runOnce(() -> isSlowMode = false));

    // reset gyro to 0 degrees
    oi.resetGyroButton()
        .onTrue(
            Commands.sequence(
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_HANDOFF)),
                new WaitCommand(0.25),
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
                                          ? visionApriltagSubsystem.getPoseEstimate().pose
                                          : new Pose2d(3.203, 4.190, new Rotation2d(0));
                                  drivetrain.resetPose(
                                      new Pose2d(
                                          visionPose.getX(), visionPose.getY(), new Rotation2d(0)));
                                  questNav.resetPose(
                                      new Pose2d(
                                          visionPose.getX(), visionPose.getY(), new Rotation2d(0)));
                                })),
                        drivetrain.run(() -> driveSlowlyDirection(Rotation2d.fromDegrees(0)))),
                    new InstantCommand(),
                    () -> true)));

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
    // oi.getSysIdQuasistaticForwarSd().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // oi.getSysIdQuasistaticReverse().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
            Commands.sequence(
                new InstantCommand(
                    () -> visionApriltagSubsystem.setPipeline(getScoringTargetPipeline())),
                intake.runOnce(() -> intake.setTargetPose(currentScoringLevelSupplier.get())),
                new ConditionalCommand(
                    // Full Auto
                    Commands.parallel(
                        // Raise Piece to scoring level
                        Commands.sequence(
                            new WaitUntilCommand(this::isRobotCloseToScoringPosition),
                            armevator.runOnce(
                                () -> armevator.setTargetPose(currentScoringLevelSupplier.get()))),
                        // Drive to scoring location
                        Commands.sequence(
                            new ConditionalCommand(
                                // Pathfind
                                getScoringPathCommand(),
                                // Drive directly to pose
                                Commands.sequence(
                                    new DriveToPose(
                                        drivetrain,
                                        () -> getPathStartingPose(scoringPathOption),
                                        driveFacingAngleRequest),
                                    new WaitCommand(0.2),
                                    getSimpleScoringPathCommand()),
                                this::isFarEnoughForPathfinding),
                            getAdjustSlowlyCommand(() -> scoringAngleMap.get(scoringPathOption)),
                            Commands.parallel(
                                Commands.sequence(new WaitCommand(0.25), new ClawBackwards(claw)),
                                drivetrain.run(
                                    () ->
                                        driveSlowlyDirection(
                                            scoringAngleMap.get(scoringPathOption)))))),
                    // Manual
                    Commands.parallel(
                        Commands.sequence(
                            new WaitCommand(0.5),
                            armevator.runOnce(
                                () -> armevator.setTargetPose(currentScoringLevelSupplier.get()))),
                        drivetrain.run(
                            () ->
                                driveFacingAngle(
                                    -oi.getTranslateX() * MaxSpeed,
                                    -oi.getTranslateY() * MaxSpeed,
                                    scoringAngleMap.get(scoringPathOption)))),
                    isFullAutoSupplier)));
    oi.scoreCoralButton()
        .onFalse(
            new ConditionalCommand(
                new InstantCommand(),
                armevator
                    .runOnce(() -> armevator.setTargetPose(ArmevatorPose.CORAL_POST_SCORE))
                    .asProxy(),
                () -> isPlucking));

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

    oi.intakeCoralButton()
        .whileTrue(
            Commands.deadline(
                Commands.sequence(
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
                Commands.runOnce(() -> currentScoringLevel = ArmevatorPose.CORAL_L1_SCORE),
                Commands.runOnce(() -> statusRgb.setScoringLevel(ScoringLevel.LEVEL_1)),
                Commands.runOnce(() -> armevator.updateScoringLevel(currentScoringLevel))));
    oi.operatorL2()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> currentScoringLevel = ArmevatorPose.CORAL_L2_SCORE),
                Commands.runOnce(() -> statusRgb.setScoringLevel(ScoringLevel.LEVEL_2)),
                Commands.runOnce(() -> armevator.updateScoringLevel(currentScoringLevel))));
    oi.operatorL3()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> currentScoringLevel = ArmevatorPose.CORAL_L3_SCORE),
                Commands.runOnce(() -> statusRgb.setScoringLevel(ScoringLevel.LEVEL_3)),
                Commands.runOnce(() -> armevator.updateScoringLevel(currentScoringLevel))));
    oi.operatorL4()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> currentScoringLevel = ArmevatorPose.CORAL_L4_SCORE),
                Commands.runOnce(() -> statusRgb.setScoringLevel(ScoringLevel.LEVEL_4)),
                Commands.runOnce(() -> armevator.updateScoringLevel(currentScoringLevel))));

    //////////////////
    // Algae Commands
    //////////////////

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
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.STARTING)),
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_HANDOFF)),
                Commands.waitSeconds(1.0),
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_HANDOFF)),
                Commands.parallel(
                    intake.run(() -> intake.stopIntake()), claw.run(() -> claw.brakeAlgae()))));

    oi.ejectAlgaeButton()
        .whileTrue(
            Commands.sequence(
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_HANDOFF)),
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_HANDOFF)),
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
            claw.runOnce(() -> claw.intakeAlgae()),
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
            intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_HANDOFF)),
            Commands.deadline(
                new ConditionalCommand(
                    getPluckPathCommand(),
                    Commands.sequence(
                        new DriveToPose(
                            drivetrain,
                            () -> getPluckPathStartingPose(scoringPathOption),
                            driveFacingAngleRequest),
                        getSimplePluckPathCommand()),
                    this::isFarEnoughForPathfindingPluck),
                Commands.sequence(
                    new WaitCommand(0.25),
                    intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_PRE_PLUCK_L2)))),
            getAdjustSlowlyCommand(() -> scoringAngleMap.get(scoringPathOption)),
            Commands.parallel(
                nonAutoPluck,
                drivetrain.run(
                    () -> driveSlowlyDirection(scoringAngleMap.get(scoringPathOption)))));

    oi.pluckAlgaeButton()
        .whileTrue(
            new ConditionalCommand(
                autoPluckCommand,
                Commands.deadline(
                    Commands.sequence(
                        Commands.runOnce(() -> isPlucking = true),
                        new PrintCommand("Non-Auto Command Started"),
                        claw.runOnce(() -> claw.intakeAlgae()),
                        intake.runOnce(
                            () -> intake.setTargetPose(ArmevatorPose.ALGAE_PRE_PLUCK_L2)),
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
                            }),
                        drivetrain
                            .run(
                                () ->
                                    driveFacingAngle(
                                        -oi.getTranslateX() * MaxSpeed,
                                        -oi.getTranslateY() * MaxSpeed,
                                        scoringAngleMap.get(scoringPathOption)))
                            .asProxy())),
                isFullAutoSupplier));

    oi.pluckAlgaeButton()
        .onFalse(
            Commands.sequence(
                Commands.runOnce(() -> isPlucking = false),
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.ALGAE_HANDOFF)),
                Commands.waitSeconds(0.25),
                armevator.runOnce(
                    () -> {
                      if (isFullAutoSupplier.getAsBoolean()) {
                        armevator.setTargetPose(ArmevatorPose.ALGAE_HANDOFF);
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
                    getDynamicNetPathCommand(),
                    new InstantCommand(),
                    () -> false), // this::isFarEnoughFromNetForPathfinding),
                new DriveToPoseSlew(
                    drivetrain,
                    () ->
                        new Pose2d(
                            7.65,
                            drivetrain.getPose().getY(),
                            preferNetRightSideSupplier.getAsBoolean()
                                ? Rotation2d.fromDegrees(135)
                                : Rotation2d.fromDegrees(-135)),
                    driveFacingAngleRequest),
                Commands.deadline(
                    Commands.sequence(
                        claw.runOnce(() -> claw.brakeAlgae()),
                        // score net command
                        armevator.runOnce(
                            () -> armevator.setTargetPose(ArmevatorPose.ALGAE_NET_SCORE)),
                        Commands.waitUntil(armevator::isAtNetReleaseAngle),
                        claw.runOnce(() -> claw.ejectAlgae()),
                        Commands.waitSeconds(0.2),
                        claw.runOnce(() -> claw.stopClaw()),
                        armevator.runOnce(
                            () -> armevator.setTargetPose(ArmevatorPose.ALGAE_HANDOFF))),
                    drivetrain.run(
                        () ->
                            driveFacingAngle(
                                0,
                                0,
                                preferNetRightSideSupplier.getAsBoolean()
                                    ? Rotation2d.fromDegrees(135)
                                    : Rotation2d.fromDegrees(-135))))));
    oi.aimAtNetButton()
        .onFalse(armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.ALGAE_HANDOFF)));

    ////////////////////
    // Climber Commands
    ////////////////////
    
    Command nonAutoClimberCommand = Commands.sequence(
        climber.runOnce(() -> climber.engageWindmill()),
        new WaitCommand(0.1),
        climber.run(() -> climber.retractClimber()));

    Command nonAutoClimberCommandCopy = Commands.sequence( // copy needed to prevent errors
        climber.runOnce(() -> climber.engageWindmill()),
        new WaitCommand(0.1),
        climber.run(() -> climber.retractClimber())); 

    Command autoClimberCommand = new SequentialCommandGroup(
        nonAutoClimberCommandCopy,
        new ConditionalCommand(
            getClimberPathCommand(),
            Commands.sequence(
                new DriveToPose(
                    drivetrain,
                    () -> getClimberPathStartingPose(climbingPathOption),
                    driveFacingAngleRequest),
                getSimpleClimberPathCommand()),
            this::isFarEnoughForPathfindingClimber)
            ); 

    oi.operatorExtendClimber()
        .whileTrue(
            Commands.sequence(
                intake.runOnce(() -> intake.setTargetPose(ArmevatorPose.CLIMB)),
                armevator.runOnce(() -> armevator.setTargetPose(ArmevatorPose.CLIMB)),
                climber.runOnce(() -> climber.disengageWindmill()),
                climber.run(() -> climber.extendClimber())));
    oi.operatorExtendClimber().onFalse(climber.runOnce(() -> climber.stopClimber()));

    oi.operatorRetractClimber()
        .whileTrue(new ConditionalCommand(autoClimberCommand, nonAutoClimberCommand, isFullAutoSupplier));
    oi.operatorRetractClimber().onFalse(climber.runOnce(() -> climber.brakeClimber()));

    oi.retractClimberSlowlySwitch().whileTrue(climber.runOnce(() -> climber.brakeClimber()));
    oi.retractClimberSlowlySwitch().onFalse(climber.runOnce(() -> climber.stopClimber()));
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
    //   this.lastAlliance = alliance.get();
    //   Field2d.getInstance().updateAlliance(this.lastAlliance);
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

  private boolean isFarEnoughForPathfinding() {
    return getDistanceFromTarget() > 2.0;
  }

  private boolean isFarEnoughForPathfindingPluck() {
    return getPluckDistanceFromTarget() > 2.0;
  }

  private boolean isFarEnoughForPathfindingClimber() {
    return getClimberDistanceFromTarget() > 2.0;
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

  private double getClimberDistanceFromTarget() {
    Pose2d targetPose = getClimberPathStartingPose(climbingPathOption);
    Pose2d currentPose = drivetrain.getPose();
    return targetPose.getTranslation().getDistance(currentPose.getTranslation());
  }

  private double getDistanceFromNet() {
    Pose2d targetPose =
        new Pose2d(NET_SCORE_LOCATION_X, drivetrain.getPose().getY(), Rotation2d.fromDegrees(135));
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

  private Command getClimberPathCommand() {
    return new SelectCommand<>(climberPathMap, () -> climbingPathOption);
  }

  private Command getPluckPathCommand() {
    return new SelectCommand<>(pluckAlgaePathMap, () -> scoringPathOption);
  }

  private Command getScoringPathCommand() {
    return new SelectCommand<>(scoringPathMap, () -> scoringPathOption);
  }

  private Command getNetPathCommand() {
    return new SelectCommand<>(NetPathMap, this::getNetPathIndex);
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
                      ? Rotation2d.fromDegrees(135)
                      : Rotation2d.fromDegrees(-135));
          return AutoBuilder.pathfindToPose(targetPose, hpPathConstraints);
        });
  }

  private Command getSimpleScoringPathCommand() {
    return new SelectCommand<>(simpleScoringPathMap, () -> scoringPathOption);
  }

  private Command getSimplePluckPathCommand() {
    return new SelectCommand<>(simplePluckScoringMap, () -> scoringPathOption);
  }

  private Command getSimpleClimberPathCommand() {
    return new SelectCommand<>(simpleClimberScoringMap, () -> climbingPathOption);
  }

  private Pose2d getPathStartingPose(ScoringPathOption scoringPathOption) {
    return getCurrentScoringPath(scoringPathOption).getStartingHolonomicPose().get();
  }

  private Pose2d getPluckPathStartingPose(ScoringPathOption scoringPathOption) {
    return getCurrentScoringPluckPath(scoringPathOption).getStartingHolonomicPose().get();
  }

  private Pose2d getClimberPathStartingPose(ClimberPathOption climberPathOption) {
    return getCurrentScoringClimberPath(climberPathOption).getStartingHolonomicPose().get();
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

  private PathPlannerPath getCurrentScoringClimberPath(ClimberPathOption climberPathOption) {
    return pathFAlgae; // TODO replace with an actual path
  }

  // run on init
  private void setupScoringPathMap() {
    scoringPathMap.put(
        ScoringPathOption.PATH_F1,
        AutoBuilder.pathfindThenFollowPath(pathF1, scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_F2,
        AutoBuilder.pathfindThenFollowPath(pathF2, scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_FL1,
        AutoBuilder.pathfindThenFollowPath(pathFL1, scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_FL2,
        AutoBuilder.pathfindThenFollowPath(pathFL2, scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_FR1,
        AutoBuilder.pathfindThenFollowPath(pathFR1, scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_FR2,
        AutoBuilder.pathfindThenFollowPath(pathFR2, scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_BL1,
        AutoBuilder.pathfindThenFollowPath(pathBL1, scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_BL2,
        AutoBuilder.pathfindThenFollowPath(pathBL2, scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_BR1,
        AutoBuilder.pathfindThenFollowPath(pathBR1, scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_BR2,
        AutoBuilder.pathfindThenFollowPath(pathBR2, scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_B1,
        AutoBuilder.pathfindThenFollowPath(pathB1, scorePathConstraints));
    scoringPathMap.put(
        ScoringPathOption.PATH_B2,
        AutoBuilder.pathfindThenFollowPath(pathB2, scorePathConstraints));

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

    climberPathMap.put(
        ClimberPathOption.PATH_LEFT,
        AutoBuilder.pathfindThenFollowPath(pathLeftClimber, climberPathConstraints));
    climberPathMap.put(
        ClimberPathOption.PATH_RIGHT,
        AutoBuilder.pathfindThenFollowPath(pathRightClimber, climberPathConstraints));
    climberPathMap.put(
        ClimberPathOption.PATH_MIDDLE,
        AutoBuilder.pathfindThenFollowPath(pathMiddleClimber, climberPathConstraints));

        simpleClimberScoringMap.put(ClimberPathOption.PATH_LEFT, AutoBuilder.followPath(pathLeftClimber));
        simpleClimberScoringMap.put(ClimberPathOption.PATH_RIGHT, AutoBuilder.followPath(pathRightClimber));
        simpleClimberScoringMap.put(ClimberPathOption.PATH_MIDDLE, AutoBuilder.followPath(pathMiddleClimber));

    NetPathMap.put(
        0,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 0),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
    NetPathMap.put(
        1,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 1),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
    NetPathMap.put(
        2,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 2),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
    NetPathMap.put(
        3,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 3),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
    NetPathMap.put(
        4,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 4),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
    NetPathMap.put(
        5,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 5),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
    NetPathMap.put(
        6,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 6),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
    NetPathMap.put(
        7,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 7),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
    NetPathMap.put(
        8,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 8),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
    NetPathMap.put(
        9,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 9),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
    NetPathMap.put(
        10,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 10),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
    NetPathMap.put(
        11,
        AutoBuilder.pathfindToPose(
            new Pose2d(
                NET_SCORE_LOCATION_X,
                4.7 + (NET_SCORE_LOCATION_X / 4.7 / 11.0 * 111),
                Rotation2d.fromDegrees(135.0)),
            hpPathConstraints));
  }

  private Integer getNetPathIndex() {
    Pose2d currentPose = drivetrain.getPose();
    double y = currentPose.getY();

    int index =
        (int)
            ((y - (4.7 - NET_SCORE_LOCATION_X / 4.7 / 11.0 / 2.0))
                / ((NET_SCORE_LOCATION_X - 4.7) / 12.0));

    if (index < 0) {
      return 0;
    }
    if (index > 11) {
      return 11;
    }
    return index;
  }

  public void updateVisionPose() {
    if (questNav.isConnected() && isVisionEnabled) {
      // drivetrain.addVisionMeasurement(
      // questNav.getRobotPose(), VecBuilder.fill(0.0, 0.0, 9999999.0));
      drivetrain.addVisionMeasurement(
          questNav.getAverageRobotPose(), VecBuilder.fill(0.0, 0.0, 0.0));
      return;
    }

    // LimelightHelpers.PoseEstimate limelightMeasurement =
    // visionApriltagSubsystem.getPoseEstimate();
    // if (limelightMeasurement != null && (limelightMeasurement.tagCount >= 2
    //     || (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist < 1.25))) {
    //   drivetrain.addVisionMeasurement(
    //       limelightMeasurement.pose,
    //       limelightMeasurement.timestampSeconds,
    //       VecBuilder.fill(.6, .6, 9999999));
    // }
  }
}
