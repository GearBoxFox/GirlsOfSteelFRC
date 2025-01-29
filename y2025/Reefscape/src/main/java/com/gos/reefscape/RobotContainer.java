// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.gos.reefscape;

import com.gos.reefscape.commands.Autos;
import com.gos.reefscape.commands.CombinedCommands;
import com.gos.reefscape.commands.MovePivotWithJoystickCommand;
import com.gos.reefscape.commands.SwerveWithJoystickCommand;
import com.gos.reefscape.commands.MoveElevatorWithJoystickCommand;
import com.gos.reefscape.subsystems.IntakeSubsystem;
import com.gos.reefscape.subsystems.ElevatorSubsystem;
import com.gos.reefscape.subsystems.PivotSubsystem;
import com.gos.reefscape.subsystems.SuperStructureViz;
import com.gos.reefscape.subsystems.drive.GOSSwerveDrive;
import com.gos.reefscape.subsystems.drive.TunerConstants;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.gos.lib.pathing.PathPlannerUtils.followChoreoPath;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    //    private final GOSSwerveDrive m_chassis = new SdsWithRevChassisSubsystem();
    private final GOSSwerveDrive m_chassis = TunerConstants.createDrivetrain();
    //    private final GOSSwerveDrive m_chassis = new DollySwerve();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
    private final CombinedCommands m_combinedCommand = new CombinedCommands(m_intakeSubsystem, m_elevator, m_pivotSubsystem);

    private final SuperStructureViz m_superStructureViz = new SuperStructureViz(m_elevator, m_pivotSubsystem); // NOPMD(UnusedPrivateField)

    private final Autos m_autos = new Autos(m_chassis, m_combinedCommand);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController m_operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        if (RobotBase.isSimulation()) {
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
            DriverStationSim.setDsAttached(true);
            DriverStationSim.setEnabled(true);
        }
        addDebugPathsToShuffleBoard();
        addIntakeDebugCommands();
        addPivotDebugCommands();
        addElevatorDebugCommands();

        createMovePIECommand();

        // PropertyManager.purgeExtraKeys();

    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        m_chassis.setDefaultCommand(new SwerveWithJoystickCommand(m_chassis, m_driverController));
        m_elevator.setDefaultCommand(new MoveElevatorWithJoystickCommand(m_elevator, m_operatorController));
        m_pivotSubsystem.setDefaultCommand(new MovePivotWithJoystickCommand(m_pivotSubsystem, m_operatorController));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return m_autos.getSelectedAuto();
    }

    private void addIntakeDebugCommands() {
        ShuffleboardTab debugTab = Shuffleboard.getTab("Intake Outtake");
        debugTab.add(m_intakeSubsystem.createMoveIntakeOutCommand());
        debugTab.add(m_intakeSubsystem.createIntakeUntilCoralCommand());
        debugTab.add(m_intakeSubsystem.createMoveIntakeInCommand());

    }

    private void addPivotDebugCommands() {
        ShuffleboardTab debugTabPivot = Shuffleboard.getTab("arm pivot");
        debugTabPivot.add(m_pivotSubsystem.createMoveArmtoAngleCommand(0.0));
        debugTabPivot.add(m_pivotSubsystem.createMoveArmtoAngleCommand(45.0));
        debugTabPivot.add(m_pivotSubsystem.createMoveArmtoAngleCommand(90.0));

    }

    private void addElevatorDebugCommands() {
        ShuffleboardTab debugTab = Shuffleboard.getTab("Elevator");
        debugTab.add(m_elevator.createMoveElevatorToHeightCommand(Units.feetToMeters(1)));
        debugTab.add(m_elevator.createMoveElevatorToHeightCommand(Units.feetToMeters(3)));
        debugTab.add(m_elevator.createMoveElevatorToHeightCommand(Units.feetToMeters(5)));
    }


    private void addDebugPathsToShuffleBoard() {
        ShuffleboardTab debugPathsTab = Shuffleboard.getTab("Debug Paths");
        debugPathsTab.add(createDebugPathCommand("TestPath_1mpss_1fps"));
        debugPathsTab.add(createDebugPathCommand("TestPath_1mpss_5fps"));
        debugPathsTab.add(createDebugPathCommand("TestPath_1mpss_10fps"));
        debugPathsTab.add(createDebugPathCommand("TestPath_1mpss_13fps"));
        debugPathsTab.add(createDebugPathCommand("TestPath_4mpss_1fps"));
        debugPathsTab.add(createDebugPathCommand("TestPath_4mpss_5fps"));
        debugPathsTab.add(createDebugPathCommand("TestPath_4mpss_10fps"));
        debugPathsTab.add(createDebugPathCommand("TestPath_4mpss_13fps"));
        debugPathsTab.add(createDebugPathCommand("TestPath_9mpss_1fps"));
        debugPathsTab.add(createDebugPathCommand("TestPath_9mpss_5fps"));
        debugPathsTab.add(createDebugPathCommand("TestPath_9mpss_10fps"));
        debugPathsTab.add(createDebugPathCommand("TestPath_9mpss_13fps"));
        debugPathsTab.add(createDebugPathCommand("AbbyPaneer"));
        debugPathsTab.add(createDebugPathCommand("PjNoodles"));


    }

    private Command createDebugPathCommand(String name) {
        return Commands.sequence(
            Commands.runOnce(() -> m_chassis.resetPose(ChoreoUtils.getPathStartingPose(name).getPose())),
            followChoreoPath(name)
        ).withName(name);
    }

    private void createMovePIECommand() {
        ShuffleboardTab debugTab = Shuffleboard.getTab("Combined Commands");
        debugTab.add(m_combinedCommand.scoreCoralCommand(PIE.L1).withName("Level One"));
        debugTab.add(m_combinedCommand.scoreCoralCommand(PIE.L2).withName("Level Two"));
        debugTab.add(m_combinedCommand.scoreCoralCommand(PIE.L3).withName("Level Three"));
        debugTab.add(m_combinedCommand.scoreCoralCommand(PIE.L4).withName("Level Four"));
        debugTab.add(m_combinedCommand.scoreCoralCommand(PIE.SCORE_INTO_NET).withName("Score Into net"));
        debugTab.add(m_combinedCommand.scoreCoralCommand(PIE.SCORE_INTO_PROCESSOR).withName("Score into processor"));
        debugTab.add(m_combinedCommand.fetchPieceFromHPStation().withName("human player station"));
        debugTab.add(m_combinedCommand.fetchAlgaeTwo().withName("Fetch Algae"));


    }

}
