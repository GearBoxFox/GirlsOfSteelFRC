package com.gos.swerve2023.subsystems;


import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.gos.lib.rev.swerve.RevSwerveChassis;
import com.gos.lib.rev.swerve.RevSwerveChassisConstants;
import com.gos.lib.rev.swerve.RevSwerveModuleConstants;
import com.gos.swerve2023.Constants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.snobotv2.module_wrappers.ctre.CtrePigeonImuWrapper;

public class ChassisSubsystem extends SubsystemBase {

    private static final double WHEEL_BASE = Units.inchesToMeters(28.25);
    private static final double TRACK_WIDTH = Units.inchesToMeters(28.25);

    public static final double MAX_TRANSLATION_SPEED = Units.feetToMeters(13);
    public static final double MAX_ROTATION_SPEED = Units.degreesToRadians(360);


    private final RevSwerveChassis m_swerveDrive;
    private final WPI_Pigeon2 m_gyro;

    private final Field2d m_field;

    public ChassisSubsystem() {
        m_gyro = new WPI_Pigeon2(Constants.PIGEON_PORT);
        m_gyro.configFactoryDefault();

        RevSwerveChassisConstants swerveConstants = new RevSwerveChassisConstants(
            Constants.FRONT_LEFT_WHEEL, Constants.FRONT_LEFT_AZIMUTH,
            Constants.BACK_LEFT_WHEEL, Constants.BACK_LEFT_AZIMUTH,
            Constants.FRONT_RIGHT_WHEEL, Constants.FRONT_RIGHT_AZIMUTH,
            Constants.BACK_RIGHT_WHEEL, Constants.BACK_RIGHT_AZIMUTH,
            RevSwerveModuleConstants.DriveMotorTeeth.T14,
            WHEEL_BASE, TRACK_WIDTH,
            MAX_TRANSLATION_SPEED,
            MAX_ROTATION_SPEED
        );
        m_swerveDrive = new RevSwerveChassis(swerveConstants, m_gyro::getRotation2d, new CtrePigeonImuWrapper(m_gyro));

        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);


            // Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            m_swerveDrive::getEstimatedPosition, // Robot pose supplier
            m_swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            m_swerveDrive::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            m_swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            this // Reference to this subsystem to set requirements
        );

    }

    @Override
    public void periodic() {
        m_swerveDrive.periodic();
        m_field.setRobotPose(m_swerveDrive.getEstimatedPosition());
    }

    @Override
    public void simulationPeriodic() {
        m_swerveDrive.updateSimulator();
    }

    public void teleopDrive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
        m_swerveDrive.driveWithJoysticks(xPercent, yPercent, rotPercent, fieldRelative);
    }


    public void resetOdometry(Pose2d pose2d) {
        m_swerveDrive.resetOdometry(pose2d);
    }

    public Command createFollowPathCommand(String pathName, boolean resetPose) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        Command followPathCommand = AutoBuilder.followPathWithEvents(path);
        if (resetPose) {
            Pose2d pose = path.getPreviewStartingHolonomicPose();
            return Commands.runOnce(() -> resetOdometry(pose)).andThen(followPathCommand);
        }
        return followPathCommand;
    }
}
