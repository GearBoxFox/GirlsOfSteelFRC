package com.gos.crescendo2024.commands;

import com.gos.crescendo2024.subsystems.ArmPivotSubsystem;
import com.gos.lib.properties.GosDoubleProperty;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class ArmPivotJoystickCommand extends Command {

    private static final double ARM_PIVOT_JOYSTICK_DEADBAND = 0.025;
    private static final GosDoubleProperty DAMPING = new GosDoubleProperty(false, "ArmPivotJoystickTranslationDamping", 0.4);
    private final CommandXboxController m_joystick;

    private final ArmPivotSubsystem m_armPivotSubsystem;


    public ArmPivotJoystickCommand(ArmPivotSubsystem armPivotSubsystem, CommandXboxController controller) {
        addRequirements(armPivotSubsystem);
        m_joystick = controller;
        m_armPivotSubsystem = armPivotSubsystem;

    }

    @Override
    public void execute() {
        double yPercent = -MathUtil.applyDeadband(m_joystick.getLeftY() * DAMPING.getValue(), ARM_PIVOT_JOYSTICK_DEADBAND);
        m_armPivotSubsystem.setArmPivotSpeed(yPercent);
    }

}
