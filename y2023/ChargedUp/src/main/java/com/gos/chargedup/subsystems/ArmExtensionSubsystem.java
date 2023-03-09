package com.gos.chargedup.subsystems;


import com.gos.chargedup.AutoPivotHeight;
import com.gos.chargedup.Constants;
import com.gos.lib.checklists.DoubleSolenoidMovesChecklist;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;

public class ArmExtensionSubsystem extends SubsystemBase {
    private static final DoubleSolenoid.Value TOP_PISTON_EXTENDED = DoubleSolenoid.Value.kReverse;
    private static final DoubleSolenoid.Value TOP_PISTON_RETRACTED = DoubleSolenoid.Value.kForward;

    private static final DoubleSolenoid.Value BOTTOM_PISTON_EXTENDED = DoubleSolenoid.Value.kReverse;
    private static final DoubleSolenoid.Value BOTTOM_PISTON_RETRACTED = DoubleSolenoid.Value.kForward;

    private static final double PNEUMATICS_WAIT = 1.3;

    //Todo: Get the actual values for the lengths for the arm
    private static final double ARM_RETRACTED_LENGTH = 1.0;

    private static final double ARM_MIDDLE_LENGTH = 1.0;

    private static final double ARM_EXTENDED_LENGTH = 1.0;

    private final DoubleSolenoid m_topPiston;
    private final DoubleSolenoid m_bottomPiston;

    private double m_currentArmLengthMeters;

    public ArmExtensionSubsystem() {
        m_topPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ARM_TOP_PISTON_OUT, Constants.ARM_TOP_PISTON_IN);
        m_bottomPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ARM_BOTTOM_PISTON_FORWARD, Constants.ARM_BOTTOM_PISTON_REVERSE);
        fullRetract();
    }

    public final void fullRetract() {
        m_topPiston.set(TOP_PISTON_EXTENDED);
        m_bottomPiston.set(BOTTOM_PISTON_RETRACTED);
        m_currentArmLengthMeters = ARM_RETRACTED_LENGTH;
    }

    public void middleRetract() {
        m_topPiston.set(TOP_PISTON_RETRACTED);
        m_bottomPiston.set(BOTTOM_PISTON_RETRACTED);
        m_currentArmLengthMeters = ARM_MIDDLE_LENGTH;
    }

    public void out() {
        m_topPiston.set(TOP_PISTON_RETRACTED);
        m_bottomPiston.set(BOTTOM_PISTON_EXTENDED);
        m_currentArmLengthMeters = ARM_EXTENDED_LENGTH;
    }

    public double getArmLengthMeters() {
        return m_currentArmLengthMeters;
    }

    public boolean isBottomPistonIn() {
        return m_bottomPiston.get() == BOTTOM_PISTON_EXTENDED;
    }

    public boolean isTopPistonIn() {
        return m_topPiston.get() == TOP_PISTON_RETRACTED;
    }

    public void setBottomPistonExtended() {
        m_bottomPiston.set(BOTTOM_PISTON_EXTENDED);
    }

    public void setBottomPistonRetracted() {
        m_bottomPiston.set(BOTTOM_PISTON_RETRACTED);
    }

    public void setTopPistonExtended() {
        m_topPiston.set(TOP_PISTON_EXTENDED);
    }

    public void setTopPistonRetracted() {
        m_topPiston.set(TOP_PISTON_RETRACTED);
    }

    ///////////////////////
    // Command Factories
    ///////////////////////

    public CommandBase commandBottomPistonExtended() {
        return runOnce(this::setBottomPistonExtended).withName("Arm Piston: Bottom Extended");
    }

    public CommandBase commandBottomPistonRetracted() {
        return runOnce(this::setBottomPistonRetracted).withName("Arm Piston: Bottom Retracted");
    }

    public CommandBase commandTopPistonExtended() {
        return runOnce(this::setTopPistonExtended).withName("Arm Piston: Top Extended");
    }

    public CommandBase commandTopPistonRetracted() {
        return runOnce(this::setTopPistonRetracted).withName("Arm Piston: Top Retracted");
    }

    public CommandBase commandFullRetractPrevention(ChassisSubsystem cs, CommandXboxController x) {
        return new ConditionalCommand(
            this.run(this::fullRetract),
            this.run(() ->  x.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1)),
            cs::canExtendArm);
    }

    public CommandBase commandFullRetract() {
        return run(this::fullRetract).withTimeout(PNEUMATICS_WAIT).withName("ArmPistonsFullRetract");
    }

    public CommandBase commandMiddleRetractPrevention(ChassisSubsystem cs, CommandXboxController x) {
        return new ConditionalCommand(
            this.run(this::middleRetract).withName("ArmPistonsMiddleRetract"),
            this.run(() ->  x.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1)),
            cs::canExtendArm);
    }

    public CommandBase commandMiddleRetract() {
        return run(this::middleRetract).withTimeout(PNEUMATICS_WAIT).withName("ArmPistonsMiddleRetract");
    }

    public CommandBase commandFullExtendPrevention(ChassisSubsystem cs, CommandXboxController x) {
        return new ConditionalCommand(
            this.run(this::out).withName("Arm Pistons: FullExtend").withTimeout(PNEUMATICS_WAIT),
            this.run(() ->  x.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1)),
            cs::canExtendArm);
    }

    public CommandBase commandFullExtend() {
        return run(this::out).withTimeout(PNEUMATICS_WAIT).withName("ArmPistonsOut");
    }

    // TODO Are these ones necessary
    public CommandBase commandOutPrevention(ChassisSubsystem cs, CommandXboxController x) {
        return new ConditionalCommand(
            this.run(this::out).withName("ArmPistonsOut"),
            this.run(() ->  x.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1)),
            cs::canExtendArm);
    }

    public CommandBase createArmToSpecifiedHeight(AutoPivotHeight height) {
        if (height == AutoPivotHeight.HIGH) {
            return commandFullExtend();
        }
        else if (height == AutoPivotHeight.MEDIUM) {
            return commandMiddleRetract();
        }
        else {
            return commandFullRetract();
        }
    }

    ////////////////
    // Checklists
    ////////////////


    public CommandBase createIsArmTopPneumaticMoving(DoubleSupplier pressureSupplier) {
        return new DoubleSolenoidMovesChecklist(this, pressureSupplier, m_topPiston, "Claw: Left Piston");
    }

    public CommandBase createIsArmBottomPneumaticMoving(DoubleSupplier pressureSupplier) {
        return new DoubleSolenoidMovesChecklist(this, pressureSupplier, m_bottomPiston, "Arm: Bottom Piston");
    }

}
