// RobotBuilder Version: 3.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SimableCANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class Collector extends SubsystemBase {
    private static final double FOURBAR_SPEED = 0.10;
    private static final double COLLECTOR_SPEED = 0.50;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final SimableCANSparkMax m_fourBarLeft;
    private final SimableCANSparkMax m_fourBarRight;
    private final SimableCANSparkMax m_collector;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
     *
     */
    public Collector() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        m_fourBarLeft = new SimableCANSparkMax(5, MotorType.kBrushed);
        m_fourBarLeft.setInverted(true);

        m_fourBarRight = new SimableCANSparkMax(6, MotorType.kBrushed);
        m_fourBarRight.setInverted(false);

        m_collector = new SimableCANSparkMax(7, MotorType.kBrushed);
        m_collector.setInverted(false);


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void extendCollector() { 
        m_fourBarLeft.set(FOURBAR_SPEED);
        m_fourBarRight.set(FOURBAR_SPEED);

    }

    public void retractCollector() {
        m_fourBarLeft.set(-FOURBAR_SPEED);
        m_fourBarRight.set(-FOURBAR_SPEED);

    }

    public void intakeBall() {
        m_collector.set(COLLECTOR_SPEED);
    }

    public void reverseBall() {
        m_collector.set(-COLLECTOR_SPEED);
    }

    public void stopIntake() {
        m_collector.set(0); 
    }

    public void stopFourbar() {
        m_fourBarLeft.set(0); 
        m_fourBarRight.set(0); 
    }
}

