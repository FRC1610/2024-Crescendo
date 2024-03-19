package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_IntakeMotor;
    private final CANSparkMax m_IntakeFollower1;
    private final CANSparkMax m_IntakeFollower2;
    
    //private final DigitalInput m_IndexSensorFront = new DigitalInput(IndexerConstants.kIndexerSensorFrontDIOPort);

    public Intake() {
        // Spark Max Sextup
        m_IntakeMotor = new CANSparkMax(4, MotorType.kBrushless);  //Front Rail
        //m_IntakeMotor.restoreFactoryDefaults();
        m_IntakeMotor.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        m_IntakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        //m_IntakeMotor.burnFlash();

        m_IntakeFollower1 = new CANSparkMax(5, MotorType.kBrushless); //Lower Inner Roller
        //m_IntakeFollower1.restoreFactoryDefaults();
        m_IntakeFollower1.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        m_IntakeFollower1.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        m_IntakeFollower1.follow(m_IntakeMotor, true);
        //m_IntakeFollower1.burnFlash();

        m_IntakeFollower2 = new CANSparkMax(6, MotorType.kBrushless); //Higher Inner Roller
        //m_IntakeFollower2.restoreFactoryDefaults();
        m_IntakeFollower2.setIdleMode(IntakeConstants.kIntakeMotorIdleMode);
        m_IntakeFollower2.setSmartCurrentLimit(IntakeConstants.kIntakeMotorCurrentLimit);
        m_IntakeFollower2.follow(m_IntakeMotor, false);
        m_IntakeFollower2.burnFlash();

    }

private void StopIntake() {
    m_IntakeMotor.set(0.0); // Stop intake motor
}

private void RunIntake(double IntakeSpeed) {
    m_IntakeMotor.set(IntakeSpeed); // Run intake motor
}

private void NoteIntake(double IntakeSpeed, double Angle){
    if(Angle > 50){
       if(Angle < 55){
        m_IntakeMotor.set(IntakeSpeed);
       } 
    }
    else{
        m_IntakeMotor.set(0.0);
    }
}

public Command StopIntakeCommand() {
    return this.runOnce(() -> this.StopIntake());
}

public Command RunIntakeCommand(double IntakeSpeed) {
    return this.run(() -> this.RunIntake(IntakeSpeed));
}

public Command NoteIntakeCommand(double IntakeSpeed, double Angle){
    return this.run(() -> this.NoteIntake(IntakeSpeed, Angle));
}

@Override
public void periodic() {
    SmartDashboard.putNumber("IntakeCurrent", m_IntakeMotor.getOutputCurrent());
    
}

}