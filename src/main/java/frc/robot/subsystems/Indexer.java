package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    private final CANSparkMax m_IndexerMotor;

    private final DigitalInput m_IndexSensorRear = new DigitalInput(IndexerConstants.kIndexerSensorRearDIOPort);
    private final DigitalInput m_IndexSensorFront = new DigitalInput(IndexerConstants.kIndexerSensorFrontDIOPort);

    public Indexer() {
        // Spark Max Sextup
        m_IndexerMotor = new CANSparkMax(7, MotorType.kBrushless);
        m_IndexerMotor.restoreFactoryDefaults();
        m_IndexerMotor.setIdleMode(IndexerConstants.kIndexerMotorIdleMode);
        m_IndexerMotor.setSmartCurrentLimit(IndexerConstants.kIndexerMotorCurrentLimit);
        m_IndexerMotor.burnFlash();
    }

private void StopIndexer() {
    m_IndexerMotor.set(0.0); // Stop indexer
}

private void Runindexer(double IndexerSpeed) {
    m_IndexerMotor.set(IndexerSpeed); // Run indexer
}

public Command StopIndexerCommand() {
    return this.runOnce(() -> this.StopIndexer());
}

public Command RunIndexerCommand(double IndexerSpeed) {
    return this.run(() -> this.Runindexer(IndexerSpeed));
}

@Override
public void periodic() {
    SmartDashboard.putNumber("IndexerCurrent", m_IndexerMotor.getOutputCurrent());
}

}