package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;


public class Launcher extends SubsystemBase {
    private final CANSparkMax m_LauncherMotor;
    private final CANSparkMax m_LauncherFollower;

    public Launcher() {
        // Spark Max Sextup
        m_LauncherMotor = new CANSparkMax(8, MotorType.kBrushless);
        m_LauncherMotor.restoreFactoryDefaults();
        m_LauncherMotor.setIdleMode(LauncherConstants.kLauncherMotorIdleMode);
        m_LauncherMotor.setSmartCurrentLimit(LauncherConstants.kLauncherMotorCurrentLimit);
        m_LauncherMotor.burnFlash();

        m_LauncherFollower = new CANSparkMax(9, MotorType.kBrushless);
        m_LauncherFollower.restoreFactoryDefaults();
        m_LauncherFollower.setIdleMode(LauncherConstants.kLauncherMotorIdleMode);
        m_LauncherFollower.setSmartCurrentLimit(LauncherConstants.kLauncherMotorCurrentLimit);
        m_LauncherFollower.follow(m_LauncherMotor, true);
        m_LauncherFollower.burnFlash();

    }

private void StopLauncher() {
    m_LauncherMotor.set(0.0); // Stop intake motor
}

private void RunLauncher(double IntakeSpeed) {
    m_LauncherMotor.set(IntakeSpeed); // Run intake motor
}

public Command StopLauncherCommand() {
    return this.runOnce(() -> this.StopLauncher());
}

public Command RunLauncherCommand(double IntakeSpeed) {
    return this.run(() -> this.RunLauncher(IntakeSpeed));
}

@Override
public void periodic() {
    SmartDashboard.putNumber("LauncherCurrent", m_LauncherMotor.getOutputCurrent());
}

}
