package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
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
        m_LauncherMotor.setOpenLoopRampRate(LauncherConstants.kLauncherRampRate);
        m_LauncherMotor.setInverted(true);
        m_LauncherMotor.burnFlash();

        m_LauncherFollower = new CANSparkMax(9, MotorType.kBrushless);
        m_LauncherFollower.restoreFactoryDefaults();
        m_LauncherFollower.setIdleMode(LauncherConstants.kLauncherMotorIdleMode);
        m_LauncherFollower.setSmartCurrentLimit(LauncherConstants.kLauncherMotorCurrentLimit);
        m_LauncherFollower.setOpenLoopRampRate(LauncherConstants.kLauncherRampRate);
        //m_LauncherFollower.follow(m_LauncherMotor, true);
        m_LauncherMotor.setInverted(true);
        m_LauncherFollower.burnFlash();
    }

private void StopLauncher() {
    m_LauncherMotor.set(0.0); // Stop launcher motors
    m_LauncherFollower.set(0);
}

private void RunLauncher(double LauncherSpeedLeft, double LauncherSpeedRight) {
    m_LauncherMotor.set(LauncherSpeedRight); // Run launcher motors
    m_LauncherFollower.set(LauncherSpeedLeft);
}

private void RunLauncherRPM(double LauncherRPM) {
    // PID Setup
    SparkPIDController LauncherPID = m_LauncherMotor.getPIDController();
    double kP = 0.1;
    double kI = 0.1;
    double kD = 0.0;
    LauncherPID.setP(kP);
    LauncherPID.setI(kI);
    LauncherPID.setD(kD);
    LauncherPID.setReference(LauncherRPM, ControlType.kVelocity); // Run launcher motors
}

public Command StopLauncherCommand() {
    return this.runOnce(() -> this.StopLauncher());
}

public Command RunLauncherCommand(double LauncherSpeedLeft, double LauncherSpeedRight) {
    return this.run(() -> this.RunLauncher(LauncherSpeedLeft, LauncherSpeedRight));
}

public Command RunLauncherRPMCommand(double LauncherRPM) {
    return this.run(() -> this.RunLauncherRPM(LauncherRPM));
}

@Override
public void periodic() {
    SmartDashboard.putNumber("LauncherCurrent", m_LauncherMotor.getOutputCurrent());
    SmartDashboard.putNumber("LauncherRPM", m_LauncherMotor.getEncoder().getVelocity());
}

}