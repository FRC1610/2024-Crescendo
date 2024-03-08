package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final CANSparkMax m_leftClimb;
    private final CANSparkMax m_rightClimb;

    public Climber() {
        // Spark Max Sextup

        // Left Climb Motor
        m_leftClimb = new CANSparkMax(18, MotorType.kBrushless);
        m_leftClimb.restoreFactoryDefaults();
        m_leftClimb.setInverted(true);
        m_leftClimb.setIdleMode(ClimberConstants.kClimberIdleMode);
        m_leftClimb.setSmartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);

        //Right Climb Motor
        m_rightClimb = new CANSparkMax(19, MotorType.kBrushless);
        m_rightClimb.restoreFactoryDefaults();
        m_rightClimb.setInverted(false);
        m_rightClimb.setIdleMode(ClimberConstants.kClimberIdleMode);
        m_rightClimb.setSmartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);
        
        //Burn flash to both controllers
        m_leftClimb.burnFlash();
        m_rightClimb.burnFlash();
    }

private void StopClimber() {
    m_leftClimb.set(0);
    m_rightClimb.set(0);
}

private void RunClimber(double leftSpeed, double rightSpeed) {
    m_leftClimb.set(leftSpeed);
    m_rightClimb.set(rightSpeed);
}

public Command RunClimberCommand (double leftSpeed, double rightSpeed){
    return this.run(() -> this.RunClimber(leftSpeed, rightSpeed));
}

public Command StopClimberCommand () {
    return this.runOnce(() -> this.StopClimber());
}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimberCurrentLeft", m_leftClimb.getOutputCurrent());
        SmartDashboard.putNumber("ClimberCurrentRight", m_rightClimb.getOutputCurrent());
    }
}
