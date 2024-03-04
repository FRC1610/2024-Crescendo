package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final CANSparkMax m_ClimbMotor;
    private final CANSparkMax m_ClimbFollower;

    public Climber() {
        // Spark Max Sextup

        // Master Controller
        m_ClimbMotor = new CANSparkMax(18, MotorType.kBrushless);
        m_ClimbMotor.restoreFactoryDefaults();
        m_ClimbMotor.setInverted(true);
        m_ClimbMotor.setIdleMode(ClimberConstants.kClimberIdleMode);
        m_ClimbMotor.setSmartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);

        //Follower Controller
        m_ClimbFollower = new CANSparkMax(19, MotorType.kBrushless);
        m_ClimbFollower.restoreFactoryDefaults();
        m_ClimbFollower.setIdleMode(ClimberConstants.kClimberIdleMode);
        m_ClimbFollower.setSmartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);
        m_ClimbFollower.follow(m_ClimbMotor, true);

        //Burn flash to both controllers
        m_ClimbMotor.burnFlash();
        m_ClimbFollower.burnFlash();
    }

private void StopClimber() {
    m_ClimbMotor.set(0);
}

private void RunClimber(double ClimbSpeed) {
    m_ClimbMotor.set(ClimbSpeed);
}

public Command RunClimberCommand (double ClimbSpeed){
    return this.run(() -> this.RunClimber(ClimbSpeed));
}

public Command StopClimberCommand () {
    return this.runOnce(() -> this.StopClimber());
}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimberCurrent", m_ClimbMotor.getOutputCurrent());
    }
}
