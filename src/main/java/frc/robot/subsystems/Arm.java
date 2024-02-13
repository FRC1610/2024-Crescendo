package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {
    private final CANSparkMax m_ArmMotor;
    private final CANSparkMax m_ArmFollower;
    private final AbsoluteEncoder m_ArmEncoder;
    private final SparkPIDController m_ArmPID;

    public Arm() {
        // Spark Max Sextup
        m_ArmMotor = new CANSparkMax(2, MotorType.kBrushless);
        m_ArmMotor.restoreFactoryDefaults();
        m_ArmMotor.setIdleMode(ArmConstants.kArmMotorIdleMode);
        m_ArmMotor.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);
        
        m_ArmEncoder = m_ArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
        
        m_ArmPID = m_ArmMotor.getPIDController();

        m_ArmFollower = new CANSparkMax(3, MotorType.kBrushless);
        m_ArmFollower.restoreFactoryDefaults();
        m_ArmFollower.setIdleMode(ArmConstants.kArmMotorIdleMode);
        m_ArmFollower.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);
        m_ArmFollower.follow(m_ArmMotor, true);

        m_ArmMotor.burnFlash();
        m_ArmFollower.burnFlash();

    }

    private void setPosition(double targetPosition){
        m_ArmPID.setP(1);
        m_ArmPID.setI(0.1);
        m_ArmPID.setD(0);
        m_ArmPID.setReference(targetPosition, ControlType.kPosition);
    }

    public Command SetPositionCommand(double targetPosition){
        return this.startEnd(() -> this.setPosition(targetPosition), () ->{});
    }
}
