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

        // Master Controller
        m_ArmMotor = new CANSparkMax(3, MotorType.kBrushless);
        //m_ArmMotor.restoreFactoryDefaults();
        m_ArmMotor.setInverted(true);
        m_ArmMotor.setIdleMode(ArmConstants.kArmMotorIdleMode);
        m_ArmMotor.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);
        m_ArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_ArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        m_ArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 135);
        m_ArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 35);

        //Master Controller PID
        m_ArmEncoder = m_ArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_ArmEncoder.setInverted(false);
        m_ArmEncoder.setPositionConversionFactor(360.0);
        m_ArmPID = m_ArmMotor.getPIDController();
        m_ArmPID.setFeedbackDevice(m_ArmEncoder);

        //Follower Controller
        m_ArmFollower = new CANSparkMax(2, MotorType.kBrushless);
        //m_ArmFollower.restoreFactoryDefaults();
        m_ArmFollower.setIdleMode(ArmConstants.kArmMotorIdleMode);
        m_ArmFollower.setSmartCurrentLimit(ArmConstants.kArmMotorCurrentLimit);
        m_ArmFollower.follow(m_ArmMotor, true);

        //Burn flash to both controllers
        //m_ArmMotor.burnFlash();
        //m_ArmFollower.burnFlash();
    }

    private void setPosition(double targetPosition){
        m_ArmPID.setP(0.02);
        m_ArmPID.setI(0);
        m_ArmPID.setD(0);
        m_ArmPID.setReference(targetPosition, ControlType.kPosition);
    }

    public double getAngle(){
        return m_ArmEncoder.getPosition();
    }

/*
    private void armUP(){
        double currentPosition = m_ArmEncoder.getPosition();
        double newPosition = currentPosition + 1;
        setPosition(newPosition);
    }

    private void armDown(){
        double currentPosition = m_ArmEncoder.getPosition();
        double newPosition = currentPosition - 1;
        setPosition(newPosition);

    }

*/

    private void resetArm(){
        m_ArmMotor.set(0); //Stop arm motors
    }

    public Command SetPositionCommand(double targetPosition){
        return this.startEnd(() -> this.setPosition(targetPosition), () ->{});
    }

    public Command RestArmCommand(){
        return this.runOnce(() -> this.resetArm());
    }

    /*
    public Command armUPcommand(){
        return this.startEnd(() -> this.armUP(), () ->{});
    }

    public Command armDOWNcommand(){
        return this.startEnd(() -> this.armDown(), () ->{});
    }
    */

    @Override
    public void periodic() {
        double rotation = m_ArmEncoder.getPosition();
        SmartDashboard.putNumber("Arm Position", rotation);
    }
}
