
// TODO Levi was here
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;

public class ClimbAxisCommand extends Command {
    private Climber climber;
    private DoubleSupplier leftAxis, rightAxis;
    public ClimbAxisCommand(Climber climber, DoubleSupplier leftAxis, DoubleSupplier rightAxis) {
        this.climber = climber;
        this.leftAxis = leftAxis;
        this.rightAxis = rightAxis;
        addRequirements(climber);
    }
    @Override
    public void execute() {
        // get axis value
        double leftValue = applyDeadband(leftAxis.getAsDouble(), OIConstants.kClimbDeadband);
        double rightValue = applyDeadband(rightAxis.getAsDouble(), OIConstants.kClimbDeadband);

        // control logic for left side
        if (leftValue > 0) {
            climber.SetLeft(ClimberConstants.kClimberExtendSpeed);
        }
        else if (leftValue < 0) {
            climber.SetLeft(ClimberConstants.kClimberRetractSpeed);
        }
        else {
            climber.SetLeft(0);
        }

        // control logic for right side
        if (rightValue > 0) {
            climber.SetRight(ClimberConstants.kClimberExtendSpeed);
        }
        else if (rightValue < 0) {
            climber.SetRight(ClimberConstants.kClimberRetractSpeed);
        }
        else {
            climber.SetRight(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // make sure motors stop when command ends
        climber.SetLeft(0);
        climber.SetRight(0);
    }

    private double applyDeadband(double value, double deadband) {
        // apply deadband
        return (Math.abs(value) > deadband) ? value : 0.0;
    }
}
