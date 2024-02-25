package frc.robot.commands.autos;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.InputStates;
import frc.robot.Constants;
import frc.robot.Variables;


public class ShootBase extends InstantCommand {

  private SuperStructure superStructure = new SuperStructure();

  private boolean OneTime = false;
  private double timer;

    public ShootBase(SuperStructure superStructure) {
        this.superStructure = superStructure;
        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        superStructure.setState(InputStates.Base);
        if (Variables.OperatorControl.isInPlace) {
            if (OneTime = false) {
                OneTime = true;
                timer = Timer.getFPGATimestamp();
            }
            superStructure.setIntakeClaiming(Constants.SuperStructure.intakeClaimSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        superStructure.setState(InputStates.Emergency);
    }


    @Override
    public boolean isFinished() {
        if(Timer.getFPGATimestamp() >= timer + 1) {
        return true;
        } else return false;
    }
}
