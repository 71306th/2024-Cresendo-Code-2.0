package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.InputStates;
import frc.robot.Constants;
import frc.robot.Variables;


public class ClaimFloor extends InstantCommand {

  private SuperStructure superStructure;

  private boolean OneTime = false;
  private double timer;

    public ClaimFloor(SuperStructure superStructure) {
        this.superStructure = superStructure;
        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        superStructure.setState(InputStates.Floor);
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
