package frc.robot.commands.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeeds;

public class ShamperShootCommand extends Command{
    private final ShamperSubsystem shamper;
    private final ShamperSpeeds speeds;
    private final IndexerSubsystem indexer;

    public ShamperShootCommand(ShamperSubsystem shamper, IndexerSubsystem indexer, ShamperSpeeds speeds) {
        this.shamper = shamper;
        this.speeds = speeds;
        this.indexer = indexer;

        addRequirements(shamper, indexer);
    }

    @Override
    public void initialize() {
        indexer.runMotors();
        shamper.setSpeed(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopMotors();
        shamper.setSpeed(ShamperSpeeds.SPEAKER_IDLE);
    }
}
