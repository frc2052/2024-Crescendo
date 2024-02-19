package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.climb.ClimberRetractCommand;
import frc.robot.commands.climb.ClimberExtendCommand;
import frc.robot.commands.shamper.ShamperAngleCommand;
import frc.robot.commands.shamper.ShamperIdleCommand;
import frc.robot.commands.shamper.ShamperShootCommand;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeed;
import frc.robot.util.AimingCalculator;

public class Superstructure extends SubsystemBase {
  private SuperstructureState state;
  
  private ShamperSubsystem shamper;

  private ClimberSubsystem climber;

  private IndexerSubsystem indexer;

  public  Superstructure(ShamperSubsystem shamper, ClimberSubsystem climber, IndexerSubsystem indexer) {
    this.shamper = shamper;
    this.climber = climber;
    this.indexer = indexer;
  }

  public void setState(SuperstructureState state) {
    this.state = state;
    switch(state){
      case DEFAULT:
        setDefault();
      case PODIUM_IDLE:
        setPodiumIdle();
      case PODIUM_SCORE:
        setPodiumScoring();
      case SPEAKER_IDLE:
        setSpeakerIdle();
      case SPEAKER_SCORE:
        setSpeakerScoring();
      case AMP_IDLE:
        setAmpIdle();
      case AMP_SCORE:
        setAmpScore();
      case CLIMBING:
        setClimbing();
    }
  }

  public SuperstructureState getState() {
    return state;
  }

  private void setDefault() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT);
    new ShamperShootCommand(shamper, ShamperSpeed.OFF);
    new ClimberRetractCommand(climber);
  }

  private void setPodiumIdle() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT);
    new ShamperIdleCommand(shamper, ShamperSpeed.SPEAKER_IDLE);
    new ClimberRetractCommand(climber);
  }

  private void setPodiumScoring() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT);
    new ShamperShootCommand(shamper, ShamperSpeed.SPEAKER_SCORE);
    new ClimberRetractCommand(climber);
  }

  private void setSpeakerIdle() {
    new ShamperAngleCommand(shamper, AimingCalculator.calculate().getShamperAngle());
    new ShamperIdleCommand(shamper, ShamperSpeed.SPEAKER_IDLE);
    new ClimberRetractCommand(climber);
  }
  
  private void setSpeakerScoring() {
    new ShamperAngleCommand(shamper, AimingCalculator.calculate().getShamperAngle());
    new ShamperShootCommand(shamper, ShamperSpeed.SPEAKER_SCORE, ShamperSpeed.SPEAKER_IDLE);
    new ClimberRetractCommand(climber);
  }
  
  private void setAmpIdle() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.AMP);
    new ShamperIdleCommand(shamper, ShamperSpeed.AMP_IDLE);
    new ClimberRetractCommand(climber);
  }  

  private void setAmpScore() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.AMP);
    new ShamperShootCommand(shamper, ShamperSpeed.AMP_SCORE, ShamperSpeed.SPEAKER_IDLE);
    new ClimberRetractCommand(climber);
  }

  private void setClimbing() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.CLIMB);
    new ShamperShootCommand(shamper, ShamperSpeed.OFF);
    new ClimberExtendCommand(climber);
  }

  @Override
  public void periodic() {
    if(state.needsRefresh()){
      setState(state);
    }
  }

  public enum SuperstructureState {
    DEFAULT(false),
    PODIUM_IDLE(false),
    PODIUM_SCORE(false),
    SPEAKER_IDLE(true),
    SPEAKER_SCORE(true),
    AMP_IDLE(false),
    AMP_SCORE(false),
    CLIMBING(false);

    private boolean refreshRequired;

    private SuperstructureState(boolean refreshRequired) {
      this.refreshRequired = refreshRequired;
    }

    public boolean needsRefresh() {
      return refreshRequired;
    }
  }
}
