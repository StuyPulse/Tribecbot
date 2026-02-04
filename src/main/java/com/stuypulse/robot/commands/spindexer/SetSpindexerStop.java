import com.stuypulse.robot.subsystems.spindexer.Spindexer;

public class SetSpindexerStop extends SetSpindexerState {
    public SetSpindexerStop() {
        super(Spindexer.SpindexerState.STOP);
    }
}