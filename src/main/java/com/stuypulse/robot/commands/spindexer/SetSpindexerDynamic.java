import com.stuypulse.robot.subsystems.spindexer.Spindexer;

public class SetSpindexerDynamic extends SetSpindexerState {
    public SetSpindexerDynamic() {
        super(Spindexer.SpindexerState.DYNAMIC);
    }
}