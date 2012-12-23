package terminal.commands;

import java.util.ArrayList;

import terminal.Terminal;


//------------- PROGRAM --------------------------------
public class Test extends Command{

	public void stop(){}
	
	/**
	 * @param terminal
	 */
	public Test(Terminal terminal) {
		super(terminal, "test");
	}
	public boolean isRelatedTo(String command){
        return isNameOfProgram(command, name);
    }
    public void execute(String command){
        sys.println("TEST PROGRAM");
        return;
    }
    
    public boolean isAutoCompleteAvailable(String command){
        return isPartOfName(command,name);
    }
    public ArrayList<String> autocomplete(String command){
    	ArrayList<String> ret = new ArrayList<String>();
    	ret.add(name);
        return ret;
    }
    
}