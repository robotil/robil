package terminal.commands;

import java.util.ArrayList;

import terminal.Terminal;

//------------- PROGRAM --------------------------------
public class History extends Command{

	@Override
	public void stop() {}

	/**
	 * @param terminal
	 */
	public History(Terminal terminal) {
		super(terminal, "history");
	}
	
    public void execute(String command){
        int i=sys.history.size();
        for(String h: sys.history){
            sys.print(""+i+") "+h+"\n");
            i--;
        }
        return;
    }
  
    public boolean isAutoCompleteAvailable(String command) {
		return isPartOfName(command, name);
	}
    
    public ArrayList<String> autocomplete(String command){
    	ArrayList<String> ret = new ArrayList<String>();
    	ret.add(name);
        return ret;
    }
    
}
