package document;

import java.util.ArrayList;

public class PlanExecutionCollection extends
		ArrayList<PlanExecution> {

	private static final long serialVersionUID = -8543571847084424873L;

	public interface EventListener {
		public void onAdd(EventListenerArgs e);
	}
	
	public class EventListenerArgs {
		public PlanExecution planResult;
		public EventListenerArgs(PlanExecution planResult) {
			this.planResult = planResult;
		}
	}
	
	private ArrayList<EventListener> _eventListeners = new ArrayList<EventListener>();
	
	private void onAdd(EventListenerArgs e) {
		for (EventListener listener : _eventListeners)
			listener.onAdd(e);
	}
	
	@Override
	public boolean add(PlanExecution planResult) {
		
		boolean added = super.add(planResult);
		
		if (added)
			onAdd(new EventListenerArgs(planResult));
		
		return added;
	}
	
	public void addEventListener(EventListener eventListener) {
		_eventListeners.add(eventListener);
	}
}
