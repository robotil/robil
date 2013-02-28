package elements.tasks;

import java.util.ArrayList;

public class TaskResultCollection extends ArrayList<TaskResult> {	
	
	public interface EventListener {
		public void onAdd(EventListenerArgs e);
	}
	
	public class EventListenerArgs {
		public TaskResult taskResult;
		public EventListenerArgs(TaskResult taskResult) {
			this.taskResult = taskResult;
		}
	}
	
	private static final long serialVersionUID = -3135641596797553701L;
	private ArrayList<EventListener> _eventListeners = new ArrayList<TaskResultCollection.EventListener>();
	
	private void onAdd(EventListenerArgs e) {
		for (EventListener listener : _eventListeners)
			listener.onAdd(e);
	}
	
	@Override
	public boolean add(TaskResult e) {
		
		boolean added = super.add(e);
		
		if (added)
			onAdd(new EventListenerArgs(e));
		
		return added;
	}
	
	public void addEventListener(EventListener eventListener) {
		_eventListeners.add(eventListener);
	}
}
