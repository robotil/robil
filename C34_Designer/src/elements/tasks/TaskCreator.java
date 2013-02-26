package elements.tasks;

import javax.swing.Icon;

import elements.GElement;

public class TaskCreator extends GElement.Creator {
	@Override
	public boolean createOnEmptyPlace() {
		return true;
	}

	@Override
	public Icon getIcon() {
		return null;
	}

	@Override
	public String getToolbarName() {
		return "Task";
	}

	@Override
	public GElement newInstance() {
		return new Task();
	}

	@Override
	public boolean ready() {
		return true;
	}

	@Override
	public String toolTip() {
		return "Create a Node (Seq, Sel, Par, Task). Select a point on the document to place a task";
	}
}
