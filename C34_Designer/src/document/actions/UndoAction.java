package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;


public class UndoAction extends AbstractDesignerAction implements
		ActionListener {

	public UndoAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		designer.getActiveTab().doc.undo();
	}
}
