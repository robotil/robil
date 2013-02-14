package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;

public class RedoAction extends AbstractDesignerAction implements
		ActionListener {

	public RedoAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		designer.getActiveTab().doc.redo();
	}
}
