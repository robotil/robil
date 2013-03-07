package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;


public class CompileAction extends AbstractDesignerAction implements
		ActionListener {

	public CompileAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent a) {
		getActiveTab().doc.compile();
	}
}