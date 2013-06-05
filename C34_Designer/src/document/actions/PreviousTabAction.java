package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;


public class PreviousTabAction extends AbstractDesignerAction implements ActionListener {

	public PreviousTabAction(BTDesigner designer) {
		super(designer);
	}
	
	@Override
	public void actionPerformed(ActionEvent e) {
		designer.previousTab();
	}

}
