package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;


public class NextTabAction extends AbstractDesignerAction implements ActionListener {

	public NextTabAction(BTDesigner designer) {
		super(designer);
	}
	
	@Override
	public void actionPerformed(ActionEvent e) {
		designer.nextTab();
	}

}
