package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;

public class CloseTabAction extends AbstractDesignerAction implements ActionListener  {

	public CloseTabAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		designer.closeCurrentTab();
	}
}
