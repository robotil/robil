package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;


/**
 * Creates a new designer tab
 * 
 * @author matan
 * 
 */
public class NewWindowAction extends AbstractDesignerAction implements
		ActionListener {

	public NewWindowAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent arg0) {
		this.designer.addNewDocumentTab();
	}

}
