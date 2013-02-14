package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.BTDesigner;
import document.Parameters;

public class OpenFileAction extends AbstractDesignerAction implements
		ActionListener {

	public OpenFileAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent a) {

//		if (this.designer.getNumberOfDocuments() == 0) {
//			JOptionPane.showMessageDialog(null, "Please open a new window",
//					"Open Plan", JOptionPane.INFORMATION_MESSAGE);
//			return;
//		}

		String fileName = Dialogs.openFile("Open XML", "xml",
				Parameters.path_to_plans);
		if (fileName == null) {
			return;
		}

		// new NewWindowAction(designer).actionPerformed(a);
		
		
//		Document document = super.getActiveTab().doc;
//		document.loadPlan(fileName);
//		String shortName = document.getShortFilePath();
//		this.designer.setTabName(this.designer.tabbedPane.getSelectedIndex(), shortName);
		
		designer.addnewDocumentTab(fileName);
	}
}