package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.designer.BTDesigner;


public class ExportTasksAction extends AbstractDesignerAction implements
		ActionListener {

	public ExportTasksAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent arg0) {

		if (this.designer.getActiveTab() != null
				&& this.designer.getActiveTab().doc != null) {
			String fileName = Dialogs.saveFile("Export tasks", "");

			if (!fileName.equals(""))
				this.designer.getActiveTab().doc.exportTasks(fileName);
		}
	}

}
