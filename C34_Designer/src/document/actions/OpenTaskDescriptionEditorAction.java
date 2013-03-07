package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.TaskDescriptionEditor;
import windows.designer.BTDesigner;


public class OpenTaskDescriptionEditorAction extends AbstractDesignerAction
		implements ActionListener {

	public OpenTaskDescriptionEditorAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		if (this.designer == null || this.designer.getActiveTab() == null
				|| this.designer.getActiveTab().doc == null
				|| this.designer.getActiveTab().doc.task_description == null)
			return;

		TaskDescriptionEditor editor = new TaskDescriptionEditor(
				this.designer.getActiveTab().doc.task_description);
		editor.setVisible(true);
	}
}
