package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JOptionPane;

import terminal.communication.Utils;

import document.BTDesigner;
import document.Document;

public class RunAction extends AbstractDesignerAction implements ActionListener {

	public RunAction(BTDesigner designer) {
		super(designer);
	}

	private void runPlan() {
		Document doc = getActiveTab().doc;
		String fileName = doc.getShortFilePath();

		if (fileName == null) {
			JOptionPane
					.showMessageDialog(
							null,
							"No plan file selected.\nPlease verify that your last changes are saved",
							"Run Plan", JOptionPane.INFORMATION_MESSAGE);
			return;
		}

		// designer.rosExecutor.runBehaviorTree(,
		// fileName);
		
		String id = Utils.randomString(10); //"Matan";
		getActiveTab().setID(id);
		designer.rosExecutor.runBehaviorTree(id, fileName);
	}

	private void resumePlan() {
		String id = getActiveTab().getID();
		designer.rosExecutor.resumeBehaviorTree(id);
	}

	private void stopPlan() {
		String id = getActiveTab().getID();
		designer.rosExecutor.stopBehaviorTree(id);
	}
	
	public void actionPerformed(ActionEvent a) {
		if (a.getActionCommand().equals("run_run_plan")) {
			runPlan();
		} else if (a.getActionCommand().equals("run_resume_plan")) {
			resumePlan();
		} else if (a.getActionCommand().equals("run_stop_plan")) {
			stopPlan();
		} else if (a.getActionCommand().equals("run_pause_plan")) {
			
		}

	}
}