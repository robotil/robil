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

	private void runPlanXML(String id, String xml) {
		designer.rosExecutor.runBehaviorTree(id, xml);
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
	
	private void stepPlan() {
		String id = getActiveTab().getID();
		designer.rosExecutor.stepBehaviorTree(id);
	}
	
	private void pausePlan() {
		String id = getActiveTab().getID();
		designer.rosExecutor.pauseBehaviorTree(id);
	}

	private void stopPlan() {
		String id = getActiveTab().getID();
		designer.rosExecutor.stopBehaviorTree(id);
	}
	
	public void actionPerformed(ActionEvent a) {
		if (a.getActionCommand().equals("run_run_plan")) {
			runPlanXML("matan", "<plan><tsk name=\"Noname\" x=\"274.5\" y=\"95.5\" test_time=\"0\" test_result=\"true\" id=\"03ccc501-6dec-4afc-b0c5-d76d3bed6970\" /></plan>");//runPlan();
		} else if (a.getActionCommand().equals("run_resume_plan")) {
			resumePlan();
		} else if (a.getActionCommand().equals("run_stop_plan")) {
			stopPlan();
		} else if (a.getActionCommand().equals("run_pause_plan")) {
			pausePlan();
		} else if (a.getActionCommand().equals("run_step_plan")) {
			stepPlan();
		}

	}
}