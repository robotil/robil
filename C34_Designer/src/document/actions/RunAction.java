package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;

import javax.swing.JOptionPane;

import terminal.communication.Utils;
import document.BTDesigner;
import document.Document;
import document.Parameters;

public class RunAction extends AbstractDesignerAction implements ActionListener {

	public RunAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent a) {
		if (a.getActionCommand().equals("run_run_plan")) {
			// runPlanXML("matan",
			// "<plan><tsk name=\"Noname\" x=\"274.5\" y=\"95.5\" test_time=\"0\" test_result=\"true\" id=\"03ccc501-6dec-4afc-b0c5-d76d3bed6970\" /></plan>");//runPlan();
			runPlan();
		} else if (a.getActionCommand().equals("run_resume_plan")) {
			resumePlan();
		}else if (a.getActionCommand().equals("run_runresume_plan")) {
			runPlan();
			resumePlan();
		} else if (a.getActionCommand().equals("run_stop_plan")) {
			stopPlan();
		} else if (a.getActionCommand().equals("run_pause_plan")) {
			pausePlan();
		} else if (a.getActionCommand().equals("run_step_plan")) {
			stepPlan();
		}

	}

	private void pausePlan() {
		String id = getActiveTab().getID();
		this.designer.rosExecutor.pauseBehaviorTree(id);
	}

	private void resumePlan() {
		String id = getActiveTab().getID();
		System.out.println("GET ACTIVE TAB ID = " + id);
		this.designer.rosExecutor.resumeBehaviorTree(id);
	}

	private void runPlan() {
		Document doc = getActiveTab().doc;
		String fileName = doc.getShortFilePath();
		if (Parameters.path_to_plans_on_executer.equals("{LOCALPATH}")) {
			fileName = doc.getAbsoluteFilePath();
		} else if (Parameters.path_to_plans_on_executer.equals("{FILENAME}")) {

		} else if (Parameters.path_to_plans_on_executer.contains("{FILENAME}")) {
			fileName = Parameters.path_to_plans_on_executer.replace(
					"{FILENAME}", doc.getShortFilePath());
		} else {
			fileName = Parameters.path_to_plans_on_executer + File.separator
					+ doc.getShortFilePath();
		}

		System.out.println("RUN PLAN FILE : " + fileName);

		if (fileName == null) {
			JOptionPane
					.showMessageDialog(
							null,
							"No plan file selected.\nPlease verify that your last changes are compiled",
							"Run Plan", JOptionPane.INFORMATION_MESSAGE);
			return;
		}

		// designer.rosExecutor.runBehaviorTree(,
		// fileName);

		String id = Utils.randomString(10); // "Matan";
		System.out.println("SET ACTIVE TAB ID = " + id);
		getActiveTab().setID(id);
		this.designer.rosExecutor.runBehaviorTree(id, fileName);
	}

	@SuppressWarnings("unused")
	private void runPlanXML(String id, String xml) {
		System.out.println("RUN PLAN XML : " + xml);

		if (id == null)
			id = Utils.randomString(10); // "Matan";
		System.out.println("SET ACTIVE TAB ID = " + id);
		getActiveTab().setID(id);

		this.designer.rosExecutor.runBehaviorTree(id, xml);
	}

	private void stepPlan() {
		String id = getActiveTab().getID();
		this.designer.rosExecutor.stepBehaviorTree(id);
	}

	private void stopPlan() {
		String id = getActiveTab().getID();
		this.designer.rosExecutor.stopBehaviorTree(id);
	}
}