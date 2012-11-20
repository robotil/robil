package document.listeners;

import document.BTDesigner;
import document.ChooseRemotePlanDialog;
import document.Document;
import document.Parameters;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;

import javax.swing.JFileChooser;
import javax.swing.filechooser.FileNameExtensionFilter;

import terminal.commands.ServiceCaller;
import terminal.communication.RosPipe;
import terminal.communication.RosPipe.RosTargets;
import terminal.lineprocessors.BatchLineProcessor;

public class LoadAndOpenAction extends AbstractDesignerAction implements ActionListener {

	public LoadAndOpenAction(BTDesigner designer) {
		super(designer);
	}

	public void actionPerformed(ActionEvent a) {

		// get plans from executer
		ServiceCaller caller = new ServiceCaller();
		ArrayList<String> filesOnHost = caller.callService("/executer/ls");

		// filter results
		ArrayList<String> plansList = new ArrayList<String>();

		for (Iterator<String> itr = filesOnHost.iterator(); itr.hasNext();) {
			String currentFile = itr.next().trim();
			boolean flagValid = true;

			if (currentFile.startsWith("content")
					|| currentFile.contains("just_names")) {
				flagValid = false;
			}

			if (currentFile.length() < 3) {
				flagValid = false;
			}

			if (!flagValid) {
				continue;
			}

			currentFile = new String(currentFile.substring(1,
					currentFile.length() - 1));
			
			if (!currentFile.endsWith("xml")) {
				continue;
			}
			plansList.add(currentFile);
		}

		if (plansList.isEmpty()) {
			return;
		}

		// let the user choose a plan
		Collections.sort(plansList);
		String[] plans = plansList.toArray(new String[0]);

		String selectedPlan = ChooseRemotePlanDialog.showDialog(null, null,
				"Available plans on remote Executer:", "Load Plan", plans,
				null, null);

		if (selectedPlan == null) {
			return;
		}

		if (!validatePlan(selectedPlan)) {

		}

//		Document document = super.getActiveDocument();
//		document.loadPlan(selectedPlan);

	}

	private boolean validatePlan(String plan) {
		return true;
	}
}