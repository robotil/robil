package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.LogConsoleWindow;

public class LogConsoleAction implements ActionListener {

	public LogConsoleAction() {
		// TODO Auto-generated constructor stub
	}

	@Override
	public void actionPerformed(ActionEvent arg0) {
		LogConsoleWindow.show("BTDesigner.log");
	}

}
