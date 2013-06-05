package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import windows.ParametersEditor;

import document.ParametersXmlHandler;

public class PropertiesAction implements ActionListener {

	public PropertiesAction() {
	}

	@Override
	public void actionPerformed(ActionEvent a) {
		ParametersEditor.show(ParametersXmlHandler.lastPath);
	}
}