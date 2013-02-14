package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import document.PropertiesEditor;
import document.PropertiesXmlHandler;

public class PropertiesAction implements ActionListener {

	public PropertiesAction() {
	}

	@Override
	public void actionPerformed(ActionEvent a) {
		PropertiesEditor.show(PropertiesXmlHandler.lastPath);
	}
}