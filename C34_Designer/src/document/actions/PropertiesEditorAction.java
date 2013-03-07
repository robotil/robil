package document.actions;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JTable;
import javax.swing.table.AbstractTableModel;

import windows.PropertiesEditor;

import document.PropertiesXmlHandler;

public class PropertiesEditorAction implements ActionListener {

	private JTable table;
	private JFrame frame;

	public PropertiesEditorAction(JTable table, JFrame frame) {
		this.table = table;
		this.frame = frame;
	}

	@Override
	public void actionPerformed(ActionEvent a) {

		//------------------ OPEN -----------------------
		if (a.getActionCommand().equals("open_properties_dialog")) {
			PropertiesEditor.show(PropertiesXmlHandler.lastPath);
			
		} else 
		//------------------ LOAD -----------------------	
		if (a.getActionCommand().equals("load_properties")) {
			try {
//				String path = Dialogs.openFile("Load Properties File", "xml", ".");
//
//				if (path != null) {
//					PropertiesXmlHandler.lastPath = path;
//				}
				PropertiesXmlHandler
						.loadAndSetProperties(PropertiesXmlHandler.lastPath);

				updateTable();

			} catch (Exception ex) {
				JOptionPane.showMessageDialog(null, "Error loading file",
						"Load properties", JOptionPane.ERROR_MESSAGE);
			}
		} else 
		//------------------ SAVE -----------------------------	
		if (a.getActionCommand().equals("save_properties")) {
			try {
//				String path = Dialogs.saveFile("Save Properties", "xml");
//
//				if (path == null || path.equals("")) {
//					return;
//				}

				// updateTable();

				PropertiesXmlHandler.setParametersFromTable(this.table);
				PropertiesXmlHandler.saveToFile(PropertiesXmlHandler.lastPath);

				JOptionPane.showMessageDialog(null, "Properties file saved");
				this.frame.dispose();
				
			} catch (Exception ex) {
				JOptionPane.showMessageDialog(null, "Error saving file", "Save properties", JOptionPane.ERROR_MESSAGE);
			}

		} else if (a.getActionCommand().equals("cancel_properties")) {
			this.frame.dispose();
		}
	}

	private void updateTable() {
		// table.getModel()
		((AbstractTableModel) this.table.getModel()).fireTableDataChanged();
	}
}