package document.actions;

import java.awt.Component;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;

import javax.swing.JFileChooser;
import javax.swing.filechooser.FileFilter;

import windows.designer.BTDesigner;

import document.Parameters;

public class OpenFileAction extends AbstractDesignerAction implements ActionListener {

	class ProjectFilter extends FileFilter {

		private String[] extension = {"xml"};

		@Override
		public boolean accept(File pathname) {

			if (pathname.isDirectory()) {

				return true;

			}

			String name = pathname.getName().toLowerCase();

			for (String anExt : extension) {

				if (name.endsWith(anExt)) {

					return true;

				}

			}

			return false;

		}

		@Override
		public String getDescription() {
			// TODO Auto-generated method stub
			return null;
		}

	}

	public OpenFileAction(BTDesigner designer) {
		super(designer);
	}

	@Override
	public void actionPerformed(ActionEvent a) {

		//		if (this.designer.getNumberOfDocuments() == 0) {
		//			JOptionPane.showMessageDialog(null, "Please open a new window",
		//					"Open Plan", JOptionPane.INFORMATION_MESSAGE);
		//			return;
		//		}

		//		String fileName = Dialogs.openFile("Open XML", "xml",
		//				Parameters.path_to_plans);
		//		if (fileName == null) {
		//			return;
		//		}

		final JFileChooser fc = new JFileChooser();
		fc.setCurrentDirectory(new File(Parameters.path_to_plans));
		fc.setFileFilter(new ProjectFilter());
		fc.setMultiSelectionEnabled(false);
		int returnVal = fc.showOpenDialog(this.designer);
		// new NewWindowAction(designer).actionPerformed(a);


		//		Document document = super.getActiveTab().doc;
		//		document.loadPlan(fileName);
		//		String shortName = document.getShortFilePath();
		//		this.designer.setTabName(this.designer.tabbedPane.getSelectedIndex(), shortName);

		//designer.addnewDocumentTab(fileName);
		File file = fc.getSelectedFile();
		designer.addnewDocumentTab(file.getPath());
	}
}