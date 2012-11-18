package document.listeners;

import java.awt.FileDialog;
import java.awt.Frame;
import java.io.File;
import java.io.FilenameFilter;

import org.omg.Dynamic.Parameter;

import document.Parameters;

public abstract class Dialogs {

	public static String openFile(String title, final String filter, String path) {
		FileDialog fileDialog = new FileDialog(new Frame(), title, FileDialog.LOAD);
		fileDialog.setDirectory(path);
		
		fileDialog.setFilenameFilter(new FilenameFilter() {
			@Override
			public boolean accept(File dir, String name) {
				return name.endsWith("." + filter);
			}
		});
		
		fileDialog.setVisible(true);
		if (fileDialog.getFile() == null) {
			return null;
		}
		return fileDialog.getDirectory()+"/"+fileDialog.getFile();
	}
	
	public static String saveFile(String title, final String filter) {
		return saveFile(title, filter, null, ".");
	};
	
	public static String saveFile(String title, final String filter, String defaultFileName, String path) {
		FileDialog fileDialog = new FileDialog(new Frame(), title, FileDialog.SAVE);
		fileDialog.setDirectory(path);
		
		fileDialog.setFilenameFilter(new FilenameFilter() {
			@Override
			public boolean accept(File dir, String name) {
				return name.endsWith("." + filter);
			}
		});
		
		if (defaultFileName != null) {
			fileDialog.setFile(defaultFileName);
		}
		
		fileDialog.setVisible(true);
		
		return fileDialog.getFile();			
	}
}
