package windows.filemanager.filelistproviders;

import java.io.File;

public class LocalFileListProvider implements FileListProvider {

	@Override
	public FileDescription[] getFiles(String path) {
		File directory = new File(path);
		File[] files = directory.listFiles();
		FileDescription[] fileDescriptions = new FileDescription[files.length];
		
		for (int i = 0; i < files.length; i++) {
			File file = files[i];
			
			FileDescription fileDescription = new FileDescription();
			fileDescription.setFilename(file.getName());
			fileDescription.setDirectory(file.isDirectory());
			fileDescriptions[i] = fileDescription;
		}
		
		return fileDescriptions;
	}

}
