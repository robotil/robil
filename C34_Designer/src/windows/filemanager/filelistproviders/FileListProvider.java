package windows.filemanager.filelistproviders;

public interface FileListProvider {
	FileDescription[] getFiles(String path);
}
