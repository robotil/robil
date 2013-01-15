package elements;

import java.awt.event.ItemEvent;
import java.util.List;

import javax.swing.DefaultComboBoxModel;
import javax.swing.JComboBox;
import javax.swing.plaf.basic.BasicComboBoxEditor;

public class Java2sAutoComboBox extends JComboBox {

	private static final long serialVersionUID = 8375007428420854617L;

	private class AutoTextFieldEditor extends BasicComboBoxEditor {

		private Java2sAutoTextField getAutoTextFieldEditor() {
			return (Java2sAutoTextField) editor;
		}

		AutoTextFieldEditor(List<String> list) {
			editor = new Java2sAutoTextField(list, Java2sAutoComboBox.this);
		}
	}

	public Java2sAutoComboBox(List<String> list) {
		isFired = false;
		autoTextFieldEditor = new AutoTextFieldEditor(list);
		setEditable(true);
		setModel(new DefaultComboBoxModel(list.toArray()) {

			/**
		 * 
		 */
			private static final long serialVersionUID = -3234807106807262423L;

			protected void fireContentsChanged(Object obj, int i, int j) {
				if (!isFired)
					super.fireContentsChanged(obj, i, j);
			}

		});
		setEditor(autoTextFieldEditor);
	}

	public boolean isCaseSensitive() {
		return autoTextFieldEditor.getAutoTextFieldEditor().isCaseSensitive();
	}

	public void setCaseSensitive(boolean flag) {
		autoTextFieldEditor.getAutoTextFieldEditor().setCaseSensitive(flag);
	}

	public boolean isStrict() {
		return autoTextFieldEditor.getAutoTextFieldEditor().isStrict();
	}

	public void setStrict(boolean flag) {
		autoTextFieldEditor.getAutoTextFieldEditor().setStrict(flag);
	}

	public List<String> getDataList() {
		return autoTextFieldEditor.getAutoTextFieldEditor().getDataList();
	}

	public void setDataList(List<String> list) {
		autoTextFieldEditor.getAutoTextFieldEditor().setDataList(list);
		setModel(new DefaultComboBoxModel(list.toArray()));
	}

	void setSelectedValue(Object obj) {
		if (isFired) {
			return;
		} else {
			isFired = true;
			setSelectedItem(obj);
			fireItemStateChanged(new ItemEvent(this, 701, selectedItemReminder,
					1));
			isFired = false;
			return;
		}
	}

	protected void fireActionEvent() {
		if (!isFired)
			super.fireActionEvent();
	}

	private AutoTextFieldEditor autoTextFieldEditor;

	private boolean isFired;

}
