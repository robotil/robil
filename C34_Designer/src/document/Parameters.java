package document;

import java.lang.reflect.Field;

import logger.Log;

public class Parameters {
	public static int test_time = 0;
	public static boolean test_result = true;
	public static String path_to_plans = "./plans";
	public static String path_to_images = "./images";
	public static String executer_service = "/executer";
	public static String path_to_lookup = "./plans/lookup.xml";
	public static String path_to_undo = "./.undo";
	public static String path_to_address = "./plans/TaskList.xml";
	public static String path_to_description = "./plans/TaskDescriptions.xml";
	public static String path_to_plans_on_executer = "{LOCALPATH}";
	public static boolean enableLinkConnection = true;
	public static boolean enableTaskIdRegeneration = false;
	public static boolean log_print_ros_commands = false;
	public static boolean log_print_ros_output = false;
	public static int log_ros_progress_print_level = 0;
	public static int log_preview_lines_limit=500;
	public static boolean log_print_running_tasks_id=false;
	public static String log_disabled_tags = "";
	
	public static boolean set(String key, String value) {
		Class<Parameters> pclass = Parameters.class;
		
		try{
			Field pfield = pclass.getField(key);
			if(pfield.getType().getName().equals(String.class.getName())){
				pfield.set(null, new String(value));
			}else
			if(pfield.getType().getName().equals(int.class.getName())){
				pfield.set(null, Integer.parseInt(value));
			}else
			if(pfield.getType().getName().equals(float.class.getName())){
				pfield.set(null, Float.parseFloat(value));
			}else
			if(pfield.getType().getName().equals(double.class.getName())){
				pfield.set(null, Double.parseDouble(value));
			}else
			if(pfield.getType().getName().equals(boolean.class.getName())){
				pfield.set(null, Boolean.parseBoolean(value));
			}else{
				Log.d("Parameters.set"+key+","+value+")  Unknown type of parameter : "+pfield.getType().getName());
				return false;
			}
			return true;
		}catch(NoSuchFieldException ex){
			Log.e("Parameters.set"+key+","+value+")  NoSuchFieldException : "+ex.getMessage());
		} catch (IllegalArgumentException ex) {
			Log.e("Parameters.set"+key+","+value+")  IllegalArgumentException : "+ex.getMessage());
		} catch (IllegalAccessException ex) {
			Log.e("Parameters.set"+key+","+value+")  IllegalAccessException : "+ex.getMessage());
		}
		return false;
	}
	
	public static void onLoaded() {
		onChanged();
		
	}
	
	public static void onChanged() {
		Log.disableTags(Parameters.log_disabled_tags);
	}
}
