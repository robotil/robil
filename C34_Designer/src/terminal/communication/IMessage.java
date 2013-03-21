package terminal.communication;

public interface IMessage<T> {
	public void clone(T source);
}
