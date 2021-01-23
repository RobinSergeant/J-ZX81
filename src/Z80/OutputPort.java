// Copyright 1998-2021 by Robin Sergeant. See license.txt distributed with this file.
// OutPort.java

/* Output port interface which classes wishing to act as CPU output ports must implement */

package Z80;

public interface OutputPort {
    public void writePort(byte low, byte high, byte value);
}

final class DefaultOutputPort implements OutputPort {
    public void writePort(byte low, byte high, byte value) {}
}
