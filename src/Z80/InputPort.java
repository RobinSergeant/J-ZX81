// Copyright 1998-2021 by Robin Sergeant. See license.txt distributed with this file.
// InputPort.java

/* Input port interface which classes wishing to act as CPU input ports must implement */

package Z80;

public interface InputPort {
    public byte readPort(byte low, byte high);
}

final class DefaultInputPort implements InputPort {
    public byte readPort(byte low, byte high) { return 0; }
}
