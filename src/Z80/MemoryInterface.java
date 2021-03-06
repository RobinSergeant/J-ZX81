// Copyright 1998-2021 by Robin Sergeant. See license.txt distributed with this file.

/* Memory interface which all Z80 memory classes must implement s*/

package Z80;

public interface MemoryInterface {
    public byte readByte(int pos);
    public short readWord(int pos);
    public void writeByte(int pos, byte b);
    public void writeWord(int pos, short w);
    public void clear();
}
