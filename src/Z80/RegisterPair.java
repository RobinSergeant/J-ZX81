// Copyright 1998-2021 by Robin Sergeant. See license.txt distributed with this file.

package Z80;

public class RegisterPair implements WordRegister {
    private Register low;
    private Register high;
    CPU cpu;

    RegisterPair(CPU cpu, Register high, Register low) {
        this.cpu = cpu;
        this.high = high;
        this.low = low;
    }

    public void set(short w) {
        low.val = (byte)w;
        high.val = (byte)(w >>> 8);
    }

    public short get() {
        return (short)((high.val << 8) | (low.val & 0xFF));
    }

    public byte readByte() {
        return cpu.ram.readByte(this.get());
    }

    public void writeByte(byte b) {
        cpu.ram.writeByte(this.get(), b);
    }

    public void swap() {
        low.swap();
        high.swap();
    }

    public void clear() {
        this.set((short)0);
        this.swap();
        this.set((short)0);
    }
}
