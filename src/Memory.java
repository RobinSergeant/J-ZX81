// Copyright 1998-2021 by Robin Sergeant. See license.txt distributed with this file.
// Memory.java

/* this class represents the ZX81 Memory and implements the CPU MemoryInterface */


import Z80.*;

public final class Memory implements MemoryInterface {
    public byte[] data;     // memory array for both RAM and ROM
    private int ram_top;    // last memory address + 1
    private int rom_top;    // last address in the ROM area + 1
    private int ram_size;   // size of RAM in bytes

    Memory(int rom_size, int ram_size) {
        rom_top = rom_size * 1024;
        ram_top = ram_size * 1024 + rom_top;
        this.ram_size = ram_size * 1024;
        data = new byte[ram_top];
    }

    public byte readByte(int pos) {
        pos = pos & 0xFFFF;     // make sure 'pos' is a 16 bit +ve index
        if (pos >= ram_top) {
            pos = rom_top + pos % ram_size;
        }
        return data[pos];
    }

    public short readWord(int pos) {
        return (short)((readByte(pos+1) << 8) | (readByte(pos) & 0xFF));
    }

    public void writeByte(int pos, byte b) {
        pos = pos & 0xFFFF;     // make sure 'pos' is a 16 bit +ve index
        if (pos >= ram_top) {
            pos = rom_top + pos % ram_size;
        }
        if (pos >= rom_top) {
            data[pos] = b;
        }
    }

    public void writeWord(int pos, short w) {
        writeByte(pos, (byte)w);
        writeByte(pos+1, (byte)(w >>> 8));
    }

    public void clear() {
        for (int c=rom_top; c<ram_top; c++) {
            data[c] = 0;
        }
    }
}
