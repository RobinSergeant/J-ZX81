// Copyright 1998-2021 by Robin Sergeant. See license.txt distributed with this file.
// Keyboard.java

/* this class represents the ZX81 keybaord, acting as a keyboard event handler and CPU input port */

import java.awt.event.*;
import java.util.*;
import Z80.*;

public final class Keyboard extends KeyAdapter implements InputPort {
    private Memory ram;
    private byte[] line = new byte[8];
    private Hashtable keys = new Hashtable();
    public short current_key;

    public Keyboard(Memory ram) {
        keys.put(new Integer(KeyEvent.VK_BACK_SLASH), new Key(0, 126));
        keys.put(new Integer(220), new Key(0, 126));
        keys.put(new Integer(KeyEvent.VK_Z), new Key(0, 125));
        keys.put(new Integer(KeyEvent.VK_X), new Key(0, 123));
        keys.put(new Integer(KeyEvent.VK_C), new Key(0, 119));
        keys.put(new Integer(KeyEvent.VK_V), new Key(0, 111));
        keys.put(new Integer(KeyEvent.VK_A), new Key(1, 126));
        keys.put(new Integer(KeyEvent.VK_S), new Key(1, 125));
        keys.put(new Integer(KeyEvent.VK_D), new Key(1, 123));
        keys.put(new Integer(KeyEvent.VK_F), new Key(1, 119));
        keys.put(new Integer(KeyEvent.VK_G), new Key(1, 111));
        keys.put(new Integer(KeyEvent.VK_Q), new Key(2, 126));
        keys.put(new Integer(KeyEvent.VK_W), new Key(2, 125));
        keys.put(new Integer(KeyEvent.VK_E), new Key(2, 123));
        keys.put(new Integer(KeyEvent.VK_R), new Key(2, 119));
        keys.put(new Integer(KeyEvent.VK_T), new Key(2, 111));
        keys.put(new Integer(KeyEvent.VK_1), new Key(3, 126));
        keys.put(new Integer(KeyEvent.VK_2), new Key(3, 125));
        keys.put(new Integer(KeyEvent.VK_3), new Key(3, 123));
        keys.put(new Integer(KeyEvent.VK_4), new Key(3, 119));
        keys.put(new Integer(KeyEvent.VK_5), new Key(3, 111));
        keys.put(new Integer(KeyEvent.VK_LEFT), new Key(3, 111));
        keys.put(new Integer(KeyEvent.VK_0), new Key(4, 126));
        keys.put(new Integer(KeyEvent.VK_BACK_SPACE), new Key(4, 126, true));
        keys.put(new Integer(KeyEvent.VK_9), new Key(4, 125));
        keys.put(new Integer(KeyEvent.VK_8), new Key(4, 123));
        keys.put(new Integer(KeyEvent.VK_RIGHT), new Key(4, 123));
        keys.put(new Integer(KeyEvent.VK_7), new Key(4, 119));
        keys.put(new Integer(KeyEvent.VK_UP), new Key(4, 119));
        keys.put(new Integer(KeyEvent.VK_6), new Key(4, 111));
        keys.put(new Integer(KeyEvent.VK_DOWN), new Key(4, 111));
        keys.put(new Integer(KeyEvent.VK_P), new Key(5, 126));
        keys.put(new Integer(KeyEvent.VK_O), new Key(5, 125));
        keys.put(new Integer(KeyEvent.VK_I), new Key(5, 123));
        keys.put(new Integer(KeyEvent.VK_U), new Key(5, 119));
        keys.put(new Integer(KeyEvent.VK_Y), new Key(5, 111));
        keys.put(new Integer(KeyEvent.VK_ENTER), new Key(6, 126));
        keys.put(new Integer(KeyEvent.VK_L), new Key(6, 125));
        keys.put(new Integer(KeyEvent.VK_K), new Key(6, 123));
        keys.put(new Integer(KeyEvent.VK_J), new Key(6, 119));
        keys.put(new Integer(KeyEvent.VK_H), new Key(6, 111));
        keys.put(new Integer(KeyEvent.VK_SPACE), new Key(7, 126));
        keys.put(new Integer(KeyEvent.VK_PERIOD), new Key(7, 126));
        keys.put(new Integer(190), new Key(7, 126));
        keys.put(new Integer(KeyEvent.VK_COMMA), new Key(7, 125));
        keys.put(new Integer(188), new Key(7, 125));
        keys.put(new Integer(KeyEvent.VK_M), new Key(7, 123));
        keys.put(new Integer(KeyEvent.VK_N), new Key(7, 119));
        keys.put(new Integer(KeyEvent.VK_B), new Key(7, 111));
        for (int c=0; c<8; c++) {
            line[c] = 127;
        }
        current_key = (short)0xFFFF;
        this.ram = ram;
    }
    public void keyPressed(KeyEvent e) {
        Key key = (Key)keys.get(new Integer(e.getKeyCode()));
        if (key != null) {
            line[key.line] = key.value;
            current_key = key.code;
            if (((e.getModifiers() & InputEvent.SHIFT_MASK) != 0) || key.shift) {
                line[0] = (byte)(line[0] & 0xFE);
                current_key = (short)(current_key & 0xFEFF);
            }
            ram.writeWord(16421, current_key);
        }
    }
    public void keyReleased(KeyEvent e) {
        Key key = (Key)keys.get(new Integer(e.getKeyCode()));
        if (key != null) {
            line[key.line] = 127;
            current_key = (short)0xFFFF;
            line[0] = 127;
            ram.writeWord(16421, current_key);
        }
    }
    public byte readPort(byte low, byte high) {
        byte res = 127;
        byte mask = 1;
        for (int c=0; c<8; c++) {
            if ((high & mask) == 0) {
                res = (byte)(res & line[c]);
            }
            mask = (byte)(mask << 1);
        }
        return res;
    }
}

final class Key {
    public int line;
    public byte value;
    public short code;
    public boolean shift;

    public Key(int line, int value, boolean shift) {
        this.line = line;
        this.value = (byte)value;
        this.shift = shift;
        code = (short)((value << 9) | 0x100 | ((int)Math.pow(2, line) ^ 0xFF));
    }
    public Key(int line, int value) {
        this(line, value, false);
    }
}
