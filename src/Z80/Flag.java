// Copyright 1998-2021 by Robin Sergeant. See license.txt distributed with this file.

package Z80;

public final class Flag {
    private Register F;
    private byte mask1;
    private byte mask2;

    Flag(Register f, int bit) {
        F = f;
        mask1 = (byte)(Math.pow(2, bit));
        mask2 = (byte)(0xFF - mask1);
    }

    public void set() {
        F.val = (byte)(F.val | mask1);
    }

    public void set(boolean value) {
        if (value) {
            this.set();
        } else {
            this.clear();
        }
    }

    public void clear() {
        F.val = (byte)(F.val & mask2);
    }

    public boolean test() {
        return ((F.val & mask1) == mask1);
    }
}
