package Z80;

final class Register {
    public byte val;
    private byte alt;   // alternative register

    public void swap() {
        byte temp = val;
        val = alt;
        alt = temp;
    }

    public void clear() {
        val = 0;
        this.swap();
        val = 0;
    }
}
