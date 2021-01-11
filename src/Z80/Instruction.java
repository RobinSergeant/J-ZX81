// CPU.java

/* main Instruction class, new Z80 instruction classes must sub class this and override the execute method */   


package Z80;

public class Instruction {
    protected static CPU cpu;
    protected static MemoryInterface ram;

    static final void setup(CPU cpu) {
        Instruction.cpu = cpu;
        ram = cpu.ram;
    }
    public int execute() {
        System.out.print("Ilegal Instruction: ");
        System.out.print(Integer.toHexString(cpu.PC-2 & 0xFFFF) + ", ");
        System.out.print(Integer.toHexString(ram.readByte(cpu.PC-2) & 0xFF));
        System.out.println(Integer.toHexString(cpu.IR & 0xFF));
        return 0;
    }
}
