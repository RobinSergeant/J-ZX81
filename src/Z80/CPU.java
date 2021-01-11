// CPU.java

/* this file contains all the major CPU classes and all the Z80 instruction classes */

package Z80;

import java.util.*;
import java.io.*;

interface WordRegister {
    void set(short w);
    short get();
}

class StackPointer implements WordRegister {
    public short val;

    public void set(short w) {
        val = w;
    }
    public short get() {
        return val;
    }
}

final class IndexRegister extends RegisterPair {
    private byte dd;

    IndexRegister(CPU cpu, Register high, Register low) {
        super(cpu, high, low);
    }

    public void setDsp() {
        dd = cpu.fetchByte();
    }

    public byte readByte() {
        return cpu.ram.readByte(this.get()+dd);
    }

    public void writeByte(byte b) {
        cpu.ram.writeByte(this.get()+dd, b);
    }
}

public final class CPU extends Thread {
    private int tstates_ms;
    private boolean check_speed = true;
    private boolean interrupt;
    private Instruction none;
    int interrupt_mode;
    boolean halt;
    MemoryInterface ram;
    int tstates;
    long timer;
    InputPort in;
    OutputPort out;
    Instruction iset[] = new Instruction[256];      // main instruction set
    Instruction iset2[] = new Instruction[256];     // CB instructions
    Instruction iset3[] = new Instruction[256];     // DD instructions (IX)
    Instruction iset4[] = new Instruction[256];     // FD instructions (IY)
    Instruction iset5[] = new Instruction[256];     // ED instructions
    Instruction iset6[] = new Instruction[256];     // DDCB insturctions
    Instruction iset7[] = new Instruction[256];     // FDCB instructions
    public static boolean parity[] = new boolean[256];
    byte IR;                // current instruction register
    public short PC;        // program counter
    StackPointer SP = new StackPointer();       // stack pointer
    boolean IFF1, IFF2;     // interrupt flip-flops

    public Register A = new Register();
    public Register F = new Register();
    public Register H = new Register();
    public Register L = new Register();
    public Register B = new Register();
    public Register C = new Register();
    public Register D = new Register();
    public Register E = new Register();
    public Register IXl = new Register();
    public Register IXh = new Register();
    public Register IYl = new Register();
    public Register IYh = new Register();
    public Register I = new Register();
    public Register R = new Register();

    public RegisterPair AF = new RegisterPair(this, A, F);
    public RegisterPair HL = new RegisterPair(this, H, L);
    public RegisterPair BC = new RegisterPair(this, B, C);
    public RegisterPair DE = new RegisterPair(this, D, E);
    public IndexRegister IX = new IndexRegister(this, IXh, IXl);
    public IndexRegister IY = new IndexRegister(this, IYh, IYl);

    public Flag C_flag = new Flag(F, 0);    // Carry flag
    public Flag N_flag = new Flag(F, 1);    // Subtract flag
    public Flag PV_flag = new Flag(F, 2);   // Parity/overflow flag
    public Flag H_flag  = new Flag(F, 4);   // Half carry flag
    public Flag Z_flag = new Flag(F, 6);    // Zero flag
    public Flag S_flag = new Flag(F, 7);    // Sign flag

    static {    // setup parity array
        for (int n=0; n<256; n++) {
            int bitCount = 0;
            for (int b=0; b<8; b++) {
                if ((n & (int)Math.pow(2,b)) != 0) {
                    bitCount++;
                }
            }
            parity[n] = ((bitCount % 2) == 0);
        }
    }

    public CPU(float speed, MemoryInterface ram, InputPort in, OutputPort out) {
        this.setSpeed(speed);
        this.ram = ram;
        if (in != null) {
            this.in = in;
        } else {
            this.in = new DefaultInputPort();
        }
        if (out != null) {
            this.out = out;
        } else {
            this.out = new DefaultOutputPort();
        }
        Instruction.setup(this);
        none = new Instruction();
        for (int i=0; i<256; i++) {
            iset[i] = none;
            iset2[i] = none;
            iset3[i] = none;
            iset4[i] = none;
            iset5[i] = none;
            iset6[i] = none;
            iset7[i] = none;
        }

        iset[0] = new NOP();                    // NOP
        iset[1] = new LD_rp_nn(BC);             // LD BC,nn
        iset[2] = new LD_rpAdr_r(BC, A);        // LD (BC),A
        iset[3] = new INC_rp(BC);               // INC BC
        iset[4] = new INC_r(B);                 // INC B
        iset[5] = new DEC_r(B);                 // DEC B
        iset[6] = new LD_r_n(B);                // LD B,n
        iset[7] = new RLCA();                   // RLCA
        iset[8] = new EX_rp(AF);                // EX AF,AF'
        iset[9] = new ADD_rp_rp(HL, BC);        // ADD HL,BC
        iset[10] = new LD_r_rpAdr(A, BC);       // LD A,(BC)
        iset[11] = new DEC_rp(BC);              // DEC BC
        iset[12] = new INC_r(C);                // INC C
        iset[13] = new DEC_r(C);                // DEC C
        iset[14] = new LD_r_n(C);               // LD C,n
        iset[15] = new RRCA();                  // RRCA
        iset[16] = new DJNZ();                  // DJNZ
        iset[17] = new LD_rp_nn(DE);            // LD DE,nn
        iset[18] = new LD_rpAdr_r(DE, A);       // LD (DE),A
        iset[19] = new INC_rp(DE);              // INC DE
        iset[20] = new INC_r(D);                // INC D
        iset[21] = new DEC_r(D);                // DEC D
        iset[22] = new LD_r_n(D);               // LD D,n
        iset[23] = new RLA();                   // RLA
        iset[24] = new JR();                    // JR dd
        iset[25] = new ADD_rp_rp(HL, DE);       // ADD HL,DE
        iset[26] = new LD_r_rpAdr(A, DE);       // LD A,(DE)
        iset[27] = new DEC_rp(DE);              // DEC DE
        iset[28] = new INC_r(E);                // INC E
        iset[29] = new DEC_r(E);                // DEC E
        iset[30] = new LD_r_n(E);               // LD E,n
        iset[31] = new RRA();                   // RRA
        iset[32] = new JR_nCond(Z_flag);        // JR NZ,dd
        iset[33] = new LD_rp_nn(HL);            // LD HL,nn
        iset[34] = new LD_adr_rp(HL);           // LD (adr),HL
        iset[35] = new INC_rp(HL);              // INC HL
        iset[36] = new INC_r(H);                // INC H
        iset[37] = new DEC_r(H);                // DEC H
        iset[38] = new LD_r_n(H);               // LD H,n
        iset[39] = new DAA();                   // DAA
        iset[40] = new JR_pCond(Z_flag);        // JR Z,dd
        iset[41] = new ADD_rp_rp(HL, HL);       // ADD HL,HL
        iset[42] = new LD_rp_adr(HL);           // LD HL,(adr)
        iset[43] = new DEC_rp(HL);              // DEC HL
        iset[44] = new INC_r(L);                // INC L
        iset[45] = new DEC_r(L);                // DEC L
        iset[46] = new LD_r_n(L);               // LD L,n
        iset[47] = new CPL();                   // CPL
        iset[48] = new JR_nCond(C_flag);        // JR NC,dd
        iset[49] = new LD_rp_nn(SP);            // LD SP,nn
        iset[50] = new LD_adr_r(A);             // LD (adr),A
        iset[51] = new INC_rp(SP);              // INC SP
        iset[52] = new INC_rpAdr(HL);           // INC (HL)
        iset[53] = new DEC_rpAdr(HL);           // DEC (HL)
        iset[54] = new LD_rpAdr_n(HL);          // LD (HL),n
        iset[55] = new SCF();                   // SCF
        iset[56] = new JR_pCond(C_flag);        // JR C,dd
        iset[57] = new ADD_rp_rp(HL, SP);       // ADD HL,SP
        iset[58] = new LD_r_adr(A);             // LD A,(adr)
        iset[59] = new DEC_rp(SP);              // DEC SP
        iset[60] = new INC_r(A);                // INC A
        iset[61] = new DEC_r(A);                // DEC A
        iset[62] = new LD_r_n(A);               // LD A,n
        iset[63] = new CCF();                   // CCF
        iset[64] = new LD_r_r(B, B);            // LD B,B
        iset[65] = new LD_r_r(B, C);            // LD B,C
        iset[66] = new LD_r_r(B, D);            // LD B,D
        iset[67] = new LD_r_r(B, E);            // LD B,E
        iset[68] = new LD_r_r(B, H);            // LD B,H
        iset[69] = new LD_r_r(B, L);            // LD B,L
        iset[70] = new LD_r_rpAdr(B, HL);       // LD B,(HL)
        iset[71] = new LD_r_r(B, A);            // LD B,A
        iset[72] = new LD_r_r(C, B);            // LD C,B
        iset[73] = new LD_r_r(C, C);            // LD C,C
        iset[74] = new LD_r_r(C, D);            // LD C,D
        iset[75] = new LD_r_r(C, E);            // LD C,E
        iset[76] = new LD_r_r(C, H);            // LD C,H
        iset[77] = new LD_r_r(C, L);            // LD C,L
        iset[78] = new LD_r_rpAdr(C, HL);       // LD C,(HL)
        iset[79] = new LD_r_r(C, A);            // LD C,A
        iset[80] = new LD_r_r(D, B);            // LD D,B
        iset[81] = new LD_r_r(D, C);            // LD D,C
        iset[82] = new LD_r_r(D, D);            // LD D,D
        iset[83] = new LD_r_r(D, E);            // LD D,E
        iset[84] = new LD_r_r(D, H);            // LD D,H
        iset[85] = new LD_r_r(D, L);            // LD D,L
        iset[86] = new LD_r_rpAdr(D, HL);       // LD D,(HL)
        iset[87] = new LD_r_r(D, A);            // LD D,A
        iset[88] = new LD_r_r(E, B);            // LD E,B
        iset[89] = new LD_r_r(E, C);            // LD E,C
        iset[90] = new LD_r_r(E, D);            // LD E,D
        iset[91] = new LD_r_r(E, E);            // LD E,E
        iset[92] = new LD_r_r(E, H);            // LD E,H
        iset[93] = new LD_r_r(E, L);            // LD E,L
        iset[94] = new LD_r_rpAdr(E, HL);       // LD E,(HL)
        iset[95] = new LD_r_r(E, A);            // LD E,A
        iset[96] = new LD_r_r(H, B);            // LD H,B
        iset[97] = new LD_r_r(H, C);            // LD H,C
        iset[98] = new LD_r_r(H, D);            // LD H,D
        iset[99] = new LD_r_r(H, E);            // LD H,E
        iset[100] = new LD_r_r(H, H);           // LD H,H
        iset[101] = new LD_r_r(H, L);           // LD H,L
        iset[102] = new LD_r_rpAdr(H, HL);      // LD H,(HL)
        iset[103] = new LD_r_r(H, A);           // LD H,A
        iset[104] = new LD_r_r(L, B);           // LD L,B
        iset[105] = new LD_r_r(L, C);           // LD L,C
        iset[106] = new LD_r_r(L, D);           // LD L,D
        iset[107] = new LD_r_r(L, E);           // LD L,E
        iset[108] = new LD_r_r(L, H);           // LD L,H
        iset[109] = new LD_r_r(L, L);           // LD L,L
        iset[110] = new LD_r_rpAdr(L, HL);      // LD L,(HL)
        iset[111] = new LD_r_r(L, A);           // LD L,A
        iset[112] = new LD_rpAdr_r(HL, B);      // LD (HL),B
        iset[113] = new LD_rpAdr_r(HL, C);      // LD (HL),C
        iset[114] = new LD_rpAdr_r(HL, D);      // LD (HL),D
        iset[115] = new LD_rpAdr_r(HL, E);      // LD (HL),E
        iset[116] = new LD_rpAdr_r(HL, H);      // LD (HL),H
        iset[117] = new LD_rpAdr_r(HL, L);      // LD (HL),L
        iset[118] = new HALT();                 // HALT
        iset[119] = new LD_rpAdr_r(HL, A);      // LD (HL),A
        iset[120] = new LD_r_r(A, B);           // LD A,B
        iset[121] = new LD_r_r(A, C);           // LD A,C
        iset[122] = new LD_r_r(A, D);           // LD A,D
        iset[123] = new LD_r_r(A, E);           // LD A,E
        iset[124] = new LD_r_r(A, H);           // LD A,H
        iset[125] = new LD_r_r(A, L);           // LD A,L
        iset[126] = new LD_r_rpAdr(A, HL);      // LD A,(HL)
        iset[127] = new LD_r_r(A, A);           // LD A,A
        iset[128] = new ADD_A_r(B);             // ADD A,B
        iset[129] = new ADD_A_r(C);             // ADD A,C
        iset[130] = new ADD_A_r(D);             // ADD A,D
        iset[131] = new ADD_A_r(E);             // ADD A,E
        iset[132] = new ADD_A_r(H);             // ADD A,H
        iset[133] = new ADD_A_r(L);             // ADD A,L
        iset[134] = new ADD_A_rpAdr(HL);        // ADD A,(HL)
        iset[135] = new ADD_A_r(A);             // ADD A,A
        iset[136] = new ADC_A_r(B);             // ADC A,B
        iset[137] = new ADC_A_r(C);             // ADC A,C
        iset[138] = new ADC_A_r(D);             // ADC A,D
        iset[139] = new ADC_A_r(E);             // ADC A,E
        iset[140] = new ADC_A_r(H);             // ADC A,H
        iset[141] = new ADC_A_r(L);             // ADC A,L
        iset[142] = new ADC_A_rpAdr(HL);        // ADC A,(HL)
        iset[143] = new ADC_A_r(A);             // ADC A,A
        iset[144] = new SUB_A_r(B);             // SUB A,B
        iset[145] = new SUB_A_r(C);             // SUB A,C
        iset[146] = new SUB_A_r(D);             // SUB A,D
        iset[147] = new SUB_A_r(E);             // SUB A,E
        iset[148] = new SUB_A_r(H);             // SUB A,H
        iset[149] = new SUB_A_r(L);             // SUB A,L
        iset[150] = new SUB_A_rpAdr(HL);        // SUB A,(HL)
        iset[151] = new SUB_A_r(A);             // SUB A,A
        iset[152] = new SBC_A_r(B);             // SBC A,B
        iset[153] = new SBC_A_r(C);             // SBC A,C
        iset[154] = new SBC_A_r(D);             // SBC A,D
        iset[155] = new SBC_A_r(E);             // SBC A,E
        iset[156] = new SBC_A_r(H);             // SBC A,H
        iset[157] = new SBC_A_r(L);             // SBC A,L
        iset[158] = new SBC_A_rpAdr(HL);        // SBC A,(HL)
        iset[159] = new SBC_A_r(A);             // SBC A,A
        iset[160] = new AND_r(B);               // AND B
        iset[161] = new AND_r(C);               // AND C
        iset[162] = new AND_r(D);               // AND D
        iset[163] = new AND_r(E);               // AND E
        iset[164] = new AND_r(H);               // AND H
        iset[165] = new AND_r(L);               // AND L
        iset[166] = new AND_rpAdr(HL);          // AND (HL)
        iset[167] = new AND_r(A);               // AND A
        iset[168] = new XOR_r(B);               // XOR B
        iset[169] = new XOR_r(C);               // XOR C
        iset[170] = new XOR_r(D);               // XOR D
        iset[171] = new XOR_r(E);               // XOR E
        iset[172] = new XOR_r(H);               // XOR H
        iset[173] = new XOR_r(L);               // XOR L
        iset[174] = new XOR_rpAdr(HL);          // XOR (HL)
        iset[175] = new XOR_r(A);               // XOR A
        iset[176] = new OR_r(B);                // OR B
        iset[177] = new OR_r(C);                // OR C
        iset[178] = new OR_r(D);                // OR D
        iset[179] = new OR_r(E);                // OR E
        iset[180] = new OR_r(H);                // OR H
        iset[181] = new OR_r(L);                // OR L
        iset[182] = new OR_rpAdr(HL);           // OR (HL)
        iset[183] = new OR_r(A);                // OR A
        iset[184] = new CP_r(B);                // CP B
        iset[185] = new CP_r(C);                // CP C
        iset[186] = new CP_r(D);                // CP D
        iset[187] = new CP_r(E);                // CP E
        iset[188] = new CP_r(H);                // CP H
        iset[189] = new CP_r(L);                // CP L
        iset[190] = new CP_rpAdr(HL);           // CP (HL)
        iset[191] = new CP_r(A);                // CP A
        iset[192] = new RET_nCond(Z_flag);      // RET NZ
        iset[193] = new POP(BC);                // POP BC
        iset[194] = new JP_nCond(Z_flag);       // JP NZ,adr
        iset[195] = new JP();                   // JP adr
        iset[196] = new CALL_nCond(Z_flag);     // CALL NZ,adr
        iset[197] = new PUSH(BC);               // PUSH BC
        iset[198] = new ADD_A_n();              // ADD A,n
        iset[199] = new RST(0x00);              // RST 00H
        iset[200] = new RET_pCond(Z_flag);      // RET Z
        iset[201] = new RET();                  // RET
        iset[202] = new JP_pCond(Z_flag);       // JP Z,adr
        iset[203] = new CB_exec();
        iset[204] = new CALL_pCond(Z_flag);     // CALL Z,adr
        iset[205] = new CALL();                 // CALL adr
        iset[206] = new ADC_A_n();              // ADC A,n
        iset[207] = new RST(0x08);              // RST 08H
        iset[208] = new RET_nCond(C_flag);      // RET NC
        iset[209] = new POP(DE);                // POP DE
        iset[210] = new JP_nCond(C_flag);       // JP NC,adr
        iset[211] = new OUT_n_A();              // OUT (n),A
        iset[212] = new CALL_nCond(C_flag);     // CALL NC,adr
        iset[213] = new PUSH(DE);               // PUSH DE
        iset[214] = new SUB_A_n();              // SUB A,n
        iset[215] = new RST(0x10);              // RST 10H
        iset[216] = new RET_pCond(C_flag);      // RET C
        iset[217] = new EXX();                  // EXX
        iset[218] = new JP_pCond(C_flag);       // JP C,adr
        iset[219] = new IN_A_n();               // IN A,(n)
        iset[220] = new CALL_pCond(C_flag);     // CALL C,adr
        iset[221] = new DD_exec();
        iset[222] = new SBC_A_n();              // SBC A,n
        iset[223] = new RST(0x18);              // RST 18H
        iset[224] = new RET_nCond(PV_flag);     // RET PO
        iset[225] = new POP(HL);                // POP HL
        iset[226] = new JP_nCond(PV_flag);      // JP PO,adr
        iset[227] = new EX_SP_rp(HL);           // EX (SP),HL
        iset[228] = new CALL_nCond(PV_flag);    // CALL PO,adr
        iset[229] = new PUSH(HL);               // PUSH HL
        iset[230] = new AND_n();                // AND n
        iset[231] = new RST(0x20);              // RST 20H
        iset[232] = new RET_pCond(PV_flag);     // RET PE
        iset[233] = new JP_rpAdr(HL);           // JP (HL)
        iset[234] = new JP_pCond(PV_flag);      // JP PE,adr
        iset[235] = new EX_rp_rp(DE, HL);       // EX DE,HL
        iset[236] = new CALL_pCond(PV_flag);    // CALL PE,adr
        iset[237] = new ED_exec();
        iset[238] = new XOR_n();                // XOR n
        iset[239] = new RST(0x28);              // RST 28H
        iset[240] = new RET_nCond(S_flag);      // RET P
        iset[241] = new POP(AF);                // POP AF
        iset[242] = new JP_nCond(S_flag);       // JP P,adr
        iset[243] = new DI();                   // DI
        iset[244] = new CALL_nCond(S_flag);     // CALL P,adr
        iset[245] = new PUSH(AF);               // PUSH AF
        iset[246] = new OR_n();                 // OR n
        iset[247] = new RST(0x30);              // RST 30H
        iset[248] = new RET_pCond(S_flag);      // RET M
        iset[249] = new LD_SP_rp(HL);           // LD SP,HL
        iset[250] = new JP_pCond(S_flag);       // JP M,adr
        iset[251] = new EI();                   // EI
        iset[252] = new CALL_pCond(S_flag);     // CALL M,adr
        iset[253] = new FD_exec();
        iset[254] = new CP_n();                 // CP n
        iset[255] = new RST(0x38);              // RST 38H

        iset2[0] = new RLC_r(B);                // RLC B
        iset2[1] = new RLC_r(C);                // RLC C
        iset2[2] = new RLC_r(D);                // RLC D
        iset2[3] = new RLC_r(E);                // RLC E
        iset2[4] = new RLC_r(H);                // RLC H
        iset2[5] = new RLC_r(L);                // RLC L
        iset2[6] = new RLC_rpAdr(HL);           // RLC (HL)
        iset2[7] = new RLC_r(A);                // RLC A
        iset2[8] = new RRC_r(B);                // RRC B
        iset2[9] = new RRC_r(C);                // RRC C
        iset2[10] = new RRC_r(D);               // RRC D
        iset2[11] = new RRC_r(E);               // RRC E
        iset2[12] = new RRC_r(H);               // RRC H
        iset2[13] = new RRC_r(L);               // RRC L
        iset2[14] = new RRC_rpAdr(HL);          // RRC (HL)
        iset2[15] = new RRC_r(A);               // RRC A
        iset2[16] = new RL_r(B);                // RL B
        iset2[17] = new RL_r(C);                // RL C
        iset2[18] = new RL_r(D);                // RL D
        iset2[19] = new RL_r(E);                // RL E
        iset2[20] = new RL_r(H);                // RL H
        iset2[21] = new RL_r(L);                // RL L
        iset2[22] = new RL_rpAdr(HL);           // RL (HL)
        iset2[23] = new RL_r(A);                // RL A
        iset2[24] = new RR_r(B);                // RR B
        iset2[25] = new RR_r(C);                // RR C
        iset2[26] = new RR_r(D);                // RR D
        iset2[27] = new RR_r(E);                // RR E
        iset2[28] = new RR_r(H);                // RR H
        iset2[29] = new RR_r(L);                // RR L
        iset2[30] = new RR_rpAdr(HL);           // RR (HL)
        iset2[31] = new RR_r(A);                // RR A
        iset2[32] = new SLA_r(B);               // SLA B
        iset2[33] = new SLA_r(C);               // SLA C
        iset2[34] = new SLA_r(D);               // SLA D
        iset2[35] = new SLA_r(E);               // SLA E
        iset2[36] = new SLA_r(H);               // SLA H
        iset2[37] = new SLA_r(L);               // SLA L
        iset2[38] = new SLA_rpAdr(HL);          // SLA (HL)
        iset2[39] = new SLA_r(A);               // SLA A
        iset2[40] = new SRA_r(B);               // SRA B
        iset2[41] = new SRA_r(C);               // SRA C
        iset2[42] = new SRA_r(D);               // SRA D
        iset2[43] = new SRA_r(E);               // SRA E
        iset2[44] = new SRA_r(H);               // SRA H
        iset2[45] = new SRA_r(L);               // SRA L
        iset2[46] = new SRA_rpAdr(HL);          // SRA (HL)
        iset2[47] = new SRA_r(A);               // SRA A
        iset2[48] = new SLL_r(B);               // SLL B [undocumented]
        iset2[49] = new SLL_r(C);               // SLL C [undocumented]
        iset2[50] = new SLL_r(D);               // SLL D [undocumented]
        iset2[51] = new SLL_r(E);               // SLL E [undocumented]
        iset2[52] = new SLL_r(H);               // SLL H [undocumented]
        iset2[53] = new SLL_r(L);               // SLL L [undocumented]
        iset2[54] = new SLL_rpAdr(HL);          // SLL (HL) [undocumented]
        iset2[55] = new SLL_r(A);               // SLL A [undocumented]
        iset2[56] = new SRL_r(B);               // SRL B
        iset2[57] = new SRL_r(C);               // SRL C
        iset2[58] = new SRL_r(D);               // SRL D
        iset2[59] = new SRL_r(E);               // SRL E
        iset2[60] = new SRL_r(H);               // SRL H
        iset2[61] = new SRL_r(L);               // SRL L
        iset2[62] = new SRL_rpAdr(HL);          // SRL (HL)
        iset2[63] = new SRL_r(A);               // SRL A
        iset2[64] = new BIT_b_r(0,B);           // BIT 0,B
        iset2[65] = new BIT_b_r(0,C);           // BIT 0,C
        iset2[66] = new BIT_b_r(0,D);           // BIT 0,D
        iset2[67] = new BIT_b_r(0,E);           // BIT 0,E
        iset2[68] = new BIT_b_r(0,H);           // BIT 0,H
        iset2[69] = new BIT_b_r(0,L);           // BIT 0,L
        iset2[70] = new BIT_b_rpAdr(0,HL);      // BIT 0,(HL)
        iset2[71] = new BIT_b_r(0,A);           // BIT 0,A
        iset2[72] = new BIT_b_r(1,B);           // BIT 1,B
        iset2[73] = new BIT_b_r(1,C);           // BIT 1,C
        iset2[74] = new BIT_b_r(1,D);           // BIT 1,D
        iset2[75] = new BIT_b_r(1,E);           // BIT 1,E
        iset2[76] = new BIT_b_r(1,H);           // BIT 1,H
        iset2[77] = new BIT_b_r(1,L);           // BIT 1,L
        iset2[78] = new BIT_b_rpAdr(1,HL);      // BIT 1,(HL)
        iset2[79] = new BIT_b_r(1,A);           // BIT 1,A
        iset2[80] = new BIT_b_r(2,B);           // BIT 2,B
        iset2[81] = new BIT_b_r(2,C);           // BIT 2,C
        iset2[82] = new BIT_b_r(2,D);           // BIT 2,D
        iset2[83] = new BIT_b_r(2,E);           // BIT 2,E
        iset2[84] = new BIT_b_r(2,H);           // BIT 2,H
        iset2[85] = new BIT_b_r(2,L);           // BIT 2,L
        iset2[86] = new BIT_b_rpAdr(2,HL);      // BIT 2,(HL)
        iset2[87] = new BIT_b_r(2,A);           // BIT 2,A
        iset2[88] = new BIT_b_r(3,B);           // BIT 3,B
        iset2[89] = new BIT_b_r(3,C);           // BIT 3,C
        iset2[90] = new BIT_b_r(3,D);           // BIT 3,D
        iset2[91] = new BIT_b_r(3,E);           // BIT 3,E
        iset2[92] = new BIT_b_r(3,H);           // BIT 3,H
        iset2[93] = new BIT_b_r(3,L);           // BIT 3,L
        iset2[94] = new BIT_b_rpAdr(3,HL);      // BIT 3,(HL)
        iset2[95] = new BIT_b_r(3,A);           // BIT 3,A
        iset2[96] = new BIT_b_r(4,B);           // BIT 4,B
        iset2[97] = new BIT_b_r(4,C);           // BIT 4,C
        iset2[98] = new BIT_b_r(4,D);           // BIT 4,D
        iset2[99] = new BIT_b_r(4,E);           // BIT 4,E
        iset2[100] = new BIT_b_r(4,H);          // BIT 4,H
        iset2[101] = new BIT_b_r(4,L);          // BIT 4,L
        iset2[102] = new BIT_b_rpAdr(4,HL);     // BIT 4,(HL)
        iset2[103] = new BIT_b_r(4,A);          // BIT 4,A
        iset2[104] = new BIT_b_r(5,B);          // BIT 5,B
        iset2[105] = new BIT_b_r(5,C);          // BIT 5,C
        iset2[106] = new BIT_b_r(5,D);          // BIT 5,D
        iset2[107] = new BIT_b_r(5,E);          // BIT 5,E
        iset2[108] = new BIT_b_r(5,H);          // BIT 5,H
        iset2[109] = new BIT_b_r(5,L);          // BIT 5,L
        iset2[110] = new BIT_b_rpAdr(5,HL);     // BIT 5,(HL)
        iset2[111] = new BIT_b_r(5,A);          // BIT 5,A
        iset2[112] = new BIT_b_r(6,B);          // BIT 6,B
        iset2[113] = new BIT_b_r(6,C);          // BIT 6,C
        iset2[114] = new BIT_b_r(6,D);          // BIT 6,D
        iset2[115] = new BIT_b_r(6,E);          // BIT 6,E
        iset2[116] = new BIT_b_r(6,H);          // BIT 6,H
        iset2[117] = new BIT_b_r(6,L);          // BIT 6,L
        iset2[118] = new BIT_b_rpAdr(6,HL);     // BIT 6,(HL)
        iset2[119] = new BIT_b_r(6,A);          // BIT 6,A
        iset2[120] = new BIT_b_r(7,B);          // BIT 7,B
        iset2[121] = new BIT_b_r(7,C);          // BIT 7,C
        iset2[122] = new BIT_b_r(7,D);          // BIT 7,D
        iset2[123] = new BIT_b_r(7,E);          // BIT 7,E
        iset2[124] = new BIT_b_r(7,H);          // BIT 7,H
        iset2[125] = new BIT_b_r(7,L);          // BIT 7,L
        iset2[126] = new BIT_b_rpAdr(7,HL);     // BIT 7,(HL)
        iset2[127] = new BIT_b_r(7,A);          // BIT 7,A
        iset2[128] = new RES_b_r(0,B);          // RES 0,B
        iset2[129] = new RES_b_r(0,C);          // RES 0,C
        iset2[130] = new RES_b_r(0,D);          // RES 0,D
        iset2[131] = new RES_b_r(0,E);          // RES 0,E
        iset2[132] = new RES_b_r(0,H);          // RES 0,H
        iset2[133] = new RES_b_r(0,L);          // RES 0,L
        iset2[134] = new RES_b_rpAdr(0,HL);     // RES 0,(HL)
        iset2[135] = new RES_b_r(0,A);          // RES 0,A
        iset2[136] = new RES_b_r(1,B);          // RES 1,B
        iset2[137] = new RES_b_r(1,C);          // RES 1,C
        iset2[138] = new RES_b_r(1,D);          // RES 1,D
        iset2[139] = new RES_b_r(1,E);          // RES 1,E
        iset2[140] = new RES_b_r(1,H);          // RES 1,H
        iset2[141] = new RES_b_r(1,L);          // RES 1,L
        iset2[142] = new RES_b_rpAdr(1,HL);     // RES 1,(HL)
        iset2[143] = new RES_b_r(1,A);          // RES 1,A
        iset2[144] = new RES_b_r(2,B);          // RES 2,B
        iset2[145] = new RES_b_r(2,C);          // RES 2,C
        iset2[146] = new RES_b_r(2,D);          // RES 2,D
        iset2[147] = new RES_b_r(2,E);          // RES 2,E
        iset2[148] = new RES_b_r(2,H);          // RES 2,H
        iset2[149] = new RES_b_r(2,L);          // RES 2,L
        iset2[150] = new RES_b_rpAdr(2,HL);     // RES 2,(HL)
        iset2[151] = new RES_b_r(2,A);          // RES 2,A
        iset2[152] = new RES_b_r(3,B);          // RES 3,B
        iset2[153] = new RES_b_r(3,C);          // RES 3,C
        iset2[154] = new RES_b_r(3,D);          // RES 3,D
        iset2[155] = new RES_b_r(3,E);          // RES 3,E
        iset2[156] = new RES_b_r(3,H);          // RES 3,H
        iset2[157] = new RES_b_r(3,L);          // RES 3,L
        iset2[158] = new RES_b_rpAdr(3,HL);     // RES 3,(HL)
        iset2[159] = new RES_b_r(3,A);          // RES 3,A
        iset2[160] = new RES_b_r(4,B);          // RES 4,B
        iset2[161] = new RES_b_r(4,C);          // RES 4,C
        iset2[162] = new RES_b_r(4,D);          // RES 4,D
        iset2[163] = new RES_b_r(4,E);          // RES 4,E
        iset2[164] = new RES_b_r(4,H);          // RES 4,H
        iset2[165] = new RES_b_r(4,L);          // RES 4,L
        iset2[166] = new RES_b_rpAdr(4,HL);     // RES 4,(HL)
        iset2[167] = new RES_b_r(4,A);          // RES 4,A
        iset2[168] = new RES_b_r(5,B);          // RES 5,B
        iset2[169] = new RES_b_r(5,C);          // RES 5,C
        iset2[170] = new RES_b_r(5,D);          // RES 5,D
        iset2[171] = new RES_b_r(5,E);          // RES 5,E
        iset2[172] = new RES_b_r(5,H);          // RES 5,H
        iset2[173] = new RES_b_r(5,L);          // RES 5,L
        iset2[174] = new RES_b_rpAdr(5,HL);     // RES 5,(HL)
        iset2[175] = new RES_b_r(5,A);          // RES 5,A
        iset2[176] = new RES_b_r(6,B);          // RES 6,B
        iset2[177] = new RES_b_r(6,C);          // RES 6,C
        iset2[178] = new RES_b_r(6,D);          // RES 6,D
        iset2[179] = new RES_b_r(6,E);          // RES 6,E
        iset2[180] = new RES_b_r(6,H);          // RES 6,H
        iset2[181] = new RES_b_r(6,L);          // RES 6,L
        iset2[182] = new RES_b_rpAdr(6,HL);     // RES 6,(HL)
        iset2[183] = new RES_b_r(6,A);          // RES 6,A
        iset2[184] = new RES_b_r(7,B);          // RES 7,B
        iset2[185] = new RES_b_r(7,C);          // RES 7,C
        iset2[186] = new RES_b_r(7,D);          // RES 7,D
        iset2[187] = new RES_b_r(7,E);          // RES 7,E
        iset2[188] = new RES_b_r(7,H);          // RES 7,H
        iset2[189] = new RES_b_r(7,L);          // RES 7,L
        iset2[190] = new RES_b_rpAdr(7,HL);     // RES 7,(HL)
        iset2[191] = new RES_b_r(7,A);          // RES 7,A
        iset2[192] = new SET_b_r(0,B);          // SET 0,B
        iset2[193] = new SET_b_r(0,C);          // SET 0,C
        iset2[194] = new SET_b_r(0,D);          // SET 0,D
        iset2[195] = new SET_b_r(0,E);          // SET 0,E
        iset2[196] = new SET_b_r(0,H);          // SET 0,H
        iset2[197] = new SET_b_r(0,L);          // SET 0,L
        iset2[198] = new SET_b_rpAdr(0,HL);     // SET 0,(HL)
        iset2[199] = new SET_b_r(0,A);          // SET 0,A
        iset2[200] = new SET_b_r(1,B);          // SET 1,B
        iset2[201] = new SET_b_r(1,C);          // SET 1,C
        iset2[202] = new SET_b_r(1,D);          // SET 1,D
        iset2[203] = new SET_b_r(1,E);          // SET 1,E
        iset2[204] = new SET_b_r(1,H);          // SET 1,H
        iset2[205] = new SET_b_r(1,L);          // SET 1,L
        iset2[206] = new SET_b_rpAdr(1,HL);     // SET 1,(HL)
        iset2[207] = new SET_b_r(1,A);          // SET 1,A
        iset2[208] = new SET_b_r(2,B);          // SET 2,B
        iset2[209] = new SET_b_r(2,C);          // SET 2,C
        iset2[210] = new SET_b_r(2,D);          // SET 2,D
        iset2[211] = new SET_b_r(2,E);          // SET 2,E
        iset2[212] = new SET_b_r(2,H);          // SET 2,H
        iset2[213] = new SET_b_r(2,L);          // SET 2,L
        iset2[214] = new SET_b_rpAdr(2,HL);     // SET 2,(HL)
        iset2[215] = new SET_b_r(2,A);          // SET 2,A
        iset2[216] = new SET_b_r(3,B);          // SET 3,B
        iset2[217] = new SET_b_r(3,C);          // SET 3,C
        iset2[218] = new SET_b_r(3,D);          // SET 3,D
        iset2[219] = new SET_b_r(3,E);          // SET 3,E
        iset2[220] = new SET_b_r(3,H);          // SET 3,H
        iset2[221] = new SET_b_r(3,L);          // SET 3,L
        iset2[222] = new SET_b_rpAdr(3,HL);     // SET 3,(HL)
        iset2[223] = new SET_b_r(3,A);          // SET 3,A
        iset2[224] = new SET_b_r(4,B);          // SET 4,B
        iset2[225] = new SET_b_r(4,C);          // SET 4,C
        iset2[226] = new SET_b_r(4,D);          // SET 4,D
        iset2[227] = new SET_b_r(4,E);          // SET 4,E
        iset2[228] = new SET_b_r(4,H);          // SET 4,H
        iset2[229] = new SET_b_r(4,L);          // SET 4,L
        iset2[230] = new SET_b_rpAdr(4,HL);     // SET 4,(HL)
        iset2[231] = new SET_b_r(4,A);          // SET 4,A
        iset2[232] = new SET_b_r(5,B);          // SET 5,B
        iset2[233] = new SET_b_r(5,C);          // SET 5,C
        iset2[234] = new SET_b_r(5,D);          // SET 5,D
        iset2[235] = new SET_b_r(5,E);          // SET 5,E
        iset2[236] = new SET_b_r(5,H);          // SET 5,H
        iset2[237] = new SET_b_r(5,L);          // SET 5,L
        iset2[238] = new SET_b_rpAdr(5,HL);     // SET 5,(HL)
        iset2[239] = new SET_b_r(5,A);          // SET 5,A
        iset2[240] = new SET_b_r(6,B);          // SET 6,B
        iset2[241] = new SET_b_r(6,C);          // SET 6,C
        iset2[242] = new SET_b_r(6,D);          // SET 6,D
        iset2[243] = new SET_b_r(6,E);          // SET 6,E
        iset2[244] = new SET_b_r(6,H);          // SET 6,H
        iset2[245] = new SET_b_r(6,L);          // SET 6,L
        iset2[246] = new SET_b_rpAdr(6,HL);     // SET 6,(HL)
        iset2[247] = new SET_b_r(6,A);          // SET 6,A
        iset2[248] = new SET_b_r(7,B);          // SET 7,B
        iset2[249] = new SET_b_r(7,C);          // SET 7,C
        iset2[250] = new SET_b_r(7,D);          // SET 7,D
        iset2[251] = new SET_b_r(7,E);          // SET 7,E
        iset2[252] = new SET_b_r(7,H);          // SET 7,H
        iset2[253] = new SET_b_r(7,L);          // SET 7,L
        iset2[254] = new SET_b_rpAdr(7,HL);     // SET 7,(HL)
        iset2[255] = new SET_b_r(7,A);          // SET 7,A

        iset3[9] = new ADD_rp_rp(IX,BC);        // ADD IX,BC
        iset3[25] = new ADD_rp_rp(IX,DE);       // ADD IX,DE
        iset3[33] = new LD_rp_nn(IX);           // LD IY,nn
        iset3[34] = new LD_adr_rp(IX);          // LD (adr),IX
        iset3[35] = new INC_rp(IX);             // INC IX
        iset3[41] = new ADD_rp_rp(IX,IX);       // ADD IX,IX
        iset3[42] = new LD_rp_adr(IX);          // LD IX,(adr)
        iset3[43] = new DEC_rp(IX);             // DEC IX
        iset3[52] = new INC_idxAdr(IX);         // INC (IX+dd
        iset3[53] = new DEC_idxAdr(IX);         // DEC (IX+dd)
        iset3[54] = new LD_idxAdr_n(IX);        // LD (IX+dd),n
        iset3[57] = new ADD_rp_rp(IX,SP);       // ADD IX,SP
        iset3[70] = new LD_r_idxAdr(B,IX);      // LD B,(IX+dd)
        iset3[78] = new LD_r_idxAdr(C,IX);      // LD C,(IX+dd)
        iset3[86] = new LD_r_idxAdr(D,IX);      // LD D,(IX+dd)
        iset3[94] = new LD_r_idxAdr(E,IX);      // LD E,(IX+dd)
        iset3[102] = new LD_r_idxAdr(H,IX);     // LD H,(IX+dd)
        iset3[110] = new LD_r_idxAdr(L,IX);     // LD L,(IX+dd)
        iset3[112] = new LD_idxAdr_r(IX,B);     // LD (IX+dd),B
        iset3[113] = new LD_idxAdr_r(IX,C);     // LD (IX+dd),C
        iset3[114] = new LD_idxAdr_r(IX,D);     // LD (IX+dd),D
        iset3[115] = new LD_idxAdr_r(IX,E);     // LD (IX+dd),E
        iset3[116] = new LD_idxAdr_r(IX,H);     // LD (IX+dd),H
        iset3[117] = new LD_idxAdr_r(IX,L);     // LD (IX+dd),L
        iset3[119] = new LD_idxAdr_r(IX,A);     // LD (IX+dd),A
        iset3[126] = new LD_r_idxAdr(A,IX);     // LD A,(IX+dd)
        iset3[134] = new ADD_A_idxAdr(IX);      // ADD A,(IX+dd)
        iset3[142] = new ADC_A_idxAdr(IX);      // ADC A,(IX+dd)
        iset3[150] = new SUB_A_idxAdr(IX);      // SUB A,(IX+dd)
        iset3[158] = new SBC_A_idxAdr(IX);      // SBC A,(IX+dd)
        iset3[166] = new AND_idxAdr(IX);        // AND (IX+dd)
        iset3[174] = new XOR_idxAdr(IX);        // XOR (IX+dd)
        iset3[182] = new OR_idxAdr(IX);         // OR (IX+dd)
        iset3[190] = new CP_idxAdr(IX);         // CP (IX+dd)
        iset3[203] = new DDCB_exec();
        iset3[225] = new POP(IX);               // POP IX
        iset3[227] = new EX_SP_rp(IX);          // EX (SP),IX
        iset3[229] = new PUSH(IX);              // PUSH IX
        iset3[233] = new JP_rpAdr(IX);          // JP (IX)
        iset3[249] = new LD_SP_rp(IX);          // LD SP,IX

        iset4[9] = new ADD_rp_rp(IY,BC);        // ADD IY,BC
        iset4[25] = new ADD_rp_rp(IY,DE);       // ADD IY,DE
        iset4[33] = new LD_rp_nn(IY);           // LD IY,nn
        iset4[34] = new LD_adr_rp(IY);          // LD (adr),IY
        iset4[35] = new INC_rp(IY);             // INC IY
        iset4[41] = new ADD_rp_rp(IY,IY);       // ADD IY,IY
        iset4[42] = new LD_rp_adr(IY);          // LD IY,(adr)
        iset4[43] = new DEC_rp(IY);             // DEC IY
        iset4[52] = new INC_idxAdr(IY);         // INC (IY+dd
        iset4[53] = new DEC_idxAdr(IY);         // DEC (IY+dd)
        iset4[54] = new LD_idxAdr_n(IY);        // LD (IY+dd),n
        iset4[57] = new ADD_rp_rp(IY,SP);       // ADD IY,SP
        iset4[70] = new LD_r_idxAdr(B,IY);      // LD B,(IY+dd)
        iset4[78] = new LD_r_idxAdr(C,IY);      // LD C,(IY+dd)
        iset4[86] = new LD_r_idxAdr(D,IY);      // LD D,(IY+dd)
        iset4[94] = new LD_r_idxAdr(E,IY);      // LD E,(IY+dd)
        iset4[102] = new LD_r_idxAdr(H,IY);     // LD H,(IY+dd)
        iset4[110] = new LD_r_idxAdr(L,IY);     // LD L,(IY+dd)
        iset4[112] = new LD_idxAdr_r(IY,B);     // LD (IY+dd),B
        iset4[113] = new LD_idxAdr_r(IY,C);     // LD (IY+dd),C
        iset4[114] = new LD_idxAdr_r(IY,D);     // LD (IY+dd),D
        iset4[115] = new LD_idxAdr_r(IY,E);     // LD (IY+dd),E
        iset4[116] = new LD_idxAdr_r(IY,H);     // LD (IY+dd),H
        iset4[117] = new LD_idxAdr_r(IY,L);     // LD (IY+dd),L
        iset4[119] = new LD_idxAdr_r(IY,A);     // LD (IY+dd),A
        iset4[126] = new LD_r_idxAdr(A,IY);     // LD A,(IY+dd)
        iset4[134] = new ADD_A_idxAdr(IY);      // ADD A,(IY+dd)
        iset4[142] = new ADC_A_idxAdr(IY);      // ADC A,(IY+dd)
        iset4[150] = new SUB_A_idxAdr(IY);      // SUB A,(IY+dd)
        iset4[158] = new SBC_A_idxAdr(IY);      // SBC A,(IY+dd)
        iset4[166] = new AND_idxAdr(IY);        // AND (IY+dd)
        iset4[174] = new XOR_idxAdr(IY);        // XOR (IY+dd)
        iset4[182] = new OR_idxAdr(IY);         // OR (IY+dd)
        iset4[190] = new CP_idxAdr(IY);         // CP (IY+dd)
        iset4[203] = new FDCB_exec();
        iset4[225] = new POP(IY);               // POP IY
        iset4[227] = new EX_SP_rp(IY);          // EX (SP),IY
        iset4[229] = new PUSH(IY);              // PUSH IY
        iset4[233] = new JP_rpAdr(IY);          // JP (IY)
        iset4[249] = new LD_SP_rp(IY);          // LD SP,IY

        iset5[64] = new IN_r_C(B);              // IN B,(C)
        iset5[65] = new OUT_C_r(B);             // OUT (C),B
        iset5[66] = new SBC_HL_rp(BC);          // SBC HL,BC
        iset5[67] = new LD_adr_rp(BC);          // LD (adr),BC
        iset5[68] = new NEG();                  // NEG
        iset5[69] = new RETN();                 // RETN
        iset5[70] = new IM(0);                  // IM 0
        iset5[71] = new LD_I_A(I);              // LD I,A
        iset5[72] = new IN_r_C(C);              // IN C,(C)
        iset5[73] = new OUT_C_r(C);             // OUT (C),C
        iset5[74] = new ADC_rp_rp(HL, BC);      // ADC HL,BC
        iset5[75] = new LD_rp_adr(BC);          // LD BC,(adr)
        iset5[77] = new RETN();                 // RETI
        iset5[79] = new LD_I_A(R);              // LD R,A
        iset5[80] = new IN_r_C(D);              // IN D,(C)
        iset5[81] = new OUT_C_r(D);             // OUT (C),D
        iset5[82] = new SBC_HL_rp(DE);          // SBC HL,DE
        iset5[83] = new LD_adr_rp(DE);          // LD (adr),DE
        iset5[86] = new IM(1);                  // IM 1
        iset5[87] = new LD_A_I(I);              // LD A,I
        iset5[88] = new IN_r_C(E);              // IN E,(C)
        iset5[89] = new OUT_C_r(E);             // OUT (C),E
        iset5[90] = new ADC_rp_rp(HL, DE);      // ADC HL,DE
        iset5[91] = new LD_rp_adr(DE);          // LD DE,(adr)
        iset5[94] = new IM(2);                  // IM 2
        iset5[95] = new LD_A_I(R);              // LD A,R
        iset5[96] = new IN_r_C(H);              // IN H,(C)
        iset5[97] = new OUT_C_r(H);             // OUT (C),H
        iset5[98] = new SBC_HL_rp(HL);          // SBC HL,HL
        iset5[103] = new RRD();                 // RRD
        iset5[104] = new IN_r_C(L);             // IN L,(C)
        iset5[105] = new OUT_C_r(L);            // OUT (C),L
        iset5[106] = new ADC_rp_rp(HL, HL);     // ADC HL,HL
        iset5[111] = new RLD();                 // RLD
        iset5[114] = new SBC_HL_rp(SP);         // SBC HL,SP
        iset5[115] = new LD_adr_rp(SP);         // LD (adr),SP
        iset5[120] = new IN_r_C(A);             // IN A,(C)
        iset5[121] = new OUT_C_r(A);            // OUT (C),A
        iset5[122] = new ADC_rp_rp(HL, SP);     // ADC HL,SP
        iset5[123] = new LD_rp_adr(SP);         // LD SP,(adr)
        iset5[160] = new LDI();                 // LDI
        iset5[161] = new CPI();                 // CPI
        iset5[162] = new INI();                 // INI
        iset5[163] = new OUTI();                // OUTI
        iset5[168] = new LDD();                 // LDD
        iset5[169] = new CPD();                 // CPD
        iset5[170] = new IND();                 // IND
        iset5[171] = new OUTD();                // OUTD
        iset5[176] = new LDIR();                // LDIR
        iset5[177] = new CPIR();                // CPIR
        iset5[178] = new INIR();                // INIR
        iset5[179] = new OTIR();                // OTIR
        iset5[184] = new LDDR();                // LDDR
        iset5[185] = new CPDR();                // CPDR
        iset5[186] = new INDR();                // INDR
        iset5[187] = new OTDR();                // OTDR

        iset6[6] = new RLC_rpAdr(IX);           // RLC (IX+dd)
        iset6[14] = new RRC_rpAdr(IX);          // RRC (IX+dd)
        iset6[22] = new RL_rpAdr(IX);           // RL (IX+dd)
        iset6[30] = new RR_rpAdr(IX);           // RR (IX+dd)
        iset6[38] = new SLA_rpAdr(IX);          // SLA (IX+dd)
        iset6[46] = new SRA_rpAdr(IX);          // SRA (IX+dd)
        iset6[54] = new SLL_rpAdr(IX);          // SLL (IX+dd) [undocumented]
        iset6[62] = new SRL_rpAdr(IX);          // SRL (IX+dd)
        iset6[70] = new BIT_b_rpAdr(0,IX);      // BIT 0,(IX+dd)
        iset6[78] = new BIT_b_rpAdr(1,IX);      // BIT 1,(IX+dd)
        iset6[86] = new BIT_b_rpAdr(2,IX);      // BIT 2,(IX+dd)
        iset6[94] = new BIT_b_rpAdr(3,IX);      // BIT 3,(IX+dd)
        iset6[102] = new BIT_b_rpAdr(4,IX);     // BIT 4,(IX+dd)
        iset6[110] = new BIT_b_rpAdr(5,IX);     // BIT 5,(IX+dd)
        iset6[118] = new BIT_b_rpAdr(6,IX);     // BIT 6,(IX+dd)
        iset6[126] = new BIT_b_rpAdr(7,IX);     // BIT 7,(IX+dd)
        iset6[134] = new RES_b_rpAdr(0,IX);     // RES 0,(IX+dd)
        iset6[142] = new RES_b_rpAdr(1,IX);     // RES 1,(IX+dd)
        iset6[150] = new RES_b_rpAdr(2,IX);     // RES 2,(IX+dd)
        iset6[158] = new RES_b_rpAdr(3,IX);     // RES 3,(IX+dd)
        iset6[166] = new RES_b_rpAdr(4,IX);     // RES 4,(IX+dd)
        iset6[174] = new RES_b_rpAdr(5,IX);     // RES 5,(IX+dd)
        iset6[182] = new RES_b_rpAdr(6,IX);     // RES 6,(IX+dd)
        iset6[190] = new RES_b_rpAdr(7,IX);     // RES 7,(IX+dd)
        iset6[198] = new SET_b_rpAdr(0,IX);     // SET 0,(IX+dd)
        iset6[206] = new SET_b_rpAdr(1,IX);     // SET 1,(IX+dd)
        iset6[214] = new SET_b_rpAdr(2,IX);     // SET 2,(IX+dd)
        iset6[222] = new SET_b_rpAdr(3,IX);     // SET 3,(IX+dd)
        iset6[230] = new SET_b_rpAdr(4,IX);     // SET 4,(IX+dd)
        iset6[238] = new SET_b_rpAdr(5,IX);     // SET 5,(IX+dd)
        iset6[246] = new SET_b_rpAdr(6,IX);     // SET 6,(IX+dd)
        iset6[254] = new SET_b_rpAdr(7,IX);     // SET 7,(IX+dd)

        iset7[6] = new RLC_rpAdr(IY);           // RLC (IY+dd)
        iset7[14] = new RRC_rpAdr(IY);          // RRC (IY+dd)
        iset7[22] = new RL_rpAdr(IY);           // RL (IY+dd)
        iset7[30] = new RR_rpAdr(IY);           // RR (IY+dd)
        iset7[38] = new SLA_rpAdr(IY);          // SLA (IY+dd)
        iset7[46] = new SRA_rpAdr(IY);          // SRA (IY+dd)
        iset7[54] = new SLL_rpAdr(IY);          // SLL (IY+dd) [undocumented]
        iset7[62] = new SRL_rpAdr(IY);          // SRL (IY+dd)
        iset7[70] = new BIT_b_rpAdr(0,IY);      // BIT 0,(IY+dd)
        iset7[78] = new BIT_b_rpAdr(1,IY);      // BIT 1,(IY+dd)
        iset7[86] = new BIT_b_rpAdr(2,IY);      // BIT 2,(IY+dd)
        iset7[94] = new BIT_b_rpAdr(3,IY);      // BIT 3,(IY+dd)
        iset7[102] = new BIT_b_rpAdr(4,IY);     // BIT 4,(IY+dd)
        iset7[110] = new BIT_b_rpAdr(5,IY);     // BIT 5,(IY+dd)
        iset7[118] = new BIT_b_rpAdr(6,IY);     // BIT 6,(IY+dd)
        iset7[126] = new BIT_b_rpAdr(7,IY);     // BIT 7,(IY+dd)
        iset7[134] = new RES_b_rpAdr(0,IY);     // RES 0,(IY+dd)
        iset7[142] = new RES_b_rpAdr(1,IY);     // RES 1,(IY+dd)
        iset7[150] = new RES_b_rpAdr(2,IY);     // RES 2,(IY+dd)
        iset7[158] = new RES_b_rpAdr(3,IY);     // RES 3,(IY+dd)
        iset7[166] = new RES_b_rpAdr(4,IY);     // RES 4,(IY+dd)
        iset7[174] = new RES_b_rpAdr(5,IY);     // RES 5,(IY+dd)
        iset7[182] = new RES_b_rpAdr(6,IY);     // RES 6,(IY+dd)
        iset7[190] = new RES_b_rpAdr(7,IY);     // RES 7,(IY+dd)
        iset7[198] = new SET_b_rpAdr(0,IY);     // SET 0,(IY+dd)
        iset7[206] = new SET_b_rpAdr(1,IY);     // SET 1,(IY+dd)
        iset7[214] = new SET_b_rpAdr(2,IY);     // SET 2,(IY+dd)
        iset7[222] = new SET_b_rpAdr(3,IY);     // SET 3,(IY+dd)
        iset7[230] = new SET_b_rpAdr(4,IY);     // SET 4,(IY+dd)
        iset7[238] = new SET_b_rpAdr(5,IY);     // SET 5,(IY+dd)
        iset7[246] = new SET_b_rpAdr(6,IY);     // SET 6,(IY+dd)
        iset7[254] = new SET_b_rpAdr(7,IY);     // SET 7,(IY+dd)

        this.setDaemon(true);
    }

    public byte fetchByte() {
        PC++;
        return ram.readByte(PC-1);
    }

    public short fetchWord() {
        PC+=2;
        return ram.readWord(PC-2);
    }

    public void push(short w) {
        SP.val-=2;
        ram.writeWord(SP.val, w);
    }

    public short pop() {
        SP.val+=2;
        return ram.readWord(SP.val-2);
    }

    public void NMI() {
        IFF1 = false;
        interrupt = true;
    }

    public void resetTimer() {
        tstates = 0;
        timer = System.currentTimeMillis();
    }

    public void setTurbo(boolean val) {
        check_speed = !val;
        this.resetTimer();
    }

    public void setSpeed(float speed) {
        tstates_ms = (int)(speed * 1000);
        this.resetTimer();
    }

    public byte registerED(Instruction inst) {
        int op = 200;
        while (iset5[op] != none) {
            op++;
        }
        iset5[op] = inst;
        return (byte)op;
    }

    public void reset() {
        PC = 0;
        SP.val = 0;
        IFF1 = false;
        IFF2 = false;
        halt = false;
        interrupt_mode = 0;
        I.clear();
        R.clear();
        AF.clear();
        HL.clear();
        BC.clear();
        DE.clear();
        IX.clear();
        IY.clear();
        C_flag.clear();
        N_flag.clear();
        PV_flag.clear();
        H_flag.clear();
        Z_flag.clear();
        S_flag.clear();
        ram.clear();
        this.resetTimer();
    }

    public void run() {
        int opcode;
        this.resetTimer();
        while (true) {
            if (tstates >= tstates_ms) {
                tstates = 0;
                timer++;
                if (interrupt) {
                    PC = 0x66;
                    interrupt = false;
                }
                if (check_speed) {
                    try {
                        Thread.sleep(Math.max(0, timer -                             System.currentTimeMillis()));
                    } catch (InterruptedException e) {}
                }
            }
            //System.out.print(Integer.toHexString(PC) + " : ");
            IR = this.fetchByte();
            opcode = IR & 0xFF;     // convert to positive integer
            R.val = (byte)((R.val + 1) | (R.val & 0x80));
            tstates += iset[opcode].execute();
        }
    }
}

final class CB_exec extends Instruction {
    public int execute() {
        int opcode;
        //System.out.print(Integer.toHexString(cpu.PC) + ":");
        cpu.IR = cpu.fetchByte();
        opcode = cpu.IR & 0xFF;
        //System.out.println(Integer.toHexString(opcode));
        cpu.R.val = (byte)((cpu.R.val + 1) | (cpu.R.val & 0x80));
        return cpu.iset2[opcode].execute() + 4;
    }
}

final class DD_exec extends Instruction {
    public int execute() {
        int opcode;
        //System.out.print(Integer.toHexString(cpu.PC) + ":");
        cpu.IR = cpu.fetchByte();
        opcode = cpu.IR & 0xFF;
        //System.out.println(Integer.toHexString(opcode));
        cpu.R.val = (byte)((cpu.R.val + 1) | (cpu.R.val & 0x80));
        return cpu.iset3[opcode].execute() + 4;
    }
}

final class FD_exec extends Instruction {
    public int execute() {
        int opcode;
        //System.out.print(Integer.toHexString(cpu.PC) + ":");
        cpu.IR = cpu.fetchByte();
        opcode = cpu.IR & 0xFF;
        //System.out.println(Integer.toHexString(opcode));
        cpu.R.val = (byte)((cpu.R.val + 1) | (cpu.R.val & 0x80));
        return cpu.iset4[opcode].execute() + 4;
    }
}

final class ED_exec extends Instruction {
    public int execute() {
        int opcode;
        //System.out.print(Integer.toHexString(cpu.PC) + ":");
        cpu.IR = cpu.fetchByte();
        opcode = cpu.IR & 0xFF;
        //System.out.println(Integer.toHexString(opcode));
        cpu.R.val = (byte)((cpu.R.val + 1) | (cpu.R.val & 0x80));
        return cpu.iset5[opcode].execute() + 4;
    }
}
final class DDCB_exec extends Instruction {
    public int execute() {
        cpu.IX.setDsp();
        int opcode;
        //System.out.print(Integer.toHexString(cpu.PC) + ":");
        cpu.IR = cpu.fetchByte();
        opcode = cpu.IR & 0xFF;
        //System.out.println(Integer.toHexString(opcode));
        return cpu.iset6[opcode].execute() + 4;
    }
}

final class FDCB_exec extends Instruction {
    public int execute() {
        cpu.IY.setDsp();
        int opcode;
        //System.out.print(Integer.toHexString(cpu.PC) + ":");
        cpu.IR = cpu.fetchByte();
        opcode = cpu.IR & 0xFF;
        //System.out.println(Integer.toHexString(opcode));
        return cpu.iset7[opcode].execute() + 4;
    }
}

/* NOP - no operation */
final class NOP extends Instruction {
    public int execute() {return 4;}
}

/* LD r,r - load the first register with the contents of the second */
final class LD_r_r extends Instruction {
    private Register r1, r2;

    LD_r_r(Register r1, Register r2) {
        this.r1 = r1;
        this.r2 = r2;
    }
    public int execute() {
        r1.val = r2.val;
        return 4;
    }
}

/* LD r,(rp) - load register from location addressed by register pair */
class LD_r_rpAdr extends Instruction {
    private Register r;
    protected RegisterPair rp;

    LD_r_rpAdr(Register r, RegisterPair rp) {
        this.r = r;
        this.rp = rp;
    }
    public int execute() {
        r.val = rp.readByte();
        return 7;
    }
}

/* LD r,(rp+dd) - load register from indexed location */
final class LD_r_idxAdr extends LD_r_rpAdr {
    LD_r_idxAdr(Register r, IndexRegister rp) {
        super(r, rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* LD (rp),r - load location addressed by register pair from register */
class LD_rpAdr_r extends Instruction {
    private Register r;
    protected RegisterPair rp;

    LD_rpAdr_r(RegisterPair rp, Register r) {
        this.r = r;
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte(r.val);
        return 7;
    }
}

/* LD (rp+dd),r - load indexed location from register */
final class LD_idxAdr_r extends LD_rpAdr_r {
    LD_idxAdr_r(IndexRegister rp, Register r) {
        super(rp, r);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* LD (rp),n - load location addressed by register pair with immediate byte */
class LD_rpAdr_n extends Instruction {
    protected RegisterPair rp;

    LD_rpAdr_n(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte(cpu.fetchByte());
        return 7;
    }
}

/* LD (rp+dd),n - load indexed location with immediate byte */
final class LD_idxAdr_n extends LD_rpAdr_n {
    LD_idxAdr_n(IndexRegister rp) {
        super(rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* LD (adr),rp - store register pair in memory using direct addressing */
final class LD_adr_rp extends Instruction {
    private WordRegister rp;

    LD_adr_rp(WordRegister rp) {
        this.rp = rp;
    }
    public int execute() {
        ram.writeWord(cpu.fetchWord(), rp.get());
        return 16;
    }
}

/* LD (adr),r - store register in memory using direct addressing */
final class LD_adr_r extends Instruction {
    private Register r;

    LD_adr_r(Register r) {
        this.r = r;
    }
    public int execute() {
        ram.writeByte(cpu.fetchWord(), r.val);
        return 13;
    }
}

/* LD rp,(adr) - load register pair from memory using direct addressing */
final class LD_rp_adr extends Instruction {
    private WordRegister rp;

    LD_rp_adr(WordRegister rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.set(ram.readWord(cpu.fetchWord()));
        return 16;
    }
}

/* LD SP,rp - load stack pointer from register pair */
final class LD_SP_rp extends Instruction {
    private RegisterPair rp;

    LD_SP_rp(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        cpu.SP.val = rp.get();
        return 6;
    }
}

/* LD r,(adr) - load register from memory using direct addressing */
final class LD_r_adr extends Instruction {
    private Register r;

    LD_r_adr(Register r) {
        this.r = r;
    }
    public int execute() {
        r.val = ram.readByte(cpu.fetchWord());
        return 13;
    }
}

/* LD r,n - load register with immediate byte */
final class LD_r_n extends Instruction {
    private Register r;

    LD_r_n(Register r) {
        this.r = r;
    }
    public int execute() {
        r.val = cpu.fetchByte();
        return 7;
    }
}

/* LD rp,nn - load register pair with immediate word */
final class LD_rp_nn extends Instruction {
    private WordRegister rp;

    LD_rp_nn(WordRegister rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.set(cpu.fetchWord());
        return 10;
    }
}

/* EX rp,rp' - exchange register pair with its alternative */
final class EX_rp extends Instruction {
    private RegisterPair rp;

    EX_rp(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.swap();
        return 4;
    }
}

/* EX rp,rp - exchnage two register pairs */
final class EX_rp_rp extends Instruction {
    private RegisterPair rp1, rp2;

    EX_rp_rp(RegisterPair rp1, RegisterPair rp2) {
        this.rp1 = rp1;
        this.rp2 = rp2;
    }
    public int execute() {
        short temp = rp1.get();
        rp1.set(rp2.get());
        rp2.set(temp);
        return 4;
    }
}

/* EXX - exchange BC,DE, and HL with their alternatives */
final class EXX extends Instruction {
    public int execute() {
        cpu.BC.swap();
        cpu.DE.swap();
        cpu.HL.swap();
        return 4;
    }
}

/* INC rp - increment register pair contents */
final class INC_rp extends Instruction {
    private WordRegister rp;

    INC_rp(WordRegister rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.set((short)(rp.get()+1));
        return 6;
    }
}

/* DEC rp - decrement register pair contents */
final class DEC_rp extends Instruction {
    private WordRegister rp;

    DEC_rp(WordRegister rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.set((short)(rp.get()-1));
        return 6;
    }
}

/* JP adr - jump to direct address */
final class JP extends Instruction {
    public int execute() {
        cpu.PC = cpu.fetchWord();
        return 10;
    }
}

/* JP (rp) - jump to location addressed by register pair */
final class JP_rpAdr extends Instruction {  // tested
    private RegisterPair rp;

    JP_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        cpu.PC = rp.get();
        return 4;
    }
}

/* JP cond,adr - jump to direct address if flag set (+ve condition) */
final class JP_pCond extends Instruction {
    private Flag flag;

    JP_pCond(Flag flag) {
        this.flag = flag;
    }
    public int execute() {
        short adr = cpu.fetchWord();
        if (flag.test()) {
            cpu.PC = adr;
        }
        return 10;
    }
}

/* JP cond,adr - jump to direct address if flag clear (-ve condition) */
final class JP_nCond extends Instruction {
    private Flag flag;

    JP_nCond(Flag flag) {
        this.flag = flag;
    }
    public int execute() {
        short adr = cpu.fetchWord();
        if (!flag.test()) {
            cpu.PC = adr;
        }
        return 10;
    }
}

/* JR dd - jump relative to PC by signed displacement byte */
final class JR extends Instruction {
    public int execute() {
        byte dd = cpu.fetchByte();
        cpu.PC += dd;
        return 12;
    }
}

/* JR cond,dd - jump relative if flag set (+ve condition) */
final class JR_pCond extends Instruction {
    private Flag flag;

    JR_pCond(Flag flag) {
        this.flag = flag;
    }
    public int execute() {
        byte dd = cpu.fetchByte();
        if (flag.test()) {
            cpu.PC += dd;
            return 12;
        }
        return 7;
    }
}

/* JR cond,dd - jump relative if flag clear (-ve condition) */
final class JR_nCond extends Instruction {
    private Flag flag;

    JR_nCond(Flag flag) {
        this.flag = flag;
    }
    public int execute() {
        byte dd = cpu.fetchByte();
        if (!flag.test()) {
            cpu.PC += dd;
            return 12;
        }
        return 7;
    }
}

/* DJNZ dd - Decerement B and then jump relative if not zero */
final class DJNZ extends Instruction {  // need to test this!
    public int execute() {
        byte dd = cpu.fetchByte();
        if (--cpu.B.val != 0) {
            cpu.PC += dd;
            return 13;
        }
        return 8;
    }
}

/* PUSH rp - Push register pair onto the stack */
final class PUSH extends Instruction {
    private RegisterPair rp;

    PUSH(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        cpu.push(rp.get());
        return 11;
    }
}

/* POP rp - Pop register pair off the stack */
final class POP extends Instruction {
    private RegisterPair rp;

    POP(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.set(cpu.pop());
        return 10;
    }
}

/* RST n - call low memory address specified by n */
final class RST extends Instruction {
    private short adr;

    RST(int adr) {
        this.adr = (short)adr;
    }
    public int execute() {
        cpu.push(cpu.PC);
        cpu.PC = adr;
        return 11;
    }
}

/* CALL adr - call direct memory address */
final class CALL extends Instruction {
    public int execute() {
        cpu.push((short)(cpu.PC+2));
        cpu.PC = cpu.fetchWord();
        return 17;
    }
}

/* CALL cond,adr - call direct address if flag set (+ve condition) */
final class CALL_pCond extends Instruction {
    private Flag flag;

    CALL_pCond(Flag flag) {
        this.flag = flag;
    }
    public int execute() {
        short adr = cpu.fetchWord();
        if (flag.test()) {
            cpu.push(cpu.PC);
            cpu.PC = adr;
            return 17;
        }
        return 10;
    }
}

/* CALL cond,adr - call direct address if flag clear (-ve condition) */
final class CALL_nCond extends Instruction {
    private Flag flag;

    CALL_nCond(Flag flag) {
        this.flag = flag;
    }
    public int execute() {
        short adr = cpu.fetchWord();
        if (!flag.test()) {
            cpu.push(cpu.PC);
            cpu.PC = adr;
            return 17;
        }
        return 10;
    }
}

/* RET - return to the address on top of stack */
final class RET extends Instruction {
    public int execute() {
        cpu.PC = cpu.pop();
        return 10;
    }
}

/* RET cond - return if flag set (+ve condition) */
final class RET_pCond extends Instruction {
    private Flag flag;

    RET_pCond(Flag flag) {
        this.flag = flag;
    }
    public int execute() {
        if (flag.test()) {
            cpu.PC = cpu.pop();
            return 11;
        }
        return 5;
    }
}

/* RET cond - return if flag clear (-ve condition) */
final class RET_nCond extends Instruction {
    private Flag flag;

    RET_nCond(Flag flag) {
        this.flag = flag;
    }
    public int execute() {
        if (!flag.test()) {
            cpu.PC = cpu.pop();
            return 11;
        }
        return 5;
    }
}

/* CCF - complement carry flag */
final class CCF extends Instruction {
    public int execute() {
        //cpu.F.val = (byte)(cpu.F.val ^ 1);
        cpu.H_flag.set(cpu.C_flag.test());
        cpu.N_flag.clear();
        cpu.C_flag.set(!cpu.C_flag.test());
        return 4;
    }
}

/* SCF - set carry flag */
final class SCF extends Instruction {
    public int execute() {
        cpu.C_flag.set();
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        return 4;
    }
}

/* CPL - complement the accumulator */
final class CPL extends Instruction {
    public int execute() {
        cpu.A.val = (byte)~cpu.A.val;
        cpu.H_flag.set();
        cpu.N_flag.set();
        return 4;
    }
}

/* SET b,r - set specified bit of register */
final class SET_b_r extends Instruction {
    private byte mask;
    private Register r;

    SET_b_r(int bit, Register r) {
        mask = (byte)(Math.pow(2, bit));
        this.r = r;
    }
    public int execute() {
        r.val = (byte)(r.val | mask);
        return 4;
    }
}

/* SET b,(rp) - set specified bit of location addressed by register pair */
final class SET_b_rpAdr extends Instruction {
    private byte mask;
    private RegisterPair rp;

    SET_b_rpAdr(int bit, RegisterPair rp) {
        mask = (byte)(Math.pow(2, bit));
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte((byte)(rp.readByte() | mask));
        return 11;
    }
}

/* RES b,r - reset specfied bit of register */
final class RES_b_r extends Instruction {
    private byte mask;
    private Register r;

    RES_b_r(int bit, Register r) {
        mask = (byte)(0xFF - Math.pow(2, bit));
        this.r = r;
    }
    public int execute() {
        r.val = (byte)(r.val & mask);
        return 4;
    }
}

/* RES b,(rp) - reset specified bit of location addressed by register pair */
final class RES_b_rpAdr extends Instruction {
    private byte mask;
    private RegisterPair rp;

    RES_b_rpAdr(int bit, RegisterPair rp) {
        mask = (byte)(0xFF - Math.pow(2, bit));
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte((byte)(rp.readByte() & mask));
        return 11;
    }
}

/* BIT b,r - test specfied bit of register */
final class BIT_b_r extends Instruction {
    private byte mask;
    private Register r;

    BIT_b_r(int bit, Register r) {
        mask = (byte)(Math.pow(2, bit));
        this.r = r;
    }
    public int execute() {
        cpu.Z_flag.set((r.val & mask) == 0);
        cpu.N_flag.clear();
        cpu.H_flag.set();
        return 4;
    }
}

/* BIT b,(rp) - test specfied bit of location addressed by register pair */
final class BIT_b_rpAdr extends Instruction {
    private byte mask;
    private RegisterPair rp;

    BIT_b_rpAdr(int bit, RegisterPair rp) {
        mask = (byte)(Math.pow(2, bit));
        this.rp = rp;
    }
    public int execute() {
        cpu.Z_flag.set((rp.readByte() & mask) == 0);
        cpu.N_flag.clear();
        cpu.H_flag.set();
        return 4;
    }
}

/* SLA r - shift register left */
final class SLA_r extends Instruction {
    private Register r;

    SLA_r(Register r) {
        this.r = r;
    }
    static byte sla(byte val) {
        cpu.C_flag.set((val & 0x80) != 0);
        val = (byte)(val << 1);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(val < 0);
        cpu.Z_flag.set(val == 0);
        cpu.PV_flag.set(cpu.parity[val & 0xFF]);
        return val;
    }
    public int execute() {
        r.val = sla(r.val);
        return 4;
    }
}

/* SLA (rp) - shift byte addressed by register pair left */
final class SLA_rpAdr extends Instruction {
    private RegisterPair rp;

    SLA_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte(SLA_r.sla(rp.readByte()));
        return 11;
    }
}

/* SLL r - shift register left logically (Undocumented) */
final class SLL_r extends Instruction {
    private Register r;

    SLL_r(Register r) {
        this.r = r;
    }
    static byte sll(byte val) {
        cpu.C_flag.set((val & 0x80) != 0);
        val = (byte)((val << 1) | 0x01);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(val < 0);
        cpu.Z_flag.set(val == 0);
        cpu.PV_flag.set(cpu.parity[val & 0xFF]);
        return val;
    }
    public int execute() {
        r.val = sll(r.val);
        return 4;
    }
}

/* SLL (rp) - shift byte addressed by register pair left logically */
final class SLL_rpAdr extends Instruction {
    private RegisterPair rp;

    SLL_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte(SLL_r.sll(rp.readByte()));
        return 11;
    }
}

/* SRA r - shift register right arithmetically (preserve sign bit) */
final class SRA_r extends Instruction {
    private Register r;

    SRA_r(Register r) {
        this.r = r;
    }
    static byte sra(byte val) {
        cpu.C_flag.set((val & 0x01) != 0);
        val = (byte)(val >> 1);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(val < 0);
        cpu.Z_flag.set(val == 0);
        cpu.PV_flag.set(cpu.parity[val & 0xFF]);
        return val;
    }
    public int execute() {
        r.val = sra(r.val);
        return 4;
    }
}

/* SRA (rp) - shift byte addressed by register pair right arithmetically  */
final class SRA_rpAdr extends Instruction {
    private RegisterPair rp;

    SRA_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte(SRA_r.sra(rp.readByte()));
        return 11;
    }
}

/* SRL r - shift register right */
final class SRL_r extends Instruction {
    private Register r;

    SRL_r(Register r) {
        this.r = r;
    }
    static byte srl(byte val) {
        cpu.C_flag.set((val & 0x01) != 0);
        val = (byte)((val >> 1) & 0x7F);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.clear();
        cpu.Z_flag.set(val == 0);
        cpu.PV_flag.set(cpu.parity[val & 0xFF]);
        return val;
    }
    public int execute() {
        r.val = srl(r.val);
        return 4;
    }
}

/* SRL (rp) - shift byte addressed by register pair right */
final class SRL_rpAdr extends Instruction {
    private RegisterPair rp;

    SRL_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte(SRL_r.srl(rp.readByte()));
        return 11;
    }
}

/* RL r - rotate register left through carry */
final class RL_r extends Instruction {
    private Register r;

    RL_r(Register r) {
        this.r = r;
    }
    static byte rl(byte val) {
        byte old = val;
        val = (byte)(old << 1);
        if (cpu.C_flag.test()) {
            val = (byte)(val | 0x01);
        }
        cpu.C_flag.set((old & 0x80) != 0);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(val < 0);
        cpu.Z_flag.set(val == 0);
        cpu.PV_flag.set(cpu.parity[val & 0xFF]);
        return val;
    }
    public int execute() {
        r.val = rl(r.val);
        return 4;
    }
}

/* RL (rp) - rotate byte addressed by register pair left through carry */
final class RL_rpAdr extends Instruction {
    private RegisterPair rp;

    RL_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte(RL_r.rl(rp.readByte()));
        return 11;
    }
}

/* RLA - rotate accumulator left through carry */
final class RLA extends Instruction {
    public int execute() {
        byte old = cpu.A.val;
        cpu.A.val = (byte)(old << 1);
        if (cpu.C_flag.test()) {
            cpu.A.val = (byte)(cpu.A.val | 0x01);
        }
        cpu.C_flag.set((old & 0x80) != 0);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        return 4;
    }
}

/* RLC r - rotate register left with branch carry */
final class RLC_r extends Instruction {
    private Register r;

    RLC_r(Register r) {
        this.r = r;
    }
    static byte rlc(byte val) {
        byte old = val;
        val = (byte)(old << 1);
        val = (byte)(val | ((old & 0x80) >>> 7));
        cpu.C_flag.set((old & 0x80) != 0);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(val < 0);
        cpu.Z_flag.set(val == 0);
        cpu.PV_flag.set(cpu.parity[val & 0xFF]);
        return val;
    }
    public int execute() {
        r.val = rlc(r.val);
        return 4;
    }
}

/* RLC (rp) - rotate byte addressed by register pair left with branch carry */
final class RLC_rpAdr extends Instruction {
    private RegisterPair rp;

    RLC_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte(RLC_r.rlc(rp.readByte()));
        return 11;
    }
}

/* RLCA - rotate accumulator left with branch carry */
final class RLCA extends Instruction {
    public int execute() {
        byte old = cpu.A.val;
        cpu.A.val = (byte)(old << 1);
        cpu.A.val = (byte)(cpu.A.val | ((old & 0x80) >>> 7));
        cpu.C_flag.set((old & 0x80) != 0);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        return 4;
    }
}

/* RR r - rotate register right through carry */
final class RR_r extends Instruction {
    private Register r;

    RR_r(Register r) {
        this.r = r;
    }
    static byte rr(byte val) {
        byte old = val;
        val = (byte)(old >>> 1);
        if (cpu.C_flag.test()) {
            val = (byte)(val | 0x80);
        } else {
            val = (byte)(val & 0x7F);
        }
        cpu.C_flag.set((old & 0x01) != 0);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(val < 0);
        cpu.Z_flag.set(val == 0);
        cpu.PV_flag.set(cpu.parity[val & 0xFF]);
        return val;
    }
    public int execute() {
        r.val = rr(r.val);
        return 4;
    }
}

/* RR (rp) - rotate byte addressed by register pair right */
final class RR_rpAdr extends Instruction {
    private RegisterPair rp;

    RR_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte(RR_r.rr(rp.readByte()));
        return 11;
    }
}

/* RRA - rotate accumulator right through carry */
final class RRA extends Instruction {
    public int execute() {
        byte old = cpu.A.val;
        cpu.A.val = (byte)(old >>> 1);
        if (cpu.C_flag.test()) {
            cpu.A.val = (byte)(cpu.A.val | 0x80);
        } else {
            cpu.A.val = (byte)(cpu.A.val & 0x7F);
        }
        cpu.C_flag.set((old & 0x01) != 0);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        return 4;
    }
}

/* RRC r - rotate register right with branch carry */
final class RRC_r extends Instruction {
    private Register r;

    RRC_r(Register r) {
        this.r = r;
    }
    static byte rrc(byte val) {
        byte old = val;
        val = (byte)(old >>> 1);
        val = (byte)((val & 0x7F) | ((old & 0x01) << 7));
        cpu.C_flag.set((old & 0x01) != 0);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(val < 0);
        cpu.Z_flag.set(val == 0);
        cpu.PV_flag.set(cpu.parity[val & 0xFF]);
        return val;
    }
    public int execute() {
        r.val = rrc(r.val);
        return 4;
    }
}

/* RRC (rp) - rotate byte addressed by register pair right with branch */
final class RRC_rpAdr extends Instruction {
    private RegisterPair rp;

    RRC_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        rp.writeByte(RRC_r.rrc(rp.readByte()));
        return 11;
    }
}

/* RRCA - rotate accumulator right with branch carry */
final class RRCA extends Instruction {
    public int execute() {
        byte old = cpu.A.val;
        cpu.A.val = (byte)(old >>> 1);
        cpu.A.val = (byte)((cpu.A.val & 0x7F) | ((old & 0x01) << 7));
        cpu.C_flag.set((old & 0x01) != 0);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        return 4;
    }
}

/* RLD - rotate one BCD digit left between A and (HL) */
final class RLD extends Instruction {
    public int execute() {
        byte old = cpu.HL.readByte();
        cpu.HL.writeByte((byte)((old << 4) | (cpu.A.val & 0x0F)));
        cpu.A.val = (byte)((cpu.A.val & 0xF0) | ((old >> 4) & 0x0F));
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(cpu.A.val < 0);
        cpu.Z_flag.set(cpu.A.val == 0);
        cpu.PV_flag.set(cpu.parity[cpu.A.val & 0xFF]);
        return 14;
    }
}

/* RRD - rotate one BCD digit right between A and (HL) */
final class RRD extends Instruction {
    public int execute() {
        byte old = cpu.HL.readByte();
        cpu.HL.writeByte((byte)(((old >> 4) & 0x0F) | (cpu.A.val << 4)));
        cpu.A.val = (byte)((cpu.A.val & 0xF0) | (old & 0x0F));
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(cpu.A.val < 0);
        cpu.Z_flag.set(cpu.A.val == 0);
        cpu.PV_flag.set(cpu.parity[cpu.A.val & 0xFF]);
        return 14;
    }
}

/* AND n - AND the accumulator with immediate byte */
class AND_n extends Instruction {
    public void and(byte value) {
        cpu.A.val = (byte)(cpu.A.val & value);
        cpu.C_flag.clear();
        cpu.N_flag.clear();
        cpu.H_flag.set();
        cpu.S_flag.set(cpu.A.val < 0);
        cpu.Z_flag.set(cpu.A.val == 0);
        cpu.PV_flag.set(cpu.parity[cpu.A.val & 0xFF]);
    }
    public int execute() {
        and(cpu.fetchByte());
        return 7;
     }
}

/* AND r - AND register with accumulator */
final class AND_r extends AND_n {
    private Register r;

    AND_r(Register r) {
        this.r = r;
    }
    public int execute() {
        and(r.val);
        return 4;
     }
}

/* AND (rp) - AND byte addressed by register pair with accumulator */
class AND_rpAdr extends AND_n {
    protected RegisterPair rp;

    AND_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        and(rp.readByte());
        return 7;
    }
}

/* AND (rp+dd) - AND indexed byte with the accumulator */
final class AND_idxAdr extends AND_rpAdr {
    AND_idxAdr(IndexRegister rp) {
        super(rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* OR n - OR the accumulator with immediate byte */
class OR_n extends Instruction {
    public void or(byte value) {
        cpu.A.val = (byte)(cpu.A.val | value);
        cpu.C_flag.clear();
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(cpu.A.val < 0);
        cpu.Z_flag.set(cpu.A.val == 0);
        cpu.PV_flag.set(cpu.parity[cpu.A.val & 0xFF]);
    }
    public int execute() {
        or(cpu.fetchByte());
        return 7;
     }
}

/* OR r - OR register with accumulator */
final class OR_r extends OR_n {
    private Register r;

    OR_r(Register r) {
        this.r = r;
    }
    public int execute() {
        or(r.val);
        return 4;
     }
}

/* OR (rp) - OR byte addressed by register pair with accumulator */
class OR_rpAdr extends OR_n {
    protected RegisterPair rp;

    OR_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        or(rp.readByte());
        return 7;
    }
}

/* OR (rp+dd) - OR indexed byte with the accumulator */
final class OR_idxAdr extends OR_rpAdr {
    OR_idxAdr(IndexRegister rp) {
        super(rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* XOR n - XOR the accumulator with immediate byte */
class XOR_n extends Instruction {
    public void xor(byte value) {
        cpu.A.val = (byte)(cpu.A.val ^ value);
        cpu.C_flag.clear();
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(cpu.A.val < 0);
        cpu.Z_flag.set(cpu.A.val == 0);
        cpu.PV_flag.set(cpu.parity[cpu.A.val & 0xFF]);
    }
    public int execute() {
        xor(cpu.fetchByte());
        return 7;
     }
}

/* XOR r - XOR register with accumulator */
final class XOR_r extends XOR_n {
    private Register r;

    XOR_r(Register r) {
        this.r = r;
    }
    public int execute() {
        xor(r.val);
        return 4;
     }
}

/* XOR (rp) - XOR byte addressed by register pair with accumulator */
class XOR_rpAdr extends XOR_n {
    protected RegisterPair rp;

    XOR_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        xor(rp.readByte());
        return 7;
    }
}

/* XOR (rp+dd) - XOR indexed byte with the accumulator */
final class XOR_idxAdr extends XOR_rpAdr {
    XOR_idxAdr(IndexRegister rp) {
        super(rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* LDD - block load with decrement */
final class LDD extends Instruction {
    public int execute() {
        cpu.DE.writeByte(cpu.HL.readByte());
        cpu.BC.set((short)(cpu.BC.get()-1));
        cpu.DE.set((short)(cpu.DE.get()-1));
        cpu.HL.set((short)(cpu.HL.get()-1));
        cpu.H_flag.clear();
        cpu.N_flag.clear();
        cpu.PV_flag.set(cpu.BC.get() != 0);
        return 12;
    }
}

/* LDI - block load with increment */
final class LDI extends Instruction {
    public int execute() {
        cpu.DE.writeByte(cpu.HL.readByte());
        cpu.BC.set((short)(cpu.BC.get()-1));
        cpu.DE.set((short)(cpu.DE.get()+1));
        cpu.HL.set((short)(cpu.HL.get()+1));
        cpu.H_flag.clear();
        cpu.N_flag.clear();
        cpu.PV_flag.set(cpu.BC.get() != 0);
        return 12;
    }
}

/* LDDR - repeating block load with decrement */
final class LDDR extends Instruction {
    public int execute() {
        cpu.DE.writeByte(cpu.HL.readByte());
        cpu.BC.set((short)(cpu.BC.get()-1));
        cpu.DE.set((short)(cpu.DE.get()-1));
        cpu.HL.set((short)(cpu.HL.get()-1));
        cpu.H_flag.clear();
        cpu.N_flag.clear();
        cpu.PV_flag.clear();
        if (cpu.BC.get() != 0) {
            cpu.PC -= 2;
            return 17;
        }
        return 12;
    }
}

/* LDIR - repeating block load with increment */
final class LDIR extends Instruction {
    public int execute() {
        cpu.DE.writeByte(cpu.HL.readByte());
        cpu.BC.set((short)(cpu.BC.get()-1));
        cpu.DE.set((short)(cpu.DE.get()+1));
        cpu.HL.set((short)(cpu.HL.get()+1));
        cpu.H_flag.clear();
        cpu.N_flag.clear();
        cpu.PV_flag.clear();
        if (cpu.BC.get() != 0) {
            cpu.PC -= 2;
            return 17;
        }
        return 12;
    }
}

/* ADD A,n - add immediate byte to the accumulator */
class ADD_A_n extends Instruction {
    public byte add(byte old, byte val, boolean carry) {
        byte res = (byte)(old + val);
        if (carry && cpu.C_flag.test()) {
            res++;
        }
        byte test = (byte)(old & val | (old ^ val) & ~res);
        cpu.C_flag.set((test & 0x80) != 0);
        cpu.H_flag.set((test & 0x08) != 0);
        cpu.PV_flag.set(((test & 0x80) != 0) ^ ((test & 0x40) != 0));
        cpu.N_flag.clear();
        cpu.S_flag.set(res < 0);
        cpu.Z_flag.set(res == 0);
        return res;
    }

    public int execute() {
        cpu.A.val = add(cpu.A.val, cpu.fetchByte(), false);
        return 7;
     }
}

/* ADD A,r - add register to the accumulator */
final class ADD_A_r extends ADD_A_n {
    private Register r;

    ADD_A_r(Register r) {
        this.r = r;
    }
    public int execute() {
        cpu.A.val = add(cpu.A.val, r.val, false);
        return 4;
     }
}

/* ADD A,(rp) - add byte addressed by register pair to the accumulator */
class ADD_A_rpAdr extends ADD_A_n {
    protected RegisterPair rp;

    ADD_A_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        cpu.A.val = add(cpu.A.val, rp.readByte(), false);
        return 7;
    }
}

/* ADD A,(rp+dd) - add indexed byte to the accumulator */
final class ADD_A_idxAdr extends ADD_A_rpAdr {
    ADD_A_idxAdr(IndexRegister rp) {
        super(rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* ADC A,(rp) - add byte addressed by register pair to accumulator with carry */
class ADC_A_rpAdr extends ADD_A_n {
    protected RegisterPair rp;

    ADC_A_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        cpu.A.val = add(cpu.A.val, rp.readByte(), true);
        return 7;
    }
}

/* ADC A,(rp+dd) - add indexed byte to the accumulator with carry */
final class ADC_A_idxAdr extends ADC_A_rpAdr {
    ADC_A_idxAdr(IndexRegister rp) {
        super(rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* ADC A,r - add register to the accumulator with carry */
final class ADC_A_r extends ADD_A_n {
    private Register r;

    ADC_A_r(Register r) {
        this.r = r;
    }
    public int execute() {
        cpu.A.val = add(cpu.A.val, r.val, true);
        return 4;
     }
}

/* ADC A,n - add immediate byte to the accumulator with carry */
final class ADC_A_n extends ADD_A_n {
    public int execute() {
        cpu.A.val = add(cpu.A.val, cpu.fetchByte(), true);
        return 7;
     }
}

/* INC r - increment register by 1 */
final class INC_r extends ADD_A_n {
    private Register r;

    INC_r(Register r) {
        this.r = r;
    }
    public int execute() {
        boolean temp = cpu.C_flag.test();
        r.val = add(r.val, (byte)1, false);
        cpu.C_flag.set(temp);   // preserve carry flag
        return 4;
     }
}

/* INC (rp) - increment addressed byte by 1 */
class INC_rpAdr extends ADD_A_n {
    protected RegisterPair rp;

    INC_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        boolean temp = cpu.C_flag.test();
        rp.writeByte(add(rp.readByte(), (byte)1, false));
        cpu.C_flag.set(temp);   // preserve carry flag
        return 11;
     }
}

/* INC (rp+dd) - increment indexed byte by 1 */
final class INC_idxAdr extends INC_rpAdr {
    INC_idxAdr(IndexRegister rp) {
        super(rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* ADD rp,rp - add the second register pair to the first */
class ADD_rp_rp extends Instruction {
    protected WordRegister rp1, rp2;

    public short add(short old, short val, boolean carry) {
        short res = (short)(old + val);
        if (carry && cpu.C_flag.test()) {
            res++;
        }
        short test = (short)(old & val | (old ^ val) & ~res);
        cpu.C_flag.set((test & 0x8000) != 0);
        cpu.H_flag.set((test & 0x0800) != 0);
        cpu.PV_flag.set(((test & 0x8000) != 0) ^ ((test & 0x4000) != 0));
        cpu.N_flag.clear();
        cpu.S_flag.set(res < 0);
        cpu.Z_flag.set(res == 0);
        return res;
    }

    ADD_rp_rp(WordRegister rp1, WordRegister rp2) {
        this.rp1 = rp1;
        this.rp2 = rp2;
    }
    public int execute() {
        rp1.set(add(rp1.get(), rp2.get(), false));
        return 11;
     }
}

/* ADC rp,rp - add the second register pair to the first with carry */
final class ADC_rp_rp extends ADD_rp_rp {
    ADC_rp_rp(WordRegister rp1, WordRegister rp2) {
        super(rp1, rp2);
    }
    public int execute() {
        rp1.set(add(rp1.get(), rp2.get(), true));
        return 11;
     }
}

/* SUB A,n - subtract immediate byte from the accumulator */
class SUB_A_n extends Instruction {
    public byte sub(byte old, byte val, boolean carry) {
        byte res = (byte)(old - val);
        if (carry && cpu.C_flag.test()) {
            res--;
        }
        byte test = (byte)(~old & val | ~(old ^ val) & res);
        cpu.C_flag.set((test & 0x80) != 0);
        cpu.H_flag.set((test & 0x08) != 0);
        cpu.PV_flag.set(((test & 0x80) != 0) ^ ((test & 0x40) != 0));
        cpu.N_flag.set();
        cpu.S_flag.set(res < 0);
        cpu.Z_flag.set(res == 0);
        return res;
    }

    public int execute() {
        cpu.A.val = sub(cpu.A.val, cpu.fetchByte(), false);
        return 7;
     }
}

/* SUB A,r - subtract register from the accumulator */
final class SUB_A_r extends SUB_A_n {
    private Register r;

    SUB_A_r(Register r) {
        this.r = r;
    }
    public int execute() {
        cpu.A.val = sub(cpu.A.val, r.val, false);
        return 4;
     }
}

/* SUB A,(rp) - subract addressed byte from the accumulator */
class SUB_A_rpAdr extends SUB_A_n {
    protected RegisterPair rp;

    SUB_A_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        cpu.A.val = sub(cpu.A.val, rp.readByte(), false);
        return 7;
    }
}

/* SUB A,(rp+dd) - subract indexed byte from the accumulator */
final class SUB_A_idxAdr extends SUB_A_rpAdr {
    SUB_A_idxAdr(IndexRegister rp) {
        super(rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* DEC r - decrement register by 1 */
final class DEC_r extends SUB_A_n {
    private Register r;

    DEC_r(Register r) {
        this.r = r;
    }
    public int execute() {
        boolean temp = cpu.C_flag.test();
        r.val = sub(r.val, (byte)1, false);
        cpu.C_flag.set(temp);   // preserve carry flag
        return 4;
     }
}

/* DEC (rp) - decrement addressed byte by 1 */
class DEC_rpAdr extends SUB_A_n {
    protected RegisterPair rp;

    DEC_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        boolean temp = cpu.C_flag.test();
        rp.writeByte(sub(rp.readByte(), (byte)1, false));
        cpu.C_flag.set(temp);   // preserve carry flag
        return 11;
     }
}

/* DEC (rp+dd) - decrement indexed byte by 1 */
final class DEC_idxAdr extends DEC_rpAdr {
    DEC_idxAdr(IndexRegister rp) {
        super(rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* SBC A,n - subtract immediate byte from the accumulator with carry */
final class SBC_A_n extends SUB_A_n {
    public int execute() {
        cpu.A.val = sub(cpu.A.val, cpu.fetchByte(), true);
        return 7;
     }
}

/* SBC A,r - subtract register from the accumulator with carry */
final class SBC_A_r extends SUB_A_n {
    private Register r;

    SBC_A_r(Register r) {
        this.r = r;
    }
    public int execute() {
        cpu.A.val = sub(cpu.A.val, r.val, true);
        return 4;
     }
}

/* SBC A,(rp) - subract addressed byte from the accumulator with carry */
class SBC_A_rpAdr extends SUB_A_n {
    protected RegisterPair rp;

    SBC_A_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        cpu.A.val = sub(cpu.A.val, rp.readByte(), true);
        return 7;
    }
}

/* SBC A,(rp+dd) - subract indexed byte from the accumulator with carry */
final class SBC_A_idxAdr extends SBC_A_rpAdr {
    SBC_A_idxAdr(IndexRegister rp) {
        super(rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* SBC HL,rp - subtract register pair from the HL with carry */
class SBC_HL_rp extends Instruction {
    private WordRegister rp;

    SBC_HL_rp(WordRegister rp) {
        this.rp = rp;
    }
    public int execute() {
        short old = cpu.HL.get();
        short val = rp.get();
        short res = (short)(old - val);
        if (cpu.C_flag.test()) {
            res--;
        }
        cpu.HL.set(res);
        short test = (short)(~old & val | ~(old ^ val) & res);
        cpu.C_flag.set((test & 0x8000) != 0);
        cpu.H_flag.set((test & 0x0800) != 0);
        cpu.PV_flag.set(((test & 0x8000) != 0) ^ ((test & 0x4000) != 0));
        cpu.N_flag.set();
        cpu.S_flag.set(res < 0);
        cpu.Z_flag.set(res == 0);
        return 11;
    }
}

/* CP r - compare register with the accumulator */
final class CP_r extends SUB_A_n {
    private Register r;

    CP_r(Register r) {
        this.r = r;
    }
    public int execute() {
        sub(cpu.A.val, r.val, false);
        return 4;
     }
}

/* CP n - compare immediate byte with the accumulator */
final class CP_n extends SUB_A_n {
    public int execute() {
        sub(cpu.A.val, cpu.fetchByte(), false);
        return 7;
     }
}

/* CP (rp) - comapre addressed byte with the accumulator */
class CP_rpAdr extends SUB_A_n {
    protected RegisterPair rp;

    CP_rpAdr(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        sub(cpu.A.val, rp.readByte(), false);
        return 7;
    }
}

/* CP (rp+dd) - compare indexed byte with the accumulator */
final class CP_idxAdr extends CP_rpAdr {
    CP_idxAdr(IndexRegister rp) {
        super(rp);
    }
    public int execute() {
        ((IndexRegister)rp).setDsp();
        return super.execute() + 8;
    }
}

/* NEG - negate accumulator (A = 0 - A) */
final class NEG extends SUB_A_n {
    public int execute() {
        cpu.A.val = sub((byte)0, cpu.A.val, false);
        return 4;
    }
}

/* CPD - compare with decrement */
class CPD extends Instruction {
    public int execute() {
        byte val = cpu.HL.readByte();
        byte res = (byte)(cpu.A.val - val);
        byte test = (byte)(~cpu.A.val & val | ~(cpu.A.val ^ val) & res);
        cpu.HL.set((short)(cpu.HL.get()-1));
        cpu.BC.set((short)(cpu.BC.get()-1));
        cpu.H_flag.set((test & 0x08) != 0);
        cpu.N_flag.set();
        cpu.S_flag.set(res < 0);
        cpu.PV_flag.set(cpu.BC.get() != 0);
        cpu.Z_flag.set(cpu.A.val == val);
        return 12;
    }
}

/* CPDR - block compare with decrement */
final class CPDR extends CPD {
    public int execute() {
        super.execute();
        if (cpu.PV_flag.test() && !cpu.Z_flag.test()) {
            cpu.PC -= 2;
            return 17;
        }
        return 12;
    }
}

/* CPI - compare with increment */
class CPI extends Instruction {
    public int execute() {
        byte val = cpu.HL.readByte();
        byte res = (byte)(cpu.A.val - val);
        byte test = (byte)(~cpu.A.val & val | ~(cpu.A.val ^ val) & res);
        cpu.HL.set((short)(cpu.HL.get()+1));
        cpu.BC.set((short)(cpu.BC.get()-1));
        cpu.H_flag.set((test & 0x08) != 0);
        cpu.N_flag.set();
        cpu.S_flag.set(res < 0);
        cpu.PV_flag.set(cpu.BC.get() != 0);
        cpu.Z_flag.set(cpu.A.val == val);
        return 12;
    }
}

/* CPIR - block compare with increment */
final class CPIR extends CPI {
    public int execute() {
        super.execute();
        if (cpu.PV_flag.test() && !cpu.Z_flag.test()) {
            cpu.PC -= 2;
            return 17;
        }
        return 12;
    }
}

/* EX (SP),rp - exchange contents of register pair and top of stack */
final class EX_SP_rp extends Instruction {
    private RegisterPair rp;

    EX_SP_rp(RegisterPair rp) {
        this.rp = rp;
    }
    public int execute() {
        short temp = ram.readWord(cpu.SP.val);
        ram.writeWord(cpu.SP.val, rp.get());
        rp.set(temp);
        return 19;
    }
}

/* IN r,(C) - load register from I/O port identified by register C */
final class IN_r_C extends Instruction {
    private Register r;

    IN_r_C(Register r) {
        this.r = r;
    }
    public int execute() {
        r.val = cpu.in.readPort(cpu.C.val, cpu.B.val);
        cpu.N_flag.clear();
        cpu.H_flag.clear();
        cpu.S_flag.set(r.val < 0);
        cpu.Z_flag.set(r.val == 0);
        cpu.PV_flag.set(cpu.parity[r.val & 0xFF]);
        return 8;
    }
}

/* IN A,(n) - load A from I/O port identified by immediate byte */
final class IN_A_n extends Instruction {
    public int execute() {
        cpu.A.val = cpu.in.readPort(cpu.fetchByte(), cpu.A.val);
        return 11;
    }
}

/* IND - input with decrement */
final class IND extends Instruction {
    public int execute() {
        cpu.HL.writeByte(cpu.in.readPort(cpu.C.val, cpu.B.val));
        cpu.B.val--;    // ?
        cpu.HL.set((short)(cpu.HL.get()-1));
        cpu.N_flag.set();
        cpu.Z_flag.set(cpu.B.val == 0);
        return 12;
    }
}

/* INDR - block input with decrement */
final class INDR extends Instruction {
    public int execute() {
        cpu.HL.writeByte(cpu.in.readPort(cpu.C.val, cpu.B.val));
        cpu.B.val--;    // ?
        cpu.HL.set((short)(cpu.HL.get()-1));
        cpu.N_flag.set();
        cpu.Z_flag.set();
        if (cpu.B.val != 0) {
            cpu.PC -= 2;
            return 17;
        }
        return 12;
    }
}

/* INI - input with increment */
final class INI extends Instruction {
    public int execute() {
        cpu.HL.writeByte(cpu.in.readPort(cpu.C.val, cpu.B.val));
        cpu.B.val--;    // ?
        cpu.HL.set((short)(cpu.HL.get()+1));
        cpu.N_flag.set();
        cpu.Z_flag.set(cpu.B.val == 0);
        return 12;
    }
}

/* INIR - block input with increment */
final class INIR extends Instruction {
    public int execute() {
        cpu.HL.writeByte(cpu.in.readPort(cpu.C.val, cpu.B.val));
        cpu.B.val--;    // ?
        cpu.HL.set((short)(cpu.HL.get()+1));
        cpu.N_flag.set();
        cpu.Z_flag.set();
        if (cpu.B.val != 0) {
            cpu.PC -= 2;
            return 17;
        }
        return 12;
    }
}

/* OUT (C),r - output register to I/O port identified by register C */
final class OUT_C_r extends Instruction {
    private Register r;

    OUT_C_r(Register r) {
        this.r = r;
    }
    public int execute() {
        cpu.out.writePort(cpu.C.val, cpu.B.val, r.val);
        return 8;
    }
}

/* OUT (n),r - output A to I/O port identified by immediate byte */
final class OUT_n_A extends Instruction {
    public int execute() {
        cpu.out.writePort(cpu.fetchByte(), cpu.A.val, cpu.A.val);
        return 11;
    }
}

/* OUTD - output with decrement */
final class OUTD extends Instruction {
    public int execute() {
        cpu.out.writePort(cpu.C.val, cpu.B.val, cpu.HL.readByte());
        cpu.B.val--;    // ?
        cpu.HL.set((short)(cpu.HL.get()-1));
        cpu.N_flag.set();
        cpu.Z_flag.set(cpu.B.val == 0);
        return 12;
    }
}

/* OTDR - block output with decrement */
final class OTDR extends Instruction {
    public int execute() {
        cpu.out.writePort(cpu.C.val, cpu.B.val, cpu.HL.readByte());
        cpu.B.val--;    // ?
        cpu.HL.set((short)(cpu.HL.get()-1));
        cpu.N_flag.set();
        cpu.Z_flag.set();
        if (cpu.B.val != 0) {
            cpu.PC -= 2;
            return 17;
        }
        return 12;
    }
}

/* OUTI - output with increment */
final class OUTI extends Instruction {
    public int execute() {
        cpu.out.writePort(cpu.C.val, cpu.B.val, cpu.HL.readByte());
        cpu.B.val--;    // ?
        cpu.HL.set((short)(cpu.HL.get()+1));
        cpu.N_flag.set();
        cpu.Z_flag.set(cpu.B.val == 0);
        return 12;
    }
}

/* OTIR - block output with increment */
final class OTIR extends Instruction {
    public int execute() {
        cpu.out.writePort(cpu.C.val, cpu.B.val, cpu.HL.readByte());
        cpu.B.val--;    // ?
        cpu.HL.set((short)(cpu.HL.get()+1));
        cpu.N_flag.set();
        cpu.Z_flag.set();
        if (cpu.B.val != 0) {
            cpu.PC -= 2;
            return 17;
        }
        return 12;
    }
}

/* HALT - halt CPU until interrupt or reset */
final class HALT extends Instruction {
    public int execute() {
        cpu.halt = true;
        return 4;
    }
}

/* IM n - set interrupt mode to n */
final class IM extends Instruction {
    private int mode;

    IM(int mode) {
        this.mode = mode;
    }
    public int execute() {
        cpu.interrupt_mode = mode;
        return 8;
    }
}

/* DI - disable interrupts */
final class DI extends Instruction {
    public int execute() {
        cpu.IFF1 = false;
        cpu.IFF2 = false;
        return 4;
    }
}

/* EI - enable interrupts */
final class EI extends Instruction {
    public int execute() {
        cpu.IFF1 = true;
        cpu.IFF2 = true;
        return 4;
    }
}

/* RETN - return to the address on top of stack from NMI */
final class RETN extends Instruction {
    public int execute() {
        cpu.PC = cpu.pop();
        cpu.IFF1 = cpu.IFF2;
        return 10;
    }
}

/* LD A,I / LD A,R - load accumulator from I or R register */
final class LD_A_I extends Instruction {
    private Register r;

    LD_A_I(Register r) {
        this.r = r;
    }
    public int execute() {
        cpu.A.val = r.val;
        cpu.Z_flag.set(r.val == 0);
        cpu.S_flag.set(r.val < 0);
        cpu.PV_flag.set(cpu.IFF2);
        cpu.H_flag.clear();
        cpu.N_flag.clear();
        return 5;
    }
}

/* LD I,A / LD R,A - load I or R register from the accumulator */
final class LD_I_A extends Instruction {
    private Register r;

    LD_I_A(Register r) {
        this.r = r;
    }
    public int execute() {
        r.val = cpu.A.val;
        return 5;
    }
}

/* DAA - decimal adjust accumulator */
final class DAA extends Instruction {
    private Hashtable daa;

    DAA() {
        daa = new Hashtable();
        for (int h=0; h<=9; h++) {
            for (int l=0; l<=9; l++) {
                daa.put(new Integer(h << 4 | l), new Integer(0));
            }
        }
        for (int h=0; h<=8; h++) {
            for (int l=0xA; l<=0xF; l++) {
                daa.put(new Integer(h << 4 | l), new Integer(6));
            }
        }
        for (int h=0; h<=9; h++) {
            for (int l=0; l<=3; l++) {
                daa.put(new Integer(h << 4 | l | 0x100), new Integer(6));
            }
        }
        for (int h=0xA; h<=0xF; h++) {
            for (int l=0; l<=9; l++) {
                daa.put(new Integer(h << 4 | l), new Integer(0x160));
            }
        }
        for (int h=9; h<=0xF; h++) {
            for (int l=0xA; l<=0xF; l++) {
                daa.put(new Integer(h << 4 | l), new Integer(0x166));
            }
        }
        for (int h=0xA; h<=0xF; h++) {
            for (int l=0; l<=3; l++) {
                daa.put(new Integer(h << 4 | l | 0x100), new Integer(0x166));
            }
        }
        for (int h=0; h<=2; h++) {
            for (int l=0; l<=9; l++) {
                daa.put(new Integer(h << 4 | l | 0x200), new Integer(0x160));
            }
        }
        for (int h=0; h<=2; h++) {
            for (int l=0xA; l<=0xF; l++) {
                daa.put(new Integer(h << 4 | l | 0x200), new Integer(0x166));
            }
        }
        for (int h=0; h<=3; h++) {
            for (int l=0; l<=3; l++) {
                daa.put(new Integer(h << 4 | l | 0x300), new Integer(0x166));
            }
        }
        for (int h=0; h<=9; h++) {
            for (int l=0; l<=9; l++) {
                daa.put(new Integer(h << 4 | l | 0x400), new Integer(0));
            }
        }
        for (int h=0; h<=8; h++) {
            for (int l=6; l<=0xF; l++) {
                daa.put(new Integer(h << 4 | l | 0x500), new Integer(0xFA));
            }
        }
        for (int h=7; h<=0xF; h++) {
            for (int l=0; l<=9; l++) {
                daa.put(new Integer(h << 4 | l | 0x600), new Integer(0x1A0));
            }
        }
        for (int h=6; h<=0xF; h++) {
            for (int l=6; l<=0xF; l++) {
                daa.put(new Integer(h << 4 | l | 0x700), new Integer(0x19A));
            }
        }
    }
    public int execute() {
        int index = cpu.A.val & 0xFF;
        if (cpu.N_flag.test()) {
            index = index | 0x400;
        }
        if (cpu.C_flag.test()) {
            index = index | 0x200;
        }
        if (cpu.H_flag.test()) {
            index = index | 0x100;
        }
        int val = ((Integer)daa.get(new Integer(index))).intValue();
        byte bval = (byte)(val & 0xFF);
        byte old = cpu.A.val;
        cpu.A.val += bval;
        byte test = (byte)(old & bval | (old ^ bval) & ~cpu.A.val);
        cpu.C_flag.set((val & 0x100) != 0);
        cpu.H_flag.set((test & 0x08) != 0);
        cpu.Z_flag.set(cpu.A.val == 0);
        cpu.S_flag.set(cpu.A.val < 0);
        cpu.PV_flag.set(cpu.parity[cpu.A.val & 0xFF]);
        return 4;
    }
}
