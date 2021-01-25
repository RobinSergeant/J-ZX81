// Copyright 1998-2021 by Robin Sergeant. See license.txt distributed with this file.
// J_ZX81.java

/* this is the main class for the emulator.  It loads and pathes the ROM, generates the user interface, and creates all the main objects */

import java.awt.*;          // import standard packages
import java.awt.event.*;
import java.io.*;
import java.net.URL;
import Z80.*;               // import Z80 emulation package

final class J_ZX81 extends Frame
implements ActionListener, ItemListener, WindowListener {
    private Screen2 screen;     // ZX81 video display object
    private Keyboard keyboard;  // ZX81 keyboard object
    private CPU cpu;            // Z80 CPU object
    private Memory ram;         // ZX81 memory object

    private CheckboxMenuItem normal, full;
    String tape_dir;    // current tape file directory

    public static void main(String[] args) {
        Frame f = new J_ZX81();     // create and setup frame
        f.setLayout(new FlowLayout(FlowLayout.CENTER, 25, 25));
        f.setBackground(Color.white);
        f.setForeground(Color.black);
        f.setTitle(" J-ZX81 - The Java ZX81 Emulator");
        f.pack();   // pack GUI components within frame
        f.show();   // show emulator window
    }

    public J_ZX81() {
        Memory ram = new Memory(16,16); // 8K ROM, 8K shadow ROM, and 16K RAM
        screen = new Screen2(ram);
        keyboard = new Keyboard(ram);
        cpu = new CPU((float)(3.25), ram, keyboard, null); // 3.25Mhz CPU

        try {   // load ROM file
            //FileInputStream in = new FileInputStream("zx81.rom");
            InputStream in = getClass().getResourceAsStream("data/zx81.rom");
            in.read(ram.data);
            in.close();
        }
        catch (Exception e) {
            System.out.println("Error: could not load \"zx81.rom\"");
        }

        // patch orginal ROM code slightly for emulator
        ram.data[0x23e] = (byte)0xed;   // READ_KEY instruction
        ram.data[0x23f] = cpu.registerED(new Read_Key(keyboard));
        ram.data[0x240] = (byte)0x28;   // JR Z,-4 (023Eh)
        ram.data[0x241] = -4;
        ram.data[0x242] = (byte)0xc9;   // RET

        ram.data[0x4cc] = (byte)0xcd;   // CALL 023Eh
        ram.data[0x4cd] = (byte)0x3e;
        ram.data[0x4d1] = (byte)0x00;   // NOP
        ram.data[0x4d2] = (byte)0x00;   // NOP
        ram.data[0x4d3] = (byte)0xe5;   // PUSH HL
        ram.data[0x4d4] = (byte)0xc1;   // POP BC
        ram.data[0x4d5] = (byte)0x00;   // NOP
        ram.data[0x4d6] = (byte)0x00;   // NOP

        ram.data[0x2fc] = (byte)0xed;   // SAVE instruction
        ram.data[0x2fd] = cpu.registerED(new Save(ram, this));
        ram.data[0x2fe] = (byte)0xc9;   // RET

        ram.data[0x343] = (byte)0x38;   // JR C,-51h (02F4h)
        ram.data[0x344] = -0x51;
        ram.data[0x345] = (byte)0xeb;   // EX DE,HL
        ram.data[0x346] = (byte)0xed;   // LOAD instruction
        ram.data[0x347] = cpu.registerED(new Load(ram, this));
        ram.data[0x348] = (byte)0xc9;   // RET

        ram.data[0xf23] = (byte)0xed;   // FAST instruction
        ram.data[0xf24] = cpu.registerED(new Fast());
        ram.data[0xf25] = (byte)0xc9;   // RET

        ram.data[0xf2b] = (byte)0xed;   // SLOW instruction
        ram.data[0xf2c] = cpu.registerED(new Slow());
        ram.data[0xf2d] = (byte)0xc9;   // RET

        ram.data[0xf3a] = (byte)0xed;   // PAUSE instruction
        ram.data[0xf3b] = cpu.registerED(new Pause(keyboard));
        ram.data[0xf3c] = (byte)0x00;   // NOP

        ram.data[0x000] = (byte)0xed;   // FRAMES_OFF instruction
        ram.data[0x001] = cpu.registerED(new Frames_Off(screen));
        ram.data[0x3c3] = (byte)0xed;   // FRAMES_OFF instruction
        ram.data[0x3c4] = ram.data[0x001];
        ram.data[0x3c5] = (byte)0x00;   // NOP
        ram.data[0x416] = (byte)0xed;   // FRAMES_ON instruction
        ram.data[0x417] = cpu.registerED(new Frames_On(screen));
        ram.data[0x418] = (byte)0x00;   // NOP

        ram.data[0x066] = (byte)0xed;   // RESET instruction
        ram.data[0x067] = cpu.registerED(new Reset());

        // copy patched ROM into 8-16K shadow area
        System.arraycopy(ram.data, 0, ram.data, 8192, 8192);

        // lookup current directory in property list and set tape_dir
        tape_dir = System.getProperty("user.dir") + File.separator;

        // add window listener to close frame
        this.addWindowListener(this);

        // create menu bar, menus, and menu items
        MenuBar menubar = new MenuBar();
        this.setMenuBar(menubar);
        Menu file = new Menu("File");
        Menu settings = new Menu("Settings");
        Menu speed = new Menu("Speed");
        Menu help = new Menu("Help");
        menubar.add(file);
        menubar.add(settings);
        menubar.add(help);
        menubar.setHelpMenu(help);
        CheckboxMenuItem pause = new CheckboxMenuItem("Pause");
        pause.addItemListener(this);
        normal = new CheckboxMenuItem("Normal", true);
        normal.addItemListener(this);
        full = new CheckboxMenuItem("Full");
        full.addItemListener(this);
        file.add(pause);
        file.insertSeparator(1);
        file.add(createItem("Reset", KeyEvent.VK_R));
        file.add(createItem("Exit"));
        settings.add(speed);
        speed.add(normal);
        speed.add(full);
        settings.add(createItem("Tape file directory"));
        help.add(createItem("Keyboard Layout"));
        help.add(createItem("About"));

        // make keyboard object listen to screen keyboard events
        screen.addKeyListener(keyboard);
        this.add(screen);
        cpu.start();        // start CPU thread
        screen.start();     // start screen refresh thread
    }

    // helper function to crate menu items
    private MenuItem createItem(String name) {
        MenuItem item = new MenuItem(name);
        item.addActionListener(this);
        return item;
    }

    // helper function to create menu items with Ctrl shortcut keys
    private MenuItem createItem(String name, int key) {
        MenuItem item = new MenuItem(name, new MenuShortcut(key));
        item.addActionListener(this);
        item.setActionCommand(name);
        return item;
    }

    // menu event handling procedure
    public void actionPerformed(ActionEvent e) {
        String choice = e.getActionCommand();
        if (choice.equals("Exit")) {
            System.exit(0);     // exit emulator
        } else if (choice.equals("Keyboard Layout")) {
            // show image of ZX81 keyboard in a dialog box
            URL image = getClass().getResource("data/zx81keyb.jpg");
            PicDialog p = new PicDialog(this, "ZX81 Keyboard", image, 555, 245);
            p.show();
        } else if (choice.equals("Reset")) {
            cpu.NMI();  // generate NMI to reset ZX81
        } else if (choice.equals("About")) {
            MsgDialog m = new MsgDialog(this, "J-ZX81 version 0.7(beta)",
                "Copyright (c) 1998-2021 Robin Sergeant", "About");
            m.show();
        } else if (choice.equals("Tape file directory")) {
            // create file dialog box
            FileDialog fd = new FileDialog(this, "Select tape file directory");
            fd.setDirectory(tape_dir);  // set to current tape_dir
            fd.show();  // show file dialog and wait until it is closed by user
            tape_dir = fd.getDirectory();   // update tape_dir
        }
    }

    public void itemStateChanged(ItemEvent e) {
        // event handling procedure for check box menu items
        ItemSelectable item = e.getItemSelectable();
        if ((item == normal) || (item == full)) {
            if (item == normal) {   // "normal speed" clicked, so
                // make "full speed" state the inverse of the new "normal" state
                full.setState(!normal.getState());
            } else {    // else modify the state of "normal speed" instead
                normal.setState(!full.getState());
            }
            cpu.setTurbo(full.getState());  // change CPU speed
        } else {
            if (e.getStateChange() == ItemEvent.SELECTED) {
                cpu.suspend();      // pause CPU thread
            } else {
                cpu.resetTimer();   // un-pause CPU thread
                cpu.resume();       // reset CPU control timer
            }
        }
    }

    public void windowClosing(WindowEvent e) { System.exit(0); }
    public void windowOpened(WindowEvent e) {}
    public void windowClosed(WindowEvent e) {}
    public void windowIconified(WindowEvent e) {}
    public void windowDeiconified(WindowEvent e) {}
    public void windowActivated(WindowEvent e) { screen.requestFocus(); }
    public void windowDeactivated(WindowEvent e) {}
}

final class MsgDialog extends Dialog implements ActionListener {
    // dialog box with two string labels, a title, and Ok button
    public MsgDialog(Frame parent, String line1, String line2, String title) {
        super(parent, title, false);    // set title and make non-modal
        this.add("North", new Label(line1, Label.CENTER));  // first label
        this.add("Center", new Label(line2, Label.CENTER)); // second label
        Panel p = new Panel();  // create layout panel to hold button
        Button ok = new Button("Ok");
        p.add(ok);
        this.add("South", p);   // add panel
        ok.addActionListener(this);
        this.pack();    // pack GUI components
    }
    public void actionPerformed(ActionEvent e) {
        // when button pressed remove dialog box
        this.setVisible(false);
        this.dispose();
    }
}

final class PicDialog extends Dialog implements WindowListener {
    // dialog box to display a GIF or JPEG image file
    private Image im;   // keyboard image

    public PicDialog(Frame parent, String title, URL file, int w, int h) {
        super(parent, title, false);
        // get AWT toolkit object
        Toolkit t = this.getToolkit();
        // use toolkit to load image
        im = t.getImage(file);
        // set dialog frame to the required size
        this.setSize(w, h);
        // add window listener
        this.addWindowListener(this);
    }
    public void paint(Graphics g) {
        // overide paint memthod to draw the image
        g.drawImage(im, 0 , 0, this);
    }
    // implement window event interface so dialog can be closed
    public void windowClosing(WindowEvent e) { this.dispose(); }
    public void windowOpened(WindowEvent e) {}
    public void windowClosed(WindowEvent e) {}
    public void windowIconified(WindowEvent e) {}
    public void windowDeiconified(WindowEvent e) {}
    public void windowActivated(WindowEvent e) {}
    public void windowDeactivated(WindowEvent e) {}
}

final class Read_Key extends Instruction {
    private int last;
    private Keyboard keyb;

    Read_Key(Keyboard keyb) {
        this.keyb = keyb;
    }

    public int execute() {
        int current = keyb.getCurrentKey() & 0xFFFF;
        cpu.HL.set((short)current);
        cpu.Z_flag.set((current == last) || (current == 0xFFFF));
        last = current;
        return 4;
    }
}

final class Pause extends Instruction {
    private Keyboard keyb;

    Pause(Keyboard keyb) {
        this.keyb = keyb;
    }

    public int execute() {
        int frames = cpu.HL.get() & 0xFFFF;
        if (frames > 32767) {
            cpu.PC = 0x23e;
            return 4;
        }
        long end_time = System.currentTimeMillis() + frames *20;
        boolean key_pressed = false;
        long time;
        do {
            if (keyb.getCurrentKey() != (short)0xFFFF) {
                key_pressed = true;
            }
            time = System.currentTimeMillis();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {}
        } while ((time < end_time) && (key_pressed == false));
        cpu.resetTimer();
        return 4;
    }
}

class Save extends Instruction {
    protected Memory ram;
    protected J_ZX81 f;

    Save(Memory ram, J_ZX81 f) {
        this.ram = ram;
        this.f = f;
    }

    public String getFname(short addr) {
        byte code;
        byte ch;
        StringBuffer sb = new StringBuffer();
        sb.append(f.tape_dir);
        do {
            code = ram.readByte(addr);
            ch = (byte)(code & 0x7f);
            if ((ch > 27) && (ch < 38)) {
                sb.append((char)(ch + 20));
            } else if ((ch > 37) && (ch < 64)) {
                sb.append((char)(ch + 59));
            } else if (ch == 22) {
                sb.append('-');
            } else {
                sb.append('_');
            }
            addr++;
        } while ((code & 0x80) == 0);
        sb.append(".p");
        return sb.toString();
    }
    public int execute() {
        String fname = getFname(cpu.HL.get());
        System.out.println("Saving \"" + fname + "\"");
        try {
            FileOutputStream out = new FileOutputStream(fname);
            out.write(ram.data, 16393, ram.readWord(16404)-16392);
            out.close();
        } catch (Exception ex) {
            MsgDialog m = new MsgDialog(f, "Could not save file:", "\"" + fname                 + "\"", "Error!");
            m.show();
        }
        return 4;
    }
}

final class Load extends Save {
    Load(Memory ram, J_ZX81 f) {
        super(ram, f);
    }

    public int execute() {
        String fname = getFname(cpu.HL.get());
        System.out.println("Loading \"" + fname + "\"");
        try {
            FileInputStream in = new FileInputStream(fname);
            in.read(ram.data, 16393, ram.data.length-16393);
            in.close();
        } catch (Exception ex) {
            MsgDialog m = new MsgDialog(f, "Could not load file:", "\"" + fname                 + "\"", "Error!");
            m.show();
        }
        return 4;
    }
}

final class Fast extends Instruction {
    public int execute() {
        cpu.setSpeed((float)3.25);
        return 4;
    }
}

final class Slow extends Instruction {
    public int execute() {
        cpu.setSpeed((float)(3.25 / 4));
        return 4;
    }
}

final class Frames_Off extends Instruction {
    private Screen2 screen;

    Frames_Off(Screen2 screen) {
        this.screen = screen;
    }
    public int execute() {
        screen.update_frames = false;
        cpu.setSpeed((float)3.25);
        return 4;
    }
}

final class Frames_On extends Instruction {
    private Screen2 screen;

    Frames_On(Screen2 screen) {
        this.screen = screen;
    }
    public int execute() {
        screen.update_frames = true;
        cpu.setSpeed((float)(3.25 / 4));
        return 4;
    }
}

final class Reset extends Instruction {
    public int execute() {
        cpu.reset();
        return 4;
    }
}
