// Copyright 1998-2021 by Robin Sergeant. See license.txt distributed with this file.
// Screen2.java

/* this class represents the ZX81 video display, providing a screen component and refresh thread */


import java.awt.*;
import java.awt.event.*;

public final class Screen2 extends Canvas implements Runnable, MouseListener {
    private Memory ram;
    private Thread refresh;
    private Image display;  // off screen image buffer
    private Graphics context;
    private Image table[] = new Image[192]; // character images

    public boolean update_frames;

    public Screen2(Memory ram) {
        this.ram = ram;
        refresh = new Thread(this);
        refresh.setDaemon(true);
        this.addMouseListener(this);
    }

    public Dimension getPreferredSize() {
        return new Dimension(256, 192);
    }

    public Dimension getMinimumSize() { return getPreferredSize(); }

    public Dimension getMaximumSize() { return getPreferredSize(); }

    public void start() { refresh.start(); }

    public void stop() { refresh.stop(); }

    public void suspend() { refresh.suspend(); }

    public void resume() { refresh.resume(); }

    synchronized public void run() {
        while (true) {
            if (update_frames) {
                ram.writeWord(16436, (short)(ram.readWord(16436)-1 | 0x8000));
            }
            repaint();
            try {
                wait();
                Thread.sleep(20);
            } catch (InterruptedException e) {}
        }
    }

    public void update(Graphics g) {
        if (display == null) {
            display = createImage(256, 192);
            context = display.getGraphics();
        }
        paint(context);
        g.drawImage(display, 0, 0, this);
    }

    synchronized public void paint(Graphics g) {
        g.clearRect(0, 0, 256, 192);
        int dfile = ram.readWord(0x400c);   // get position of DFILE
        int addr, mask;
        int x = 0;
        int y = -8;
        int ch;
        boolean inv;
        Graphics gt;
        while ((y < 192) && (x < 257)) {
            ch = ram.readByte(dfile) & 0xFF;    // get next character
            dfile++;
            if (ch == 118) {    // new line
                x = -8;
                y += 8;
            } else if (((ch > 0) && (ch < 64)) || ((ch > 127) && (ch < 192))) {
                if (table[ch] == null) {    // create character image if null
                    table[ch] = this.createImage(8,8);
                    gt = table[ch].getGraphics();
                    addr = 7680 + ((ch & 0x7F) << 3);   // find character in table
                    inv = ((ch & 0x80) != 0);   // check for inverse bit
                    for (int c=0; c<8; c++) {   // loop through each row
                        mask = 1;
                        for (int c2=7; c2>=0; c2--) {   // check each pixel
                            if (((ram.data[c+addr] & mask) != 0) ^ inv) {
                                gt.drawLine(c2, c, c2, c);
                            }
                            mask = mask << 1;   // multiply by 2
                        }
                    }
                }
                g.drawImage(table[ch], x, y, this); // draw char image
            }
            x += 8;
        }
        notifyAll();    // wake up refresh thread
    }

    public void mouseClicked(MouseEvent e) {}
    public void mouseEntered(MouseEvent e) {}
    public void mouseExited(MouseEvent e) {}
    public void mousePressed(MouseEvent e) { this.requestFocus(); }
    public void mouseReleased(MouseEvent e) {}
}
