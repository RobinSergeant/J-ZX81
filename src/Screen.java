import java.awt.*;

public final class Screen extends Canvas implements Runnable {
	private Memory ram;
	private Thread refresh;
	private Image display;
	private Graphics context;

	public Screen(Memory ram) {
		this.ram = ram;
		refresh = new Thread(this);
		refresh.setDaemon(true);
	}

	public Dimension getPreferredSize() {
		return new Dimension(256, 192);
	}

	public Dimension getMinimumSize() { return getPreferredSize(); }

	public Dimension getMaximumSize() { return getPreferredSize(); }

	public void start() { refresh.start(); }

	public void suspend() { refresh.suspend(); }

	public void resume() { refresh.resume(); }

	public void run() {
		while (true) {
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {}
			repaint();
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

	public void paint(Graphics g) {
		long t = System.currentTimeMillis();
		g.clearRect(0, 0, 256, 192);
		int dfile = ram.readWord(0x400c);
		int addr, mask;
		int x = 0;
		int y = -8;
		byte ch;
		boolean inv;
		while (y < 192) {
			ch = ram.data[dfile];
			dfile++;
			if (ch == 118) {
				x = -8;
				y += 8;
			} else if (ch != 0) {
				addr = 7680 + ((ch & 0x7F) << 3);
				inv = ((ch & 0x80) != 0);
				for (int c=0; c<8; c++) {
					mask = 1;
					for (int c2=7; c2>=0; c2--) {
						if (((ram.data[c+addr] & mask) != 0) ^ inv) {
							g.drawLine(x+c2,y+c,x+c2,y+c);
						}
						mask = mask << 1;
					}
				}
			}
			x += 8;
		}
		System.out.println(System.currentTimeMillis()-t);
	}
}