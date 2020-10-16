package actr.tasks.driving;

import java.util.Random;

import java.awt.Graphics;
import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.FontFormatException;
import java.awt.Graphics2D;
import java.io.*;
import java.awt.GraphicsEnvironment;

/**
 * The class that defines the specifics of the speed signs.
 * 
 * @author Moritz Held
 */
public class Speedsign extends Road {

    double signOnset = 0;
    String speedlimit = "60"; // starting speed
    int speedI = 0;
    Position signPos;
    double signFrac;
    int sign_count = 0;
    String[] allLimits = { "60", "70", "80", "90", "100", "110", "120", "130", "140" };
    String[] tmpLim = { "90", "80", "70", "80", "90", "60", "100", "90", "80" };
    boolean visible = false;
    boolean newSign = false;

    void newSign(Env env) {
        double time = env.time;
        if (env.simcar.nearPoint != null) {
            // pick a random speed with delta<30
            String previousLimit = speedlimit;
            while (Math.abs(Integer.parseInt(speedlimit) - Integer.parseInt(previousLimit)) > 30
                    || (speedlimit == previousLimit)) {
                int rnd = new Random().nextInt(allLimits.length);
                speedlimit = allLimits[rnd];
            }
            speedlimit = tmpLim[sign_count]; // tmp
            sign_count += 1; // tmp
            // signFrac = env.simcar.fracIndex + (env.simcar.speed*env.sampleTime*400);
            signFrac = env.simcar.fracIndex + 100;
            signPos = Road.location(signFrac, env.road.lanes + 1.3);
            signPos.y = 1.5;
            signOnset = time;
        }
    }

    void update(Env env) {
        double time = env.time;
        if (time >= 10) {
            if ((int) time % 20 == 0 && visible == false && (speedI < allLimits.length)) {
                newSign(env);
                newSign = true;
            }
        }
    }

    void drawSign(Graphics g, Env env) {
        Coordinate cs = env.world2image(signPos);
        if (cs == null)
            return;

        // painting the metal rod
        Position pos1 = Road.location(signFrac, env.road.lanes + 1.28);
        pos1.y = 0.0;
        Coordinate im1 = env.world2image(pos1);

        Position pos2 = Road.location(signFrac, env.road.lanes + 1.32);
        pos2.y = 1.6; // For reference: A car is 1 unit tall.
        Coordinate im2 = env.world2image(pos2);

        g.setColor(Color.GRAY);
        g.fillRect(im1.x, im2.y, im2.x - im1.x, im1.y - im2.y);

        // painting the sign
        int r = (int) (im1.y - im2.y) / 4; // radius of the sign = height of rod/4
        int wr = (int) (0.8 * r);
        Position pos3 = Road.location(signFrac, env.road.lanes + 1.3);
        pos3.y = pos2.y;
        Coordinate im3 = env.world2image(pos3);

        // red circle
        int cx = im3.x;
        int cy = im3.y;
        cx = cx - r;
        cy = cy - r;
        g.setColor(Color.red);
        g.fillOval(cx, cy, r * 2, r * 2);

        // white circle
        cx = im3.x;
        cy = im3.y;
        cx = cx - wr;
        cy = cy - wr;
        g.setColor(Color.white);
        g.fillOval(cx, cy, wr * 2, wr * 2);

        // text
        Font speedFont = speedFont();
        speedFont = speedFont.deriveFont(Font.BOLD);
        speedFont = speedFont.deriveFont((float) wr);
        g.setFont(speedFont);
        g.setColor(Color.black);
        FontMetrics fm = g.getFontMetrics();
        int textwidth = (wr * 2 - fm.stringWidth(speedlimit) / 2);
        int textheight = ((wr * 2 - fm.getHeight()) / 2) + fm.getAscent();

        Graphics2D g2d = (Graphics2D) g.create();
        g2d.setColor(Color.black);
        g2d.drawString(speedlimit, (im3.x - textwidth / 2), (cy + textheight)); // bit off center
    }

    private Font speedFont() {
        Font speedFont = null;
        try {
            speedFont = Font.createFont(Font.TRUETYPE_FONT,
                    new File(System.getProperty("user.dir") + "/src/resources/speedsign_font.TTF"));
            GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
            ge.registerFont(speedFont);
        } catch (IOException e) {
            e.printStackTrace();
            speedFont = new Font("serif", Font.PLAIN, 24);
            ;
        } catch (FontFormatException e) {
            e.printStackTrace();
            speedFont = new Font("serif", Font.PLAIN, 24);
            ;
        }
        return speedFont;
    }

}