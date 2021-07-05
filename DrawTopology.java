package cn.act.ict.crowdsource.draw;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FilterOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.imageio.ImageIO;
import javax.swing.*;

import cn.ac.ict.crowdsource.bean.AtomTrajectoryBean;

/**
 * 
 * ���˹���
 * 
 * @author Administrator
 *
 */
public class DrawTopology extends JFrame {

	private static final long serialVersionUID = 1L;
	private Random random;
	private static int MAP_SCALE = 10; // ��ͼ����Ŵ����
	private static double radis = 0.5;

	private List<List<AtomTrajectoryBean>> data = new ArrayList<List<AtomTrajectoryBean>>();

	public DrawTopology(List<List<AtomTrajectoryBean>> data) {
		random = new Random();
		this.data = data;
		this.setTitle("crowd_map");
		this.setSize(1000, 800);
		this.getContentPane().setBackground(Color.green);
		this.getContentPane().setVisible(false);// �����Ϊtrue��ô�ͱ���˺�ɫ��
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		this.setVisible(true);

	}

	@Override
	public void paint(Graphics g) {
		for (int i = 0; i < data.size(); i++) {
			g.setColor(new Color(random.nextInt(255), random.nextInt(255), random.nextInt(255)));
			List<AtomTrajectoryBean> temp = data.get(i);
			for (int j = 0; j < temp.size(); j++) {
				AtomTrajectoryBean line = temp.get(j);
				// ����
				int stepCount = line.getStepList().size();
				// ����
				double stepLength = line.getStepLength();
				// �켣�ķ�������,�Ѿ��û������ڴ����
				List<Double> oritation =line.getOrientation();
				// ǰһʱ�̺�����
				double preX = line.getStartX() + random.nextGaussian() * radis * MAP_SCALE;
				// ǰһʱ��������
				double preY = line.getStartY() + random.nextGaussian() * radis * MAP_SCALE;
				// ÿһ���ķ������
				int oritationCountPerStep = oritation.size() / stepCount;
				// ��������
				double beginX = line.getStartX();
				// ���������
				double beginY = line.getStartY();
				// �յ������
				double endX = line.getEndX();
				// �յ�������
				double endY = line.getEndY();

				g.drawRect((int) preX, (int) preY, 1, 1);
				for (int k = 0; k < stepCount; k++) {
					double aveOritation = getDealWithOritation(getAveOritation(
							oritation, oritationCountPerStep * k,
							oritationCountPerStep))
							+ (random.nextDouble() * 2 - 1) * 10;
					System.out.println("��" + j + "����" + aveOritation);

					double tempPreX = preX, tempPreY = preY;

					preX += stepLength * Math.sin(Math.toRadians(aveOritation)) * MAP_SCALE;
					preY -= stepLength * Math.cos(Math.toRadians(aveOritation)) * MAP_SCALE;

					g.drawRect((int) preX, (int) preY, 1, 1);

					// ���Ӻ����������һ���ڵ�͵�ǰ�ڵ�
					g.drawLine((int) tempPreX, (int) tempPreY, (int) preX,
							(int) preY);
					g.drawLine((int) preX, (int) preY, (int) tempPreX,
							(int) tempPreY);
				}

				// ���Ӻ�������������յ�
//				 g.drawLine((int)beginX, (int)beginY, (int)preX, (int)preY);
//				 g.drawLine((int)preX, (int)preY, (int)beginX, (int)beginY);

				// ��ȷ�������յ�
//				g.drawLine((int) beginX, (int) beginY, (int) endX, (int) endY);
//				g.fillRect((int) beginX, (int) beginY, (int) (endX - beginX),
//						(int) (endY - beginY));

			}
		}
	}

	/**
	 * 
	 * ע��180�Ⱥ�-180��
	 * 
	 * @param orientation
	 *            ��������
	 * @param i
	 *            ��ʼλ��
	 * @param j
	 *            ����
	 * @return
	 */
	private double getAveOritation(List<Double> orientation, int i, int j) {
		double positive = 0, negative = 0;
		for (int k = i; k < i + j; k++) {
			double temp = orientation.get(k);
			if (temp >= 0)
				positive += temp;
			else
				negative += temp;
		}

		if (positive == 0)
			return negative / j;
		else if (negative == 0)
			return positive / j;
		else {
			double res = (positive - negative) / j;
			return positive > -negative ? res : -res;
		}
	}

	private double getDealWithOritation(double orientation) {
		if (orientation > 45 && orientation < 135)
			return 90;
		else if (orientation > -45 && orientation <= 45)
			return 0;
		else if (orientation > -135 && orientation <= -45)
			return -90;
		else if(orientation <= -135)
			return -180;
		else if (orientation > 135) {
			return 180;
		}
		else 
			return 0;
	}



}
