package david.augusto.luan;

import java.awt.Color;

import robocode.HitByBulletEvent;
import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import robocode.Robot;
import robocode.ScannedRobotEvent;
import robocode.WinEvent;

public class Fantasminha extends Robot {
	public void run() {
		// Definem as cores do robô
		setBodyColor(Color.white);
		setGunColor(Color.white);
		setRadarColor(Color.white);
		setScanColor(Color.white);
		// setBulletColor(Color.writh);

		while (true) {
			turnRadarRight(360);
			ahead(100);
			turnGunRight(360);
			back(100);

		}
	}

	// Detecta os outros robôs
	public void onScannedRobot(ScannedRobotEvent e) {
		double max = 100;

		// Faz um controle da energia que é gasta no que diz
		// respeito à potência do tiro
		if (e.getEnergy() < max) {
			max = e.getEnergy();
			miraCanhao(e.getBearing(), max, getEnergy());
		} else if (e.getEnergy() >= max) {
			max = e.getEnergy();
			miraCanhao(e.getBearing(), max, getEnergy());
		} else if (getOthers() == 1) {
			max = e.getEnergy();
			miraCanhao(e.getBearing(), max, getEnergy());
		}
	}

	// quando o seu robo colide com outro robo
	public void onHitRobot(HitRobotEvent e) {
		tiroFatal(e.getBearing(), e.getEnergy(), getEnergy());

	}

	// Quando seu roboô leva um tiro
	public void onHitByBullet(HitByBulletEvent e) {
		turnLeft(90);
		back(100);
	}

	// Fornece as coordenadas para o ajuste do canhão.
	public void miraCanhao(double PosIni, double energiaIni, double minhaEnergia) {
		double Distancia = PosIni;
		double Coordenadas = getHeading() + PosIni - getGunHeading();
		double PontoQuarenta = (energiaIni / 4) + .1;
		if (!(Coordenadas > -180 && Coordenadas <= 180)) {
			while (Coordenadas <= -180) {
				Coordenadas += 360;
			}
			while (Coordenadas > 180) {
				Coordenadas -= 360;
			}
		}
		turnGunRight(Coordenadas);

		if (Distancia > 200 || minhaEnergia < 15 || energiaIni > minhaEnergia) {
			fire(1);
		} else if (Distancia > 50) {
			fire(2);
		} else {
			fire(PontoQuarenta);
		}
	}

	// É chamado quando o robô bate na parede,
	public void onHitWall(HitWallEvent e) {
		turnLeft(90);
		ahead(200);
	}

	// Dança da vitória
	public void onWin(WinEvent e) {
		turnRight(72000);
	}

	public void tiroFatal(double PosIni, double energiaIni, double minhaEnergia) {
	  double Coordenadas = getHeading() + PosIni - getGunHeading();
	  double PontoQuarenta = (energiaIni / 4) + .1;
	  if (!(Coordenadas > -180 && Coordenadas <= 180)) {
	     while (Coordenadas <= -180) {
	        Coordenadas += 360;
		 }
		 while (Coordenadas > 180) {
	        Coordenadas -= 360;
	     }
	  }
	  turnGunRight(Coordenadas);
	  fire(PontoQuarenta);
       
   }
}