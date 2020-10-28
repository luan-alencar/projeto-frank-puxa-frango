package david.augusto.luan;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import robocode.AdvancedRobot;
import robocode.BorderSentry;
import robocode.RobotDeathEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

public class Ultron extends AdvancedRobot implements BorderSentry {

	// Constants
	final double PODERDEFOGO = 3; // poder maximo de 3
	final double HALF_ROBOT_SIZE = 18; // Robot size is 36x36 units, so the half size is 18 units

	Map<String, RobotData> enemyMap;

	// direção da varredura, onde o radar vira para a direita com valores positivos
	// e vira
	// a esquerda com valores negativos.
	double scanDir = 1;

	// Robô verificado mais antigo. Pode ser nulo.
	RobotData oldestScanned;

	// Robô alvo para a arma. Pode ser nulo, o que significa que não há robô de
	// destino no momento.
	RobotData target;

	// A última vez que o robô mudou de direção
	long lastDirectionShift;

	// Direção atual, onde 1 significa à frente (para frente) e -1 significa para
	// trás
	int direction = 1;

	/**
	 * contrutor do robo
	 */
	public Ultron() {
		// Inicializamos um HashMap especializado que usa uma lista vinculada para o
		// pedido de acesso.
		// Isso significa que a última entrada do robô acessada é listada primeiro,
		// quando iteramos sobre sua
		// valores. Este robô sempre varre o radar em direção ao robô mais antigo
		// escaneado.
		enemyMap = new LinkedHashMap<String, RobotData>(5, 2, true);
	}

	@Override
	public void run() {
		// Faça a inicialização aqui antes do loop
		initialize();

		// Loop para sempre. Se o robô não agir, o jogo irá desabilitar nosso robô!
		while (true) {
			// Segure uma única volta ...

			// Controle o radar que verifica os robôs inimigos
			handleRadar();
			// Manuseie a arma girando-a e atire em nosso alvo
			handleGun();
			// Mova o robô no campo de batalha
			moveRobot();

			// Procure outros robôs. Observe que este método irá executar todos os comandos
			// pendentes para
			// a próxima curva. Portanto, scan () termina o turno para nosso robô.
			scan();
		}
	}

	/**
	 * Este método é chamado pelo jogo quando seu robô vê outro robô, ou seja,
	 * quando a varredura do radar do robô "atinge" outro robô.
	 * 
	 * @param scannedRobotEvent é um evento ScannedRobotEvent.
	 */
	@Override
	public void onScannedRobot(ScannedRobotEvent scannedRobotEvent) {
		// Verifique se o robô verificado não é um robô sentinela
		if (!scannedRobotEvent.isSentryRobot()) {
			// O robô verificado não é um robô sentinela ...

			// Atualize o mapa do inimigo
			updateEnemyMap(scannedRobotEvent);

			// Atualize a direção da varredura
			updateScanDirection(scannedRobotEvent);

			// Atualize as posições dos alvos inimigos
			updateEnemyTargetPositions();
		}
	}

	/**
	 * Este método é chamado pelo jogo quando outro robô morre.
	 * 
	 * @param robotDeathEvent é o RobotDeathEvent que ocorre quando outro robô
	 *                        morre, que contém dados do robô que morreu.
	 */
	@Override
	public void onRobotDeath(RobotDeathEvent robotDeathEvent) {
		// Obtém o nome do robô que morreu
		final String deadRobotName = robotDeathEvent.getName();

		// Remova os dados do robô para o robô que morreu do mapa inimigo
		enemyMap.remove(deadRobotName);

		// Remova a entrada de dados para o robô mais antigo verificado, se tivermos tal
		// entrada
		if (oldestScanned != null && oldestScanned.name.equals(deadRobotName)) {
			oldestScanned = null;
		}
		if (target != null && target.name.equals(deadRobotName)) {
			target = null;
		}
	}

	/**
	 * Este método é chamado sempre que o robô é pintado. Em ordem para ver a
	 * pintura, certifique-se de habilitar o botão Paint no console do robô para
	 * este robô.
	 * 
	 * @param g is the {@link Graphics2D} object, which is the graphics context used
	 *          for painting various shapes like rectangles, circles, lines etc. on
	 *          top of the battlefield for debugging graphics.
	 */
	@Override
	public void onPaint(Graphics2D g) {
		// Defina a largura da linha em 2 pixels
		g.setStroke(new BasicStroke(2f));

		// Prepare colors for painting the scanned coordinate and target coordinate
		Color color1 = new Color(0x00, 0xFF, 0x00, 0x40); // Green with 25% alpha blending
		Color color2 = new Color(0xFF, 0xFF, 0x00, 0x40); // Yellow with 25% alhpa blending

		// Paint a two circles for each robot in the enemy map. One circle where the
		// robot was
		// scanned the last time, and another circle where our robot must point the gun
		// in order to
		// hit it (target coordinate). In addition, a line is drawn between these
		// circles.
		for (RobotData robot : enemyMap.values()) {
			// Paint the two circles and a line
			fillCircle(g, robot.scannedX, robot.scannedY, color1); // scanned coordinate
			fillCircle(g, robot.targetX, robot.targetY, color2); // target coordinate
			g.setColor(color1);
			g.drawLine((int) robot.scannedX, (int) robot.scannedY, (int) robot.targetX, (int) robot.targetY);
		}

		// Paint a two circles for the target robot. One circle where the robot was
		// scanned the last time, and another circle where our robot must point the gun
		// in order to
		// hit it (target coordinate). In addition, a line is drawn between these
		// circles.
		if (target != null) {
			// Prepare colors for painting the scanned coordinate and target coordinate
			color1 = new Color(0xFF, 0x7F, 0x00, 0x40); // Orange with 25% alpha blending
			color2 = new Color(0xFF, 0x00, 0x00, 0x80); // Red with 50% alpha blending

			// Paint the two circles and a line
			fillCircle(g, target.scannedX, target.scannedY, color1); // scanned coordinate
			fillCircle(g, target.targetX, target.targetY, color2); // target coordinate
			g.setColor(color1);
			g.drawLine((int) target.scannedX, (int) target.scannedY, (int) target.targetX, (int) target.targetY);
		}
	}

	/**
	 * Initializes this robot before a new round in a battle.
	 */
	private void initialize() {
		// Inicializa este robô antes de uma nova rodada em uma batalha.
		setAdjustRadarForGunTurn(true);
		setAdjustGunForRobotTurn(true);

		// Set cores
		setBodyColor(Color.pink);
		setGunColor(Color.pink);
		setRadarColor(Color.pink);
		setBulletColor(Color.pink);
		setScanColor(Color.pink);
	}

	/**
	 * Este método lida com o radar que verifica os robôs inimigos.
	 */
	private void handleRadar() {
		// Defina o radar para virar infinitamente para a direita se a direção da
		// varredura for
		// positivo;
		// caso contrário, o radar é movido para a esquerda, se a direção da varredura
		// for negativa.
		// Observe que onScannedRobot (ScannedRobotEvent) é responsável por determinar
		// a varredura
		// direção.
		setTurnRadarRightRadians(scanDir * Double.POSITIVE_INFINITY);
	}

	/**
	 * Método que manuseia a arma girando-a e atirando em um alvo.
	 */
	private void handleGun() {
		// Atualize nosso robô alvo para atirar em
		updateTarget();
		// Atualize a direção da arma
		updateGunDirection();
		// Dispara a arma, quando estiver pronta
		fireGunWhenReady();
	}

	/**
	 * Método que move nosso robô pelo campo de batalha.
	 */
	private void moveRobot() {

		// A estratégia de movimento é mover-se o mais próximo possível do nosso robô
		// alvo.
		// Nosso robô deve se mover ao longo das fronteiras o tempo todo, verticalmente
		// ou
		// horizontalmente.
		// Quando nos aproximamos de nosso alvo, ou não temos para onde ir, nosso robô
		// deve
		// muda seu
		// direção de um lado para o outro para que não fique parado em nenhum momento.
		// Se o robô ficar parado, será um alvo fácil para robôs inimigos.
		int newDirection = direction;

		// Aproxime-se do nosso alvo se tivermos um robô alvo
		if (target != null) {
			// Calcule o alcance das paredes / bordas, nosso robô deve se manter dentro
			int borderRange = getSentryBorderSize() - 20;

			// Os sinalizadores horizontais e verticais são usados ​​para determinar se o
			// nosso robô devemos mover horizontal ou vertical.
			boolean horizontal = false;
			boolean vertical = false;

			// Inicialize o novo rumo do robô para o rumo atual do robô
			double newHeading = getHeadingRadians();

			// Verifique se o nosso robô está na borda superior ou inferior e, portanto,
			// deve se mover horizontalmente
			if (getY() < borderRange || getY() > getBattleFieldHeight() - borderRange) {
				horizontal = true;
			}
			// Verifique se o nosso robô está na borda esquerda ou direita e, portanto, deve
			// se mover
			// verticalmente
			if (getX() < borderRange || getX() > getBattleFieldWidth() - borderRange) {
				vertical = true;
			}

			// Se estivermos em um dos cantos do campo de batalha, poderíamos mover ambos
			// horizontalmente
			// ou verticalmente.
			// Nesta situação, precisamos escolher uma das duas direções.
			if (horizontal && vertical) {
				// Se a distância horizontal ao nosso alvo for menor que a distância vertical,
				// então escolhemos mover verticalmente e, portanto, limpamos a sinalização
				// horizontal.
				if (Math.abs(target.targetX - getX()) <= Math.abs(target.targetY - getY())) {
					horizontal = false; // Não mova horizontalmente => mova verticalmente
				}
			}
			// Ajuste o rumo do nosso robô com 90 graus, se ele deve se mover
			// horizontalmente.
			// Caso contrário, o rumo calculado será movido verticalmente.
			if (horizontal) {
				newHeading -= Math.PI / 2;
			}
			// Configure o robô para virar à esquerda a quantidade de radianos que acabamos
			// de calcular
			setTurnLeftRadians(Utils.normalRelativeAngle(newHeading));

			// Verifique se nosso robô terminou de girar, ou seja, tem menos de 1 grau
			// restante para
			// virar
			if (Math.abs(getTurnRemaining()) < 1 || Math.abs(getVelocity()) < 0.01) {
				// Se devemos mover horizontalmente, o conjunto do robô para avançar com o
				// distância horizontal até o robô alvo. Caso contrário, use o vertical
				// distância.
				double delta;// delta é a distância delta para mover
				if (horizontal) {
					delta = target.targetX - getX();
				} else {
					delta = target.targetY - getY();
				}
				setAhead(delta);

				// Defina a nova direção do nosso robô para 1 (ou seja, avançar) se o delta
				// a distância é positiva; caso contrário, é definido como -1 (significando
				// mover para trás).
				newDirection = delta > 0 ? 1 : -1;

				// Verifique se mais de 10 curvas se passaram desde que mudamos a direção da
				// última
				// Tempo
				if (getTime() - lastDirectionShift > 10) {
					// Em caso afirmativo, defina a nova direção para ser a direção reversa se a
					// velocidade <1
					if (Math.abs(getVelocity()) < 1) {
						newDirection = direction * -1;
					}
					// Verifique se a direção realmente mudou
					if (newDirection != direction) {
						// Se a nova direção for != da direção atual, defina a direção atual
						// para ser a nova direção e salvar a hora atual para sabermos quando
						// mudou a direção da última vez.
						direction = newDirection;
						lastDirectionShift = getTime();
					}
				}
			}
		}
		// Avance 100 unidades para a frente ou para trás, dependendo da direção
		setAhead(100 * direction);
	}

	/**
	 * O método atualiza o mapa inimigo com base em novos dados de varredura para um
	 * robô varrido.
	 * 
	 * @param scannedRobotEvent é um evento ScannedRobotEvent que contém dados sobre
	 *                          um robô verificado.
	 */
	private void updateEnemyMap(ScannedRobotEvent scannedRobotEvent) {
		// Obtém o nome do robô verificado
		final String scannedRobotName = scannedRobotEvent.getName();

		// Obtenha dados do robô para o robô verificado, se tivermos uma entrada no mapa
		// inimigo
		RobotData scannedRobot = enemyMap.get(scannedRobotName);

		// Verifique se a entrada de dados existe para o robô verificado
		if (scannedRobot == null) {
			// Não existe entrada de dados => Criar uma nova entrada de dados para o robô
			// verificado
			scannedRobot = new RobotData(scannedRobotEvent);
			// Coloque a nova entrada de dados no mapa inimigo
			enemyMap.put(scannedRobotName, scannedRobot);
		} else {
			// Entrada de dados existe => Atualizar a entrada atual com novos dados
			// digitalizados
			scannedRobot.update(scannedRobotEvent);
		}
	}

	/**
	 * Método que atualiza a direção do radar com base em novos dados de varredura
	 * para um robô verificado.
	 * 
	 * @param scannedRobotEvent é um evento ScannedRobotEvent que contém dados sobre
	 *                          um robô verificado.
	 */
	private void updateScanDirection(ScannedRobotEvent scannedRobotEvent) {
		// Obtém o nome do robô verificado
		final String scannedRobotName = scannedRobotEvent.getName();

		// Mude a direção da varredura se e somente se não tivermos registro do mais
		// antigo
		// digitalizado
		// robô ou o robô verificado É o robô verificado mais antigo (com base no nome)
		// E o inimigo
		// mapa contém entradas de dados escaneados para TODOS os robôs (o tamanho do
		// mapa inimigo
		// é igual a
		// o número de robôs oponentes encontrados chamando o método getOthers ()).
		if ((oldestScanned == null || scannedRobotName.equals(oldestScanned.name)) && enemyMap.size() == getOthers()) {

			// Obtenha os dados mais antigos do robô verificado em nosso LinkedHashMap, onde
			// o primeiro valor contém a entrada acessada mais antiga, que é o robô que
			// precisamos obter.
			RobotData oldestScannedRobot = enemyMap.values().iterator().next();

			// Obtenha a posição verificada recentemente (x, y) do robô verificado mais
			// antigo
			double x = oldestScannedRobot.scannedX;
			double y = oldestScannedRobot.scannedY;

			// Obtenha a direção do nosso robô
			double ourHeading = getRadarHeadingRadians();

			// Calcule o rumo do robô mais antigo verificado.
			// O rumo é o ângulo delta entre o rumo do nosso robô e o outro robô,que pode
			// ser um ângulo positivo ou negativo.
			double bearing = bearingTo(ourHeading, x, y);

			// Atualize a direção da varredura com base no rumo.
			// Se o rumo for positivo, o radar será movido para a direita.
			// Se a direção for negativa, o radar será movido para a esquerda.
			scanDir = bearing;
		}
	}

	/**
	 * Atualiza as posições de destino para todos os inimigos. A posição alvo é a
	 * posição em que nosso robô deve atirar para atingir o robô alvo. Este robô usa
	 * a segmentação linear (solução não iterativa exata), conforme descrito no
	 * RoboWiki here: https://robowiki.net/wiki/Linear_Targeting
	 */
	private void updateEnemyTargetPositions() {
		// Passe por todos os robôs no mapa inimigo
		for (RobotData enemy : enemyMap.values()) {

			// Variáveis ​​prefixadas com e- referem-se ao inimigo e b- referem-se ao
			// marcador
			double bV = Rules.getBulletSpeed(PODERDEFOGO);
			double eX = enemy.scannedX;
			double eY = enemy.scannedY;
			double eV = enemy.scannedVelocity;
			double eH = enemy.scannedHeading;

			// Essas constantes tornam o cálculo dos coeficientes quadráticos abaixo mais
			// fácil
			double A = (eX - getX()) / bV;
			double B = (eY - getY()) / bV;
			double C = eV / bV * Math.sin(eH);
			double D = eV / bV * Math.cos(eH);

			// Coeficientes quadráticos: a * (1 / t) ^ 2 + b * (1 / t) + c = 0
			double a = A * A + B * B;
			double b = 2 * (A * C + B * D);
			double c = (C * C + D * D - 1);

			// Se o discriminante da fórmula quadrática é> = 0, temos uma solução
			// significa que em algum momento, t, a bala atingirá o robô inimigo se
			// atirarmos nela agora.
			double discrim = b * b - 4 * a * c;
			if (discrim >= 0) {
				// Fórmula recíproca de quadrática. Calcule as duas soluções possíveis para o
				// tempo, t
				double t1 = 2 * a / (-b - Math.sqrt(discrim));
				double t2 = 2 * a / (-b + Math.sqrt(discrim));

				// Escolha o tempo mínimo positivo ou selecione aquele mais próximo de 0,
				// se o tempo é negativo
				double t = Math.min(t1, t2) >= 0 ? Math.min(t1, t2) : Math.max(t1, t2);

				// Calcule a posição do alvo (x, y) para o inimigo. Esse é o ponto que nosso
				// arma de fogo deve apontar para acertar o inimigo no momento, t.
				double targetX = eX + eV * t * Math.sin(eH);
				double targetY = eY + eV * t * Math.cos(eH);

				// Suponha que o inimigo pare nas paredes. Portanto, limitamos essa posição de
				// destino no
				// paredes.
				double minX = HALF_ROBOT_SIZE;
				double minY = HALF_ROBOT_SIZE;
				double maxX = getBattleFieldWidth() - HALF_ROBOT_SIZE;
				double maxY = getBattleFieldHeight() - HALF_ROBOT_SIZE;

//				enemy.targetX = limit(targetX, minX, maxX);
//				enemy.targetY = limit(targetY, minY, maxY);
			}
		}
	}

	/**
	 * Atualiza qual robô inimigo do mapa inimigo que deve ser nosso atual alvo.
	 */
	private void updateTarget() {
		// Defina o alvo como nulo, o que significa que ainda não temos um robô alvo
		target = null;

		// Crie uma lista de possíveis robôs de destino que é
		// uma cópia dos dados do robô do mapa do inimigo
		List<RobotData> targets = new ArrayList<RobotData>(enemyMap.values());

		// Percorra todos os robôs-alvo possíveis e remova aqueles que estão fora do
		// ataque alcance para este robô sentinela de fronteira, pois nosso robô não
		// pode fazer mal aos robôs fora de seu alcance.
		Iterator<RobotData> it = targets.iterator();
		while (it.hasNext()) {
			RobotData robot = it.next();
			if (isOutsideAttackRange(robot.targetX, robot.targetY)) {
				it.remove();
			}
		}

		// Defina o robô alvo para ser aquele entre todos os robôs alvo possíveis que é
		// mais perto de nosso robô.
		double minDist = Double.POSITIVE_INFINITY;
		for (RobotData robot : targets) {
			double dist = distanceTo(robot.targetX, robot.targetY);
			if (dist < minDist) {
				minDist = dist;
				target = robot;
			}
		}

		// Se ainda não temos um robô alvo, então pegue o primeiro de nossa lista
		// do alvo robôs se a lista não estiver vazia.
		if (target == null && targets.size() > 0) {
			target = targets.get(0);
		}
	}

	/**
	 * Método que atualiza a direção da arma para apontar para o alvo atual.
	 */
	private void updateGunDirection() {
		// Apenas atualize a direção da arma, se tivermos um alvo atual
		if (target != null) {
			// Calcule o rumo entre a arma e o alvo,
			// que pode ser positivo ou negativo
			double targetBearing = bearingTo(getGunHeadingRadians(), target.targetX, target.targetY);
			// Defina a arma para virar à direita a quantidade de radianos definidos pelo
			// rumo para o alvo
			setTurnGunRightRadians(targetBearing); // positivo => vire à direita, negativo => vire à esquerda
		}
	}

	/**
	 * Método que dispara uma bala quando a arma está pronta para disparar.
	 */
	private void fireGunWhenReady() {
		// Nós apenas disparamos a diversão, quando temos um robô alvo
		if (target != null) {
			// Atire apenas quando o ângulo da arma estiver apontando para nosso robô alvo
			// (virtual)

			// Calcule a distância entre o nosso robô e o robô alvo
			double dist = distanceTo(target.targetX, target.targetY);
			// Ângulo que "cobre" o robô alvo do centro até a borda
			double angle = Math.atan(HALF_ROBOT_SIZE / dist);

			// Verifique se o ângulo restante (giro) para mover a arma é menor que nosso
			// cobertura calculada
			// angulo
			if (Math.abs(getGunTurnRemaining()) < angle) {
				// Em caso afirmativo, nossa arma deve estar apontando para nosso alvo para que
				// possamos acertá-lo => fogo !!
				setFire(PODERDEFOGO);
			}
		}
	}

	/**
	 * Método que verifica se uma coordenada (x, y) está fora do Border Sentry raio
	 * de ataque.
	 * 
	 * @param x é a coordenada x.
	 * @param y é a coordenada y.
	 * @return true se a coordenada estiver fora da faixa de ataque; caso contrário,
	 *         false.
	 */
	private boolean isOutsideAttackRange(double x, double y) {
		double minBorderX = getSentryBorderSize();
		double minBorderY = getSentryBorderSize();
		double maxBorderX = getBattleFieldWidth() - getSentryBorderSize();
		double maxBorderY = getBattleFieldHeight() - getSentryBorderSize();

		return (x > minBorderX) && (y > minBorderY) && (x < maxBorderX) && (y < maxBorderY);
	}

	/**
	 * Método que retorna um valor que está dentro de um intervalo de valores
	 * definido por um valor mínimo e máximo com base em um valor de entrada. <br>
	 * Se o valor de entrada for menor que o valor mínimo, o valor retornado será
	 * ser definido com o valor mínimo. <br>
	 * Se o valor de entrada for maior que o valor máximo, o valor retornado será
	 * ser definido com o valor máximo. <br>
	 * Caso contrário, o valor retornado será igual ao valor de entrada.
	 *
	 * @param value é o valor de entrada a ser limitado.
	 * @param min   é o valor mínimo permitido.
	 * @param max   é o valor máximo permitido.
	 * @return o valor de entrada limitado que é garantido estar dentro do
	 *         especificado alcance mínimo e máximo. / private double limit(double
	 *         value, double min, double max) { return Math.min(max, Math.max(min,
	 *         value)); }
	 * 
	 *         / ** Métodos que retornam a distância para uma coordenada (x, y) de
	 *         nosso robô.
	 *
	 * @param x é a coordenada x.
	 * @param y é a coordenada y.
	 * @retorne a distância para a coordenada (x, y).
	 */
	private double distanceTo(double x, double y) {
		return Math.hypot(x - getX(), y - getY());
	}

	/**
	 * Método que retorna o ângulo para uma coordenada (x, y) de nosso robô.
	 *
	 * @param x é a coordenada x.
	 * @param y é a coordenada y.
	 * @retorne o ângulo para a coordenada (x, y).
	 */
	private double angleTo(double x, double y) {
		return Math.atan2(x - getX(), y - getY());
	}

	/**
	 * Method that returns the bearing to a coordinate (x,y) from the position and
	 * heading of our robot. The bearing is the delta angle between the heading of
	 * our robot and the angle of the specified coordinate.
	 * 
	 * @param x is the x coordinate.
	 * @param y is the y coordinate.
	 * @return the angle to the coordinate (x,y).
	 */
	private double bearingTo(double heading, double x, double y) {
		return Utils.normalRelativeAngle(angleTo(x, y) - heading);
	}

	/**
	 * Method that paints a filled circle at the specified coordinate (x,y) and
	 * given color. The circle will have a radius of 20 pixels (meaning that the
	 * diameter will be 40 pixels).
	 * 
	 * @param gfx   is the graphics context to draw within.
	 * @param x     is the x coordinate for the center of the circle.
	 * @param y     is the y coordinate for the center of the circle.
	 * @param color is the color of the filled circle.
	 */
	private void fillCircle(Graphics2D gfx, double x, double y, Color color) {
		// Set the pen color
		gfx.setColor(color);
		// Paint a filled circle (oval) that has a radius of 20 pixels with a center at
		// the input
		// coordinates.
		gfx.fillOval((int) x - 20, (int) y - 20, 40, 40);
	}

	/**
	 * Esta classe é usada para armazenar dados sobre um robô que foi verificado.
	 * <br>
	 * Os dados são principalmente um instantâneo de dados escaneados específicos,
	 * como o escaneado posição (x, y), velocidade e rumo, coloque também o previsto
	 * calculado posição alvo do robô quando nosso robô precisa atirar no escaneado
	 * robô. <br>
	 * Observe que esta classe calcula a posição (x, y) do robô verificado como
	 * nosso robô se move e, portanto, dados como o ângulo e distância do escaneado
	 * o robô mudará com o tempo. usando a posição, é fácil calcular um novo ângulo
	 * e distância para o robô.
	 */
	class RobotData {
		final String name; // nome do robo verificado
		double scannedX; // x coordenada do robô verificado com base na última atualização
		double scannedY; // y coordenada do robô verificado com base na última atualização
		double scannedVelocity; // velocidade do robô verificado desde a última atualização
		double scannedHeading; // cabeçalho do robô verificado da última atualização
		double targetX; // coordenada x predicada para apontar nossa arma, ao atirar no robô
		double targetY; // coordenada y predicada para apontar nossa arma, ao atirar no robô

		/**
		 * Cria uma nova entrada de dados do robô com base em novos dados de varredura
		 * para um robô varrido.
		 *
		 * O evento @param é um evento ScannedRobotEvent que contém dados sobre um robô.
		 */
		RobotData(ScannedRobotEvent event) {
			// Armazene o nome do robô verificado
			name = event.getName();
			// Atualiza todos os fatos digitalizados, como posição, velocidade e direção
			update(event);
			// Inicialize as coordenadas (x, y) para disparar na posição digitalizada
			// atualizada
			targetX = scannedX;
			targetY = scannedY;
		}

		/**
		 * Atualiza os dados verificados com base em novos dados de verificação para um
		 * robô verificado.
		 *
		 * O evento @param é um evento ScannedRobotEvent que contém dados sobre um robô.
		 */
		void update(ScannedRobotEvent event) {
			// Obtenha a posição do robô verificado com base no ScannedRobotEvent
			Point2D.Double pos = getPosition(event);
			// Armazene a posição digitalizada (x, y)
			scannedX = pos.x;
			scannedY = pos.y;
			// Armazene a velocidade e direção digitalizadas
			scannedVelocity = event.getVelocity();
			scannedHeading = event.getHeadingRadians();
		}

		/**
		 * Retorna a posição do robô verificado com base em novos dados de verificação
		 * para um robô verificado.
		 *
		 * O evento @param é um evento ScannedRobotEvent que contém dados sobre um robô.
		 * 
		 * @return a posição (x, y) do robô verificado.
		 */
		Point2D.Double getPosition(ScannedRobotEvent event) {
			// Obtém a distância até o robô verificado
			double distance = event.getDistance();
			// Calcule o ângulo para o robô digitalizado (nosso robô rumo + rumo para
			// digitalizado
			// robô)
			double angle = getHeadingRadians() + event.getBearingRadians();

			// Calcule as coordenadas (x, y) do robô verificado
			double x = getX() + Math.sin(angle) * distance;
			double y = getY() + Math.cos(angle) * distance;

			// Retorne a posição como um ponto (x, y)
			return new Point2D.Double(x, y);
		}
	}
}