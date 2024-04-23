#include "catenary_checker/catenary_checker.hpp"
#include "catenary_checker/obstacle_2d.hpp"
#include "catenary_checker/parabola.hpp"
#include <string>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLineSeries>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QAreaSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QPolarChart>
#include <QtCharts/QChartView>
#include <QtCore/QDebug>

using namespace std;

// Add the Qt namespaces for god sake!!
QT_CHARTS_USE_NAMESPACE

QChartView *represent_problem(const std::vector<Obstacle2D> &scenario, const Point2D &A,
			     const Point2D &B, const Parabola &parabol);

int main(int argc, char **argv) {
  QApplication a(argc, argv);
  std::vector<Obstacle2D> scenario;

  Point2D p1, p2;
  p1.x = 3;   p1.y = 3.0; 
  p2.x = 4.2; p2.y = 4.7;

  scenario.push_back(Obstacle2D::rectangle(p1, p2));
  
  p1.x = 2.0; p1.y = 2.8; 
  scenario.push_back(Obstacle2D::randomObstacle(p1, 20, 0.2));
 
  p1.x = 5.8; p1.y = 3;
  scenario.push_back(Obstacle2D::randomObstacle(p1, 20, 0.2));

  p1.x = 3; p1.y = 4;
  p2.x = 6; p2.y = 5;
  scenario.push_back(Obstacle2D::rectangle(p1, p2));

  int i = 0;

  Parabola parabola;
  // Put the origin (p1) and target (p2) coordinates
  p1.x = 1.5; p1.y = 2;
  p2.x = 8;   p2.y = 8;
  if (parabola.approximateParabola(scenario, p1, p2)) {
    std::cout << "Parabola OK. Parabola params: " <<  parabola.toString() << std::endl;
  } else {
    std::cout << "Parabola failed miserably!\n";
  }

  auto chart_view = represent_problem(scenario, p1, p2, parabola);

  QMainWindow window;
  window.setCentralWidget(chart_view);
  window.resize(800,600);
  window.show();
  
  return a.exec();
}


QChartView *represent_problem(const std::vector<Obstacle2D> &scenario, const Point2D &A,
			     const Point2D &B, const Parabola &parabol) {
  QChartView *ret = new QChartView();

  QChart *chart = new QChart();

  QColor colours[10] = {QColor("cyan"), QColor("magenta"), QColor("red"),
                      QColor("darkRed"), QColor("darkCyan"), QColor("darkMagenta"),
                      QColor("green"), QColor("darkGreen"), QColor("yellow"),
                      QColor("blue")};  
  
  int i = 0;
  for (auto &obs:scenario) {
    ostringstream obs_name;
    obs_name << "obstacle_" << i++;
    chart->addSeries(obs.toSeries(obs_name.str(), 20.0, colours[i%10]));
  }

  QScatterSeries *a_serie = new QScatterSeries();
  a_serie->setName("A");
  a_serie->setMarkerShape(QScatterSeries::MarkerShapeCircle);
  a_serie->setMarkerSize(30.0);
  a_serie->append(A.x, A.y);
  a_serie->append(B.x, B.y);
  chart->addSeries(a_serie);

  // Get the parabola spline
  chart->addSeries(parabol.toSeries("parabola", A.x, B.x));

  chart->setTitle("Collision-Free Catenary");
  chart->createDefaultAxes();
  chart->setDropShadowEnabled(false);

  ret->setChart(chart);
  ret->setRenderHint(QPainter::Antialiasing);
  
  return ret;

}
