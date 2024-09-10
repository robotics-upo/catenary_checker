#include "catenary_checker/catenary_checker.hpp"
#include "catenary_checker/obstacle_2d.hpp"
#include "catenary_checker/parabola.hpp"
#include "catenary_checker/catenary.hpp"
#include <string>
#include <random> 
#include <iostream>
#include <fstream>

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

QChartView *represent_problem(const Point2D &A, const Point2D &B, const Catenary &cat, const Parabola &parabol, const double &l);
Point2D getThirdPoint(const Point2D &A, const Point2D &B);
double genRandomValue(const double &min_, const double &max_);
void computeCurvesError(const Point2D &A, const Point2D &B, const Parabola &Par, const Catenary &Cat,
                        double &e_sum, double &e_max, double &e_min, double &e_avg);
void saveDataInFile(const Point2D &A, const Point2D &B, double l_par_approx_, double l_par_, double l_cat_approx_, double l_cat_, 
                    double &e_sum, double &e_max, double &e_min, double &e_avg, string name_);

std::ifstream ifile;
std::ofstream ofs;
std::string file_;

int main(int argc, char **argv) {
  QApplication a(argc, argv);
  std::vector<Obstacle2D> scenario;
  QMainWindow window;
  window.resize(800,600);

  Point2D p1, p2, p3;
  int n_iter = 100;
  int count = 1;
  while (count < n_iter+1){
    std::cout << "["<< count << "/"<< n_iter <<"] ITERATION" << std::endl;

    p1.x = 0;   p1.y = 1.0; // Init point
    //   p2.x = 9.2; p2.y = 4.7; // Final point
    p2.x = genRandomValue(2.0, 30.0); // Final point, X pose
    p2.y = genRandomValue(2.0, 10.0); // Final point, Y pose
    p3 = getThirdPoint(p1, p2); // Third parable random point

    Parabola parabola(p1, p2, p3);
    double length_par_approx = parabola.getLengthApprox(p1.x, p2.x);
    double length_par = parabola.getLength(p1.x, p2.x);
    if (length_par_approx > 30.0)
        continue;
    // Put the origin (p1) and target (p2) coordinates
    std::cout << "P1: " <<  p1.toString() << "\t";
    std::cout << "P2: " <<  p2.toString() << "\t";
    std::cout << "P3: " <<  p3.toString() << std::endl;
    std::cout << "Parabola params: " <<  parabola.toString();

    std::cout << "Parabola length_approx: " <<  length_par_approx << " , length: " <<  length_par <<std::endl;

    Catenary cat;
    std::cout << "Aproximation byLength " << std::endl;
    cat.approximateByLength(p1, p2, length_par_approx);
    double length_cat = cat.getLength(p1.x, p2.x);
    double length_cat_approx = cat.getLengthApprox(p1.x, p2.x);
    std::cout << "Catenary length_approx: " <<  length_cat_approx << " , length_approx: " <<  length_cat <<std::endl;

    double e_sum, e_max, e_min, e_avg;
    computeCurvesError(p1, p2, parabola, cat, e_sum, e_max, e_min, e_avg);
    saveDataInFile(p1, p2, length_par, length_par_approx, length_cat, length_cat_approx, e_sum, e_max, e_min, e_avg, "byLength");

    auto chart_view = represent_problem(p1, p2, cat, parabola,length_par_approx);

    window.setCentralWidget(chart_view);
    window.show();

    // Process the events to ensure the plot is shown
    a.processEvents();
    a.exec();
    count++;
  }

  return a.exec();
}

QChartView *represent_problem(const Point2D &A,
			     const Point2D &B, const Catenary &cat, const Parabola &parabol, const double &l ) {
  QChartView *ret = new QChartView();

  QChart *chart = new QChart();

  QColor colours[10] = {QColor("cyan"), QColor("magenta"), QColor("red"),
                      QColor("darkRed"), QColor("darkCyan"), QColor("darkMagenta"),
                      QColor("green"), QColor("darkGreen"), QColor("yellow"),
                      QColor("blue")};  
  int i = 0;

  QScatterSeries *a_serie = new QScatterSeries();
  a_serie->setName("A");
  a_serie->setMarkerShape(QScatterSeries::MarkerShapeCircle);
  a_serie->setMarkerSize(30.0);
  a_serie->append(A.x, A.y);
  a_serie->append(B.x, B.y);
  chart->addSeries(a_serie);

  // Get the parabola spline and customize its color
  QLineSeries *parabola_series = parabol.toSeries("parabola", A.x, B.x);
  parabola_series->setColor(colours[2 % 10]);  // Set the color for the parabola
  chart->addSeries(parabola_series);

  // Get the catenary spline and customize its color
  QLineSeries *catenary_series = cat.toSeries("catenary", A.x, B.x);
  catenary_series->setColor(colours[9 % 10]);  // Set the color for the catenary
  chart->addSeries(catenary_series);

  chart->setTitle(QString::fromStdString("Parabola v/s Catenary  length: "+ to_string(l)));
  chart->createDefaultAxes();
  chart->setDropShadowEnabled(false);

  ret->setChart(chart);
  ret->setRenderHint(QPainter::Antialiasing);
  
  return ret;
}

Point2D getThirdPoint(const Point2D &A, const Point2D &B){
    Point2D C;
    C.x = genRandomValue(A.x+0.5, B.x-0.5);
    double Ymax = ((B.y - A.y)/(B.x - A.x))*(C.x - A.x)+ A.y;
    C.y = genRandomValue(-5.0, Ymax);

    return C;
}

double genRandomValue(const double &min_, const double &max_){

    std::random_device rd;   // Random Origen
    std::mt19937 gen(rd());  // Mersenne Twister: generador de números pseudoaleatorios
    std::uniform_real_distribution<double> distrib(min_, max_); // Distribution number between values
    
    return (round((distrib(gen))* 10.0) / 10.0);
}

void computeCurvesError(const Point2D &A, const Point2D &B, 
                        const Parabola &Par, const Catenary &Cat,
                        double &e_sum, double &e_max, double &e_min, double &e_avg){
    double delta_t = 0.01;
    e_sum = e_max = e_avg = 0.0;
    e_min = 1000.0;
    for (double x = A.x + delta_t; x <= B.x; x += delta_t) {
        double y_par = Par.apply(x);
        double y_cat = Cat.apply(x);
        double error = fabs(y_par - y_cat);
        e_sum = e_sum + error; // sum error
        if(e_max < error)
            e_max = error;  //max error
        if(e_min > error)
            e_min = error;  //min error
    }
    e_avg = e_sum/(fabs(B.x-A.x)/delta_t ); //mean error
}

void saveDataInFile(const Point2D &A, const Point2D &B, 
                    double l_par_approx_, double l_par_, double l_cat_approx_, double l_cat_, 
                    double &e_sum, double &e_max, double &e_min, double &e_avg,
                    string name_){

    const char* home = getenv("HOME"); // Extender a ruta explicita de home
    if (home != nullptr) 
        file_ = std::string(home) + "/results_error_catenary_vs_parabola_"+name_+".txt";
    else
        file_ = "results_error_catenary_vs_parabola_"+name_+".txt";  // Ruta de fallback

    ifile.open(file_);
    if(ifile)
        std::cout << "      " << file_ <<" : File exists !!!!!!!!!! " << std::endl;
    else
        std::cout << "      " << file_ <<" : File doesn't exist !!!!!!!!!! " << std::endl;
    ofs.open(file_.c_str(), std::ofstream::app);

    if (ofs.is_open()) {
        ofs << A.x << ","
            << A.y << ","
            << B.x << ","
            << B.y << ","
            << l_par_approx_ << ","
            << l_par_ << "," 
            << l_cat_approx_ << ","
            << l_cat_ << ","
            << e_sum << ","
            << e_max << "," 
            << e_min << ","
            << e_avg << ","
            << std::endl;
    } 
    else 
        std::cout << "Couldn't open " << file_ << " for writing." << std::endl;
    ofs.close(); 
}   