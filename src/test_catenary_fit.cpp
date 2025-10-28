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

QChartView *represent_problem(const Point2D &A,
                              const Point2D &B, const Catenary &cat, const Catenary &cat_f,const Catenary &cat_p,
                              const Parabola &parabol, const double &l, string mode_ );
Point2D getThirdPoint(const Point2D &A, const Point2D &B);
double genRandomValue(const double &min_, const double &max_);
void computeCurvesError(const Point2D &A, const Point2D &B, const Parabola &Par, const Catenary &Cat,
                        double &e_sum, double &e_max, double &e_min, double &e_avg);
void saveDataInFile(const Point2D &A, const Point2D &B, double l_par_approx_, double l_par_, double l_cat_approx_, double l_cat_, 
                    double &e_sum, double &e_max, double &e_min, double &e_avg, string name_);
void getPointParabola(const Parabola &Par, const Point2D &p1, const Point2D &p2, Point2D &C, Point2D &D, Point2D &E);
std::ifstream ifile;
std::ofstream ofs;
std::string file_, arg_str;
bool by_fitting = false;
bool by_length = false;

int main(int argc, char **argv) {

  QApplication a(argc, argv);
  QMainWindow window;
  window.resize(800,600);

  Point2D p1, p2, p3;
  int n_iter = 1;
  int count = 1;
  double max_length_allow = 30.0;

  QChartView *chart_view = NULL;

  if (argc > 1) {
    n_iter = atoi(argv[1]);
  }

  while (count < n_iter+1){
    std::cout << "["<< count << "/"<< n_iter <<"] ITERATION" << std::endl;

    p1.x = 0;   p1.y = 1.0; // Init point
    //   p2.x = 9.2; p2.y = 4.7; // Final point
    p2.x = genRandomValue(2.0, 40.0); // Final point, X pose
    p2.y = genRandomValue(2.0, 10.0); // Final point, Y pose
    p3 = getThirdPoint(p1, p2); // Third parable random point To get First parabola
 
    double euclidian_d = sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));

    Parabola parabola(p1, p2, p3);
    double length_par_approx = parabola.getLengthApprox(p1.x, p2.x);
    double length_par = parabola.getLength(p1.x, p2.x);
    if (length_par > max_length_allow || fabs(length_par_approx-euclidian_d) < 0.1){
    std::cout << "\t\t Continue : length_par_approx("<< length_par_approx << ") > max_length_allow(" << max_length_allow << ") ,or,  length_par_approx("<< length_par_approx <<") - euclidian_d(" << euclidian_d <<") < 0.1" <<std::endl;
        continue;
    }
    // Put the origin (p1) and target (p2) coordinates
    std::cout << "P1: " <<  p1.toString() << "\t";
    std::cout << "P2: " <<  p2.toString() << "\t";
    std::cout << "P3: " <<  p3.toString() << std::endl;
    std::cout << "Parabola params: " <<  parabola.toString();

    std::cout << "Parabola length: " << length_par <<std::endl;

    Catenary cat;
    double length_cat, length_cat_approx;
    /*----------- By Length Method Stuff -----------*/
    std::cout << "*** Aproximating Catenary byLength: ***" << std::endl;
    cat.approximateByLength(p1, p2, length_par);
    length_cat = cat.getLength(p1.x, p2.x);
    length_cat_approx = cat.getLengthApprox(p1.x, p2.x);
    std::cout << "Catenary length_approx: " <<  length_cat_approx << " , length: " <<  length_cat <<std::endl;
    // Compute Errors and Save data for Analysis 
         double e_sum, e_max, e_min, e_avg;
    computeCurvesError(p1, p2, parabola, cat, e_sum, e_max, e_min, e_avg);
    saveDataInFile(p1, p2, length_par, length_par_approx, length_cat, length_cat_approx, e_sum, e_max, e_min, e_avg, "byLength");

    /*----------- By Fitting Method Stuff -----------*/
    
    std::cout << "*** Aproximating Catenary byFitting: ***" << std::endl;
    Catenary cat_fit;
    cat_fit.approximateByFitting(p1, p2, length_par, parabola, max_length_allow, a);
    length_cat = cat_fit.getLength(p1.x, p2.x);
    length_cat_approx = cat_fit.getLengthApprox(p1.x, p2.x);
    std::cout << "Catenary length_approx: " <<  length_cat_approx << " , length: " <<  length_cat <<std::endl;
    // Compute Errors and Save data for Analysis 
         
    e_sum = e_max = e_min = e_avg = 0.0;
    computeCurvesError(p1, p2, parabola, cat_fit, e_sum, e_max, e_min, e_avg);
    saveDataInFile(p1, p2, length_par, length_par_approx, length_cat, length_cat_approx, e_sum, e_max, e_min, e_avg, "byFitting");
        
    /*----------- By Points Method Stuff -----------*/
    std::cout << "*** Aproximating Catenary byPoints: ***" << std::endl;
    Point2D C, D, E;
    getPointParabola(parabola, p1, p2, C, D, E);
    Catenary cat_point;
    cat_point.approximateByPoints(p1, p2, C, D, E);
    length_cat = cat_point.getLength(p1.x, p2.x);
    length_cat_approx = cat_point.getLengthApprox(p1.x, p2.x);
    std::cout << "Catenary length_approx: " <<  length_cat_approx << " , length: " <<  length_cat <<std::endl;

    // Compute Errors and Save data for Analysis

    e_sum = e_max = e_min = e_avg = 0.0;
    computeCurvesError(p1, p2, parabola, cat_point, e_sum, e_max, e_min, e_avg);
    saveDataInFile(p1, p2, length_par, length_par_approx, length_cat, length_cat_approx, e_sum, e_max, e_min, e_avg, "byPoints");
    // // Plot Graph

    chart_view = represent_problem(p1, p2, cat, cat_fit, cat_point, parabola,length_par, "byPoints");
    count++;
  }

  if (chart_view != NULL) {
    window.setCentralWidget(chart_view);
    window.show();
    a.processEvents(); // Process the events to ensure the plot is shown
  }
  return a.exec();

}

QChartView *represent_problem(const Point2D &A,
                              const Point2D &B, const Catenary &cat, const Catenary &cat_f,const Catenary &cat_p,
                              const Parabola &parabol, const double &l, string mode_ ) {
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
  QLineSeries *catenary_series = cat.toSeries("catenary length", A.x, B.x);
  catenary_series->setColor(colours[9 % 10]);  // Set the color for the catenary
  chart->addSeries(catenary_series);

  QLineSeries *catenary_series2 = cat_f.toSeries("catenary fit", A.x, B.x);
  catenary_series2->setColor(colours[0 % 10]);  // Set the color for the catenary
  chart->addSeries(catenary_series2);

  QLineSeries *catenary_series3 = cat_p.toSeries("catenary_points", A.x, B.x);
  catenary_series3->setColor(colours[1 % 10]);  // Set the color for the catenary
  chart->addSeries(catenary_series3);


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

void getPointParabola(const Parabola &Par, const Point2D &p1, const Point2D &p2, Point2D &C, Point2D &D, Point2D &E){
    C.x = (p1.x+p2.x)/2.0;
    D.x = (p1.x+C.x)/2.0;
    E.x = (C.x+p2.x)/2.0;
    C.y = Par.apply(C.x);
    D.y = Par.apply(D.x);
    E.y = Par.apply(E.x);
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
