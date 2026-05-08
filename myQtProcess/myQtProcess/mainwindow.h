#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSocketNotifier>
#include <QMap>
#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QLogValueAxis>
#include <QtCharts/QValueAxis>

/* Must match ad5940_bia_demo.c / dummy_Qt.c */
#define BIA_SOCK_PATH  "/tmp/bia_sample.sock"

QT_CHARTS_USE_NAMESPACE

struct bia_sample_t {
    float magnitude;     /* |Z| in Ohms */
    float phase;         /* angle(Z) in degrees */
    float resistance;    /* Real part R in Ohms */
    float reactance;     /* Imaginary part X in Ohms */
    quint32 freq_hz;     /* Excitation frequency in Hz */
    qint32 curr_real;    /* Raw DFT: current channel real part (18-bit signed) */
    qint32 curr_imag;    /* Raw DFT: current channel imaginary part */
    qint32 volt_real;    /* Raw DFT: voltage channel real part */
    qint32 volt_imag;    /* Raw DFT: voltage channel imaginary part */
};

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onDataReady();

private:
    void initSocket();
    void initChart();
    void refreshChart();

    Ui::MainWindow *ui;

    /* Socket */
    int m_dataFd = -1;
    QSocketNotifier *m_notifier = nullptr;

    /* Data: freq → latest sample */
    QMap<quint32, bia_sample_t> m_samples;

    /* Chart */
    QChartView *m_chartView = nullptr;
    QChart *m_chart = nullptr;
    QLineSeries *m_magSeries = nullptr;
    QLineSeries *m_phaseSeries = nullptr;
    QLogValueAxis *m_xAxis = nullptr;
    QValueAxis *m_yMagAxis = nullptr;
    QValueAxis *m_yPhaseAxis = nullptr;
};
#endif // MAINWINDOW_H
