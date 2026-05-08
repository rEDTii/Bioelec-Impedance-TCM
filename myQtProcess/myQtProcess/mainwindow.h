#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSocketNotifier>
#include <QMap>
#include <QProcess>
#include <QPushButton>
#include <QLabel>
#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QLogValueAxis>
#include <QtCharts/QValueAxis>

/* Must match ad5940_bia_demo.c */
#define BIA_SOCK_PATH  "/tmp/bia_sample.sock"

/* IIO sysfs paths — may be device0 or device1, auto-detected */
#define IIO_DEVICE_GLOB  "/sys/bus/iio/devices/iio:device*"

/*
 * Search paths for ad5940_bia_demo, tried in order.
 * Edit if your deployment puts the binary elsewhere.
 */
static const char * const DEMO_BIN_PATHS[] = {
    "/mydrivers/ad5940_bia_demo", // 开发时使用的目录
    "/usr/local/bin/ad5940_bia_demo",
    "/usr/bin/ad5940_bia_demo",
    "/opt/ad5940/ad5940_bia_demo",
    "./ad5940_bia_demo",
};

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
    void onToggleButton();
    void onDataReady();
    void onDemoFinished(int exitCode, QProcess::ExitStatus status);

private:
    void initChart();
    void initSocket();
    void refreshChart();
    void startAcquisition();
    void stopAcquisition();
    void setAcquiring(bool on);

    int  readSweepPoints();
    QString findIioDevice();
    bool enableIioBuffer(bool enable);

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

    /* Acquisition control */
    QPushButton *m_btnToggle = nullptr;
    QLabel *m_statusLabel = nullptr;
    QProcess *m_demoProcess = nullptr;
    bool m_acquiring = false;
    int m_sweepPoints = 0;      /* 0 = not yet read */
    int m_receivedPoints = 0;   /* unique freq points received this round */
    QString m_iioDevicePath;    /* e.g. /sys/bus/iio/devices/iio:device1 */
};
#endif // MAINWINDOW_H
