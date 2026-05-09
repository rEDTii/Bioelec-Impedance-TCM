#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSocketNotifier>
#include <QMap>
#include <QPushButton>
#include <QLabel>
#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QLogValueAxis>
#include <QtCharts/QValueAxis>

/* Must match ad5940_bia_daemon.c */
#define BIA_DATA_SOCK_PATH  "/tmp/bia_sample.sock"
#define BIA_CMD_SOCK_PATH   "/tmp/bia_cmd.sock"

/* Command bytes (single-char datagrams) */
#define CMD_START  'S'
#define CMD_STOP   'T'
#define CMD_STATUS '?'
#define CMD_QUIT   'Q'

/* Meta-info magic: distinguishes meta packet from data sample */
#define BIA_META_MAGIC  0xB1A00000u

QT_CHARTS_USE_NAMESPACE

struct bia_sample_t {
    float magnitude;
    float phase;
    float resistance;
    float reactance;
    quint32 freq_hz;
    qint32 curr_real;
    qint32 curr_imag;
    qint32 volt_real;
    qint32 volt_imag;
};

/* Sent by demo before first data after each START */
struct bia_meta_t {
    quint32 magic;         /* = BIA_META_MAGIC */
    quint32 sweep_points;  /* number of frequency points */
    quint32 sweep_type;    /* 0=linear, 1=log, 2=custom */
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

private:
    void initChart();
    void initDataSocket();
    bool sendCommand(char cmd);
    void startAcquisition();
    void stopAcquisition();
    void setAcquiring(bool on);
    void refreshChart();

    Ui::MainWindow *ui;

    /* Data socket (receives samples from demo) */
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
    bool m_acquiring = false;
    int m_sweepPoints = 0;
    int m_receivedPoints = 0;
};
#endif // MAINWINDOW_H
