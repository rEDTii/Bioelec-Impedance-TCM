#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>

#include <QDebug>
#include <QVBoxLayout>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("BIA Impedance Spectrum");
    resize(1280, 720);

    initChart();
    initSocket();
}

MainWindow::~MainWindow()
{
    delete m_notifier;
    if (m_dataFd >= 0)
        ::close(m_dataFd);
    unlink(BIA_SOCK_PATH);
    delete ui;
}

/* ------------------------------------------------------------------ */
/*  Chart setup                                                       */
/* ------------------------------------------------------------------ */

void MainWindow::initChart()
{
    m_chart = new QChart();
    m_chart->setTitle("BIA Impedance Spectrum");
    m_chart->setAnimationOptions(QChart::NoAnimation);
    m_chart->legend()->setVisible(true);
    m_chart->legend()->setAlignment(Qt::AlignBottom);

    /* ---- Magnitude series ---- */
    m_magSeries = new QLineSeries();
    m_magSeries->setName("|Z| (Ω)");
    QPen magPen(Qt::blue);
    magPen.setWidth(2);
    m_magSeries->setPen(magPen);
    m_chart->addSeries(m_magSeries);

    /* ---- Phase series ---- */
    m_phaseSeries = new QLineSeries();
    m_phaseSeries->setName("Phase (°)");
    QPen phasePen(Qt::red);
    phasePen.setWidth(2);
    m_phaseSeries->setPen(phasePen);
    m_chart->addSeries(m_phaseSeries);

    /* ---- X axis — logarithmic frequency ---- */
    m_xAxis = new QLogValueAxis();
    m_xAxis->setTitleText("Frequency (Hz)");
    m_xAxis->setBase(10);
    m_xAxis->setRange(1e3, 2e5);
    m_xAxis->setLabelFormat("%.0f");
    m_xAxis->setMinorTickCount(-1);
    m_chart->addAxis(m_xAxis, Qt::AlignBottom);
    m_magSeries->attachAxis(m_xAxis);
    m_phaseSeries->attachAxis(m_xAxis);

    /* ---- Left Y axis — Magnitude ---- */
    m_yMagAxis = new QValueAxis();
    m_yMagAxis->setTitleText("|Z| (Ω)");
    m_yMagAxis->setLabelFormat("%.1f");
    m_yMagAxis->setRange(0, 5000);
    m_chart->addAxis(m_yMagAxis, Qt::AlignLeft);
    m_magSeries->attachAxis(m_yMagAxis);

    /* ---- Right Y axis — Phase ---- */
    m_yPhaseAxis = new QValueAxis();
    m_yPhaseAxis->setTitleText("Phase (°)");
    m_yPhaseAxis->setLabelFormat("%.2f");
    m_yPhaseAxis->setRange(-90, 90);
    m_chart->addAxis(m_yPhaseAxis, Qt::AlignRight);
    m_phaseSeries->attachAxis(m_yPhaseAxis);

    /* ---- Chart view ---- */
    m_chartView = new QChartView(m_chart);
    m_chartView->setRenderHint(QPainter::Antialiasing);

    QWidget *central = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(central);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_chartView);
    setCentralWidget(central);

    showMaximized();
}

/* ------------------------------------------------------------------ */
/*  Socket setup (non-blocking)                                       */
/* ------------------------------------------------------------------ */

void MainWindow::initSocket()
{
    m_dataFd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (m_dataFd < 0) {
        qCritical("socket() failed: %s", strerror(errno));
        return;
    }

    unlink(BIA_SOCK_PATH);

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, BIA_SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (bind(m_dataFd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        qCritical("bind() failed: %s", strerror(errno));
        ::close(m_dataFd);
        m_dataFd = -1;
        return;
    }

    int rcvbuf = 65536;
    setsockopt(m_dataFd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    /* Non-blocking: recvfrom returns EAGAIN when empty */
    int flags = fcntl(m_dataFd, F_GETFL, 0);
    fcntl(m_dataFd, F_SETFL, flags | O_NONBLOCK);

    m_notifier = new QSocketNotifier(m_dataFd, QSocketNotifier::Read, this);
    connect(m_notifier, &QSocketNotifier::activated,
            this, &MainWindow::onDataReady);

    qDebug("Listening on %s", BIA_SOCK_PATH);
}

/* ------------------------------------------------------------------ */
/*  Data receive + immediate refresh                                   */
/*  Socket is non-blocking, so this never stalls the event loop.      */
/*  Each QSocketNotifier activation drains all pending datagrams,      */
/*  then refreshes chart once — real-time, no timer needed.            */
/* ------------------------------------------------------------------ */

void MainWindow::onDataReady()
{
    bia_sample_t sample;
    bool gotData = false;

    /* Drain all available datagrams (non-blocking) */
    while (true) {
        ssize_t n = recvfrom(m_dataFd, &sample, sizeof(sample), 0,
                              NULL, NULL);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break;
            qWarning("recvfrom error: %s", strerror(errno));
            break;
        }

        if ((size_t)n != sizeof(sample)) {
            qWarning("Truncated datagram: %zd/%zu bytes", n, sizeof(sample));
            continue;
        }

        m_samples[sample.freq_hz] = sample;
        gotData = true;

        qDebug("Freq=%6u Hz  |Z|=%10.2f Ω  Phase=%8.2f °",
               sample.freq_hz, sample.magnitude, sample.phase);
    }

    if (gotData)
        refreshChart();
}

/* ------------------------------------------------------------------ */
/*  Chart refresh — rebuild series from m_samples                      */
/* ------------------------------------------------------------------ */

void MainWindow::refreshChart()
{
    if (m_samples.isEmpty())
        return;

    /* ---- Rebuild magnitude series ---- */
    m_chart->removeSeries(m_magSeries);
    delete m_magSeries;
    m_magSeries = new QLineSeries();
    m_magSeries->setName("|Z| (Ω)");
    QPen magPen(Qt::blue);
    magPen.setWidth(2);
    m_magSeries->setPen(magPen);

    /* ---- Rebuild phase series ---- */
    m_chart->removeSeries(m_phaseSeries);
    delete m_phaseSeries;
    m_phaseSeries = new QLineSeries();
    m_phaseSeries->setName("Phase (°)");
    QPen phasePen(Qt::red);
    phasePen.setWidth(2);
    m_phaseSeries->setPen(phasePen);

    double magMin = 1e9, magMax = 0;
    double phMin = 180, phMax = -180;
    double freqMin = 1e9, freqMax = 0;

    for (auto it = m_samples.cbegin(); it != m_samples.cend(); ++it) {
        double freq = it.key();
        double mag  = it.value().magnitude;
        double ph   = it.value().phase;

        m_magSeries->append(freq, mag);
        m_phaseSeries->append(freq, ph);

        freqMin = qMin(freqMin, freq);
        freqMax = qMax(freqMax, freq);
        magMin  = qMin(magMin, mag);
        magMax  = qMax(magMax, mag);
        phMin   = qMin(phMin, ph);
        phMax   = qMax(phMax, ph);
    }

    m_chart->addSeries(m_magSeries);
    m_chart->addSeries(m_phaseSeries);

    m_magSeries->attachAxis(m_xAxis);
    m_magSeries->attachAxis(m_yMagAxis);
    m_phaseSeries->attachAxis(m_xAxis);
    m_phaseSeries->attachAxis(m_yPhaseAxis);

    /* Auto-adjust axes */
    m_xAxis->setRange(freqMin * 0.8, freqMax * 1.25);

    /* |Z| Y-axis: min=0 fixed, max=dynamic with 5% padding */
    m_yMagAxis->setRange(0, magMax * 1.05);

    /* Phase Y-axis: fixed -90 to 90 */
    m_yPhaseAxis->setRange(-90, 90);

    m_chartView->update();
}
