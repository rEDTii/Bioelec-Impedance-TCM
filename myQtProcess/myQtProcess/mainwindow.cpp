#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <glob.h>
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QDir>

/* ------------------------------------------------------------------ */
/*  IIO sysfs helpers                                                  */
/* ------------------------------------------------------------------ */

/*
 * findIioDevice — Locate the AD5940 IIO device sysfs path.
 * Scans /sys/bus/iio/devices/iio:device* for one whose "name"
 * file contains "ad5940".
 */
QString MainWindow::findIioDevice()
{
    glob_t gl;
    if (glob(IIO_DEVICE_GLOB, 0, NULL, &gl) != 0)
        return QString();

    QString result;
    for (size_t i = 0; i < gl.gl_pathc; i++) {
        QString path = QString::fromLocal8Bit(gl.gl_pathv[i]);
        QString namePath = path + "/name";
        QFile f(namePath);
        if (f.open(QIODevice::ReadOnly)) {
            QString name = QString::fromUtf8(f.readAll()).trimmed();
            if (name == "ad5940") {
                result = path;
                break;
            }
        }
    }
    globfree(&gl);
    return result;
}

/*
 * readSweepPoints — Read sweep_points from driver module parameter sysfs.
 * Path: /sys/module/ad5940/parameters/sweep_points
 */
int MainWindow::readSweepPoints()
{
    QFile f("/sys/module/ad5940/parameters/sweep_points");
    if (!f.open(QIODevice::ReadOnly)) {
        qWarning() << "Cannot read sweep_points:" << f.errorString();
        return 0;
    }
    bool ok;
    int val = QString::fromUtf8(f.readAll()).trimmed().toInt(&ok);
    return ok ? val : 0;
}

/*
 * enableIioBuffer — Enable or disable the IIO triggered buffer.
 * Writes "1" or "0" to <device>/buffer/enable.
 */
bool MainWindow::enableIioBuffer(bool enable)
{
    if (m_iioDevicePath.isEmpty()) {
        qWarning() << "IIO device path not found";
        return false;
    }
    QString path = m_iioDevicePath + "/buffer/enable";
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        qWarning() << "Cannot open" << path << ":" << f.errorString();
        return false;
    }
    if (f.write(enable ? "1" : "0") != 1) {
        qWarning() << "Write failed to" << path;
        return false;
    }
    qDebug() << "IIO buffer" << (enable ? "enabled" : "disabled");
    return true;
}

/* ------------------------------------------------------------------ */
/*  Acquisition control                                                */
/* ------------------------------------------------------------------ */

void MainWindow::startAcquisition()
{
    /* Read sweep_points for round-completion detection */
    m_sweepPoints = readSweepPoints();
    if (m_sweepPoints <= 0) {
        QMessageBox::warning(this, tr("Error"),
                             tr("Cannot read sweep_points from driver.\n"
                                "Is the ad5940 module loaded?"));
        return;
    }

    /* Clear previous data */
    m_samples.clear();
    m_receivedPoints = 0;

    /* Enable IIO buffer (starts WUPT in driver) */
    if (!enableIioBuffer(true)) {
        QMessageBox::warning(this, tr("Error"),
                             tr("Failed to enable IIO buffer.\n"
                                "Device: %1").arg(m_iioDevicePath));
        return;
    }

    /* Launch ad5940_bia_demo (reads IIO stream, sends to Unix socket) */
    if (!m_demoProcess) {
        m_demoProcess = new QProcess(this);
        connect(m_demoProcess,
                QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, &MainWindow::onDemoFinished);
    }

    /* Find ad5940_bia_demo binary */
    QString demoBin;
    for (auto p : DEMO_BIN_PATHS) {
        if (QFile::exists(QString::fromUtf8(p))) {
            demoBin = QString::fromUtf8(p);
            break;
        }
    }
    if (demoBin.isEmpty()) {
        QString searched;
        for (auto p : DEMO_BIN_PATHS)
            searched += QString("  %1\n").arg(p);
        QMessageBox::warning(this, tr("Error"),
                             tr("Cannot find ad5940_bia_demo.\n"
                                "Searched:\n%1")
                             .arg(searched));
        enableIioBuffer(false);
        return;
    }

    m_demoProcess->start(demoBin, QStringList());
    if (!m_demoProcess->waitForStarted(3000)) {
        QMessageBox::warning(this, tr("Error"),
                             tr("Failed to start %1\n%2")
                             .arg(demoBin)
                             .arg(m_demoProcess->errorString()));
        enableIioBuffer(false);
        return;
    }

    setAcquiring(true);
    qDebug() << "Acquisition started, sweep_points =" << m_sweepPoints;
}

void MainWindow::stopAcquisition()
{
    /* Terminate demo process */
    if (m_demoProcess && m_demoProcess->state() != QProcess::NotRunning) {
        m_demoProcess->terminate();
        if (!m_demoProcess->waitForFinished(2000))
            m_demoProcess->kill();
    }

    /* Disable IIO buffer (stops WUPT in driver) */
    enableIioBuffer(false);

    setAcquiring(false);
    qDebug() << "Acquisition stopped,"
             << "received" << m_receivedPoints << "of"
             << m_sweepPoints << "points";
}

void MainWindow::setAcquiring(bool on)
{
    m_acquiring = on;
    if (on) {
        m_btnToggle->setText(tr("取消"));
        m_btnToggle->setStyleSheet(
            "QPushButton { background-color: #e74c3c; color: white; "
            "font-size: 16px; padding: 8px 24px; border-radius: 4px; }");
        m_statusLabel->setText(tr("采集中... 0/%1 个频点").arg(m_sweepPoints));
    } else {
        m_btnToggle->setText(tr("启动"));
        m_btnToggle->setStyleSheet(
            "QPushButton { background-color: #27ae60; color: white; "
            "font-size: 16px; padding: 8px 24px; border-radius: 4px; }");
        if (m_receivedPoints >= m_sweepPoints && m_sweepPoints > 0)
            m_statusLabel->setText(tr("采集完成 (%1 个频点)").arg(m_receivedPoints));
        else
            m_statusLabel->setText(tr("就绪"));
    }
}

/* ------------------------------------------------------------------ */
/*  Constructor / Destructor                                           */
/* ------------------------------------------------------------------ */

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("BIA Impedance Spectrum");
    resize(1280, 720);

    /* Discover IIO device path */
    m_iioDevicePath = findIioDevice();
    if (m_iioDevicePath.isEmpty())
        qWarning() << "AD5940 IIO device not found!";

    initChart();
    initSocket();

    /* Initial state: not acquiring */
    setAcquiring(false);
}

MainWindow::~MainWindow()
{
    if (m_acquiring)
        stopAcquisition();
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
    m_xAxis->setRange(100, 200000);
    m_xAxis->setLabelFormat("%.0f");
    m_xAxis->setMinorTickCount(8);
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

    /* ---- Bottom control bar: button + status label ---- */
    m_btnToggle = new QPushButton(tr("启动"));
    m_btnToggle->setMinimumSize(120, 40);
    connect(m_btnToggle, &QPushButton::clicked,
            this, &MainWindow::onToggleButton);

    m_statusLabel = new QLabel(tr("就绪"));
    m_statusLabel->setStyleSheet("font-size: 14px; color: #555;");

    QHBoxLayout *ctrlLayout = new QHBoxLayout();
    ctrlLayout->addWidget(m_btnToggle);
    ctrlLayout->addSpacing(16);
    ctrlLayout->addWidget(m_statusLabel);
    ctrlLayout->addStretch();

    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->addWidget(m_chartView, 1);
    mainLayout->addLayout(ctrlLayout);

    QWidget *central = new QWidget(this);
    central->setLayout(mainLayout);
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
/*  Button handler                                                     */
/* ------------------------------------------------------------------ */

void MainWindow::onToggleButton()
{
    if (m_acquiring) {
        /* Cancel: stop acquisition mid-way */
        stopAcquisition();
    } else {
        /* Start a new acquisition round */
        startAcquisition();
    }
}

/* ------------------------------------------------------------------ */
/*  Demo process finished handler                                      */
/* ------------------------------------------------------------------ */

void MainWindow::onDemoFinished(int exitCode, QProcess::ExitStatus status)
{
    Q_UNUSED(exitCode);
    Q_UNUSED(status);
    qDebug() << "ad5940_bia_demo process finished";

    /* If we were acquiring, clean up the driver side too */
    if (m_acquiring)
        stopAcquisition();
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

        /* Count unique frequencies for round-completion detection */
        if (!m_samples.contains(sample.freq_hz))
            m_receivedPoints++;

        m_samples[sample.freq_hz] = sample;
        gotData = true;

        qDebug("Freq=%6u Hz  |Z|=%10.2f Ω  Phase=%8.2f °  DFT: I=(%d,%d) V=(%d,%d)",
               sample.freq_hz, sample.magnitude, sample.phase,
               sample.curr_real, sample.curr_imag,
               sample.volt_real, sample.volt_imag);
    }

    if (gotData) {
        /* Update status label with progress */
        if (m_acquiring && m_sweepPoints > 0) {
            m_statusLabel->setText(
                tr("采集中... %1/%2 个频点")
                .arg(m_receivedPoints).arg(m_sweepPoints));
        }

        refreshChart();

        /* Check if one full sweep round is complete */
        if (m_acquiring && m_sweepPoints > 0
            && m_receivedPoints >= m_sweepPoints) {
            qDebug() << "Sweep round complete:"
                     << m_receivedPoints << "points received";
            stopAcquisition();
        }
    }
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

    /* Auto-adjust X axis (log scale — ensure positive minimum) */
    if (freqMin < 1.0) freqMin = 1.0;
    m_xAxis->setRange(freqMin * 0.5, freqMax * 2.0);

    /* |Z| Y-axis: min=0 fixed, max=dynamic with 5% padding */
    m_yMagAxis->setRange(0, magMax * 1.05);

    /* Phase Y-axis: fixed -90 to 90 */
    m_yPhaseAxis->setRange(-90, 90);

    m_chartView->update();
}
