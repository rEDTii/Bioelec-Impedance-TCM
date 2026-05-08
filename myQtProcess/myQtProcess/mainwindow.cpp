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
#include <QHBoxLayout>
#include <QMessageBox>

/* ------------------------------------------------------------------ */
/*  Command socket helper                                              */
/* ------------------------------------------------------------------ */

/*
 * sendCommand — Send a single-byte command to ad5940_bia_demo.
 * Uses DGRAM (connectionless), so open → send → close each time.
 */
bool MainWindow::sendCommand(char cmd)
{
    int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) {
        qWarning("cmd socket(): %s", strerror(errno));
        return false;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, BIA_CMD_SOCK_PATH, sizeof(addr.sun_path) - 1);

    ssize_t n = sendto(fd, &cmd, 1, 0,
                        (struct sockaddr *)&addr, sizeof(addr));
    ::close(fd);

    if (n != 1) {
        qWarning("sendCommand('%c') failed: %zd", cmd, n);
        return false;
    }
    qDebug("sendCommand('%c') OK", cmd);
    return true;
}

/* ------------------------------------------------------------------ */
/*  Acquisition control                                                */
/* ------------------------------------------------------------------ */

void MainWindow::startAcquisition()
{
    /* Clear previous data */
    m_samples.clear();
    m_receivedPoints = 0;

    /* Send START to daemon — it will enable IIO buffer and stream data */
    if (!sendCommand(CMD_START)) {
        QMessageBox::warning(this, tr("Error"),
                             tr("Failed to send START command.\n"
                                "Is ad5940_bia_demo running?\n"
                                "Listening on %1").arg(BIA_CMD_SOCK_PATH));
        return;
    }

    setAcquiring(true);
}

void MainWindow::stopAcquisition()
{
    /* Send STOP to daemon — it will disable IIO buffer */
    sendCommand(CMD_STOP);

    setAcquiring(false);
    qDebug("Acquisition stopped, received %d/%d points",
           m_receivedPoints, m_sweepPoints);
}

void MainWindow::setAcquiring(bool on)
{
    m_acquiring = on;
    if (on) {
        m_btnToggle->setText(tr("取消"));
        m_btnToggle->setStyleSheet(
            "QPushButton { background-color: #e74c3c; color: white; "
            "font-size: 16px; padding: 8px 24px; border-radius: 4px; }");
        m_statusLabel->setText(tr("等待数据..."));
    } else {
        m_btnToggle->setText(tr("启动"));
        m_btnToggle->setStyleSheet(
            "QPushButton { background-color: #27ae60; color: white; "
            "font-size: 16px; padding: 8px 24px; border-radius: 4px; }");
        if (m_sweepPoints > 0 && m_receivedPoints >= m_sweepPoints)
            m_statusLabel->setText(
                tr("采集完成 (%1/%2 个频点)")
                .arg(m_receivedPoints).arg(m_sweepPoints));
        else if (m_receivedPoints > 0)
            m_statusLabel->setText(
                tr("已停止 (%1/%2 个频点)")
                .arg(m_receivedPoints).arg(m_sweepPoints));
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

    initChart();
    initDataSocket();

    setAcquiring(false);
}

MainWindow::~MainWindow()
{
    if (m_acquiring)
        stopAcquisition();
    delete m_notifier;
    if (m_dataFd >= 0)
        ::close(m_dataFd);
    unlink(BIA_DATA_SOCK_PATH);
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

    /* Magnitude series */
    m_magSeries = new QLineSeries();
    m_magSeries->setName("|Z| (Ω)");
    QPen magPen(Qt::blue);
    magPen.setWidth(2);
    m_magSeries->setPen(magPen);
    m_chart->addSeries(m_magSeries);

    /* Phase series */
    m_phaseSeries = new QLineSeries();
    m_phaseSeries->setName("Phase (°)");
    QPen phasePen(Qt::red);
    phasePen.setWidth(2);
    m_phaseSeries->setPen(phasePen);
    m_chart->addSeries(m_phaseSeries);

    /* X axis — logarithmic frequency */
    m_xAxis = new QLogValueAxis();
    m_xAxis->setTitleText("Frequency (Hz)");
    m_xAxis->setBase(10);
    m_xAxis->setRange(100, 200000);
    m_xAxis->setLabelFormat("%.0f");
    m_xAxis->setMinorTickCount(8);
    m_chart->addAxis(m_xAxis, Qt::AlignBottom);
    m_magSeries->attachAxis(m_xAxis);
    m_phaseSeries->attachAxis(m_xAxis);

    /* Left Y axis — Magnitude */
    m_yMagAxis = new QValueAxis();
    m_yMagAxis->setTitleText("|Z| (Ω)");
    m_yMagAxis->setLabelFormat("%.1f");
    m_yMagAxis->setRange(0, 5000);
    m_chart->addAxis(m_yMagAxis, Qt::AlignLeft);
    m_magSeries->attachAxis(m_yMagAxis);

    /* Right Y axis — Phase */
    m_yPhaseAxis = new QValueAxis();
    m_yPhaseAxis->setTitleText("Phase (°)");
    m_yPhaseAxis->setLabelFormat("%.2f");
    m_yPhaseAxis->setRange(-90, 90);
    m_chart->addAxis(m_yPhaseAxis, Qt::AlignRight);
    m_phaseSeries->attachAxis(m_yPhaseAxis);

    /* Chart view */
    m_chartView = new QChartView(m_chart);
    m_chartView->setRenderHint(QPainter::Antialiasing);

    /* Control bar */
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
/*  Data socket setup (receives samples from demo)                     */
/* ------------------------------------------------------------------ */

void MainWindow::initDataSocket()
{
    m_dataFd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (m_dataFd < 0) {
        qCritical("socket() failed: %s", strerror(errno));
        return;
    }

    unlink(BIA_DATA_SOCK_PATH);

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, BIA_DATA_SOCK_PATH, sizeof(addr.sun_path) - 1);

    if (bind(m_dataFd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        qCritical("bind(%s): %s", BIA_DATA_SOCK_PATH, strerror(errno));
        ::close(m_dataFd);
        m_dataFd = -1;
        return;
    }

    int rcvbuf = 65536;
    setsockopt(m_dataFd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    int flags = fcntl(m_dataFd, F_GETFL, 0);
    fcntl(m_dataFd, F_SETFL, flags | O_NONBLOCK);

    m_notifier = new QSocketNotifier(m_dataFd, QSocketNotifier::Read, this);
    connect(m_notifier, &QSocketNotifier::activated,
            this, &MainWindow::onDataReady);

    qDebug("Listening on %s", BIA_DATA_SOCK_PATH);
}

/* ------------------------------------------------------------------ */
/*  Button handler                                                     */
/* ------------------------------------------------------------------ */

void MainWindow::onToggleButton()
{
    if (m_acquiring)
        stopAcquisition();
    else
        startAcquisition();
}

/* ------------------------------------------------------------------ */
/*  Data receive + chart refresh                                      */
/*  Handles both bia_meta_t (meta info) and bia_sample_t (data).       */
/* ------------------------------------------------------------------ */

void MainWindow::onDataReady()
{
    char buf[sizeof(bia_meta_t) > sizeof(bia_sample_t) ?
             sizeof(bia_meta_t) : sizeof(bia_sample_t)];
    bool gotData = false;

    while (true) {
        ssize_t n = recvfrom(m_dataFd, buf, sizeof(buf), 0, NULL, NULL);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            qWarning("recvfrom error: %s", strerror(errno));
            break;
        }

        /*
         * Distinguish packet type by checking the first field:
         *   If magic == BIA_META_MAGIC → meta info packet
         *   Otherwise               → data sample
         *
         * Both types share the same first-4-bytes layout at offset 0,
         * but a valid freq_hz will never be 0xB1A00000 (~3GB),
         * and a valid magnitude is a normal float.
         */
        uint32_t first_word;
        memcpy(&first_word, buf, sizeof(first_word));

        if ((size_t)n == sizeof(bia_meta_t) && first_word == BIA_META_MAGIC) {
            /* Meta-info packet from demo after START */
            const bia_meta_t *meta = (const bia_meta_t *)buf;
            m_sweepPoints = meta->sweep_points;
            qDebug("META: sweep_points=%d, sweep_type=%d",
                   meta->sweep_points, meta->sweep_type);
            continue;  /* not data, don't refresh chart */
        }

        if ((size_t)n != sizeof(bia_sample_t)) {
            qWarning("Unexpected datagram size: %zd (expected %zu)",
                     n, sizeof(bia_sample_t));
            continue;
        }

        /* Regular data sample */
        const bia_sample_t *sample = (const bia_sample_t *)buf;
        if (!m_samples.contains(sample->freq_hz))
            m_receivedPoints++;

        m_samples[sample->freq_hz] = *sample;
        gotData = true;

        qDebug("Freq=%6u Hz  |Z|=%10.2f Ω  Phase=%8.2f °",
               sample->freq_hz, sample->magnitude, sample->phase);
    }

    if (gotData) {
        /* Update status with progress */
        if (m_acquiring) {
            if (m_sweepPoints > 0)
                m_statusLabel->setText(
                    tr("采集中... %1/%2 个频点")
                    .arg(m_receivedPoints).arg(m_sweepPoints));
            else
                m_statusLabel->setText(
                    tr("采集中... 已收到 %1 个频点")
                    .arg(m_receivedPoints));
        }

        refreshChart();

        /* Check round completion */
        if (m_acquiring && m_sweepPoints > 0
            && m_receivedPoints >= m_sweepPoints) {
            qDebug("Sweep complete: %d points", m_receivedPoints);
            stopAcquisition();
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Chart refresh                                                     */
/* ------------------------------------------------------------------ */

void MainWindow::refreshChart()
{
    if (m_samples.isEmpty())
        return;

    /* Rebuild magnitude series */
    m_chart->removeSeries(m_magSeries);
    delete m_magSeries;
    m_magSeries = new QLineSeries();
    m_magSeries->setName("|Z| (Ω)");
    QPen magPen(Qt::blue);
    magPen.setWidth(2);
    m_magSeries->setPen(magPen);

    /* Rebuild phase series */
    m_chart->removeSeries(m_phaseSeries);
    delete m_phaseSeries;
    m_phaseSeries = new QLineSeries();
    m_phaseSeries->setName("Phase (°)");
    QPen phasePen(Qt::red);
    phasePen.setWidth(2);
    m_phaseSeries->setPen(phasePen);

    double magMin = 1e9, magMax = 0;
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
    }

    m_chart->addSeries(m_magSeries);
    m_chart->addSeries(m_phaseSeries);

    m_magSeries->attachAxis(m_xAxis);
    m_magSeries->attachAxis(m_yMagAxis);
    m_phaseSeries->attachAxis(m_xAxis);
    m_phaseSeries->attachAxis(m_yPhaseAxis);

    if (freqMin < 1.0) freqMin = 1.0;
    m_xAxis->setRange(freqMin * 0.5, freqMax * 2.0);
    m_yMagAxis->setRange(0, magMax * 1.05);
    m_yPhaseAxis->setRange(-90, 90);

    m_chartView->update();
}
