#include "tfwindow.h"
#include "ui_tfwindow.h"

#include <Eigen/Geometry>

static const double PI(3.141592653589793238462);

TFwindow::TFwindow(const Eigen::Matrix4d& tf, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TFwindow)
{
    ui->setupUi(this);

	auto setWidget = [](double data, QSlider* sld, QLabel* lb)
	{
		int32_t val = static_cast<int32_t>(std::round(data*500+500));
		if (val <0 )
		{
			val = 0;
		}
		if(val > 1000)
		{
			val = 1000;
		}
		sld->setValue(val);
        lb->setText(QString("%1").arg((val-500.0)/500.0, 6, 'f',3,' '));
	};

	setWidget(tf(0,3), ui->tx_slide, ui->tx_text);
	setWidget(tf(1,3), ui->ty_slide, ui->ty_text);
	setWidget(tf(2,3), ui->tz_slide, ui->tz_text);

	// xyz - euler
	Eigen::Matrix3d rotation = tf.topLeftCorner(3,3);
	// [0:pi]x[-pi:pi]x[-pi:pi]
	Eigen::Vector3d angle = rotation.eulerAngles(0, 1, 2)/PI*180;
	Eigen::Vector3i r = angle.cast<int>();

	uint16_t v = r(0) <0 ? 0 : (r(0)>180? 180: r(0));
	ui->rx_slide->setValue( v );
	ui->rx_text->setText( QString::number(v) );

	v = r(1) < -180 ? 0 : (r(1) > 180 ? 360 : r(1) +180);
	ui->ry_slide->setValue( v );
	ui->ry_text->setText( QString::number(r(1)) );

	v = r(2) < -180 ? 0 : (r(2) > 180 ? 360 : r(2) +180);
    ui->rz_slide->setValue( v );
    ui->rz_text->setText( QString::number(r(2)) );


    connect(ui->rx_slide, &QSlider::valueChanged, this, &TFwindow::process);
    connect(ui->ry_slide, &QSlider::valueChanged, this, &TFwindow::process);
    connect(ui->rz_slide, &QSlider::valueChanged, this, &TFwindow::process);
    connect(ui->tx_slide, &QSlider::valueChanged, this, &TFwindow::process);
    connect(ui->ty_slide, &QSlider::valueChanged, this, &TFwindow::process);
    connect(ui->tz_slide, &QSlider::valueChanged, this, &TFwindow::process);
}

TFwindow::~TFwindow()
{
    delete ui;
}

void TFwindow::process()
{
    Eigen::Matrix4d tf;
    tf.setIdentity();
    tf(0,3) = (ui->tx_slide->value() - 500)/500.0;
    ui->tx_text->setText(QString::number(tf(0,3)));
    tf(1,3) = (ui->ty_slide->value() - 500)/500.0;
    ui->ty_text->setText(QString::number(tf(1,3)));
    tf(2,3) = (ui->tz_slide->value() - 500)/500.0;
    ui->tz_text->setText(QString::number(tf(2,3)));

    ui->rx_text->setText(QString::number(ui->rx_slide->value()));
    double rx = ui->rx_slide->value()/180.0*PI;
    ui->ry_text->setText(QString::number(ui->ry_slide->value()-180));
    double ry = (ui->ry_slide->value()-180.0)/180.0*PI;
    ui->rz_text->setText(QString::number(ui->rz_slide->value()-180));
    double rz = (ui->rz_slide->value()-180.0)/180.0*PI;

    Eigen::Quaterniond ag =  Eigen::AngleAxisd(rx,  Eigen::Vector3d::UnitX())
                             *  Eigen::AngleAxisd(ry,  Eigen::Vector3d::UnitY())
                             *  Eigen::AngleAxisd(rz,  Eigen::Vector3d::UnitZ());
    tf.topLeftCorner(3,3) = ag.matrix();
    tf.row(3) << 0,0,0,1;

    emit newTransformation(tf);
}


void TFwindow::closeEvent(QCloseEvent *event)
{
    emit tfwindowClose();
    event->accept();
}
