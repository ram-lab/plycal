#ifndef TFWINDIOW_H
#define TFWINDIOW_H

#include <QMainWindow>
#include <QCloseEvent>

#include <Eigen/Dense>

namespace Ui {
class TFwindow;
}

class TFwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit TFwindow(const Eigen::Matrix4d& tf, QWidget *parent = 0);
    ~TFwindow();

signals:
    void newTransformation(Eigen::Matrix4d tf);
//    void newTransformation();
    void tfwindowClose();

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    Ui::TFwindow *ui;

private slots:
	void process();

};

#endif // TFWINDIOW_H
