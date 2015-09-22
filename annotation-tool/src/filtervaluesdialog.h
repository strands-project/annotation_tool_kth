#ifndef FILTERVALUESDIALOG_H
#define FILTERVALUESDIALOG_H

#include <QDialog>

namespace Ui {
class filtervaluesdialog;
}

class filtervaluesdialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit filtervaluesdialog(QWidget *parent = 0);
    ~filtervaluesdialog();

    float getLeafSize();
    

private Q_SLOTS:

 void on_pushButton_clicked();

private:
    Ui::filtervaluesdialog *ui;

    float _leafValue;
};

#endif // FILTERVALUESDIALOG_H
