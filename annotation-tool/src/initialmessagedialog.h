#ifndef INITIALMESSAGEDIALOG_H
#define INITIALMESSAGEDIALOG_H

#include <QDialog>

namespace Ui {
class initialmessagedialog;
}

class initialmessagedialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit initialmessagedialog(QWidget *parent = 0);
    ~initialmessagedialog();

    bool getBoolValue();
    

private Q_SLOTS:

void on_pushButton_clicked();

private:
    Ui::initialmessagedialog *ui;

    bool _showMessageAgain;
};

#endif // INITIALMESSAGEDIALOG_H
