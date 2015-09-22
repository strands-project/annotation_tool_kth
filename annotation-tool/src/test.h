#ifndef CHOOSEOBJECTDIALOG_H
#define CHOOSEOBJECTDIALOG_H

#include <QDialog>

namespace Ui {
class chooseObjectDialog;
}

class chooseObjectDialog : public QDialog
{
    Q_OBJECT
    
public:
    // If type = 0, is to insert a new object in the list of objects
    // If type = 1, is to define if the plane is a desk, floor or whatever
    explicit chooseObjectDialog(QWidget *parent = 0, int type=0);
    ~chooseObjectDialog();
    QString getObjectName();
    
private slots:
    void on_insert_clicked();

private:
    Ui::chooseObjectDialog *ui;
    QString objectName;
};

#endif // CHOOSEOBJECTDIALOG_H
