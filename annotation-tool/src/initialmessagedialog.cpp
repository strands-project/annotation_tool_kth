#include "initialmessagedialog.h"
#include "ui_initialmessagedialog.h"

initialmessagedialog::initialmessagedialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::initialmessagedialog)
{
    ui->setupUi(this);
    setWindowTitle("Welcome to 3D annotation tool");
}

initialmessagedialog::~initialmessagedialog()
{
    delete ui;
}

void initialmessagedialog::on_pushButton_clicked()
{
    _showMessageAgain = ui->checkBox->isChecked();
    hide();
}

bool initialmessagedialog::getBoolValue(){
    return _showMessageAgain;
}
