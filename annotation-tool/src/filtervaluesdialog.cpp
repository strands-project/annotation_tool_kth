#include "filtervaluesdialog.h"
#include "ui_filtervaluesdialog.h"

filtervaluesdialog::filtervaluesdialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::filtervaluesdialog)
{
    ui->setupUi(this);
    ui->leafSize->setValue(1);
}

filtervaluesdialog::~filtervaluesdialog()
{
    delete ui;
}

void filtervaluesdialog::on_pushButton_clicked()
{
    _leafValue = ui->leafSize->value();
    hide();
}

float filtervaluesdialog::getLeafSize(){
    return _leafValue/100;
}
