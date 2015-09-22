#include "newobjectdialog.h"
#include "ui_newobjectdialog.h"
#include "QMessageBox"

chooseObjectDialog::chooseObjectDialog(QWidget *parent, int type) :
    QDialog(parent),
    ui(new Ui::chooseObjectDialog)
{
    ui->setupUi(this);
    if(type == 0){
        setWindowTitle("New object");
        ui->label->setText("Insert new object: ");
    }
    else{
        setWindowTitle("Insert scenario");
        //ui->label->setText("Insert the dominant plane (desk, floor...)");
    }
}

chooseObjectDialog::~chooseObjectDialog(){
    delete ui;
}

void chooseObjectDialog::on_insert_clicked(){
    QString text = ui->lineEdit->text();

    if (text.isEmpty()) {
        QMessageBox::information(this, tr("Empty Field"), tr("Please enter an object."));
        return;
    } else {
        objectName = text;
        ui->lineEdit->clear();
        hide();
    }
}

QString chooseObjectDialog::getObjectName(){
    return objectName;
}
