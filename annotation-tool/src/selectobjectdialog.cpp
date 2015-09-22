#include "selectobjectdialog.h"
#include "ui_selectobjectdialog.h"
#include "newobjectdialog.h"


#include <iostream>
#include "QDir"
#include "QTextStream"

#include <sstream>
#include <boost/property_tree/xml_parser.hpp>

using namespace boost::property_tree;

selectObject::selectObject(QWidget *parent, int type) :
    QDialog(parent),
    ui(new Ui::selectObject)
{
    ui->setupUi(this);
    readData(type);
}

selectObject::~selectObject()
{
    delete ui;
}


void selectObject::setObjectList(QStringList listObjects){
    _listOfObjects.clear();
    _listOfObjects = listObjects;
    ui->comboBox->clear();
    ui->comboBox->addItems(_listOfObjects);
}

void selectObject::readData(int type){
    if(type == 2){
        setWindowTitle("Select object to remove");
        ui->objectSelected->setText(QString("Remove"));
    }
    else{
        // read data
        _list_doc = qApp->applicationDirPath();
        _list_doc.remove("bin");
        _list_doc.append("/list_objects.xml");;

        ptree root;
        read_xml(_list_doc.toStdString(), root);

        _listOfObjects.clear();

        ptree& allObjects = root.get_child("List_of_objects");
        for(ptree::iterator it = allObjects.begin(); it != allObjects.end(); it++){
            _listOfObjects += QString::fromStdString(allObjects.get<std::string>("Object"));
            allObjects.pop_front();
        }

        ui->comboBox->clear();
        ui->comboBox->addItems(_listOfObjects);

        if(!type){
            setWindowTitle("Select the object");
            ui->comboBox->insertSeparator(ui->comboBox->count());
            ui->comboBox->addItem(QString("Insert new ..."));
            ui->comboBox->addItem(QString("Remove from the list"));
        }
        else{
            setWindowTitle("Select object to remove");
            ui->objectSelected->setText(QString("Remove"));
        }
    }
}


void selectObject::on_objectSelected_clicked(){
    _objectName = ui->comboBox->currentText();
    hide();
}

QString selectObject::getObjectName(){
    return _objectName;
}


void selectObject::on_comboBox_currentIndexChanged(const QString &arg1){
    if(!arg1.compare(QString("Insert new ..."))){
        chooseObjectDialog choose;
        choose.exec();
        _objectName = choose.getObjectName();

        _listOfObjects += _objectName;

        readWriteXml();

        hide();
    }
    if(!arg1.compare(QString("Remove from the list"))){
        selectObject removeObject(0, 1);
        removeObject.exec();
        QString objectToRemove = removeObject.getObjectName();
        _listOfObjects.removeOne(objectToRemove);

        readWriteXml();
    }
}

void selectObject::readWriteXml(){
    ptree root;
    ptree* node = &root.add("List_of_objects", "");

    // Read
    for(int i = 0; i < _listOfObjects.size(); i++){
        std::string object;
        std::stringstream str;
        str << _listOfObjects.at(i).toStdString() << std::endl;
        str >> object;
        node->add("Object", object);
    }

    xml_writer_settings<char> settings(' ', 3);
    write_xml(_list_doc.toStdString(), root, std::locale(), settings);

    readData(0);
}
