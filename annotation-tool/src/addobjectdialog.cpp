#include "addobjectdialog.h"
#include "ui_addobjectdialog.h"
#include "newobjectdialog.h"


#include <iostream>
#include "QDir"
#include "QTextStream"

#include <QFile>
#include <QTextStream>
#include <QCompleter>
#include <QEvent>
#include <QApplication>

#include <sstream>
#include <boost/property_tree/xml_parser.hpp>

using namespace boost::property_tree;

AddObject::AddObject(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddObject)
{
    ui->setupUi(this);
    readData();
    initList();
}

AddObject::~AddObject()
{
    delete ui;
}


void AddObject::setObjectList(QStringList listObjects){
    _listOfObjects.clear();
    _listOfObjects = listObjects;
}

void AddObject::readData(){
            // read data
        _list_doc = qApp->applicationDirPath();
        _list_doc.remove("bin");
        _list_doc.append("/list_objects.xml");;

        ptree root;
        read_xml(_list_doc.toStdString(), root);

        _listOfObjects.clear();

        ptree& allObjects = root.get_child("List_of_objects");
        for(ptree::iterator it = allObjects.begin(); it != allObjects.end(); it++){
            _listOfObjects += QString::fromStdString(allObjects.get<std::string>("Object")).toLower();
            allObjects.pop_front();
        }
    }


void AddObject::on_objectSelected_clicked(){

    if (ui->lineEdit->text()!= "")
    {
         _objectName = ui->lineEdit->text().toLower();
        if(!_listOfObjects.contains(_objectName, Qt::CaseInsensitive))
        {
           _listOfObjects += _objectName;
           readWriteXml();
        }
    }

    hide();
}

QString AddObject::getObjectName(){
    return _objectName;
}

void AddObject::initList()
{
    QCompleter *completer = new QCompleter(_listOfObjects, this);//
    completer->setCaseSensitivity(Qt::CaseInsensitive);
    ui->lineEdit->setCompleter(completer);
}

void AddObject::readWriteXml(){
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

    readData();
}

void AddObject::on_lineEdit_textEdited(const QString &arg1)
{
}
