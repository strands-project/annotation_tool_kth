#ifndef ADDOBJECT_H
#define ADDOBJECT_H

#include <QDialog>

namespace Ui {
class AddObject;
}

class AddObject : public QDialog
{
    Q_OBJECT

public:

    explicit AddObject(QWidget *parent = 0);
    ~AddObject();

    QString getObjectName();

    void setObjectList(QStringList listObjects);


private Q_SLOTS:
   void on_objectSelected_clicked();

    void on_lineEdit_textEdited(const QString &arg1);

private:
    Ui::AddObject *ui;

    void readData();

    void readWriteXml();

    QString _list_doc;

    QString _objectName;

    QStringList _listOfObjects;

    QString loadFileText();

    void initList();
};

#endif // ADDOBJECT_H
