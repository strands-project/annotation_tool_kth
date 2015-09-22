#ifndef SELECTOBJECT_H
#define SELECTOBJECT_H

#include <QDialog>

namespace Ui {
class selectObject;
}

class selectObject : public QDialog
{
    Q_OBJECT
    
public:
    // If type = 0, is to select the objects
    // If type = 1, is to remove one of the objects from the list of objects previously inserted
    // If type = 2, is to remove one of the annotated objects
    explicit selectObject(QWidget *parent = 0, int type = 0);
    ~selectObject();

    QString getObjectName();

    void setObjectList(QStringList listObjects);
    
private Q_SLOTS:

    void on_objectSelected_clicked();

    void on_comboBox_currentIndexChanged(const QString &arg1);

private:
    Ui::selectObject *ui;

    void readData(int type);

    void readWriteXml();

    QString _list_doc;

    QString _objectName;

    QStringList _listOfObjects;
};

#endif // SELECTOBJECT_H
