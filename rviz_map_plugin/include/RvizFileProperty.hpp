#ifndef RVIZ_FILE_PROPERTY_H
#define RVIZ_FILE_PROPERTY_H

#include <string>
#include <rviz/properties/property.h>

namespace rviz
{
class FileProperty : public Property
{
  Q_OBJECT
public:
  FileProperty(const QString& name = QString(), const QString& default_value = QString(),
               const QString& description = QString(), Property* parent = nullptr, const char* changed_slot = nullptr,
               QObject* receiver = nullptr);

  std::string getFilename()
  {
    return getValue().toString().toStdString();
  }

  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem&);

public Q_SLOTS:
  bool setFilename(const QString& str)
  {
    return setValue(str);
  }
  bool setStdFilename(const std::string& std_str)
  {
    return setValue(QString::fromStdString(std_str));
  }
};

}  // end namespace rviz

#endif
