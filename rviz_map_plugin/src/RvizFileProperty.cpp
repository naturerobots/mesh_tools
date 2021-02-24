#include "RvizFileProperty.hpp"

#include <QFileDialog>

namespace rviz
{
FileProperty::FileProperty(const QString& name, const QString& default_value, const QString& description,
                           Property* parent, const char* changed_slot, QObject* receiver)
  : Property(name, default_value, description, parent, changed_slot, receiver)
{
}

QWidget* FileProperty::createEditor(QWidget* parent, const QStyleOptionViewItem&)
{
  QFileDialog* editor = new QFileDialog(nullptr);

  QStringList filenameFilters;
  filenameFilters << tr("*.h5");
  filenameFilters << tr("*");
  editor->setNameFilters(filenameFilters);

  editor->setViewMode(QFileDialog::Detail);

  if (editor->exec())
  {
    QStringList fileNames = editor->selectedFiles();
    if (fileNames.size() == 0)
    {
      setStdFilename("");
    }
    else
    {
      setFilename(fileNames.at(0));
    }
  }

  return nullptr;
}

}  // namespace rviz
