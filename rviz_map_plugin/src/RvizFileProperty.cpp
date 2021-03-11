/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2021, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  RvizFileProperty.cpp
 *
 *
 *  authors:
 *
 *    Malte kleine Piening <malte@klpiening.de>
 */

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
