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
 *  RvizFileProperty.hpp
 *
 *
 *  authors:
 *
 *    Malte kleine Piening <malte@klpiening.de>
 */

#ifndef RVIZ_FILE_PROPERTY_HPP
#define RVIZ_FILE_PROPERTY_HPP

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
