//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "ZGui/zapplicationstyle.h"

#include <QApplication>
#include <QPalette>
#include <QStyleFactory>

namespace Z3D::ZApplicationStyle
{

void applyStyle(ZStyle style)
{
    /// set custom style
    QApplication::setStyle(QStyleFactory::create("Fusion"));

    QPalette p;

    switch (style) {
    case LightStyle:
        p = qApp->palette();
        p.setColor(QPalette::Window, Qt::white);
        break;
    case DarkStyle:
        QColor windowColor(63, 63, 63);
        QColor baseColor(85, 85, 85);
        QColor buttonColor(93, 93, 93);
        QColor textColor(236, 236, 236);
        p.setColor(QPalette::Window, windowColor);
        p.setColor(QPalette::WindowText, textColor);
        p.setColor(QPalette::Disabled, QPalette::WindowText, textColor.darker(125));
        p.setColor(QPalette::Base, baseColor); // color de los edits por ej
        p.setColor(QPalette::Disabled, QPalette::Base, baseColor.lighter(125));
        p.setColor(QPalette::AlternateBase, baseColor.lighter(125));
        p.setColor(QPalette::ToolTipBase, Qt::white);
        p.setColor(QPalette::ToolTipText, Qt::white);
        p.setColor(QPalette::Text, textColor);
        p.setColor(QPalette::Disabled, QPalette::Text, textColor.darker(125));
        p.setColor(QPalette::Button, buttonColor);
        p.setColor(QPalette::Disabled, QPalette::Button, buttonColor.darker(125));
        p.setColor(QPalette::ButtonText, textColor);
        p.setColor(QPalette::Disabled, QPalette::ButtonText, textColor.darker(150));
        p.setColor(QPalette::BrightText, Qt::red);
        p.setColor(QPalette::Link, QColor(42, 130, 218));

        p.setColor(QPalette::Highlight, QColor(255, 87, 34));// deep orange
        //p.setColor(QPalette::Highlight, QColor(244, 67, 54)); // red
        //p.setColor(QPalette::Highlight, QColor(0, 150, 136)); // teal
        p.setColor(QPalette::HighlightedText, Qt::white);

        qApp->setStyleSheet(
                    "QToolTip {"
                    "    border-radius: 2px;"
                    "    color: lightGray;"
                    "    background-color: black;"
                    "    border: 1px solid gray;"
                    "    padding: 2px;"
                    "}"
                    );
        break;
    }

    qApp->setPalette(p);
}

} // namespace Z3D::ZApplicationStyle
