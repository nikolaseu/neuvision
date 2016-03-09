#pragma once

#include <QWidget>

namespace Ui {
class ZIncompleteCircleGridPatternFinderConfigWidget;
}

namespace Z3D
{

/// forward declaration
class ZIncompleteCircleGridPatternFinder;

class ZIncompleteCircleGridPatternFinderConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ZIncompleteCircleGridPatternFinderConfigWidget(ZIncompleteCircleGridPatternFinder *patternFinder, QWidget *parent = 0);
    ~ZIncompleteCircleGridPatternFinderConfigWidget();

private:
    Ui::ZIncompleteCircleGridPatternFinderConfigWidget *ui;

    ZIncompleteCircleGridPatternFinder *m_patternFinder;
};

} // namespace Z3D
