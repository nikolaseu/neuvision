#include <ZCore/zsettingsitem.h>

#include <QtTest/QAbstractItemModelTester>
#include <QtTest/QTest>

using namespace Z3D;

class ZSettingsItemTests : public QObject
{
    Q_OBJECT

private slots:
    void tst_empty()
    {
        const std::vector<ZSettingsItemPtr> settings;

        auto *model = new ZSettingsItemModel(settings);
        auto *tester = new QAbstractItemModelTester(model);

        QCOMPARE(model->rowCount(), 0);
    }

    void tst_some()
    {
        const std::vector<ZSettingsItemPtr> settings {
            ZSettingsItemPtr(new ZSettingsItemInt("path", "label", "description", [](){ return 3; }, [](auto){ return true; }))
        };

        auto *model = new ZSettingsItemModel(settings);
        auto *tester = new QAbstractItemModelTester(model);

        QCOMPARE(model->rowCount(), 1);
    }
};

QTEST_MAIN(ZSettingsItemTests)
#include "tst_zsettingsitem.moc"
