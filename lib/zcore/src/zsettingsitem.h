#pragma once

#include "qqmlobjectlistmodel.h"
#include "zcore_fwd.h"
#include "zcore_global.h"

#include <QObject>
#include <QVariant>

namespace Z3D
{

class Z3D_CORE_SHARED_EXPORT ZSettingsItem : public QObject
{
    Q_OBJECT

    Q_PROPERTY(SettingsItemType type READ type CONSTANT)
    Q_PROPERTY(QString path READ path CONSTANT)
    Q_PROPERTY(QString label READ label CONSTANT)
    Q_PROPERTY(QString description READ description CONSTANT)
    Q_PROPERTY(bool readable READ readable WRITE setReadable NOTIFY readableChanged)
    Q_PROPERTY(bool writable READ writable WRITE setWritable NOTIFY writableChanged)
    Q_PROPERTY(QVariant value READ value WRITE setValue NOTIFY valueChanged)

public:
    enum SettingsItemType {
        SettingsItemTypeUnknown = 0,
        SettingsItemTypeBool,
        SettingsItemTypeString,
        SettingsItemTypeInt,
        SettingsItemTypeLong,
        SettingsItemTypeFloat,
        SettingsItemTypeEnum,
        SettingsItemTypeCommand,
        SettingsItemTypeCategory
    };
    Q_ENUM(SettingsItemType)

    explicit ZSettingsItem(const SettingsItemType type,
                           const QString &path,
                           const QString &label,
                           const QString &description,
                           QObject *parent = nullptr);
    virtual ~ZSettingsItem();

    SettingsItemType type() const;
    const QString &path() const;
    const QString &label() const;
    const QString &description() const;
    bool readable() const;
    bool writable() const;

    virtual QVariant value() const = 0;

signals:
    void readableChanged(bool readable);
    void writableChanged(bool writable);

    void valueChanged(QVariant value);

public slots:
    void setReadable(bool readable);
    void setNonReadable(bool nonReadable);
    void setWritable(bool writable);
    void setNonWritable(bool nonWritable);

    virtual bool setValue(QVariant value) = 0;

private:
    const SettingsItemType m_type;
    const QString m_path;
    const QString m_label;
    const QString m_description;
    bool m_readable = true;
    bool m_writable = true;
};

class Z3D_CORE_SHARED_EXPORT ZSettingsItemModel : public QQmlObjectListModel<ZSettingsItem>
{
    Q_OBJECT

public:
    explicit ZSettingsItemModel(const std::vector<ZSettingsItemPtr> settings, QObject *parent = nullptr)
        : QQmlObjectListModel<ZSettingsItem>(parent)
    {
        for (ZSettingsItemPtr setting : settings) {
            append(setting.get());
        }
    }
};

template<typename ValueType>
class ZSettingsItemValueType
{
public:
    explicit ZSettingsItemValueType(std::function<ValueType()> getValue,
                                    std::function<bool(ValueType)> setValue)
        : m_getValue(getValue)
        , m_setValue(setValue)
    {

    }

    virtual ~ZSettingsItemValueType()
    {

    }

public:
    ValueType value() const
    {
        return m_getValue();
    }

    // signals:
    virtual void valueChanged(QVariant value) = 0;

    // public slots:
    bool setValue(ValueType value)
    {
        if (m_getValue() == value) {
            return true;
        }

        if (!m_setValue(value)) {
            return false;
        }

        emit valueChanged(value);
        return true;
    }

private:
    const std::function<ValueType()> m_getValue;
    const std::function<bool(ValueType)> m_setValue;
};

class Z3D_CORE_SHARED_EXPORT ZSettingsItemBool : public ZSettingsItem, public ZSettingsItemValueType<bool>
{
    Q_OBJECT

public:
    explicit ZSettingsItemBool(const QString &path, const QString &label, const QString &description,
                               std::function<bool()> getValue,
                               std::function<bool(bool)> setValue,
                               QObject *parent = nullptr)
        : ZSettingsItem(ZSettingsItem::SettingsItemTypeBool, path, label, description, parent)
        , ZSettingsItemValueType(getValue, setValue)
    {

    }

    // ZSettingsItemValueType interface
signals:
    void valueChanged(QVariant value) override;

    // ZSettingsItem interface
public:
    virtual QVariant value() const override
    {
        return ZSettingsItemValueType::value();
    }

public slots:
    virtual bool setValue(QVariant value) override
    {
        if (!value.canConvert<bool>()) {
            return false;
        }
        return ZSettingsItemValueType::setValue(value.toBool());
    }
};

class Z3D_CORE_SHARED_EXPORT ZSettingsItemString : public ZSettingsItem, public ZSettingsItemValueType<QString>
{
    Q_OBJECT

public:
    explicit ZSettingsItemString(const QString &path, const QString &label, const QString &description,
                                 std::function<QString()> getValue,
                                 std::function<bool(QString)> setValue,
                                 QObject *parent = nullptr)
        : ZSettingsItem(ZSettingsItem::SettingsItemTypeString, path, label, description, parent)
        , ZSettingsItemValueType(getValue, setValue)
    {

    }
};

class Z3D_CORE_SHARED_EXPORT ZSettingsItemInt : public ZSettingsItem, public ZSettingsItemValueType<int>
{
    Q_OBJECT

    Q_PROPERTY(int minimum READ minimum CONSTANT)
    Q_PROPERTY(int maximum READ maximum CONSTANT)

public:
    explicit ZSettingsItemInt(const QString &path, const QString &label, const QString &description,
                              std::function<int()> getValue,
                              std::function<bool(int)> setValue,
                              const int minimumValue = std::numeric_limits<int>::min(),
                              const int maximumValue = std::numeric_limits<int>::max(),
                              QObject *parent = nullptr)
        : ZSettingsItem(ZSettingsItem::SettingsItemTypeInt, path, label, description, parent)
        , ZSettingsItemValueType(getValue, setValue)
        , m_minimumValue(minimumValue)
        , m_maximumValue(maximumValue)
    {

    }

    int minimum() const
    {
        return m_minimumValue;
    }

    int maximum() const
    {
        return m_maximumValue;
    }

    // ZSettingsItemValueType interface
signals:
    void valueChanged(QVariant value) override;

    // ZSettingsItem interface
public:
    virtual QVariant value() const override
    {
        return ZSettingsItemValueType::value();
    }

public slots:
    virtual bool setValue(QVariant value) override
    {
        if (!value.canConvert<int>()) {
            return false;
        }
        return ZSettingsItemValueType::setValue(value.toInt());
    }

private:
    const int m_minimumValue = std::numeric_limits<int>::min();
    const int m_maximumValue = std::numeric_limits<int>::max();
};

class Z3D_CORE_SHARED_EXPORT ZSettingsItemLong : public ZSettingsItem, public ZSettingsItemValueType<long long>
{
    Q_OBJECT

    Q_PROPERTY(long long minimum READ minimum CONSTANT)
    Q_PROPERTY(long long maximum READ maximum CONSTANT)

public:
    explicit ZSettingsItemLong(const QString &path, const QString &label, const QString &description,
                               std::function<long long()> getValue,
                               std::function<bool(long long)> setValue,
                               long long minimumValue = std::numeric_limits<long long>::min(),
                               long long maximumValue = std::numeric_limits<long long>::max(),
                               QObject *parent = nullptr)
        : ZSettingsItem(ZSettingsItem::SettingsItemTypeLong, path, label, description, parent)
        , ZSettingsItemValueType(getValue, setValue)
        , m_minimumValue(minimumValue)
        , m_maximumValue(maximumValue)
    {

    }

    long long minimum() const
    {
        return m_minimumValue;
    }

    long long maximum() const
    {
        return m_maximumValue;
    }

    // ZSettingsItemValueType interface
signals:
    void valueChanged(QVariant value) override;

    // ZSettingsItem interface
public:
    virtual QVariant value() const override
    {
        return ZSettingsItemValueType::value();
    }

public slots:
    virtual bool setValue(QVariant value) override
    {
        if (!value.canConvert<long long>()) {
            return false;
        }
        return ZSettingsItemValueType::setValue(value.toLongLong());
    }

private:
    const long long m_minimumValue = std::numeric_limits<long long>::min();
    const long long m_maximumValue = std::numeric_limits<long long>::max();
};

class Z3D_CORE_SHARED_EXPORT ZSettingsItemFloat : public ZSettingsItem, public ZSettingsItemValueType<double>
{
    Q_OBJECT

    Q_PROPERTY(double minimum READ minimum CONSTANT)
    Q_PROPERTY(double maximum READ maximum CONSTANT)

public:
    explicit ZSettingsItemFloat(const QString &path, const QString &label, const QString &description,
                                std::function<double()> getValue,
                                std::function<bool(double)> setValue,
                                const double minimumValue = std::numeric_limits<double>::min(),
                                const double maximumValue = std::numeric_limits<double>::max(),
                                QObject *parent = nullptr)
        : ZSettingsItem(ZSettingsItem::SettingsItemTypeFloat, path, label, description, parent)
        , ZSettingsItemValueType(getValue, setValue)
        , m_minimumValue(minimumValue)
        , m_maximumValue(maximumValue)
    {

    }

    double minimum() const
    {
        return m_minimumValue;
    }

    double maximum() const
    {
        return m_maximumValue;
    }

    // ZSettingsItemValueType interface
signals:
    void valueChanged(QVariant value) override;

    // ZSettingsItem interface
public:
    virtual QVariant value() const override
    {
        return ZSettingsItemValueType::value();
    }

public slots:
    virtual bool setValue(QVariant value) override
    {
        if (!value.canConvert<double>()) {
            return false;
        }
        return ZSettingsItemValueType::setValue(value.toDouble());
    }

private:
    const double m_minimumValue = std::numeric_limits<double>::min();
    const double m_maximumValue = std::numeric_limits<double>::max();
};

class Z3D_CORE_SHARED_EXPORT ZSettingsItemEnum : public ZSettingsItem
{
    Q_OBJECT

    Q_PROPERTY(QStringList options READ options NOTIFY optionsChanged)

public:
    explicit ZSettingsItemEnum(const QString &path, const QString &label, const QString &description,
                               std::function<std::vector<QString>()> getEnumNames,
                               std::function<int()> getSelectedIndex,
                               std::function<bool(int)> setSelectedIndex,
                               QObject *parent = nullptr)
        : ZSettingsItem(ZSettingsItem::SettingsItemTypeEnum, path, label, description, parent)
        , m_getEnumNames(getEnumNames)
        , m_getSelectedIndex(getSelectedIndex)
        , m_setSelectedIndex(setSelectedIndex)
    {

    }

    QStringList options() const
    {
        QStringList sl;
        for (const auto &option : m_getEnumNames()) {
            sl << option;
        }
        return sl;
    }

    // ZSettingsItem interface
    virtual QVariant value() const override
    {
        return m_getSelectedIndex();
    }

signals:
    void optionsChanged();

public slots:
    virtual bool setValue(QVariant value) override
    {
        const int index = value.toInt();
        if (index == m_getSelectedIndex()) {
            return true;
        }

        return m_setSelectedIndex(index);
    }

private:
    const std::function<std::vector<QString>()> m_getEnumNames;
    const std::function<int()> m_getSelectedIndex;
    const std::function<bool(int)> m_setSelectedIndex;
};

class Z3D_CORE_SHARED_EXPORT ZSettingsItemCommand : public ZSettingsItem
{
    Q_OBJECT

public:
    explicit ZSettingsItemCommand(const QString &path, const QString &label, const QString &description,
                                  std::function<bool()> runCommand,
                                  QObject *parent = nullptr)
        : ZSettingsItem(ZSettingsItem::SettingsItemTypeCommand, path, label, description, parent)
        , m_runCommand(runCommand)
    {

    }

    virtual QVariant value() const override
    {
        return QVariant();
    }

public slots:
    virtual bool setValue(QVariant value) override
    {
        return false;
    }

    bool execute()
    {
        return m_runCommand();
    }

private:
    const std::function<bool()> m_runCommand;
};

class Z3D_CORE_SHARED_EXPORT ZSettingsItemCategory : public ZSettingsItem
{
    Q_OBJECT

public:
    explicit ZSettingsItemCategory(const QString &path, const QString &label, const QString &description,
                                   std::vector<ZSettingsItemPtr> settings,
                                   QObject *parent = nullptr)
        : ZSettingsItem(ZSettingsItem::SettingsItemTypeCategory, path, label, description, parent)
        , m_settings(settings)
    {

    }

public:
    virtual QVariant value() const override
    {
        return QVariant();
    }

    const std::vector<ZSettingsItemPtr> &settings() const
    {
        return m_settings;
    }

private:
    const std::vector<ZSettingsItemPtr> m_settings;
};

} // namespace Z3D
