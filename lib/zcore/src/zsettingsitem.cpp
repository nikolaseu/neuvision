#include "zsettingsitem.h"

namespace Z3D
{

ZSettingsItem::ZSettingsItem(const SettingsItemType type,
                             const QString &path,
                             const QString &label,
                             const QString &description,
                             QObject *parent)
    : QObject(parent)
    , m_type(type)
    , m_path(path)
    , m_label(label)
    , m_description(description)
{

}

ZSettingsItem::~ZSettingsItem()
{

}

ZSettingsItem::SettingsItemType ZSettingsItem::type() const
{
    return m_type;
}

const QString &ZSettingsItem::path() const
{
    return m_path;
}

const QString &ZSettingsItem::label() const
{
    return m_label;
}

const QString &ZSettingsItem::description() const
{
    return m_description;
}

bool ZSettingsItem::readable() const
{
    return m_readable;
}

bool ZSettingsItem::writable() const
{
    return m_writable;
}

void ZSettingsItem::setReadable(bool readable)
{
    if (m_readable == readable) {
        return;
    }

    m_readable = readable;
    emit readableChanged(m_readable);
}

void ZSettingsItem::setNonReadable(bool nonReadable)
{
    setReadable(!nonReadable);
}

void ZSettingsItem::setWritable(bool writable)
{
    if (m_writable == writable) {
        return;
    }

    m_writable = writable;
    emit writableChanged(m_writable);
}

void ZSettingsItem::setNonWritable(bool nonWritable)
{
    setWritable(!nonWritable);
}

} // namespace Z3D
