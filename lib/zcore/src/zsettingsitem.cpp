#include "zsettingsitem.h"

#include "qqmlobjectlistmodel.h"

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

class ZSettingsItemModelPrivate : public QQmlObjectListModel<ZSettingsItem>
{
    Q_OBJECT

public:
    friend class ZSettingsItemModel;

    explicit ZSettingsItemModelPrivate(const std::vector<ZSettingsItemPtr> settings, QObject *parent = nullptr)
        : QQmlObjectListModel<ZSettingsItem>(parent)
    {
        for (ZSettingsItemPtr setting : settings) {
            append(setting.get());
        }
    }
};

ZSettingsItemModel::ZSettingsItemModel(const std::vector<ZSettingsItemPtr> settings, QObject *parent)
    : QAbstractListModel(parent)
    , m_p(new ZSettingsItemModelPrivate(settings, this))
{
    // we have to forward all signals :(

    //! These begin* signals are not available
//    QObject::connect(m_p, &ZSettingsItemModelPrivate::beginMoveRows,
//                     this, &ZSettingsItemModel::beginMoveRows);
//    QObject::connect(m_p, &ZSettingsItemModelPrivate::beginInsertRows,
//                     this, &ZSettingsItemModel::beginInsertRows);
//    QObject::connect(m_p, &ZSettingsItemModelPrivate::beginRemoveRows,
//                     this, &ZSettingsItemModel::beginRemoveRows);
//    QObject::connect(m_p, &ZSettingsItemModelPrivate::beginResetModel,
//                     this, &ZSettingsItemModel::beginResetModel);
//    QObject::connect(m_p, &ZSettingsItemModelPrivate::beginMoveColumns,
//                     this, &ZSettingsItemModel::beginMoveColumns);
//    QObject::connect(m_p, &ZSettingsItemModelPrivate::beginInsertColumns,
//                     this, &ZSettingsItemModel::beginInsertColumns);
//    QObject::connect(m_p, &ZSettingsItemModelPrivate::beginRemoveColumns,
//                     this, &ZSettingsItemModel::beginRemoveColumns);

    QObject::connect(m_p, &ZSettingsItemModelPrivate::rowsAboutToBeMoved,
                     this, &ZSettingsItemModel::rowsAboutToBeMoved);
    QObject::connect(m_p, &ZSettingsItemModelPrivate::rowsAboutToBeRemoved,
                     this, &ZSettingsItemModel::rowsAboutToBeRemoved);
    QObject::connect(m_p, &ZSettingsItemModelPrivate::rowsAboutToBeInserted,
                     this, &ZSettingsItemModel::rowsAboutToBeInserted);
    QObject::connect(m_p, &ZSettingsItemModelPrivate::rowsMoved,
                     this, &ZSettingsItemModel::rowsMoved);
    QObject::connect(m_p, &ZSettingsItemModelPrivate::rowsRemoved,
                     this, &ZSettingsItemModel::rowsRemoved);
    QObject::connect(m_p, &ZSettingsItemModelPrivate::rowsInserted,
                     this, &ZSettingsItemModel::rowsInserted);

    QObject::connect(m_p, &ZSettingsItemModelPrivate::dataChanged,
                     this, &ZSettingsItemModel::dataChanged);
}

int ZSettingsItemModel::rowCount(const QModelIndex &parent) const
{
    return m_p->rowCount(parent);
}

QVariant ZSettingsItemModel::data(const QModelIndex &index, int role) const
{
    return m_p->data(index, role);
}

QHash<int, QByteArray> ZSettingsItemModel::roleNames() const
{
    return m_p->roleNames();
}

} // namespace Z3D

// we have a Q_OBJECT in the cpp, so we have to force this
#include "zsettingsitem.moc"
