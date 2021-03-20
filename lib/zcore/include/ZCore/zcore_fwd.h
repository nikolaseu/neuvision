#pragma once

#include <QtCore/QMetaType>
#include <memory>

namespace Z3D
{

class ZCorePlugin;
class ZSettingsItem;
class ZSettingsItemModel;

typedef std::shared_ptr<ZSettingsItem> ZSettingsItemPtr;

} // namespace Z3D

Q_DECLARE_OPAQUE_POINTER(Z3D::ZSettingsItemModel*);
