#ifndef Z3D_CORE___ZAPPLICATIONSTYLE_H
#define Z3D_CORE___ZAPPLICATIONSTYLE_H

namespace Z3D
{

namespace ZApplicationStyle
{

    enum ZStyle {
        LightStyle,
        DarkStyle
    };

    void applyStyle(ZStyle style);

} // namespace ZApplicationStyle

} // namespace Z3D

#endif // Z3D_CORE___ZAPPLICATIONSTYLE_H
