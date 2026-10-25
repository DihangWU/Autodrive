"""在同一 SUMO GUI 中显示双坐标，并添加 SUMO/CARLA 两种右键复制。

转换使用当前加载路网的 netOffset，不使用手工偏移，不改变路网或视图方向。
"""
from pathlib import Path

VIEW_SOURCE = Path('utils/gui/windows/GUISUMOAbstractView.cpp')
WINDOW_SOURCE = Path('gui/GUIApplicationWindow.cpp')
OBJECT_SOURCE = Path('utils/gui/globjects/GUIGlObject.cpp')
MENU_SOURCE = Path('utils/gui/globjects/GUIGLObjectPopupMenu.cpp')


def source_patches():
    return {
        VIEW_SOURCE: [
            ('myApp->getCartesianLabel()->setText(("x:" + toString(pos.x()) + ", y:" + toString(pos.y())).c_str());',
             '''const Position offset = GeoConvHelper::getFinal().getOffset();
    myApp->getCartesianLabel()->setText(("SUMO x:" + toString(pos.x()) + ", y:" + toString(pos.y())
        + " | CARLA x:" + toString(pos.x() - offset.x())
        + ", y:" + toString(offset.y() - pos.y())).c_str());
    myApp->getCartesianLabel()->setTipText("Mouse position in meters. CARLA coordinates use the loaded network's netOffset.");'''),
        ],
        WINDOW_SOURCE: [
            ('myCartesianFrame->setWidth(textWidth);', 'myCartesianFrame->setWidth(textWidth * 3);'),
        ],
        OBJECT_SOURCE: [
            ('GUIDesigns::buildFXMenuCommand(ret, TL("Copy cursor position to clipboard"), nullptr, ret, MID_COPY_CURSOR_POSITION);',
             '''GUIDesigns::buildFXMenuCommand(ret, "Copy SUMO coordinates", nullptr, ret, MID_COPY_CURSOR_POSITION);
    GUIDesigns::buildFXMenuCommand(ret, "Copy CARLA coordinates", nullptr, ret, MID_LAST + 1);'''),
        ],
        MENU_SOURCE: [
            ('FXMAPFUNC(SEL_COMMAND,  MID_COPY_CURSOR_POSITION,    GUIGLObjectPopupMenu::onCmdCopyCursorPosition),',
             '''FXMAPFUNC(SEL_COMMAND,  MID_COPY_CURSOR_POSITION,    GUIGLObjectPopupMenu::onCmdCopyCursorPosition),
    FXMAPFUNC(SEL_COMMAND,  MID_LAST + 1, GUIGLObjectPopupMenu::onCmdCopyCursorPosition),'''),
            ('''GUIGLObjectPopupMenu::onCmdCopyCursorPosition(FXObject*, FXSelector, void*) {
    GUIUserIO::copyToClipboard(*myParent->getApp(), toString(myNetworkPosition));''',
             '''GUIGLObjectPopupMenu::onCmdCopyCursorPosition(FXObject*, FXSelector selector, void*) {
    // Use the saved right-click location, not the pointer over the menu.
    Position pos = myNetworkPosition;
    if (FXSELID(selector) == MID_LAST + 1) {
        const Position offset = GeoConvHelper::getFinal().getOffset();
        pos = Position(pos.x() - offset.x(), offset.y() - pos.y());
    }
    GUIUserIO::copyToClipboard(*myParent->getApp(),
        toString(pos.x(), 6) + ", " + toString(pos.y(), 6));'''),
        ],
    }
